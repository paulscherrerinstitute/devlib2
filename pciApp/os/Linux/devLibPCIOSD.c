
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <libgen.h>

#include <unistd.h>
#include <dirent.h>
#include <pthread.h>
#include <signal.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <epicsStdio.h>
#include <errlog.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsInterrupt.h>
#include <compilerDependencies.h>


#include "devLibPCIImpl.h"

/**@file devLibPCIOSD.c
 * @brief Userspace PCI access in Linux
 *
 * Full implementation of devLibPCI interface (search, mapping, ISR).
 *
 * Searching is general for all PCI devices (via sysfs).
 *
 * MMIO and connecting ISRs requires a UIO kernel driver.
 *
 * See README.pci in this directory and
 * Documentation/DocBook/uio-howto.tmpl in the kernel source tree.
 *
 * Note on locking: When taking both pciLock and a device lock
 *                  always take pciLock first.
 */

#ifndef CONTAINER
# ifdef __GNUC__
#   define CONTAINER(ptr, structure, member) ({                     \
        const __typeof(((structure*)0)->member) *_ptr = (ptr);      \
        (structure*)((char*)_ptr - offsetof(structure, member));    \
    })
# else
#   define CONTAINER(ptr, structure, member) \
        ((structure*)((char*)(ptr) - offsetof(structure, member)))
# endif
#endif

/* The following defintions taken from RTEMS */
#define PCI_BASE_ADDRESS_SPACE         0x01  /* 0 = memory, 1 = I/O */
#define PCI_BASE_ADDRESS_SPACE_IO      0x01
#define PCI_BASE_ADDRESS_SPACE_MEMORY  0x00
#define PCI_BASE_ADDRESS_MEM_TYPE_MASK 0x06
#define PCI_BASE_ADDRESS_MEM_TYPE_32   0x00  /* 32 bit address */
#define PCI_BASE_ADDRESS_MEM_TYPE_1M   0x02  /* Below 1M */
#define PCI_BASE_ADDRESS_MEM_TYPE_64   0x04  /* 64 bit address */
#define PCI_BASE_ADDRESS_MEM_PREFETCH  0x08  /* prefetchable? */
#define PCI_BASE_ADDRESS_MEM_MASK      (~0x0fUL)
#define PCI_BASE_ADDRESS_IO_MASK      (~0x03UL)

/**@brief Info of a single PCI device
 *
 * Lifetime: Created in linuxDevPCIInit and free'd in linuxDevFinal
 *
 * Access after init is guarded by devLock
 */
struct osdPCIDevice {
    epicsPCIDevice dev; /* "public" data */
    unsigned int domain; /* PCI domain */

    /* result of mmap(), add offset before passing to user */
    volatile void *base[PCIBARCOUNT];
    /* offset from start of page to start of BAR */
    epicsUInt32    offset[PCIBARCOUNT];
    /* BAR length (w/o offset) */
    epicsUInt32    len[PCIBARCOUNT];
    volatile void *erom;                                            /* UNUSED */
    epicsUInt32    eromlen;                                         /* UNUSED */

    epicsUInt32 displayBAR[PCIBARCOUNT]; /* Raw PCI address */      /* UNUSED */
    epicsUInt32 displayErom;                                        /* UNUSED */

    char *linuxDriver;                                              /* UNUSED */

    int fd; /* /dev/uio# */
    char* devname;

    epicsMutexId devLock; /* guard access to isrs list */

    ELLNODE node;

    ELLLIST isrs; /* contains struct osdISR */
};
typedef struct osdPCIDevice osdPCIDevice;

#define dev2osd(dev) CONTAINER(dev, osdPCIDevice, dev)

struct osdISR {
    ELLNODE node;

    epicsThreadId waiter;
    epicsEventId done;
    enum {
        osdISRStarting=0, /* started, id not filled */
        osdISRRunning, /* id filled, normal operation */
        osdISRStopping, /* stop required */
        osdISRDone, /* thread done, can free resources */
    } waiter_status;

    osdPCIDevice *osd;

    EPICSTHREADFUNC fptr;
    void  *param;
};
typedef struct osdISR osdISR;

static
void isrThread(void*);

static
void stopIsrThread(osdISR *isr);

static
ELLLIST devices = {{NULL,NULL},0}; /* list of osdPCIDevices::node */

/* guard access to 'devices' list */
static
epicsMutexId pciLock=NULL;

static
long pagesize;

#define BUSBASE "/sys/bus/pci/devices/%04x:%02x:%02x.%1x/"

#define UIONUM     "uio%u"

#define fbad(FILE) ( feof(FILE) || ferror(FILE))

/* vsprintf() w/ allocation.  The result must be free'd!
 */
static
char*
vallocPrintf(const char *format, va_list args)
{
    va_list nargs;
    char* ret=NULL;
    int size, size2;

    /* May use a va_list only *once* (on some implementations it may
     * be a reference to something that holds internal state information
     *
     * Luckily, C99 provides va_copy.
     * but some compilers only have __va_copy instead
     */

    #ifndef va_copy
    #define va_copy(nargs, args) __va_copy(nargs, args)
    #endif
     
    va_copy(nargs, args);

    /* Take advantage of the fact that sprintf will tell us how much space to allocate */
    size=vsnprintf("",0,format,nargs);

    va_end(nargs);

    if (size<=0) {
        errlogPrintf("vaprintf: Failed to convert format '%s'\n",format);
        goto fail;
    }
    ret=malloc(size+1);
    if (!ret) {
        errlogPrintf("vaprintf: Failed to allocate memory for format '%s'\n",format);
        goto fail;
    }
    size2=vsnprintf(ret,size+1,format,args);
    if (size!=size2) {
        errlogPrintf("vaprintf: Format yielded different size %d %d : %s\n",size,size2,format);
        goto fail;
    }

    return ret;
fail:
    free(ret);
    return NULL;
}

static
char*
allocPrintf(const char *format, ...) EPICS_PRINTF_STYLE(1,2);

static
char*
allocPrintf(const char *format, ...)
{
    char* ret;
    va_list args;
    va_start(args, format);
    ret=vallocPrintf(format,args);
    va_end(args);
    return ret;
}

/* Read a file containing only an integer number, hex or decimal.
 * This format is common in the /sys/ tree.
 */
static
unsigned long
vread_sysfs(int *err, const char *fileformat, va_list args)
{
    long ret=0;
    int size;
    char *scratch=NULL;
    FILE *fd=NULL;

    if (*err) return ret;
    *err=1;

    scratch=vallocPrintf(fileformat, args);
    if (!scratch)
        goto done;

    fd=fopen(scratch, "r");
    if (!fd) {
        errlogPrintf("vread_sysfs: Failed to open %s\n", scratch);
        goto done;
    }
    size=fscanf(fd, "%li",&ret);
    if(size!=1 || ferror(fd)) {
        errlogPrintf("vread_sysfs: Failed to read %s\n", scratch);
        goto done;
    }

    *err=0;
done:
    if (fd) fclose(fd);
    free(scratch);
    return ret;
}

static
unsigned long
read_sysfs(int *err, const char *fileformat, ...) EPICS_PRINTF_STYLE(2,3);

static
unsigned long
read_sysfs(int *err, const char *fileformat, ...)
{
    unsigned long ret;
    va_list args;
    va_start(args, fileformat);
    ret=vread_sysfs(err,fileformat,args);
    va_end(args);
    return ret;
}

/* location of UIO entries in sysfs tree
 *
 * circa 2.6.28
 * in /sys/bus/pci/devices/0000:%02x:%02x.%1x/
 * called uio:uio#
 *
 * circa 2.6.32
 * in /sys/bus/pci/devices/0000:%02x:%02x.%1x/uio/
 * called uio#
 */
static const
struct locations_t {
    const char *dir, *name;
} locations[] = {
{BUSBASE,        "uio:" UIONUM},
{BUSBASE "uio/", UIONUM},
{NULL,NULL}
};

static
int match_uio(const char *pat, const struct dirent *ent)
{
    unsigned int X;
    /* poor mans regexp... */
    return sscanf(ent->d_name, pat, &X)==1;
}

static
int find_uio_number2(const char* dname, const char* pat)
{
    int ret=-1;
    DIR *d;
    struct dirent *ent;

    d=opendir(dname);
    if (!d)
        return ret;

    while ((ent=readdir(d))!=NULL) {
        if (!match_uio(pat, ent))
            continue;
        if (sscanf(ent->d_name, pat, &ret)==1) {
            break;
        }
        ret=-1;
    }

    closedir(d);

    if (ret==-1)
        errno=ENOENT;

    return ret;
}

/* Each PCI device with a UIO instance attached to in should have
 * a directory like /sys/bus/pci/devices/xxxxxx/uio:uio#/...
 * and this call should return '#' it such a directory exists.
 */
static
int
find_uio_number(const struct osdPCIDevice* osd)
{
    int ret=-1;
    char *devdir=NULL;
    const struct locations_t *curloc;

    for(curloc=locations; curloc->dir; ++curloc)
    {
        free(devdir);

        devdir=allocPrintf(curloc->dir, osd->domain, osd->dev.bus, osd->dev.device, osd->dev.function);
        if (!devdir)
            goto fail;
        ret=find_uio_number2(devdir, curloc->name);
        if (ret<0) {
            if(errno==ENOENT)
                continue;

            errlogPrintf("find_uio_number: Search of %s failed\n",devdir);
            perror("opendir");
            goto fail;

        } else
            break;
    }

/*
    if (ret==-1) {
        errlogPrintf("After looking:\n");
        for(curloc=locations; curloc->dir; ++curloc)
        {
            devdir=allocPrintf(curloc->dir, osd->domain,osd->dev.bus, osd->dev.device, osd->dev.function);
            errlogPrintf("in %s for %s\n",devdir,curloc->name);
            free(devdir);
        }
        devdir=NULL;
        errlogPrintf("Failed to find device %u:%u.%u\n",
                     osd->dev.bus, osd->dev.device, osd->dev.function);
    }
*/

    /* ret set by sscanf */
fail:
    free(devdir);
    return ret;
}

static
int
open_uio(struct osdPCIDevice* osd)
{
    int uio, ret=1;
    if (osd->fd!=-1) return 0;

    uio=find_uio_number(osd);
    if (uio<0) goto fail;

    osd->devname=allocPrintf("/dev/uio%u", uio);
    if (!osd->devname) goto fail;

    /* First try to open /dev/uio# */
    osd->fd=open(osd->devname,O_RDWR);
    if (osd->fd==-1) {
        /* TODO: try to create? */
        perror("Failed to open UIO device file");
        errlogPrintf("Could not open device file %s.\n",osd->devname);
        goto fail;
    }

    ret=0;
fail:
    return ret;
}

static
void
close_uio(struct osdPCIDevice* osd)
{
    int i;

    for(i=0; i<PCIBARCOUNT; i++) {
        if (!osd->base[i]) continue;

        munmap((void*)osd->base[i], osd->offset[i]+osd->len[i]);
        osd->base[i]=NULL;
    }

    if (osd->fd!=-1) close(osd->fd);
    osd->fd=-1;
}

static
int linuxDevPCIInit(void)
{
    DIR* sysfsPci_dir = NULL;
    struct dirent* dir;
    int i;
    osdPCIDevice *osd=NULL;
    pciLock = epicsMutexMustCreate();

    pagesize=sysconf(_SC_PAGESIZE);
    if (pagesize==-1) {
        perror("Failed to get pagesize");
        goto fail;
    }

    sysfsPci_dir = opendir("/sys/bus/pci/devices");
    if(!sysfsPci_dir){
    	errlogPrintf("Could not open /sys/bus/pci/devices!\n");
    	goto fail;
    }

    while ((dir=readdir(sysfsPci_dir))) {
        int fail=0;
        int matched;

    	if (!dir->d_name || dir->d_name[0]=='.') continue; /* Skip invalid entries */

        osd=calloc(1, sizeof(osdPCIDevice));
        if (!osd) {
            errMessage(S_dev_noMemory, "Out of memory");
            goto fail;
        }
        osd->fd=-1;


        matched = sscanf(dir->d_name, "%4x:%2x:%2x.%x", &osd->domain, &osd->dev.bus, &osd->dev.device, &osd->dev.function);
        if (matched != 4) {
            errlogPrintf("Could not decode PCI device directory %s\n",dir->d_name);
        }

        osd->dev.id.vendor=read_sysfs(&fail, BUSBASE "vendor",osd->domain, osd->dev.bus, osd->dev.device, osd->dev.function);
        osd->dev.id.device=read_sysfs(&fail, BUSBASE "device",osd->domain, osd->dev.bus, osd->dev.device, osd->dev.function);
        osd->dev.irq=read_sysfs(&fail, BUSBASE "irq", osd->domain, osd->dev.bus, osd->dev.device, osd->dev.function);
        osd->dev.id.sub_vendor=read_sysfs(&fail, BUSBASE "subsystem_vendor", osd->domain, osd->dev.bus, osd->dev.device, osd->dev.function);
        osd->dev.id.sub_device=read_sysfs(&fail, BUSBASE "subsystem_device",osd->domain,
                                              osd->dev.bus, osd->dev.device, osd->dev.function);
        osd->dev.id.pci_class= read_sysfs(&fail, BUSBASE "class",osd->domain,
                                              osd->dev.bus, osd->dev.device, osd->dev.function);
        osd->dev.id.revision=0;

        if (fail) {
            errlogPrintf("Warning: Failed to read some attributes of PCI %u:%u.%u\n"
                         "         This may cause some searches to fail\n",
                         osd->dev.bus, osd->dev.device, osd->dev.function);
            fail=0;
        }

        if(devPCIDebug>=1) {
            errlogPrintf("linuxDevPCIInit found %d.%d.%d\n",
                         osd->dev.bus, osd->dev.device, osd->dev.function);
            errlogPrintf(" as pri %04x:%04x sub %04x:%04x cls %06x\n",
                         osd->dev.id.vendor, osd->dev.id.device,
                         osd->dev.id.sub_vendor, osd->dev.id.sub_device,
                         osd->dev.id.pci_class);
            errlogFlush();
        }

        /* Read driver name */

        char *driver_dir = allocPrintf(BUSBASE "driver", osd->domain, osd->dev.bus, osd->dev.device, osd->dev.function);
        if (driver_dir)
        {
            /* driver_dir is a symlink of the form ../../../../bus/pci/drivers/drivername  (28+x chars) */
            char link [80];
            memset(link, 0, sizeof(link));
            readlink(driver_dir, link, sizeof(link)-1);
            osd->linuxDriver = epicsStrDup(basename(link));
            free(driver_dir);
        }
        if (!osd->linuxDriver)
            errlogPrintf("Warning: Failed to copy driver name\n");

        /* Read BAR info */

        /* Base address */
        char* resource_filename = allocPrintf(BUSBASE"resource",osd->domain,osd->dev.bus,osd->dev.device,osd->dev.function);
        FILE* resource_fptr = fopen(resource_filename,"r");
        if(!resource_fptr){
        	printf("Could not open resource file!\n");
        	continue;
        }
                
        for (i=0; i<=PCIBARCOUNT; i++) {
            unsigned long long int start,stop,len,flags;
            int match = fscanf(resource_fptr, "0x%16llx 0x%16llx 0x%16llx\n", &start, &stop, &flags);
            if (match != 3) {
                printf("Could not parse resource file line %d column %d. Aborting\n", i+1, match);
                break;
            }

            if (i < PCIBARCOUNT)
            {
                osd->dev.bar[i].ioport=(flags & PCI_BASE_ADDRESS_SPACE)==PCI_BASE_ADDRESS_SPACE_IO;
                if (osd->dev.bar[i].ioport) {
                    /* This BAR is I/O ports */
                    osd->dev.bar[i].below1M=0;
                    osd->dev.bar[i].addr64=0;
                } else {
                    /* This BAR is memory mapped */
                    osd->dev.bar[i].below1M=!!(flags&PCI_BASE_ADDRESS_MEM_TYPE_1M);
                    osd->dev.bar[i].addr64=!!(flags&PCI_BASE_ADDRESS_MEM_TYPE_64);
                }
                /* region start */
                osd->displayBAR[i] = start;
                /* offset from start of page to start of BAR */
                osd->offset[i] = start&(pagesize-1);
                /* region length */
                len = stop-start;
                if (len) len++;
                osd->len[i] = len;
            } else {
                /* rom */
                osd->displayErom = start;
                osd->eromlen = (stop-start+1);
            }
        }

        osd->devLock = epicsMutexMustCreate();

        ellAdd(&devices, &osd->node);
        osd=NULL;
    }

    return 0;
fail:
    if (sysfsPci_dir) closedir(sysfsPci_dir);
    epicsMutexDestroy(pciLock);
    return S_dev_badInit;
}

static
int linuxDevFinal(void)
{
    ELLNODE *cur, *next, *isrcur, *isrnext;
    osdPCIDevice *curdev=NULL;
    osdISR *isr;

    epicsMutexMustLock(pciLock);
    for(cur=ellFirst(&devices), next=cur ? ellNext(cur) : NULL;
        cur;
        cur=next, next=next ? ellNext(next) : NULL )
    {
        curdev=CONTAINER(cur,osdPCIDevice,node);

        epicsMutexMustLock(curdev->devLock);

        for(isrcur=ellFirst(&curdev->isrs), isrnext=isrcur ? ellNext(isrcur) : NULL;
            isrcur;
            isrcur=isrnext, isrnext=isrnext ? ellNext(isrnext) : NULL )
        {
            isr=CONTAINER(isrcur,osdISR,node);

            stopIsrThread(isr);

            ellDelete(&curdev->isrs,cur);
            free(isr);

        }

        close_uio(curdev);

        epicsMutexUnlock(curdev->devLock);
        epicsMutexDestroy(curdev->devLock);
        free(curdev->linuxDriver);
        free(curdev);
    }
    epicsMutexUnlock(pciLock);
    epicsMutexDestroy(pciLock);

    return 0;
}

static
int
linuxDevPCIFindCB(
     const epicsPCIID *idlist,
     devPCISearchFn searchfn,
     void *arg,
     unsigned int opt /* always 0 */
)
{
  int err=0, ret=0;
  ELLNODE *cur;
  osdPCIDevice *curdev=NULL;
  const epicsPCIID *search;

  if(!searchfn || !idlist)
    return S_dev_badArgument;

  if(epicsMutexLock(pciLock)!=epicsMutexLockOK)
      return S_dev_internal;

  cur=ellFirst(&devices);
  for(; cur; cur=ellNext(cur)){
      curdev=CONTAINER(cur,osdPCIDevice,node);
      if(epicsMutexLock(curdev->devLock)!=epicsMutexLockOK) {
          ret=S_dev_internal;
          goto done;
      }

      for(search=idlist; search->device!=DEVPCI_LAST_DEVICE; search++){

          if(search->device!=DEVPCI_ANY_DEVICE &&
             search->device!=curdev->dev.id.device)
              continue;
          else
              if(search->vendor!=DEVPCI_ANY_DEVICE &&
                 search->vendor!=curdev->dev.id.vendor)
                  continue;
          else
              if( search->sub_device!=DEVPCI_ANY_SUBDEVICE &&
                  search->sub_device!=curdev->dev.id.sub_device
                  )
                  continue;
          else
              if( search->sub_vendor!=DEVPCI_ANY_SUBVENDOR &&
                  search->sub_vendor!=curdev->dev.id.sub_vendor
                  )
                  continue;
          else
              if( search->pci_class!=DEVPCI_ANY_CLASS &&
                  search->pci_class!=curdev->dev.id.pci_class
                  )
                  continue;
          else
              if( search->revision!=DEVPCI_ANY_REVISION &&
                  search->revision!=curdev->dev.id.revision
                  )
                  continue;

          /* Match found */

          err=searchfn(arg,&curdev->dev);
          if(err==0) /* Continue search */
              continue;
          else if(err==1) /* Abort search OK */
              ret=0;
          else /* Abort search Err */
              ret=err;
          epicsMutexUnlock(curdev->devLock);
          goto done;

      }

      epicsMutexUnlock(curdev->devLock);

  }

done:
  epicsMutexUnlock(pciLock);

  return ret;
}

static
int
linuxDevPCIToLocalAddr(
  const epicsPCIDevice* dev,
  unsigned int bar,
  volatile void **ppLocalAddr,
  unsigned int opt
)
{
    int mapno,i;
    osdPCIDevice *osd=CONTAINER((epicsPCIDevice*)dev,osdPCIDevice,dev);

    if(epicsMutexLock(osd->devLock)!=epicsMutexLockOK)
        return S_dev_internal;

    if (open_uio(osd)) {
        epicsMutexUnlock(osd->devLock);
        return S_dev_addrMapFail;
    }

    if (!osd->base[bar]) {

        if ( osd->dev.bar[bar].ioport ) {
            errlogPrintf("Failed to MMAP BAR %u of %u:%u.%u -- mapping of IOPORTS is not possible\n", bar,
                         osd->dev.bus, osd->dev.device, osd->dev.function);
            return S_dev_addrMapFail;
        }

        if (opt&DEVLIB_MAP_UIOCOMPACT) {
            /* mmap requires the number of *mappings* times pagesize;
             * valid mappings are only PCI memory regions.
             * Let's count them here
             */
            for ( i=0, mapno=bar; i<=bar; i++ ) {
                if ( osd->dev.bar[i].ioport ) {
                    mapno--;
                }
            }
        } else
            mapno=bar;

        printf ("mmap(NULL,%#x,PROT_READ|PROT_WRITE, MAP_SHARED, open(\"%s\",O_RDWR), %#lx)\n",
            osd->len[bar], osd->devname, mapno*pagesize);
        osd->base[bar] = mmap(NULL,osd->len[bar],
                              PROT_READ|PROT_WRITE, MAP_SHARED,
                              osd->fd, mapno*pagesize);

        if (osd->base[bar]==MAP_FAILED) {
            perror("Failed to map BAR");
            errlogPrintf("Failed to MMAP BAR %u of %u:%u.%u\n", bar,
                         osd->dev.bus, osd->dev.device, osd->dev.function);
            epicsMutexUnlock(osd->devLock);
            return S_dev_addrMapFail;
        }

/*        printf("MMAP%d sucess!\n", bar); */

    }

    *ppLocalAddr=osd->base[bar] + osd->offset[bar];

    epicsMutexUnlock(osd->devLock);
    return 0;
}

static
int
linuxDevPCIBarLen(
  const epicsPCIDevice* dev,
  unsigned int bar,
  epicsUInt32 *len
)
{
    osdPCIDevice *osd=CONTAINER(dev,osdPCIDevice,dev);

    if(epicsMutexLock(osd->devLock)!=epicsMutexLockOK)
        return -1;
    *len=osd->len[bar];
    epicsMutexUnlock(osd->devLock);
    return 0;
}

static
int linuxDevPCIConnectInterrupt(
  const epicsPCIDevice *dev,
  void (*pFunction)(void *),
  void  *parameter,
  unsigned int opt
)
{
    char name[10];
    ELLNODE *cur;
    osdPCIDevice *osd=CONTAINER((epicsPCIDevice*)dev,osdPCIDevice,dev);
    osdISR *other, *isr=calloc(1,sizeof(osdISR));

    if (!isr) return S_dev_noMemory;

    isr->fptr=pFunction;
    isr->param=parameter;
    isr->osd=osd;
    isr->waiter_status=osdISRStarting;
    isr->done=epicsEventCreate(epicsEventEmpty);

    if(!isr->done || epicsMutexLock(osd->devLock)!=epicsMutexLockOK) {
        free(isr);
        return S_dev_internal;
    }

    for(cur=ellFirst(&osd->isrs); cur; cur=ellNext(cur))
    {
        other=CONTAINER(cur,osdISR,node);
        if (other->fptr==isr->fptr && other->param==isr->param) {
            epicsMutexUnlock(osd->devLock);
            errlogPrintf("ISR already registered\n");
            goto error;
        }
    }

    epicsSnprintf(name,NELEMENTS(name),"%02xPCIISR",dev->irq);
    name[NELEMENTS(name)-1]='\0';

    /* Ensure that "IRQ" thread has higher priority
     * then all other EPICS threads.
     */
    isr->waiter = epicsThreadCreate(name,
                                    epicsThreadPriorityMax-1,
                                    epicsThreadGetStackSize(epicsThreadStackMedium),
                                    isrThread,
                                    isr
                                    );
    if (!isr->waiter) {
        epicsMutexUnlock(osd->devLock);
        errlogPrintf("Failed to create ISR thread %s\n", name);
        goto error;
    }

    ellAdd(&osd->isrs,&isr->node);
    epicsMutexUnlock(osd->devLock);

    return 0;
error:
    epicsEventDestroy(isr->done);
    free(isr);
    return S_dev_vecInstlFail;
}

static
void isrThread(void* arg)
{
    osdISR *isr=arg;
    osdPCIDevice *osd=isr->osd;
    int interrupted=0, ret;
    epicsInt32 event, next=0;
    const char* name;
    int isrflag;

    name=epicsThreadGetNameSelf();

    if(epicsMutexLock(osd->devLock)!=epicsMutexLockOK) {
        errlogMessage("Can't lock ISR thread");
        return;
    }

    if (isr->waiter_status!=osdISRStarting) {
        isr->waiter_status = osdISRDone;
        epicsMutexUnlock(osd->devLock);
        return;
    }

    isr->waiter_status = osdISRRunning;

    while (isr->waiter_status==osdISRRunning) {
        epicsMutexUnlock(osd->devLock);

        /* The interrupted flag lets us check
         * the status flag (taking devLock)
         * once each iteration
         */
        if (interrupted) {
            interrupted=0;
            isrflag=epicsInterruptLock();
            (isr->fptr)(isr->param);
            epicsInterruptUnlock(isrflag);
        }

        ret=read(osd->fd, &event, sizeof(event));
        if (ret==-1) {
            switch(errno) {
            case EINTR: /* interrupted by a signal */
                break;
            default:
                errlogPrintf("isrThread '%s' read error %d\n",
                             name,errno);
                epicsThreadSleep(0.5);
            }
        } else
            interrupted=1;

        if (next!=event && next!=0) {
            errlogPrintf("isrThread '%s' missed %d events\n",
                         name, event-next);
        }
        next=event+1;

        if(epicsMutexLock(osd->devLock)!=epicsMutexLockOK) {
            errlogMessage("Failed to relock ISR thread\n");
            isr->waiter_status = osdISRDone;
            return;
        }
    }

    isr->waiter_status = osdISRDone;

    epicsMutexUnlock(osd->devLock);
    epicsEventSignal(isr->done);
}

/* Caller must take devLock */
static
void
stopIsrThread(osdISR *isr)
{
    if (isr->waiter_status==osdISRDone)
        return;

    isr->waiter_status = osdISRStopping;

    while (isr->waiter_status!=osdISRDone) {
        epicsMutexUnlock(isr->osd->devLock);

        epicsEventWait(isr->done);

        epicsMutexMustLock(isr->osd->devLock);
    }
}

static
int linuxDevPCIDisconnectInterrupt(
  const epicsPCIDevice *dev,
  void (*pFunction)(void *),
  void  *parameter
)
{
    int ret=S_dev_intDisconnect;
    ELLNODE *cur;
    osdISR *isr;
    osdPCIDevice *osd=CONTAINER((epicsPCIDevice*)dev,osdPCIDevice,dev);

    if(epicsMutexLock(osd->devLock)!=epicsMutexLockOK)
        return S_dev_internal;

    for(cur=ellFirst(&osd->isrs); cur; cur=ellNext(cur))
    {
        isr=CONTAINER(cur,osdISR,node);

        if (pFunction==isr->fptr && parameter==isr->param) {

            stopIsrThread(isr);

            ellDelete(&osd->isrs,cur);
            epicsEventDestroy(isr->done);
            free(isr);

            ret=0;
            break;
        }
    }
    epicsMutexUnlock(osd->devLock);

    return ret;
}

devLibPCI plinuxPCI = {
  "native",
  linuxDevPCIInit, linuxDevFinal,
  linuxDevPCIFindCB,
  linuxDevPCIToLocalAddr,
  linuxDevPCIBarLen,
  linuxDevPCIConnectInterrupt,
  linuxDevPCIDisconnectInterrupt
};
#include <epicsExport.h>

void devLibPCIRegisterBaseDefault(void)
{
    devLibPCIRegisterDriver(&plinuxPCI);
}
epicsExportRegistrar(devLibPCIRegisterBaseDefault);
