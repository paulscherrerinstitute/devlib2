
#include <stdlib.h>
#include <epicsAssert.h>

#include <vxWorks.h>
#include <types.h>
#include <sysLib.h>
#include <intLib.h>
#include <iv.h>
#include <drv/pci/pciIntLib.h>
#include <vxLib.h>
#include <sysSymTbl.h>

#include <dbDefs.h>

#include "devLibPCIImpl.h"
#include "osdPciShared.h"

#if defined(VXPCIINTOFFSET)
/* do nothing */

#elif defined(INT_NUM_IRQ0)
#define VXPCIINTOFFSET INT_NUM_IRQ0

#elif defined(INT_VEC_IRQ0)
#define VXPCIINTOFFSET INT_VEC_IRQ0

#else
#define VXPCIINTOFFSET 0

#endif

/*
   We cannot be sure that the BSP supports pci at all.
   To avoid load errors, we need to link dynamically.
*/

static STATUS (*pciIntConnectFunction)(VOIDFUNCPTR * vector, VOIDFUNCPTR routine, int parameter) = NULL;
static STATUS (*pciIntDisconnectFunction)(VOIDFUNCPTR * vector, VOIDFUNCPTR routine, int parameter) = NULL;

static
int vxworksDevPCIConnectInterrupt(
  const epicsPCIDevice *dev,
  void (*pFunction)(void *),
  void  *parameter,
  unsigned int opt
)
{
  int status;
  struct osdPCIDevice *osd=pcidev2osd(dev);
  
  if (pciIntConnectFunction)
  {
    status=pciIntConnectFunction((void*)INUM_TO_IVEC(VXPCIINTOFFSET + osd->dev.irq),
                       pFunction, (int)parameter);
    if (status == 0) return 0;
  }
  return S_dev_vecInstlFail;
}

static
int vxworksDevPCIDisconnectInterrupt(
  const epicsPCIDevice *dev,
  void (*pFunction)(void *),
  void  *parameter
)
{
  int status;
  struct osdPCIDevice *osd=pcidev2osd(dev);

  if (pciIntDisconnectFunction)
  {
    status=pciIntDisconnectFunction((void*)INUM_TO_IVEC(VXPCIINTOFFSET + osd->dev.irq),
                       pFunction, (int)parameter);
    if (status == 0) return 0;
  }
  return S_dev_intDisconnect;
}

static
int vxworksPCIToLocalAddr(const epicsPCIDevice* dev,
                          unsigned int bar, volatile void **loc,
                          unsigned int o)
{
  int ret, space=0;
  volatile void *pci;

  ret=sharedDevPCIToLocalAddr(dev, bar, &pci, o);

  if(ret)
    return ret;

#if defined(PCI_SPACE_MEMIO_PRI)
  if(!dev->bar[bar].ioport) {
    space = PCI_SPACE_MEMIO_PRI;
  }
#endif

#if defined(PCI_SPACE_IO_PRI)
  if(dev->bar[bar].ioport) {
    space = PCI_SPACE_IO_PRI;
  }
#endif

  if(space) {
    if(sysBusToLocalAdrs(space, (char*)pci, (char**)loc))
      return -1;
  } else {
    *loc=pci;
  }

  return 0;
}

devLibPCI pvxworksPCI = {
  "native",
  sharedDevPCIInit, NULL,
  sharedDevPCIFindCB,
  vxworksPCIToLocalAddr,
  sharedDevPCIBarLen,
  vxworksDevPCIConnectInterrupt,
  vxworksDevPCIDisconnectInterrupt
};
#include <epicsExport.h>

void devLibPCIRegisterBaseDefault(void)
{
    SYM_TYPE t;
    
    symFindByNameAndType(sysSymTbl, "pciIntConnect", (char**)&pciIntConnectFunction, &t, SYM_TEXT, SYM_MASK_ALL);
    symFindByNameAndType(sysSymTbl, "pciIntDisconnect", (char**)&pciIntDisconnectFunction, &t, SYM_TEXT, SYM_MASK_ALL);
    symFindByNameAndType(sysSymTbl, "pciIntDisconnect2", (char**)&pciIntDisconnectFunction, &t, SYM_TEXT, SYM_MASK_ALL);

    devLibPCIRegisterDriver(&pvxworksPCI);
}
epicsExportRegistrar(devLibPCIRegisterBaseDefault);
