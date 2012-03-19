include /ioc/tools/driver.makefile

EXCLUDE_VERSIONS = 3.13
BUILDCLASSES += Linux

USR_CFLAGS += -I ../common
USR_CFLAGS += -I ../common/os/$(OS_CLASS)
USR_CFLAGS += -I ../common/os/default
USR_CFLAGS += -I ../pciApp/os/$(OS_CLASS)

SOURCES += pciApp/devLibPCI.c
SOURCES += pciApp/pcish.c
SOURCES_vxWorks += pciApp/osdPciShared.c
SOURCES_vxWorks += pciApp/os/vxWorks/devLibPCIOSD.c
SOURCES_Linux += pciApp/os/Linux/devLibPCIOSD.c

SOURCES += vmeApp/devcsr.c
SOURCES += vmeApp/iocreg.c
SOURCES += vmeApp/vmesh.c
SOURCES += vmeApp/devlib_compat.c

ifneq ($(findstring $(EPICS_MODIFICATION),1 2 3 4 5 6 7 8 9),)
SOURCES += vmeApp/devLibVME.c
SOURCES_vxWorks += vmeApp/os/vxWorks/devLibVMEOSD.c
SOURCES_Linux += vmeApp/os/default/devLibVMEOSD.c
HEADERS += devLibVME.h
endif

DBDS += pciApp/epicspci.dbd
DBDS += vmeApp/epicsvme.dbd
