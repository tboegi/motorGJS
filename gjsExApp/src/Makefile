TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = gjs

# <name>.dbd will be created from <name>Include.dbd
DBD += gjs.dbd
gjs_DBD += base.dbd
gjs_DBD += asyn.dbd
gjs_DBD += drvAsynIPPort.dbd
gjs_DBD += motorSupport.dbd
gjs_DBD += devSoftMotor.dbd
gjs_LIBS += motor
gjs_LIBS += softMotor

ifneq (,$(findstring /,$(CALC)))
  gjs_DBD += calcSupport.dbd
  gjs_LIBS += calc
endif

ifdef BASE_7_0
 gjs_DBD += PVAServerRegister.dbd
 gjs_DBD += PVACommonRegister.dbd
 gjs_DBD += softIocPVA.dbd
 gjs_DBD += registerChannelProviderLocal.dbd
endif

gjs_DBD += gjsSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
gjs_SRCS += gjs_registerRecordDeviceDriver.cpp
gjs_SRCS_DEFAULT += gjsMain.cpp

#add a definition for each support application used by this application
gjs_LIBS += asyn
gjs_LIBS += motor
gjs_LIBS += softMotor

ifneq (,$(findstring /,$(CALC)))
gjs_LIBS += calc
endif

ifdef BASE_7_0
gjs_LIBS += qsrv
gjs_LIBS += pvDatabase
gjs_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
endif

gjs_LIBS += gjsSupport
gjs_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
