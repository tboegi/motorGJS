TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += gjsSupport

DBD += gjsSupport.dbd

# The following are compiled and added to the Support library

gjsSupport_SRCS += GJSDriver.cpp
#gjsSupport_SRCS += gjsAxis.cpp
gjsSupport_LIBS += asyn

ifneq (,$(findstring /,$(CALC)))
gjsSupport_LIBS += calc
endif
ifneq (,$(findstring /,$(MOTOR)))
gjsSupport_LIBS += motor
gjsSupport_LIBS += softMotor
endif
ifneq (,$(findstring /,$(STREAM)))
gjsSupport_LIBS += stream
endif


gjsSupport_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
