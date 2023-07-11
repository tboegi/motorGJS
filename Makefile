#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG

DIRS += configure gjsApp
gjsApp_DEPEND_DIRS   = configure

DIRS += gjsExApp
gjsExApp_DEPEND_DIRS = configure gjsApp

include $(TOP)/configure/RULES_TOP


