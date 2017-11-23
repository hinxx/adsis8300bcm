#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += bcmApp

# do not build IOCs
#ifeq ($(BUILD_IOCS), YES)
#DIRS += bcmDemoApp
#bcmDemoApp_DEPEND_DIRS += bcmApp
#iocBoot_DEPEND_DIRS += bcmDemoApp
#DIRS += iocBoot
#endif

include $(TOP)/configure/RULES_TOP
