#Makefile at top of application tree
TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure
DIRS += bcmApp

ifeq ($(BUILD_IOCS), YES)
DIRS += bcmDemoApp
bcmDemoApp_DEPEND_DIRS += bcmApp
iocBoot_DEPEND_DIRS += bcmDemoApp
DIRS += iocBoot
endif

include $(TOP)/configure/RULES_TOP

uninstall: uninstall_iocs
uninstall_iocs:
	$(MAKE) -C iocs uninstall
.PHONY: uninstall uninstall_iocs

realuninstall: realuninstall_iocs
realuninstall_iocs:
	$(MAKE) -C iocs realuninstall
.PHONY: realuninstall realuninstall_iocs

