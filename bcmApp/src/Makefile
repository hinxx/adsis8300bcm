TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

# The following gets rid of the -fno-implicit-templates flag on vxWorks, 
# so we get automatic template instantiation.
# This is what we want for miscellaneous/asynPortDriver.cpp
ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
CODE_CXXFLAGS=
endif

#$(warning T_A = $(T_A))
ifeq (debug,$(findstring debug, $(T_A)))
	USR_CPPFLAGS += -D_DBG=1
else
	USR_CPPFLAGS += -D_DBG=0
endif
#$(warning USR_CPPFLAGS = $(USR_CPPFLAGS))

LIBRARY_IOC = bcm

# EPICS specfic headers and sources
INC += bcm.h
LIB_SRCS += bcm.cpp

DBD += bcmSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

