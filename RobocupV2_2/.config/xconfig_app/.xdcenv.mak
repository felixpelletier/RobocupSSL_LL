#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /opt/ti/bios_6_35_04_50/packages;/opt/ti/ccsv5/ccs_base;/home/mathieu/sourceRepos/Ccs/RobocupSSL_LL/RobocupV2_2/.config
override XDCROOT = /opt/ti/xdctools_3_25_03_72
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /opt/ti/bios_6_35_04_50/packages;/opt/ti/ccsv5/ccs_base;/home/mathieu/sourceRepos/Ccs/RobocupSSL_LL/RobocupV2_2/.config;/opt/ti/xdctools_3_25_03_72/packages;..
HOSTOS = Linux
endif
