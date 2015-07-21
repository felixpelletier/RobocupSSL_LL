## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,28L linker.cmd package/cfg/app_p28L.o28L

linker.cmd: package/cfg/app_p28L.xdl
	$(SED) 's"^\"\(package/cfg/app_p28Lcfg.cmd\)\"$""\"/home/mathieu/sourceRepos/Ccs/RobocupSSL_LL/RobocupV2_2/.config/xconfig_app/\1\""' package/cfg/app_p28L.xdl > $@
