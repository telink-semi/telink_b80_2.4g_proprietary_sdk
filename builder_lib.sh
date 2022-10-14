#!/bin/bash
C:/TelinkSDK/bin/find ./esb_ll/ -name "*.o" -type f -print -exec tc32-elf-ar -r esb_ll.a {} \;
if [ $? == 0 ]
then
    cp esb_ll.a ../esb_ll
    if [ $? == 0 ]
    then
        cd ../esb_ll
        rm esb_ll.c
        if [ $? == 0 ]
        then
        echo "****create esb_ll.a success****"
        fi
    fi
fi

cd ../BUILD_CORE_LIB
C:/TelinkSDK/bin/find ./tl_esb_ll/ -name "*.o" -type f -print -exec tc32-elf-ar -r tl_esb_ll.a {} \;
if [ $? == 0 ]
then
    cp tl_esb_ll.a ../tl_esb_ll
    if [ $? == 0 ]
    then
        cd ../tl_esb_ll
        rm tl_esb_ll.c
        if [ $? == 0 ]
        then
        echo "****create tl_esb_ll.a success****"
        fi
    fi
fi

cd ../BUILD_CORE_LIB
C:/TelinkSDK/bin/find ./genfsk_ll/ -name "*.o" -type f -print -exec tc32-elf-ar -r genfsk_ll.a {} \;
if [ $? == 0 ]
then
    cp genfsk_ll.a ../genfsk_ll
    if [ $? == 0 ]
    then
        cd ../genfsk_ll
        rm genfsk_ll.c
        if [ $? == 0 ]
        then
        echo "****create genfsk_ll.a success****"
        fi
    fi
fi

cd ../BUILD_CORE_LIB/drivers
C:/TelinkSDK/opt/tc32/bin/tc32-elf-ar -r 8208_core_drv_lib.a rf_drv.o pm.o pm_32k_rc.o pm_32k_xtal.o pm_32k_calib.o
if [ $? == 0 ]
then
    cp 8208_core_drv_lib.a ../../drivers
    if [ $? == 0 ]
    then
        cd ../../drivers
        rm rf_drv.c pm.c pm_32k_rc.c pm_32k_xtal.c pm_32k_calib.c
        if [ $? == 0 ]
        then
        echo "***create 8208_core_drv_lib.a success****"
        fi
    fi
fi


