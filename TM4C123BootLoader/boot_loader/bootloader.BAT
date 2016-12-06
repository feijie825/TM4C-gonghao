SET PATH=C:\Keil\ARM\ARMCC\bin\;C:\Program Files\AMD APP\bin\x86;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\Program Files\ATI Technologies\ATI.ACE\Core-Static;D:\安装到D盘软件\Flash Magic;C:\Program Files\Microsoft SQL Server\100\Tools\Binn\;C:\Program Files\Microsoft SQL Server\100\DTS\Binn\;C:\Program Files\Microsoft SQL Server\100\Tools\Binn\VSShell\Common7\IDE\;C:\Program Files\Microsoft Visual Studio 9.0\Common7\IDE\PrivateAssemblies\;C:\WINDOWS\system32\WindowsPowerShell\v1.0;D:\安装到D盘软件\AT91-ISP v1.13\Library\;D:\安装到D盘软件\AT91-ISP v1.13\sam-ba 2.9;D:\安装到D盘软件\WPS Office Personal\8.1.0.3526\office6;D:\安装到D盘软件\AT91-ISP v1.12\SAM-BA v2.8;;D:\安装到~1\FLASHM~1
SET CPU_TYPE=LM4F212E5QC
SET CPU_VENDOR=Texas Instruments
SET UV2_TARGET=bootloader
SET CPU_CLOCK=0x007A1200
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_autobaud._1i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_can._1i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_check._1i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_decrypt._1i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_flash._1i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_main._1i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_packet._1i"
"C:\Keil\ARM\ARMCC\bin\ArmAsm" --Via "bl_startup_rvmdk._ia"
"C:\Keil\ARM\ARMCC\bin\ArmLink" --Via "bootloader.l1p"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_autobaud._2i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_can._2i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_check._2i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_decrypt._2i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_flash._2i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_main._2i"
"C:\Keil\ARM\ARMCC\bin\ArmCC" --Via "bl_packet._2i"
"C:\Keil\ARM\ARMCC\bin\ArmAsm" --Via "bl_startup_rvmdk._ia"
"C:\Keil\ARM\ARMCC\bin\ArmLink" --Via "bootloader.l2p"
C:\Keil\ARM\ARMCC\bin\fromelf.exe "bootloader.axf" --i32combined --output "bootloader.hex"
c:\keil\arm\armcc\bin\fromelf.exe --bin -o .\output\bootloader.bin bootloader.axf
