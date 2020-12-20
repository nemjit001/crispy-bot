@echo off

SET TOOLCHAIN_PATH=C:\nxp\MCUXpressoIDE_11.2.1_4149\ide\tools\bin
SET IDE_PATH=C:\nxp\MCUXpressoIDE_11.2.1_4149
SET IDE=%IDE_PATH%/ide/mcuxpressoide.exe

ECHO %PATH% | findstr /i /c:"%TOOLCHAIN_PATH:"=%">nul || set PATH=%PATH%;%TOOLCHAIN_PATH%

ECHO Building Project...
"%IDE%" -nosplash --launcher.suppressErrors -printErrorMarkers -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build nxp_cup/Release
"%TOOLCHAIN_PATH%\arm-none-eabi-objcopy.exe" -v -O ihex "./Release/nxp_cup.axf" "./Release/nxp_cup.hex"
ECHO Done
