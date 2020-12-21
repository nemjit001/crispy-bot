#!/bin/bash

export IDE=/usr/local/mcuxpressoide-11.2.1_4149/ide/mcuxpressoide

echo "Building Project..."
$IDE -nosplash --launcher.suppressErrors -printErrorMarkers -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build nxp_cup/Release
arm-none-eabi-objcopy -v -O ihex "./Release/nxp_cup.axf" "./Release/nxp_cup.hex"
echo "Done"
