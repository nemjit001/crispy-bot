#!/bin/bash

export IDE=

echo "Building Project..."
$IDE -nosplash --launcher.suppressErrors -printErrorMarkers -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build nxp_cup/Release
arm-none-eabi-objcopy -v -O ihex "./Release/nxp_cup.axf" "./Release/nxp_cup.hex"
echo "Done"
