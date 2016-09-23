#/usr/bin/env sh
#
# Install the software needed for a build.
#
#*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*
# 
# Copyright (c) 2015 Jeroen de Bruijn
# 
# This file is part of MPLABX_Travis-CI_Example which is released under the
# MIT License (MIT).
# For full license details see file "LICENSE" or go to
# https://opensource.org/licenses/MIT
#
#*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*
#
# Downloads the software from Microchip and saves them in a folder outside
# of the project directory. Than extracts and installs them.
# 
# Versions:
#  MPLAB® X IDE:            v3.40
#  MPLAB® XC16 Compiler:    v1.26
#  MPLAB® Pheriperhal Lib:  v2.00
#

export MPLABVER=v3.40
export XC16VER=v1.26
export PERIPHERAL_LIBRARY=v2.00

# Save the current working directory
pushd .

# Create a folder for the installation files.
mkdir ../install-dependencies
cd ../install-dependencies



### IDE
# Install MPLAB X IDE 
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading MPLAB X IDE $MPLABVER... +"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
wget http://ww1.microchip.com/downloads/en/DeviceDoc/MPLABX-$MPLABVER-linux-installer.tar
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Download finished...                          -"
echo "- Unpacking MPLABX-$MPLABVER-linux-installer.tar... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
tar -xvf MPLABX-$MPLABVER-linux-installer.tar
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Unpack finished...              -"
echo "- Installing MPLAB X IDE $MPLABVER... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
sudo ./MPLABX-$MPLABVER-linux-installer.sh -- --mode unattended
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Installation of MPLAB X IDE $MPLABVER finished... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo 

### Compiler
# Install MPLAB XC16 Compiler
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Downloading MPLAB XC16 Compiler $XC16VER -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
wget http://ww1.microchip.com/downloads/en/DeviceDoc/xc16-$XC16VER-full-install-linux-installer.run
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Download finished...        -"
echo "- Adding excecution rights... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
chmod +x xc16-$XC16VER-full-install-linux-installer.run
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Installing MPLAB XC16 Compiler $XC16VER +"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
sudo ./xc16-$XC16VER-full-install-linux-installer.run --mode unattended --netservername dontknow
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Installation of MPLAB XC16 Compiler $XC16VER finished... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo 

### Compiler
# Install MPLAB XC16 Peripheral libraries for pic24 and dspic
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Downloading MPLAB XC16 Peripheral libraries for pic24 and dspic $PERIPHERAL_LIBRARY -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
wget "http://ww1.microchip.com/downloads/en/DeviceDoc/peripheral-libraries-for-pic24-and-dspic-$PERIPHERAL_LIBRARY-linux-installer.run"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Download finished...        -"
echo "- Adding excecution rights... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
chmod +x "peripheral-libraries-for-pic24-and-dspic-$PERIPHERAL_LIBRARY-linux-installer.run"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Installing MPLAB XC16 Peripheral libraries for pic24 and dspic $PERIPHERAL_LIBRARY  -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
sudo ./"peripheral-libraries-for-pic24-and-dspic-$PERIPHERAL_LIBRARY-linux-installer.run" --mode unattended --prefix "/opt/microchip/xc16/$XC16VER"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo "- Installation MPLAB XC16 Peripheral libraries for pic24 and dspic $PERIPHERAL_LIBRARY... -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-"
echo 

# Return to the saved working directory
popd

echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- All installations finished +"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
