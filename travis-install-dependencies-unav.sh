#/usr/bin/env sh
#
# Install the uNAV required libraries needed for a build.
#
#*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*
# 
# Copyright (c) 2016 Raffaello Bonghi
# 
# This file is part of uNAV project which is released under the
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
#  MPLAB® X IDE:         v3.15
#  MPLAB® XC16 Compiler: v1.25
#

# Save the current working directory
pushd .

# Go to the home folder for the unav libraries.
cd ../

export MPLABVER=v1.25
echo "- MPLAB VER: &MPLABVER -"

echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading or_bus_c.X library     -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
git clone https://github.com/officinerobotiche/or_bus_c.X
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Build or_bus_c.X library           -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
cd or_bus_c.X/
/opt/microchip/mplabx/v3.15/mplab_ide/bin/prjMakefilesGenerator.sh .
make
cd ../
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading or_kernel_c.X library  -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
git clone https://github.com/officinerobotiche/or_kernel_c.X
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Build or_kernel_c.X library        -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
cd or_kernel_c.X/
/opt/microchip/mplabx/v3.15/mplab_ide/bin/prjMakefilesGenerator.sh .
make
cd ../
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading or_common_c.X library  -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
git clone https://github.com/officinerobotiche/or_common_c.X
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Build or_commont_c.X library       -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
cd or_common_c.X/
/opt/microchip/mplabx/v3.15/mplab_ide/bin/prjMakefilesGenerator.sh .
make
cd ../

# Return to the saved working directory
popd

echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- All libraries are downloaded       -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
