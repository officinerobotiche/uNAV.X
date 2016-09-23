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

# Create a folder for the unav libraries.
mkdir ../unav-libraries
cd ../unav-libraries

echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading or_bus_c.X library     -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
git clone https://github.com/officinerobotiche/or_bus_c.X
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading or_kernel_c.X library  -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
git clone https://github.com/officinerobotiche/or_kernel_c.X
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- Downloading or_common_c.X library  -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
git clone https://github.com/officinerobotiche/or_common_c.X

# Return to the saved working directory
popd

echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
echo "- All libraries are downloaded       -"
echo "-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+"
