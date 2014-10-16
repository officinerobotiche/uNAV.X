#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-XC16_dsPIC33FJ128MC804.mk)" "nbproject/Makefile-local-XC16_dsPIC33FJ128MC804.mk"
include nbproject/Makefile-local-XC16_dsPIC33FJ128MC804.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=XC16_dsPIC33FJ128MC804
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=src/I2c.c src/main.c src/communication/decode_packet.c src/communication/parsing_packet.c src/communication/serial.c src/system/system.c src/system/traps.c src/system/user.c src/system/configuration_bits.c src/system/interrupts.c src/control/high_level_control.c src/control/motors_PID.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/src/I2c.o ${OBJECTDIR}/src/main.o ${OBJECTDIR}/src/communication/decode_packet.o ${OBJECTDIR}/src/communication/parsing_packet.o ${OBJECTDIR}/src/communication/serial.o ${OBJECTDIR}/src/system/system.o ${OBJECTDIR}/src/system/traps.o ${OBJECTDIR}/src/system/user.o ${OBJECTDIR}/src/system/configuration_bits.o ${OBJECTDIR}/src/system/interrupts.o ${OBJECTDIR}/src/control/high_level_control.o ${OBJECTDIR}/src/control/motors_PID.o
POSSIBLE_DEPFILES=${OBJECTDIR}/src/I2c.o.d ${OBJECTDIR}/src/main.o.d ${OBJECTDIR}/src/communication/decode_packet.o.d ${OBJECTDIR}/src/communication/parsing_packet.o.d ${OBJECTDIR}/src/communication/serial.o.d ${OBJECTDIR}/src/system/system.o.d ${OBJECTDIR}/src/system/traps.o.d ${OBJECTDIR}/src/system/user.o.d ${OBJECTDIR}/src/system/configuration_bits.o.d ${OBJECTDIR}/src/system/interrupts.o.d ${OBJECTDIR}/src/control/high_level_control.o.d ${OBJECTDIR}/src/control/motors_PID.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/src/I2c.o ${OBJECTDIR}/src/main.o ${OBJECTDIR}/src/communication/decode_packet.o ${OBJECTDIR}/src/communication/parsing_packet.o ${OBJECTDIR}/src/communication/serial.o ${OBJECTDIR}/src/system/system.o ${OBJECTDIR}/src/system/traps.o ${OBJECTDIR}/src/system/user.o ${OBJECTDIR}/src/system/configuration_bits.o ${OBJECTDIR}/src/system/interrupts.o ${OBJECTDIR}/src/control/high_level_control.o ${OBJECTDIR}/src/control/motors_PID.o

# Source Files
SOURCEFILES=src/I2c.c src/main.c src/communication/decode_packet.c src/communication/parsing_packet.c src/communication/serial.c src/system/system.c src/system/traps.c src/system/user.c src/system/configuration_bits.c src/system/interrupts.c src/control/high_level_control.c src/control/motors_PID.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-XC16_dsPIC33FJ128MC804.mk dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ128MC804
MP_LINKER_FILE_OPTION=,--script=p33FJ128MC804.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/I2c.o: src/I2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/I2c.o.d 
	@${RM} ${OBJECTDIR}/src/I2c.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/I2c.c  -o ${OBJECTDIR}/src/I2c.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/I2c.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/I2c.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/main.o: src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/main.o.d 
	@${RM} ${OBJECTDIR}/src/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/main.c  -o ${OBJECTDIR}/src/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/communication/decode_packet.o: src/communication/decode_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/communication 
	@${RM} ${OBJECTDIR}/src/communication/decode_packet.o.d 
	@${RM} ${OBJECTDIR}/src/communication/decode_packet.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/communication/decode_packet.c  -o ${OBJECTDIR}/src/communication/decode_packet.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/communication/decode_packet.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/communication/decode_packet.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/communication/parsing_packet.o: src/communication/parsing_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/communication 
	@${RM} ${OBJECTDIR}/src/communication/parsing_packet.o.d 
	@${RM} ${OBJECTDIR}/src/communication/parsing_packet.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/communication/parsing_packet.c  -o ${OBJECTDIR}/src/communication/parsing_packet.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/communication/parsing_packet.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/communication/parsing_packet.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/communication/serial.o: src/communication/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/communication 
	@${RM} ${OBJECTDIR}/src/communication/serial.o.d 
	@${RM} ${OBJECTDIR}/src/communication/serial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/communication/serial.c  -o ${OBJECTDIR}/src/communication/serial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/communication/serial.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/communication/serial.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/system.o: src/system/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/system.o.d 
	@${RM} ${OBJECTDIR}/src/system/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/system.c  -o ${OBJECTDIR}/src/system/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/system.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/system.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/traps.o: src/system/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/traps.o.d 
	@${RM} ${OBJECTDIR}/src/system/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/traps.c  -o ${OBJECTDIR}/src/system/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/traps.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/user.o: src/system/user.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/user.o.d 
	@${RM} ${OBJECTDIR}/src/system/user.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/user.c  -o ${OBJECTDIR}/src/system/user.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/user.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/user.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/configuration_bits.o: src/system/configuration_bits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/configuration_bits.o.d 
	@${RM} ${OBJECTDIR}/src/system/configuration_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/configuration_bits.c  -o ${OBJECTDIR}/src/system/configuration_bits.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/configuration_bits.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/configuration_bits.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/interrupts.o: src/system/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/interrupts.o.d 
	@${RM} ${OBJECTDIR}/src/system/interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/interrupts.c  -o ${OBJECTDIR}/src/system/interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/interrupts.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/control/high_level_control.o: src/control/high_level_control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/control 
	@${RM} ${OBJECTDIR}/src/control/high_level_control.o.d 
	@${RM} ${OBJECTDIR}/src/control/high_level_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/control/high_level_control.c  -o ${OBJECTDIR}/src/control/high_level_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/control/high_level_control.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/control/high_level_control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/control/motors_PID.o: src/control/motors_PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/control 
	@${RM} ${OBJECTDIR}/src/control/motors_PID.o.d 
	@${RM} ${OBJECTDIR}/src/control/motors_PID.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/control/motors_PID.c  -o ${OBJECTDIR}/src/control/motors_PID.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/control/motors_PID.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1    -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/control/motors_PID.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/src/I2c.o: src/I2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/I2c.o.d 
	@${RM} ${OBJECTDIR}/src/I2c.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/I2c.c  -o ${OBJECTDIR}/src/I2c.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/I2c.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/I2c.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/main.o: src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/main.o.d 
	@${RM} ${OBJECTDIR}/src/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/main.c  -o ${OBJECTDIR}/src/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/main.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/communication/decode_packet.o: src/communication/decode_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/communication 
	@${RM} ${OBJECTDIR}/src/communication/decode_packet.o.d 
	@${RM} ${OBJECTDIR}/src/communication/decode_packet.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/communication/decode_packet.c  -o ${OBJECTDIR}/src/communication/decode_packet.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/communication/decode_packet.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/communication/decode_packet.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/communication/parsing_packet.o: src/communication/parsing_packet.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/communication 
	@${RM} ${OBJECTDIR}/src/communication/parsing_packet.o.d 
	@${RM} ${OBJECTDIR}/src/communication/parsing_packet.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/communication/parsing_packet.c  -o ${OBJECTDIR}/src/communication/parsing_packet.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/communication/parsing_packet.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/communication/parsing_packet.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/communication/serial.o: src/communication/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/communication 
	@${RM} ${OBJECTDIR}/src/communication/serial.o.d 
	@${RM} ${OBJECTDIR}/src/communication/serial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/communication/serial.c  -o ${OBJECTDIR}/src/communication/serial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/communication/serial.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/communication/serial.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/system.o: src/system/system.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/system.o.d 
	@${RM} ${OBJECTDIR}/src/system/system.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/system.c  -o ${OBJECTDIR}/src/system/system.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/system.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/system.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/traps.o: src/system/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/traps.o.d 
	@${RM} ${OBJECTDIR}/src/system/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/traps.c  -o ${OBJECTDIR}/src/system/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/traps.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/user.o: src/system/user.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/user.o.d 
	@${RM} ${OBJECTDIR}/src/system/user.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/user.c  -o ${OBJECTDIR}/src/system/user.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/user.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/user.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/configuration_bits.o: src/system/configuration_bits.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/configuration_bits.o.d 
	@${RM} ${OBJECTDIR}/src/system/configuration_bits.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/configuration_bits.c  -o ${OBJECTDIR}/src/system/configuration_bits.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/configuration_bits.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/configuration_bits.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/system/interrupts.o: src/system/interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/system 
	@${RM} ${OBJECTDIR}/src/system/interrupts.o.d 
	@${RM} ${OBJECTDIR}/src/system/interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/system/interrupts.c  -o ${OBJECTDIR}/src/system/interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/system/interrupts.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/system/interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/control/high_level_control.o: src/control/high_level_control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/control 
	@${RM} ${OBJECTDIR}/src/control/high_level_control.o.d 
	@${RM} ${OBJECTDIR}/src/control/high_level_control.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/control/high_level_control.c  -o ${OBJECTDIR}/src/control/high_level_control.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/control/high_level_control.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/control/high_level_control.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/src/control/motors_PID.o: src/control/motors_PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src/control 
	@${RM} ${OBJECTDIR}/src/control/motors_PID.o.d 
	@${RM} ${OBJECTDIR}/src/control/motors_PID.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  src/control/motors_PID.c  -o ${OBJECTDIR}/src/control/motors_PID.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/src/control/motors_PID.o.d"        -g -omf=elf -mlarge-data -O0 -I"includes" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/src/control/motors_PID.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  libdsp-elf.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    libdsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf  -mreserve=data@0x800:0x81F -mreserve=data@0x820:0x821 -mreserve=data@0x822:0x823 -mreserve=data@0x824:0x825 -mreserve=data@0x826:0x84F   -Wl,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  libdsp-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    libdsp-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}/xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/uNAV.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/XC16_dsPIC33FJ128MC804
	${RM} -r dist/XC16_dsPIC33FJ128MC804

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
