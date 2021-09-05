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
ifeq "$(wildcard nbproject/Makefile-local-PUBLISHER_UNIT_TESTS.mk)" "nbproject/Makefile-local-PUBLISHER_UNIT_TESTS.mk"
include nbproject/Makefile-local-PUBLISHER_UNIT_TESTS.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PUBLISHER_UNIT_TESTS
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../RC_servo.X/RC_servo.c ../System_timer.X/System_timer.c ../RC_RX.X/RC_RX.c ../nmea0183v4.X/nmea0183v4.c ../MavSerial.X/MavSerial.c ../ICM-20948.X/ICM_20948.c ../Board.X/Board.c Publisher.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1548322204/RC_servo.o ${OBJECTDIR}/_ext/698669614/System_timer.o ${OBJECTDIR}/_ext/88768175/RC_RX.o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ${OBJECTDIR}/_ext/528463759/MavSerial.o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ${OBJECTDIR}/_ext/36105697/Board.o ${OBJECTDIR}/Publisher.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1548322204/RC_servo.o.d ${OBJECTDIR}/_ext/698669614/System_timer.o.d ${OBJECTDIR}/_ext/88768175/RC_RX.o.d ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d ${OBJECTDIR}/_ext/528463759/MavSerial.o.d ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d ${OBJECTDIR}/_ext/36105697/Board.o.d ${OBJECTDIR}/Publisher.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1548322204/RC_servo.o ${OBJECTDIR}/_ext/698669614/System_timer.o ${OBJECTDIR}/_ext/88768175/RC_RX.o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ${OBJECTDIR}/_ext/528463759/MavSerial.o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ${OBJECTDIR}/_ext/36105697/Board.o ${OBJECTDIR}/Publisher.o

# Source Files
SOURCEFILES=../RC_servo.X/RC_servo.c ../System_timer.X/System_timer.c ../RC_RX.X/RC_RX.c ../nmea0183v4.X/nmea0183v4.c ../MavSerial.X/MavSerial.c ../ICM-20948.X/ICM_20948.c ../Board.X/Board.c Publisher.c



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
	${MAKE}  -f nbproject/Makefile-PUBLISHER_UNIT_TESTS.mk dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/cc91ec7bf234fd0aae1f85092c4a4f45a445e151.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/2b49804ded5ffd1520beb39c036cb28a606e27f2.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/a1d704d05716b38847813a7558e395cddc44a2e3.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/3caacd3e04010bc8de1f0fa93fa20a3dd38fa813.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/82ea1cab59d5cc0e52b420decf5f76c2a434388d.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/d2a52118973d400efd54906cc10ece688fcd6915.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/6ecb1d8fad769e3e698233e03a40ae1c25307dc2.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/Publisher.o: Publisher.c  .generated_files/dc73991db6715af749fd39901c4a6cc5e82aeea4.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Publisher.o.d 
	@${RM} ${OBJECTDIR}/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/Publisher.o.d" -o ${OBJECTDIR}/Publisher.o Publisher.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/afc4a3b321e3cab738d74e34852edaae4be52852.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/29a108260a4ea2ea3d1f1e27ebb5cf95084b87b7.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/8795899adf511caa57e0d20af97d96afb0acc842.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/93ad658edd6a210bab829584468b7874aaac43bd.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/17e9345d4fce769e6b1741cb0a52ed569ba8b728.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/a0275993cf39ced97a01cd243e0341335694fc82.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/2b73dd4c11248725896374beb01646178582a7d7.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/Publisher.o: Publisher.c  .generated_files/dea339d4b0a87c4eb11cfa980fac4763e04b523e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Publisher.o.d 
	@${RM} ${OBJECTDIR}/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/Publisher.o.d" -o ${OBJECTDIR}/Publisher.o Publisher.c    -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_PUBLISHER_UNIT_TESTS=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PUBLISHER_UNIT_TESTS
	${RM} -r dist/PUBLISHER_UNIT_TESTS

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
