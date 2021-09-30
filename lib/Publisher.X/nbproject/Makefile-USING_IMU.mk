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
ifeq "$(wildcard nbproject/Makefile-local-USING_IMU.mk)" "nbproject/Makefile-local-USING_IMU.mk"
include nbproject/Makefile-local-USING_IMU.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=USING_IMU
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
	${MAKE}  -f nbproject/Makefile-USING_IMU.mk dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/8149793dd28cc5fa11e7b8634c3eb0a92b220e66.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/b0850ea8e0fd34fe5abecbf41227504e5e391e47.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/a0989245f7aa1a17a15c3e1264db1290e89d88b9.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/675b0edf8120d727546e384eb9c606fb226f5041.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/69d8c6aa846af188f4ba847e15e0aca9a9fd9e3f.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/a9053ca3519bc8efce21af2318085f23212871f4.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/68df470e3765233adfb6c655981ba5b4e6b60703.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/Publisher.o: Publisher.c  .generated_files/313ad9b8cfc6a9cb7a782cdf932730b52c762e61.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Publisher.o.d 
	@${RM} ${OBJECTDIR}/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/Publisher.o.d" -o ${OBJECTDIR}/Publisher.o Publisher.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/e3c0d74bf8a7797f8248dee3660e5b710e63f185.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/122ccebbb5055791d31aef71f2926865fde8dc6.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/1d7f4f1e9bd27f2a7ae3e9a0cc6eb4baa1101920.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/89c17729aba3a89470171892fd1f840a1919e06f.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/13d984f251b3095fc8514bc9f3f47ce56df50f7b.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/f407fd76f33d8065c5e31b47134875d8995120da.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/a89f34bea4ba125fc57b4cee14a5faa253620766.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/Publisher.o: Publisher.c  .generated_files/137ed2dd4f21a034c0313a9c75dc0cb291932b3f.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Publisher.o.d 
	@${RM} ${OBJECTDIR}/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/Publisher.o.d" -o ${OBJECTDIR}/Publisher.o Publisher.c    -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_USING_IMU=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/USING_IMU
	${RM} -r dist/USING_IMU

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
