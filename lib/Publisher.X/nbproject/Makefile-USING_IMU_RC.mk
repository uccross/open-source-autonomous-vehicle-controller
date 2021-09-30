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
ifeq "$(wildcard nbproject/Makefile-local-USING_IMU_RC.mk)" "nbproject/Makefile-local-USING_IMU_RC.mk"
include nbproject/Makefile-local-USING_IMU_RC.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=USING_IMU_RC
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
	${MAKE}  -f nbproject/Makefile-USING_IMU_RC.mk dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/f497d79c4d89c5a071d3a00c309307f62ef5eb75.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/41e320066144ff7226af6c2042582a4d2618c2ff.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/5c632b6215ca9171ebf2290d1b45ebfddf604be.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/4dd855c95c20640e57140fb4b89455ed592f4158.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/f6087e548d33336d1ee1c12653ba4a9cb06503ea.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/c3c116ae8a43e456fc0bafbf439ff81f82e911c3.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/d0c572c258d37765c7c835474b872b83ac2497b3.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/Publisher.o: Publisher.c  .generated_files/80b91a30240d168beac9cc7405c802a056b06370.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Publisher.o.d 
	@${RM} ${OBJECTDIR}/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/Publisher.o.d" -o ${OBJECTDIR}/Publisher.o Publisher.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/d8782062d5d16c4e2eae4377967e7133147d1c39.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/49ef70d974c807e3dea76de59d4ed03b5b2130ae.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/750cdc10bbe9d259c0fa5be0b2b7f8eb7fd087ed.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/fc520e28f049ec4d6d9ec7738fc12f25149295e6.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/74f7eedaeb97f38c3344f0f41c2b20a3350ddf73.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/63763a41e30a15dbc8591755e47a6e4e612ebcf.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/e9e21aee896eb1bda7b11bb550752dd93b4f863f.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/Publisher.o: Publisher.c  .generated_files/375ddc1eedbaf142c79e6204ffcdd7727fe76d96.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Publisher.o.d 
	@${RM} ${OBJECTDIR}/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPUBLISHER_TEST -DSAM_M8Q_GPS -DUSING_IMU -DUSING_RC -I"../Board.X" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../RC_RX.X" -I"../System_timer.X" -I"../RC_servo.X" -I"../../modules/c_library_v2/common" -I"../../modules/c_library_v2" -Wall -MP -MMD -MF "${OBJECTDIR}/Publisher.o.d" -o ${OBJECTDIR}/Publisher.o Publisher.c    -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_USING_IMU_RC=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Publisher.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/USING_IMU_RC
	${RM} -r dist/USING_IMU_RC

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
