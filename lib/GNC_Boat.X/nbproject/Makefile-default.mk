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
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
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
SOURCEFILES_QUOTED_IF_SPACED=../Board.X/Board.c ../ICM-20948.X/ICM_20948.c ../MavSerial.X/MavSerial.c ../nmea0183v4.X/nmea0183v4.c ../RC_RX.X/RC_RX.c ../RC_servo.X/RC_servo.c ../System_timer.X/System_timer.c main_gnc_boat.c ../linear_trajectory.X/linear_trajectory.c ../pid_controller.X/pid_controller.c ../Publisher.X/Publisher.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/36105697/Board.o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ${OBJECTDIR}/_ext/528463759/MavSerial.o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ${OBJECTDIR}/_ext/88768175/RC_RX.o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ${OBJECTDIR}/_ext/698669614/System_timer.o ${OBJECTDIR}/main_gnc_boat.o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ${OBJECTDIR}/_ext/32917207/pid_controller.o ${OBJECTDIR}/_ext/433160951/Publisher.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/36105697/Board.o.d ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d ${OBJECTDIR}/_ext/528463759/MavSerial.o.d ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d ${OBJECTDIR}/_ext/88768175/RC_RX.o.d ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d ${OBJECTDIR}/_ext/698669614/System_timer.o.d ${OBJECTDIR}/main_gnc_boat.o.d ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d ${OBJECTDIR}/_ext/32917207/pid_controller.o.d ${OBJECTDIR}/_ext/433160951/Publisher.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/36105697/Board.o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ${OBJECTDIR}/_ext/528463759/MavSerial.o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ${OBJECTDIR}/_ext/88768175/RC_RX.o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ${OBJECTDIR}/_ext/698669614/System_timer.o ${OBJECTDIR}/main_gnc_boat.o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ${OBJECTDIR}/_ext/32917207/pid_controller.o ${OBJECTDIR}/_ext/433160951/Publisher.o

# Source Files
SOURCEFILES=../Board.X/Board.c ../ICM-20948.X/ICM_20948.c ../MavSerial.X/MavSerial.c ../nmea0183v4.X/nmea0183v4.c ../RC_RX.X/RC_RX.c ../RC_servo.X/RC_servo.c ../System_timer.X/System_timer.c main_gnc_boat.c ../linear_trajectory.X/linear_trajectory.c ../pid_controller.X/pid_controller.c ../Publisher.X/Publisher.c



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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/eeadb385bfde92b5f88160c2583f27daa35b7812.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/ec31be8f6407b13e848170f49f2cac6813a6921a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/a25598ef5d164cca93f16a4377f195c285e77648.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/80d773a9a8eb2183137198125498653e40e59483.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/b1bf4b7771ead6dcd8da77f3d41388e80385ea02.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/77cd5e973a4320ecabc94ea69ecd7c1732ecfe79.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/d1e785b210d6af95ca38aeae06fb283025e0c942.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/4b2c720283ab45cc8d480a216a9a19d2728496e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/f10e79b9cd3a890f3bb86db63596b6b05f5a8950.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/1912cf219a4614392393e55e735f34590a92f1f6.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/aab24eb50a916399de76d18bd9b447fbe90faaeb.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/f0d188a6faa3ac8275aaf5c2cec59eb726c4293d.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/1992e9bac5cbac2bf0b9cd7756024cdca64dc5c7.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/ab30657755c0c4619c2a6db5395fbff3e0e6d754.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/800906013d7cffc778a05e9685385756ba13bfc5.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/1d3d79ab7ca443a7266be1527b59b52dfbb7f41e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/dc78443986818d26fef151e1ce70b3a0134d947a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/8b6cc5ad241f7af503f2cebfbc457cf1a66e814c.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/702506604b02417df1445c8bf894490e253ac72f.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/f422c1074001f317d5113281ab132731c04badac.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/821deb9d276cc7b716751330e1d9a1844c3d56ea.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/8c6a757f2c46904bf47140201ad113e7d2e69eec.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
