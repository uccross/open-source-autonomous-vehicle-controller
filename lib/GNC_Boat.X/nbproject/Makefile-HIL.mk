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
ifeq "$(wildcard nbproject/Makefile-local-HIL.mk)" "nbproject/Makefile-local-HIL.mk"
include nbproject/Makefile-local-HIL.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=HIL
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
SOURCEFILES_QUOTED_IF_SPACED=../Board.X/Board.c ../ICM-20948.X/ICM_20948.c ../MavSerial.X/MavSerial.c ../nmea0183v4.X/nmea0183v4.c ../RC_RX.X/RC_RX.c ../RC_servo.X/RC_servo.c ../System_timer.X/System_timer.c main_gnc_boat.c ../linear_trajectory.X/linear_trajectory.c ../pid_controller.X/pid_controller.c ../Publisher.X/Publisher.c ../Lin_alg.X/Lin_alg_float.c ../cf_ahrs.X/cf_ahrs.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/36105697/Board.o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ${OBJECTDIR}/_ext/528463759/MavSerial.o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ${OBJECTDIR}/_ext/88768175/RC_RX.o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ${OBJECTDIR}/_ext/698669614/System_timer.o ${OBJECTDIR}/main_gnc_boat.o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ${OBJECTDIR}/_ext/32917207/pid_controller.o ${OBJECTDIR}/_ext/433160951/Publisher.o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/36105697/Board.o.d ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d ${OBJECTDIR}/_ext/528463759/MavSerial.o.d ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d ${OBJECTDIR}/_ext/88768175/RC_RX.o.d ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d ${OBJECTDIR}/_ext/698669614/System_timer.o.d ${OBJECTDIR}/main_gnc_boat.o.d ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d ${OBJECTDIR}/_ext/32917207/pid_controller.o.d ${OBJECTDIR}/_ext/433160951/Publisher.o.d ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/36105697/Board.o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ${OBJECTDIR}/_ext/528463759/MavSerial.o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ${OBJECTDIR}/_ext/88768175/RC_RX.o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ${OBJECTDIR}/_ext/698669614/System_timer.o ${OBJECTDIR}/main_gnc_boat.o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ${OBJECTDIR}/_ext/32917207/pid_controller.o ${OBJECTDIR}/_ext/433160951/Publisher.o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o

# Source Files
SOURCEFILES=../Board.X/Board.c ../ICM-20948.X/ICM_20948.c ../MavSerial.X/MavSerial.c ../nmea0183v4.X/nmea0183v4.c ../RC_RX.X/RC_RX.c ../RC_servo.X/RC_servo.c ../System_timer.X/System_timer.c main_gnc_boat.c ../linear_trajectory.X/linear_trajectory.c ../pid_controller.X/pid_controller.c ../Publisher.X/Publisher.c ../Lin_alg.X/Lin_alg_float.c ../cf_ahrs.X/cf_ahrs.c



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
	${MAKE}  -f nbproject/Makefile-HIL.mk dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/198a356238ce3886778d477d978497e82491fe27.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/f65b09956621fa8e5fec74cbfaaaa7c1834caa39.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/a336b75f0fe392b5139b9a1e24f69ebf1dee8128.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/19249dba4767e3a286859a29b4753b86acf61c9a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/e842434de0338a5c7c1526d751be9a5b4f709a95.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/5b3e6831c8d9019ef7460b08b969571512151b1.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/bf55198d0ad6ad82b3a4c6d1227c6458a6a4110e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/eb90de86d9f8cb3c4a5b4d5de4e98431a2d3b2d8.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/938fec509252800c747b0613818e6d6f433b7814.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/36a6cd0f3eb92bda8ac8f8775d88deaa6d285f18.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/d1948aa9a470d2fe0086a3e34975f7cf7f2e07c3.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/21014615/Lin_alg_float.o: ../Lin_alg.X/Lin_alg_float.c  .generated_files/e09bc2d27eb9db7f7b2247faa9cce32d02d431f0.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/21014615" 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d" -o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ../Lin_alg.X/Lin_alg_float.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/60001887/cf_ahrs.o: ../cf_ahrs.X/cf_ahrs.c  .generated_files/ae21f6acb84da0d13686c017b098d2a2f11503f6.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/60001887" 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o ../cf_ahrs.X/cf_ahrs.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/730f5d3bcc9b6e50583d3c401a74472174b98efb.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/c0bb5dc96b8a72787c602cdaa800a2dfd732f226.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/32378fc2f3f2582d7dbfa84458dbea7449318b10.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/e1bfd334a22c120d909ee10b032bc4eeb7533bc1.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/9398e4202f8c000d66ec90ab6c5082a7dff7e0eb.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/dde383d783f81f76f9badc83f8878e1e58c2e368.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/7f5f8c10ca0c109012fa1195aa48424f5cf51103.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/4c0994fb55d352bd40445e56e643133d868cb0ab.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/d7cf83ce3987014c106a45fff747c487b60d3bab.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/1ea191b55d1352a66c38287fda4c4f9cc3a2b11a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/d6a8bbffcb06a7efa74f02f726b4e473c8f1d4e5.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/21014615/Lin_alg_float.o: ../Lin_alg.X/Lin_alg_float.c  .generated_files/a2849f2e705087fcf0e26c8fb0549d8104e1fa52.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/21014615" 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d" -o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ../Lin_alg.X/Lin_alg_float.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/60001887/cf_ahrs.o: ../cf_ahrs.X/cf_ahrs.c  .generated_files/590762d54fa9b59f872df990f3675191846a1ccd.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/60001887" 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -DHIL -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o ../cf_ahrs.X/cf_ahrs.c    -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_HIL=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/HIL
	${RM} -r dist/HIL

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
