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
ifeq "$(wildcard nbproject/Makefile-local-AUTONOMOUS_MODE_TEST.mk)" "nbproject/Makefile-local-AUTONOMOUS_MODE_TEST.mk"
include nbproject/Makefile-local-AUTONOMOUS_MODE_TEST.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=AUTONOMOUS_MODE_TEST
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
	${MAKE}  -f nbproject/Makefile-AUTONOMOUS_MODE_TEST.mk dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/7c2025b3389b4926201cf422a2d80a9858cd993e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/c490a04b83b6bee2a9a853b90e01ed06e1043ea0.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/42a7e5b9cf0c208d0d182265abba4895bc6941d6.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/e578aa74ea24196b891b67c03f6c5e6de352c768.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/7197498df57027f9797f99ae7159c5326bd3184c.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/e20caa6abef351bc4ce4cad685c3ec166fa88ea0.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/9518d2b6917947d1b32b01484d4bdf956ee3d275.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/d6bac1d5ea86b200fd72e9ab9ee6b36b51210e88.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/e25f970ecad0f3beef02f1069940f2823894a220.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/309d03631eac457fe6b711ac226268ec64e8de26.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/8f3916c292ad240bcdccec24fc5ba43fd09c8e8c.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/21014615/Lin_alg_float.o: ../Lin_alg.X/Lin_alg_float.c  .generated_files/262d83b428e79a07c44adac1dad9917d90dc5737.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/21014615" 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d" -o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ../Lin_alg.X/Lin_alg_float.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/60001887/cf_ahrs.o: ../cf_ahrs.X/cf_ahrs.c  .generated_files/b9b681189d8bada8b22b25f98c95739dce36a39a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/60001887" 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o ../cf_ahrs.X/cf_ahrs.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/38a75ce8245a68f7fb64b91a254a66e6befd8b4a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/6b338a4df25e29f2d97f5137f837c14ce640dd5e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/63a3892e9be32982c00d49c5e216a1f13665879c.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/2a4195c39bb9bc78cd65c1b12b81aa964cce3b0.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/9508681de648affc6a8c807c18ad5e2e530f2f1e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/c312a77e60a8189504f76e183505d0839b115df2.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/b6f59ca6d7a0fa58ae7f1fd7b2ba1dd9c0192ee1.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/8487aacb593b3f145183606c9117fd6303dd8ef2.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/ef27c4f0c41c3d6b4b3550af9f5988ee5e9f9777.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/123b714b72b30973e187f6ea70abda46feda9807.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/5831c41e4dcd0f75a2e32e23f4c0de9611657c34.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/21014615/Lin_alg_float.o: ../Lin_alg.X/Lin_alg_float.c  .generated_files/e71a0f934de20822b7de91fd3b0e3a1760547538.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/21014615" 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d" -o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ../Lin_alg.X/Lin_alg_float.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/60001887/cf_ahrs.o: ../cf_ahrs.X/cf_ahrs.c  .generated_files/dbe4db70f4d092eea2048978d8efcc2784018299.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/60001887" 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -DAUTONOMOUS_MODE_TEST -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o ../cf_ahrs.X/cf_ahrs.c    -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_AUTONOMOUS_MODE_TEST=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/GNC_Boat.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/AUTONOMOUS_MODE_TEST
	${RM} -r dist/AUTONOMOUS_MODE_TEST

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
