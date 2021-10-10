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
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/5b4eb0500d1f072deb5f6120323abb1973a1fa41.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/891bc5337cbddaa063111011ee82cd296e532a70.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/767167bf83077a43075658281d4d7f392948e250.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/f0172477d106fbffdf9eb94570e43cabdb49e9f4.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/b2ce2993e3a20be4e9be0fbef4174e6161ad16c4.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/cfa5715d546abeb0d201d6fe66b5dac03aaa35ab.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/7e7dcab12a7211b6b28c8aa13261284f7b960034.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/e694bc95ed160147a86b9bfc647dc69276164e87.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/ff310caf47639db725d1cec05b90b3c88fc4a13a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/eee8c76b0aaf5e8682000d554067318dcda06e6.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/62c436ea690c3acbafcd305ff4d65cf17bb26073.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/21014615/Lin_alg_float.o: ../Lin_alg.X/Lin_alg_float.c  .generated_files/a351c58901dae5c44ad7fbdba88fb2dc59a12369.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/21014615" 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d" -o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ../Lin_alg.X/Lin_alg_float.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/60001887/cf_ahrs.o: ../cf_ahrs.X/cf_ahrs.c  .generated_files/4bb2f0fde18cdf490d6f393743b1df0da108e505.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/60001887" 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o ../cf_ahrs.X/cf_ahrs.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/36105697/Board.o: ../Board.X/Board.c  .generated_files/e8887b37ad67490a35d151d6908e731b848355a2.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/36105697" 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o.d 
	@${RM} ${OBJECTDIR}/_ext/36105697/Board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/36105697/Board.o.d" -o ${OBJECTDIR}/_ext/36105697/Board.o ../Board.X/Board.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/994540416/ICM_20948.o: ../ICM-20948.X/ICM_20948.c  .generated_files/8fd59937e003766e8a9e0adee686480ef57595d5.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/994540416" 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o.d 
	@${RM} ${OBJECTDIR}/_ext/994540416/ICM_20948.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/994540416/ICM_20948.o.d" -o ${OBJECTDIR}/_ext/994540416/ICM_20948.o ../ICM-20948.X/ICM_20948.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/528463759/MavSerial.o: ../MavSerial.X/MavSerial.c  .generated_files/f2f763b64424420ab1dd2194184e93f7a09eff44.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/528463759" 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o.d 
	@${RM} ${OBJECTDIR}/_ext/528463759/MavSerial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/528463759/MavSerial.o.d" -o ${OBJECTDIR}/_ext/528463759/MavSerial.o ../MavSerial.X/MavSerial.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/992960722/nmea0183v4.o: ../nmea0183v4.X/nmea0183v4.c  .generated_files/3e91694690fa91a28e10117e3ef057845a4a4706.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/992960722" 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d 
	@${RM} ${OBJECTDIR}/_ext/992960722/nmea0183v4.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/992960722/nmea0183v4.o.d" -o ${OBJECTDIR}/_ext/992960722/nmea0183v4.o ../nmea0183v4.X/nmea0183v4.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/88768175/RC_RX.o: ../RC_RX.X/RC_RX.c  .generated_files/e108236d8a699399f6a2c4aea15831e85aacbf42.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/88768175" 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o.d 
	@${RM} ${OBJECTDIR}/_ext/88768175/RC_RX.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/88768175/RC_RX.o.d" -o ${OBJECTDIR}/_ext/88768175/RC_RX.o ../RC_RX.X/RC_RX.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/1548322204/RC_servo.o: ../RC_servo.X/RC_servo.c  .generated_files/34d59b7525790c82a737bc941fa7da3409344a49.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1548322204" 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o.d 
	@${RM} ${OBJECTDIR}/_ext/1548322204/RC_servo.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1548322204/RC_servo.o.d" -o ${OBJECTDIR}/_ext/1548322204/RC_servo.o ../RC_servo.X/RC_servo.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/698669614/System_timer.o: ../System_timer.X/System_timer.c  .generated_files/fa296a6f4d23ad77379696abbc4affab2e8f11e5.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/698669614" 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o.d 
	@${RM} ${OBJECTDIR}/_ext/698669614/System_timer.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/698669614/System_timer.o.d" -o ${OBJECTDIR}/_ext/698669614/System_timer.o ../System_timer.X/System_timer.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/main_gnc_boat.o: main_gnc_boat.c  .generated_files/e8f9378af08b7a26766cc3a360543d111ecc8d63.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o.d 
	@${RM} ${OBJECTDIR}/main_gnc_boat.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/main_gnc_boat.o.d" -o ${OBJECTDIR}/main_gnc_boat.o main_gnc_boat.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/547705276/linear_trajectory.o: ../linear_trajectory.X/linear_trajectory.c  .generated_files/6c021d3e295ddebbb171a43adbb530741e74c14e.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/547705276" 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d 
	@${RM} ${OBJECTDIR}/_ext/547705276/linear_trajectory.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/547705276/linear_trajectory.o.d" -o ${OBJECTDIR}/_ext/547705276/linear_trajectory.o ../linear_trajectory.X/linear_trajectory.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/32917207/pid_controller.o: ../pid_controller.X/pid_controller.c  .generated_files/8f95ffaa5551fe76412336b740a28681fb80b2a3.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/32917207" 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o.d 
	@${RM} ${OBJECTDIR}/_ext/32917207/pid_controller.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/32917207/pid_controller.o.d" -o ${OBJECTDIR}/_ext/32917207/pid_controller.o ../pid_controller.X/pid_controller.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/433160951/Publisher.o: ../Publisher.X/Publisher.c  .generated_files/c62fe7dfce087d1c7c6615601ced9f5be3305f8c.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/433160951" 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o.d 
	@${RM} ${OBJECTDIR}/_ext/433160951/Publisher.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/433160951/Publisher.o.d" -o ${OBJECTDIR}/_ext/433160951/Publisher.o ../Publisher.X/Publisher.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/21014615/Lin_alg_float.o: ../Lin_alg.X/Lin_alg_float.c  .generated_files/ce121fc784c7a8f58680e3076d9fd3efa8b7956f.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/21014615" 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d 
	@${RM} ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/21014615/Lin_alg_float.o.d" -o ${OBJECTDIR}/_ext/21014615/Lin_alg_float.o ../Lin_alg.X/Lin_alg_float.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/60001887/cf_ahrs.o: ../cf_ahrs.X/cf_ahrs.c  .generated_files/a2cc2d5a483c0fa43da2f923714e903d9a19a272.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/60001887" 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/60001887/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DSAM_M8Q_GPS -I"../" -I"../../modules" -I"../Board.X" -I"../../modules/c_library_v2" -I"../../modules/c_library_v2/common" -I"../ICM-20948.X" -I"../MavSerial.X" -I"../nmea0183v4.X" -I"../Publisher.X" -I"../RC_RX.X" -I"../RC_servo.X" -I"../System_timer.X" -I"../linear_trajectory.X" -I"../pid_controller.X" -I"../Lin_alg.X" -I"../cf_ahrs.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/60001887/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/60001887/cf_ahrs.o ../cf_ahrs.X/cf_ahrs.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
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
