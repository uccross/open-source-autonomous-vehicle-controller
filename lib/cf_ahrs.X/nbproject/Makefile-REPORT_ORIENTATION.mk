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
ifeq "$(wildcard nbproject/Makefile-local-REPORT_ORIENTATION.mk)" "nbproject/Makefile-local-REPORT_ORIENTATION.mk"
include nbproject/Makefile-local-REPORT_ORIENTATION.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=REPORT_ORIENTATION
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
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
SOURCEFILES_QUOTED_IF_SPACED=C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c ../lin_alg_pack.X/lin_alg_pack.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1972374124/cf_ahrs.o ${OBJECTDIR}/_ext/2092238454/board.o ${OBJECTDIR}/_ext/191137912/serial.o ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d ${OBJECTDIR}/_ext/2092238454/board.o.d ${OBJECTDIR}/_ext/191137912/serial.o.d ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1972374124/cf_ahrs.o ${OBJECTDIR}/_ext/2092238454/board.o ${OBJECTDIR}/_ext/191137912/serial.o ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o

# Source Files
SOURCEFILES=C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c ../lin_alg_pack.X/lin_alg_pack.c



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
	${MAKE}  -f nbproject/Makefile-REPORT_ORIENTATION.mk dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/1972374124/cf_ahrs.o: C\:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c  .generated_files/16ccbbf992354726ce0403ef06e2a95a2d330f6d.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1972374124" 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/2092238454/board.o: C\:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c  .generated_files/9a9321ad058d81a5f919bfa2d60969241999f665.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/2092238454" 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o.d 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2092238454/board.o.d" -o ${OBJECTDIR}/_ext/2092238454/board.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/191137912/serial.o: C\:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c  .generated_files/331e1276528766ff1aa4d6a23a49f879cee12a59.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/191137912" 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/191137912/serial.o.d" -o ${OBJECTDIR}/_ext/191137912/serial.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o: ../lin_alg_pack.X/lin_alg_pack.c  .generated_files/9b1de1c71c1f768bc8e1051c67a1abbc3036577a.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/2132384669" 
	@${RM} ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o.d 
	@${RM} ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG   -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o.d" -o ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o ../lin_alg_pack.X/lin_alg_pack.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
else
${OBJECTDIR}/_ext/1972374124/cf_ahrs.o: C\:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c  .generated_files/a030193acf987fec41ab203583968697fbe2fbe5.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1972374124" 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/2092238454/board.o: C\:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c  .generated_files/fc23118255d87dc3dd765980182a97388ea7dfdb.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/2092238454" 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o.d 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2092238454/board.o.d" -o ${OBJECTDIR}/_ext/2092238454/board.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/191137912/serial.o: C\:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c  .generated_files/d4c055d193fbffcfa1453a3c4cab5ddb1823c699.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/191137912" 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/191137912/serial.o.d" -o ${OBJECTDIR}/_ext/191137912/serial.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o: ../lin_alg_pack.X/lin_alg_pack.c  .generated_files/4d2fd02039363f27ed20423abcca85516f412d3.flag .generated_files/afce40dd9192c5cd0a2cd5e27d88fc5e8d2abc0.flag
	@${MKDIR} "${OBJECTDIR}/_ext/2132384669" 
	@${RM} ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o.d 
	@${RM} ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DREPORT_ORIENTATION -I"../board.X" -I"../lin_alg_pack.X" -Wall -MP -MMD -MF "${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o.d" -o ${OBJECTDIR}/_ext/2132384669/lin_alg_pack.o ../lin_alg_pack.X/lin_alg_pack.c    -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -mdfp="${DFP_DIR}"  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g   -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)      -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_REPORT_ORIENTATION=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml -mdfp="${DFP_DIR}"
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/REPORT_ORIENTATION
	${RM} -r dist/REPORT_ORIENTATION

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
