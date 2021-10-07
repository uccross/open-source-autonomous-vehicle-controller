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
SOURCEFILES_QUOTED_IF_SPACED=C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c C:/Users/ladyb/OneDrive/Documents/uLAPack/src/ulapack.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1972374124/cf_ahrs.o ${OBJECTDIR}/_ext/247353088/ulapack.o ${OBJECTDIR}/_ext/2092238454/board.o ${OBJECTDIR}/_ext/191137912/serial.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d ${OBJECTDIR}/_ext/247353088/ulapack.o.d ${OBJECTDIR}/_ext/2092238454/board.o.d ${OBJECTDIR}/_ext/191137912/serial.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1972374124/cf_ahrs.o ${OBJECTDIR}/_ext/247353088/ulapack.o ${OBJECTDIR}/_ext/2092238454/board.o ${OBJECTDIR}/_ext/191137912/serial.o

# Source Files
SOURCEFILES=C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c C:/Users/ladyb/OneDrive/Documents/uLAPack/src/ulapack.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

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
${OBJECTDIR}/_ext/1972374124/cf_ahrs.o: C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1972374124" 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/247353088/ulapack.o: C:/Users/ladyb/OneDrive/Documents/uLAPack/src/ulapack.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/247353088" 
	@${RM} ${OBJECTDIR}/_ext/247353088/ulapack.o.d 
	@${RM} ${OBJECTDIR}/_ext/247353088/ulapack.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/247353088/ulapack.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/247353088/ulapack.o.d" -o ${OBJECTDIR}/_ext/247353088/ulapack.o C:/Users/ladyb/OneDrive/Documents/uLAPack/src/ulapack.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2092238454/board.o: C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2092238454" 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o.d 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2092238454/board.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/2092238454/board.o.d" -o ${OBJECTDIR}/_ext/2092238454/board.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/191137912/serial.o: C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/191137912" 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/191137912/serial.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -DPICkit3PlatformTool=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/191137912/serial.o.d" -o ${OBJECTDIR}/_ext/191137912/serial.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/1972374124/cf_ahrs.o: C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1972374124" 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d 
	@${RM} ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/1972374124/cf_ahrs.o.d" -o ${OBJECTDIR}/_ext/1972374124/cf_ahrs.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/cf_ahrs.X/cf_ahrs.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/247353088/ulapack.o: C:/Users/ladyb/OneDrive/Documents/uLAPack/src/ulapack.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/247353088" 
	@${RM} ${OBJECTDIR}/_ext/247353088/ulapack.o.d 
	@${RM} ${OBJECTDIR}/_ext/247353088/ulapack.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/247353088/ulapack.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/247353088/ulapack.o.d" -o ${OBJECTDIR}/_ext/247353088/ulapack.o C:/Users/ladyb/OneDrive/Documents/uLAPack/src/ulapack.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/2092238454/board.o: C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2092238454" 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o.d 
	@${RM} ${OBJECTDIR}/_ext/2092238454/board.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2092238454/board.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/2092238454/board.o.d" -o ${OBJECTDIR}/_ext/2092238454/board.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/board.X/board.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/191137912/serial.o: C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/191137912" 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o.d 
	@${RM} ${OBJECTDIR}/_ext/191137912/serial.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/191137912/serial.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DULAPACK_USE_STATIC_ALLOC -I"../../../uLAPack/src" -I"../board.X" -I"../serial.X" -Wall -MMD -MF "${OBJECTDIR}/_ext/191137912/serial.o.d" -o ${OBJECTDIR}/_ext/191137912/serial.o C:/Users/ladyb/OneDrive/Documents/reboat/libraries/serial.X/serial.c    -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD) 
	
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
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -DPICkit3PlatformTool=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D=__DEBUG_D,--defsym=PICkit3PlatformTool=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/cf_ahrs.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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
