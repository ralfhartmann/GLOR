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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1643945058/mifare.o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o ${OBJECTDIR}/_ext/1360937237/as3911.o ${OBJECTDIR}/_ext/1360937237/as3911_com.o ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o ${OBJECTDIR}/_ext/1360937237/as3911_stream.o ${OBJECTDIR}/_ext/1360937237/beep.o ${OBJECTDIR}/_ext/1360937237/board.o ${OBJECTDIR}/_ext/1360937237/clock.o ${OBJECTDIR}/_ext/1360937237/crc.o ${OBJECTDIR}/_ext/1360937237/delay.o ${OBJECTDIR}/_ext/1360937237/dispatcher.o ${OBJECTDIR}/_ext/1360937237/ic.o ${OBJECTDIR}/_ext/1360937237/irq_table.o ${OBJECTDIR}/_ext/1360937237/iso14443_common.o ${OBJECTDIR}/_ext/1360937237/iso14443a.o ${OBJECTDIR}/_ext/1360937237/iso14443b.o ${OBJECTDIR}/_ext/1360937237/iso15693_2.o ${OBJECTDIR}/_ext/1360937237/iso15693_3.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/mifare_ul.o ${OBJECTDIR}/_ext/1360937237/nfc.o ${OBJECTDIR}/_ext/1360937237/topaz.o ${OBJECTDIR}/_ext/1360937237/uart.o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ${OBJECTDIR}/_ext/1360937237/utils.o ${OBJECTDIR}/_ext/1933841525/bootloadable.o ${OBJECTDIR}/_ext/1933841525/logger.o ${OBJECTDIR}/_ext/1933841525/spi_driver.o ${OBJECTDIR}/_ext/1933841525/system_clock.o ${OBJECTDIR}/_ext/1933841525/usb_device.o ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o ${OBJECTDIR}/_ext/1360937237/felica.o ${OBJECTDIR}/_ext/1933841525/i2c_driver.o ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1643945058/mifare.o.d ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d ${OBJECTDIR}/_ext/1360937237/as3911.o.d ${OBJECTDIR}/_ext/1360937237/as3911_com.o.d ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d ${OBJECTDIR}/_ext/1360937237/beep.o.d ${OBJECTDIR}/_ext/1360937237/board.o.d ${OBJECTDIR}/_ext/1360937237/clock.o.d ${OBJECTDIR}/_ext/1360937237/crc.o.d ${OBJECTDIR}/_ext/1360937237/delay.o.d ${OBJECTDIR}/_ext/1360937237/dispatcher.o.d ${OBJECTDIR}/_ext/1360937237/ic.o.d ${OBJECTDIR}/_ext/1360937237/irq_table.o.d ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d ${OBJECTDIR}/_ext/1360937237/iso14443a.o.d ${OBJECTDIR}/_ext/1360937237/iso14443b.o.d ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d ${OBJECTDIR}/_ext/1360937237/nfc.o.d ${OBJECTDIR}/_ext/1360937237/topaz.o.d ${OBJECTDIR}/_ext/1360937237/uart.o.d ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d ${OBJECTDIR}/_ext/1360937237/utils.o.d ${OBJECTDIR}/_ext/1933841525/bootloadable.o.d ${OBJECTDIR}/_ext/1933841525/logger.o.d ${OBJECTDIR}/_ext/1933841525/spi_driver.o.d ${OBJECTDIR}/_ext/1933841525/system_clock.o.d ${OBJECTDIR}/_ext/1933841525/usb_device.o.d ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d ${OBJECTDIR}/_ext/1360937237/felica.o.d ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1643945058/mifare.o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o ${OBJECTDIR}/_ext/1360937237/as3911.o ${OBJECTDIR}/_ext/1360937237/as3911_com.o ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o ${OBJECTDIR}/_ext/1360937237/as3911_stream.o ${OBJECTDIR}/_ext/1360937237/beep.o ${OBJECTDIR}/_ext/1360937237/board.o ${OBJECTDIR}/_ext/1360937237/clock.o ${OBJECTDIR}/_ext/1360937237/crc.o ${OBJECTDIR}/_ext/1360937237/delay.o ${OBJECTDIR}/_ext/1360937237/dispatcher.o ${OBJECTDIR}/_ext/1360937237/ic.o ${OBJECTDIR}/_ext/1360937237/irq_table.o ${OBJECTDIR}/_ext/1360937237/iso14443_common.o ${OBJECTDIR}/_ext/1360937237/iso14443a.o ${OBJECTDIR}/_ext/1360937237/iso14443b.o ${OBJECTDIR}/_ext/1360937237/iso15693_2.o ${OBJECTDIR}/_ext/1360937237/iso15693_3.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/mifare_ul.o ${OBJECTDIR}/_ext/1360937237/nfc.o ${OBJECTDIR}/_ext/1360937237/topaz.o ${OBJECTDIR}/_ext/1360937237/uart.o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ${OBJECTDIR}/_ext/1360937237/utils.o ${OBJECTDIR}/_ext/1933841525/bootloadable.o ${OBJECTDIR}/_ext/1933841525/logger.o ${OBJECTDIR}/_ext/1933841525/spi_driver.o ${OBJECTDIR}/_ext/1933841525/system_clock.o ${OBJECTDIR}/_ext/1933841525/usb_device.o ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o ${OBJECTDIR}/_ext/1360937237/felica.o ${OBJECTDIR}/_ext/1933841525/i2c_driver.o ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o


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

# The following macros may be used in the pre and post step lines
Device=PIC24FJ128GB108
ProjectDir="C:\archive\Gamaya\Rama\gitHub\AS3911_GP_Src_2.0.6 with Mifare\AS3911\firmware\application\AS3911_Firmware.X"
ConfName=default
ImagePath="dist\default\${IMAGE_TYPE}\AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}"
ImageDir="dist\default\${IMAGE_TYPE}"
ImageName="AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}"

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
	@echo "--------------------------------------"
	@echo "User defined post-build step: [..\..\..\..\common\Tools\objcopy.exe -I ihex -O binary $(ImagePath) $(ImageDir)\AS3911_Firmware.X.${IMAGE_TYPE}.bin]"
	@..\..\..\..\common\Tools\objcopy.exe -I ihex -O binary $(ImagePath) $(ImageDir)\AS3911_Firmware.X.${IMAGE_TYPE}.bin
	@echo "--------------------------------------"

MP_PROCESSOR_OPTION=24FJ128GB108
MP_LINKER_FILE_OPTION=,--script="..\src\p24FJ128GB108.gld"
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
${OBJECTDIR}/_ext/1643945058/mifare.o: ../mifare_src/mifare.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare.o.ok ${OBJECTDIR}/_ext/1643945058/mifare.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare.o ../mifare_src/mifare.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o: ../mifare_src/mifare_crypto1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o ../mifare_src/mifare_crypto1.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o: ../mifare_src/mifare_crypto1_clean.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o ../mifare_src/mifare_crypto1_clean.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o: ../mifare_src/mifare_parity_data_t.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o ../mifare_src/mifare_parity_data_t.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o: ../mifare_src/mifare_raw_request.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o ../mifare_src/mifare_raw_request.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o: ../mifare_src/mifare_uint64emu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o ../mifare_src/mifare_uint64emu.c    
	
${OBJECTDIR}/_ext/1360937237/as3911.o: ../src/as3911.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911.o.ok ${OBJECTDIR}/_ext/1360937237/as3911.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911.o ../src/as3911.c    
	
${OBJECTDIR}/_ext/1360937237/as3911_com.o: ../src/as3911_com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_com.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_com.o.ok ${OBJECTDIR}/_ext/1360937237/as3911_com.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911_com.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911_com.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911_com.o ../src/as3911_com.c    
	
${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o: ../src/as3911_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.ok ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o ../src/as3911_interrupt.c    
	
${OBJECTDIR}/_ext/1360937237/as3911_stream.o: ../src/as3911_stream.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.ok ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911_stream.o ../src/as3911_stream.c    
	
${OBJECTDIR}/_ext/1360937237/beep.o: ../src/beep.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/beep.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/beep.o.ok ${OBJECTDIR}/_ext/1360937237/beep.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/beep.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/beep.o.d" -o ${OBJECTDIR}/_ext/1360937237/beep.o ../src/beep.c    
	
${OBJECTDIR}/_ext/1360937237/board.o: ../src/board.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/board.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/board.o.ok ${OBJECTDIR}/_ext/1360937237/board.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/board.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/board.o.d" -o ${OBJECTDIR}/_ext/1360937237/board.o ../src/board.c    
	
${OBJECTDIR}/_ext/1360937237/clock.o: ../src/clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clock.o.ok ${OBJECTDIR}/_ext/1360937237/clock.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/clock.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/clock.o.d" -o ${OBJECTDIR}/_ext/1360937237/clock.o ../src/clock.c    
	
${OBJECTDIR}/_ext/1360937237/crc.o: ../src/crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/crc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/crc.o.ok ${OBJECTDIR}/_ext/1360937237/crc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/crc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/crc.o.d" -o ${OBJECTDIR}/_ext/1360937237/crc.o ../src/crc.c    
	
${OBJECTDIR}/_ext/1360937237/delay.o: ../src/delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/delay.o.ok ${OBJECTDIR}/_ext/1360937237/delay.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/delay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/delay.o.d" -o ${OBJECTDIR}/_ext/1360937237/delay.o ../src/delay.c    
	
${OBJECTDIR}/_ext/1360937237/dispatcher.o: ../src/dispatcher.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/dispatcher.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/dispatcher.o.ok ${OBJECTDIR}/_ext/1360937237/dispatcher.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/dispatcher.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/dispatcher.o.d" -o ${OBJECTDIR}/_ext/1360937237/dispatcher.o ../src/dispatcher.c    
	
${OBJECTDIR}/_ext/1360937237/ic.o: ../src/ic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/ic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/ic.o.ok ${OBJECTDIR}/_ext/1360937237/ic.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/ic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/ic.o.d" -o ${OBJECTDIR}/_ext/1360937237/ic.o ../src/ic.c    
	
${OBJECTDIR}/_ext/1360937237/irq_table.o: ../src/irq_table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/irq_table.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/irq_table.o.ok ${OBJECTDIR}/_ext/1360937237/irq_table.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/irq_table.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/irq_table.o.d" -o ${OBJECTDIR}/_ext/1360937237/irq_table.o ../src/irq_table.c    
	
${OBJECTDIR}/_ext/1360937237/iso14443_common.o: ../src/iso14443_common.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.ok ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso14443_common.o ../src/iso14443_common.c    
	
${OBJECTDIR}/_ext/1360937237/iso14443a.o: ../src/iso14443a.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443a.o.ok ${OBJECTDIR}/_ext/1360937237/iso14443a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso14443a.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso14443a.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso14443a.o ../src/iso14443a.c    
	
${OBJECTDIR}/_ext/1360937237/iso14443b.o: ../src/iso14443b.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443b.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443b.o.ok ${OBJECTDIR}/_ext/1360937237/iso14443b.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso14443b.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso14443b.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso14443b.o ../src/iso14443b.c    
	
${OBJECTDIR}/_ext/1360937237/iso15693_2.o: ../src/iso15693_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.ok ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso15693_2.o ../src/iso15693_2.c    
	
${OBJECTDIR}/_ext/1360937237/iso15693_3.o: ../src/iso15693_3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.ok ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso15693_3.o ../src/iso15693_3.c    
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.ok ${OBJECTDIR}/_ext/1360937237/main.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    
	
${OBJECTDIR}/_ext/1360937237/mifare_ul.o: ../src/mifare_ul.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.ok ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d" -o ${OBJECTDIR}/_ext/1360937237/mifare_ul.o ../src/mifare_ul.c    
	
${OBJECTDIR}/_ext/1360937237/nfc.o: ../src/nfc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc.o.ok ${OBJECTDIR}/_ext/1360937237/nfc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/nfc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/nfc.o.d" -o ${OBJECTDIR}/_ext/1360937237/nfc.o ../src/nfc.c    
	
${OBJECTDIR}/_ext/1360937237/topaz.o: ../src/topaz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/topaz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/topaz.o.ok ${OBJECTDIR}/_ext/1360937237/topaz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/topaz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/topaz.o.d" -o ${OBJECTDIR}/_ext/1360937237/topaz.o ../src/topaz.c    
	
${OBJECTDIR}/_ext/1360937237/uart.o: ../src/uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart.o.ok ${OBJECTDIR}/_ext/1360937237/uart.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart.o ../src/uart.c    
	
${OBJECTDIR}/_ext/1360937237/usb_descriptors.o: ../src/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.ok ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" -o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ../src/usb_descriptors.c    
	
${OBJECTDIR}/_ext/1360937237/utils.o: ../src/utils.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/utils.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/utils.o.ok ${OBJECTDIR}/_ext/1360937237/utils.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/utils.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/utils.o.d" -o ${OBJECTDIR}/_ext/1360937237/utils.o ../src/utils.c    
	
${OBJECTDIR}/_ext/1933841525/bootloadable.o: ../../../../common/firmware/microchip/src/bootloadable.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/bootloadable.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/bootloadable.o.ok ${OBJECTDIR}/_ext/1933841525/bootloadable.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/bootloadable.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/bootloadable.o.d" -o ${OBJECTDIR}/_ext/1933841525/bootloadable.o ../../../../common/firmware/microchip/src/bootloadable.c    
	
${OBJECTDIR}/_ext/1933841525/logger.o: ../../../../common/firmware/microchip/src/logger.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/logger.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/logger.o.ok ${OBJECTDIR}/_ext/1933841525/logger.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/logger.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/logger.o.d" -o ${OBJECTDIR}/_ext/1933841525/logger.o ../../../../common/firmware/microchip/src/logger.c    
	
${OBJECTDIR}/_ext/1933841525/spi_driver.o: ../../../../common/firmware/microchip/src/spi_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/spi_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/spi_driver.o.ok ${OBJECTDIR}/_ext/1933841525/spi_driver.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/spi_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/spi_driver.o.d" -o ${OBJECTDIR}/_ext/1933841525/spi_driver.o ../../../../common/firmware/microchip/src/spi_driver.c    
	
${OBJECTDIR}/_ext/1933841525/system_clock.o: ../../../../common/firmware/microchip/src/system_clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/system_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/system_clock.o.ok ${OBJECTDIR}/_ext/1933841525/system_clock.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/system_clock.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/system_clock.o.d" -o ${OBJECTDIR}/_ext/1933841525/system_clock.o ../../../../common/firmware/microchip/src/system_clock.c    
	
${OBJECTDIR}/_ext/1933841525/usb_device.o: ../../../../common/firmware/microchip/src/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_device.o.ok ${OBJECTDIR}/_ext/1933841525/usb_device.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_device.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_device.o ../../../../common/firmware/microchip/src/usb_device.c    
	
${OBJECTDIR}/_ext/1933841525/usb_function_generic.o: ../../../../common/firmware/microchip/src/usb_function_generic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.ok ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o ../../../../common/firmware/microchip/src/usb_function_generic.c    
	
${OBJECTDIR}/_ext/1933841525/usb_function_hid.o: ../../../../common/firmware/microchip/src/usb_function_hid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.ok ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o ../../../../common/firmware/microchip/src/usb_function_hid.c    
	
${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o: ../../../../common/firmware/microchip/src/usb_hal_pic24.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.ok ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o ../../../../common/firmware/microchip/src/usb_hal_pic24.c    
	
${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o: ../../../../common/firmware/microchip/src/stream_dispatcher.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.ok ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d" -o ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o ../../../../common/firmware/microchip/src/stream_dispatcher.c    
	
${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o: ../../../../common/firmware/microchip/src/usb_hid_stream_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.ok ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o ../../../../common/firmware/microchip/src/usb_hid_stream_driver.c    
	
${OBJECTDIR}/_ext/1360937237/felica.o: ../src/felica.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/felica.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/felica.o.ok ${OBJECTDIR}/_ext/1360937237/felica.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/felica.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/felica.o.d" -o ${OBJECTDIR}/_ext/1360937237/felica.o ../src/felica.c    
	
${OBJECTDIR}/_ext/1933841525/i2c_driver.o: ../../../../common/firmware/microchip/src/i2c_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.ok ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d" -o ${OBJECTDIR}/_ext/1933841525/i2c_driver.o ../../../../common/firmware/microchip/src/i2c_driver.c    
	
${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o: ../../../../common/firmware/microchip/src/weak_stream_functions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.ok ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d" -o ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o ../../../../common/firmware/microchip/src/weak_stream_functions.c    
	
else
${OBJECTDIR}/_ext/1643945058/mifare.o: ../mifare_src/mifare.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare.o.ok ${OBJECTDIR}/_ext/1643945058/mifare.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare.o ../mifare_src/mifare.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o: ../mifare_src/mifare_crypto1.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1.o ../mifare_src/mifare_crypto1.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o: ../mifare_src/mifare_crypto1_clean.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_crypto1_clean.o ../mifare_src/mifare_crypto1_clean.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o: ../mifare_src/mifare_parity_data_t.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_parity_data_t.o ../mifare_src/mifare_parity_data_t.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o: ../mifare_src/mifare_raw_request.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_raw_request.o ../mifare_src/mifare_raw_request.c    
	
${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o: ../mifare_src/mifare_uint64emu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1643945058 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d 
	@${RM} ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.ok ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o.d" -o ${OBJECTDIR}/_ext/1643945058/mifare_uint64emu.o ../mifare_src/mifare_uint64emu.c    
	
${OBJECTDIR}/_ext/1360937237/as3911.o: ../src/as3911.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911.o.ok ${OBJECTDIR}/_ext/1360937237/as3911.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911.o ../src/as3911.c    
	
${OBJECTDIR}/_ext/1360937237/as3911_com.o: ../src/as3911_com.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_com.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_com.o.ok ${OBJECTDIR}/_ext/1360937237/as3911_com.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911_com.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911_com.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911_com.o ../src/as3911_com.c    
	
${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o: ../src/as3911_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.ok ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911_interrupt.o ../src/as3911_interrupt.c    
	
${OBJECTDIR}/_ext/1360937237/as3911_stream.o: ../src/as3911_stream.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.ok ${OBJECTDIR}/_ext/1360937237/as3911_stream.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/as3911_stream.o.d" -o ${OBJECTDIR}/_ext/1360937237/as3911_stream.o ../src/as3911_stream.c    
	
${OBJECTDIR}/_ext/1360937237/beep.o: ../src/beep.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/beep.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/beep.o.ok ${OBJECTDIR}/_ext/1360937237/beep.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/beep.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/beep.o.d" -o ${OBJECTDIR}/_ext/1360937237/beep.o ../src/beep.c    
	
${OBJECTDIR}/_ext/1360937237/board.o: ../src/board.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/board.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/board.o.ok ${OBJECTDIR}/_ext/1360937237/board.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/board.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/board.o.d" -o ${OBJECTDIR}/_ext/1360937237/board.o ../src/board.c    
	
${OBJECTDIR}/_ext/1360937237/clock.o: ../src/clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clock.o.ok ${OBJECTDIR}/_ext/1360937237/clock.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/clock.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/clock.o.d" -o ${OBJECTDIR}/_ext/1360937237/clock.o ../src/clock.c    
	
${OBJECTDIR}/_ext/1360937237/crc.o: ../src/crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/crc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/crc.o.ok ${OBJECTDIR}/_ext/1360937237/crc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/crc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/crc.o.d" -o ${OBJECTDIR}/_ext/1360937237/crc.o ../src/crc.c    
	
${OBJECTDIR}/_ext/1360937237/delay.o: ../src/delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/delay.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/delay.o.ok ${OBJECTDIR}/_ext/1360937237/delay.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/delay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/delay.o.d" -o ${OBJECTDIR}/_ext/1360937237/delay.o ../src/delay.c    
	
${OBJECTDIR}/_ext/1360937237/dispatcher.o: ../src/dispatcher.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/dispatcher.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/dispatcher.o.ok ${OBJECTDIR}/_ext/1360937237/dispatcher.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/dispatcher.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/dispatcher.o.d" -o ${OBJECTDIR}/_ext/1360937237/dispatcher.o ../src/dispatcher.c    
	
${OBJECTDIR}/_ext/1360937237/ic.o: ../src/ic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/ic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/ic.o.ok ${OBJECTDIR}/_ext/1360937237/ic.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/ic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/ic.o.d" -o ${OBJECTDIR}/_ext/1360937237/ic.o ../src/ic.c    
	
${OBJECTDIR}/_ext/1360937237/irq_table.o: ../src/irq_table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/irq_table.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/irq_table.o.ok ${OBJECTDIR}/_ext/1360937237/irq_table.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/irq_table.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/irq_table.o.d" -o ${OBJECTDIR}/_ext/1360937237/irq_table.o ../src/irq_table.c    
	
${OBJECTDIR}/_ext/1360937237/iso14443_common.o: ../src/iso14443_common.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.ok ${OBJECTDIR}/_ext/1360937237/iso14443_common.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso14443_common.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso14443_common.o ../src/iso14443_common.c    
	
${OBJECTDIR}/_ext/1360937237/iso14443a.o: ../src/iso14443a.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443a.o.ok ${OBJECTDIR}/_ext/1360937237/iso14443a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso14443a.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso14443a.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso14443a.o ../src/iso14443a.c    
	
${OBJECTDIR}/_ext/1360937237/iso14443b.o: ../src/iso14443b.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443b.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso14443b.o.ok ${OBJECTDIR}/_ext/1360937237/iso14443b.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso14443b.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso14443b.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso14443b.o ../src/iso14443b.c    
	
${OBJECTDIR}/_ext/1360937237/iso15693_2.o: ../src/iso15693_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.ok ${OBJECTDIR}/_ext/1360937237/iso15693_2.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso15693_2.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso15693_2.o ../src/iso15693_2.c    
	
${OBJECTDIR}/_ext/1360937237/iso15693_3.o: ../src/iso15693_3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.ok ${OBJECTDIR}/_ext/1360937237/iso15693_3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/iso15693_3.o.d" -o ${OBJECTDIR}/_ext/1360937237/iso15693_3.o ../src/iso15693_3.c    
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.ok ${OBJECTDIR}/_ext/1360937237/main.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c    
	
${OBJECTDIR}/_ext/1360937237/mifare_ul.o: ../src/mifare_ul.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.ok ${OBJECTDIR}/_ext/1360937237/mifare_ul.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/mifare_ul.o.d" -o ${OBJECTDIR}/_ext/1360937237/mifare_ul.o ../src/mifare_ul.c    
	
${OBJECTDIR}/_ext/1360937237/nfc.o: ../src/nfc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc.o.ok ${OBJECTDIR}/_ext/1360937237/nfc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/nfc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/nfc.o.d" -o ${OBJECTDIR}/_ext/1360937237/nfc.o ../src/nfc.c    
	
${OBJECTDIR}/_ext/1360937237/topaz.o: ../src/topaz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/topaz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/topaz.o.ok ${OBJECTDIR}/_ext/1360937237/topaz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/topaz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/topaz.o.d" -o ${OBJECTDIR}/_ext/1360937237/topaz.o ../src/topaz.c    
	
${OBJECTDIR}/_ext/1360937237/uart.o: ../src/uart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart.o.ok ${OBJECTDIR}/_ext/1360937237/uart.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart.o ../src/uart.c    
	
${OBJECTDIR}/_ext/1360937237/usb_descriptors.o: ../src/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.ok ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/usb_descriptors.o.d" -o ${OBJECTDIR}/_ext/1360937237/usb_descriptors.o ../src/usb_descriptors.c    
	
${OBJECTDIR}/_ext/1360937237/utils.o: ../src/utils.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/utils.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/utils.o.ok ${OBJECTDIR}/_ext/1360937237/utils.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/utils.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/utils.o.d" -o ${OBJECTDIR}/_ext/1360937237/utils.o ../src/utils.c    
	
${OBJECTDIR}/_ext/1933841525/bootloadable.o: ../../../../common/firmware/microchip/src/bootloadable.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/bootloadable.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/bootloadable.o.ok ${OBJECTDIR}/_ext/1933841525/bootloadable.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/bootloadable.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/bootloadable.o.d" -o ${OBJECTDIR}/_ext/1933841525/bootloadable.o ../../../../common/firmware/microchip/src/bootloadable.c    
	
${OBJECTDIR}/_ext/1933841525/logger.o: ../../../../common/firmware/microchip/src/logger.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/logger.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/logger.o.ok ${OBJECTDIR}/_ext/1933841525/logger.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/logger.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/logger.o.d" -o ${OBJECTDIR}/_ext/1933841525/logger.o ../../../../common/firmware/microchip/src/logger.c    
	
${OBJECTDIR}/_ext/1933841525/spi_driver.o: ../../../../common/firmware/microchip/src/spi_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/spi_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/spi_driver.o.ok ${OBJECTDIR}/_ext/1933841525/spi_driver.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/spi_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/spi_driver.o.d" -o ${OBJECTDIR}/_ext/1933841525/spi_driver.o ../../../../common/firmware/microchip/src/spi_driver.c    
	
${OBJECTDIR}/_ext/1933841525/system_clock.o: ../../../../common/firmware/microchip/src/system_clock.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/system_clock.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/system_clock.o.ok ${OBJECTDIR}/_ext/1933841525/system_clock.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/system_clock.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/system_clock.o.d" -o ${OBJECTDIR}/_ext/1933841525/system_clock.o ../../../../common/firmware/microchip/src/system_clock.c    
	
${OBJECTDIR}/_ext/1933841525/usb_device.o: ../../../../common/firmware/microchip/src/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_device.o.ok ${OBJECTDIR}/_ext/1933841525/usb_device.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_device.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_device.o ../../../../common/firmware/microchip/src/usb_device.c    
	
${OBJECTDIR}/_ext/1933841525/usb_function_generic.o: ../../../../common/firmware/microchip/src/usb_function_generic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.ok ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_function_generic.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_function_generic.o ../../../../common/firmware/microchip/src/usb_function_generic.c    
	
${OBJECTDIR}/_ext/1933841525/usb_function_hid.o: ../../../../common/firmware/microchip/src/usb_function_hid.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.ok ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_function_hid.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_function_hid.o ../../../../common/firmware/microchip/src/usb_function_hid.c    
	
${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o: ../../../../common/firmware/microchip/src/usb_hal_pic24.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.ok ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_hal_pic24.o ../../../../common/firmware/microchip/src/usb_hal_pic24.c    
	
${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o: ../../../../common/firmware/microchip/src/stream_dispatcher.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.ok ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o.d" -o ${OBJECTDIR}/_ext/1933841525/stream_dispatcher.o ../../../../common/firmware/microchip/src/stream_dispatcher.c    
	
${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o: ../../../../common/firmware/microchip/src/usb_hid_stream_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.ok ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o.d" -o ${OBJECTDIR}/_ext/1933841525/usb_hid_stream_driver.o ../../../../common/firmware/microchip/src/usb_hid_stream_driver.c    
	
${OBJECTDIR}/_ext/1360937237/felica.o: ../src/felica.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/felica.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/felica.o.ok ${OBJECTDIR}/_ext/1360937237/felica.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/felica.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1360937237/felica.o.d" -o ${OBJECTDIR}/_ext/1360937237/felica.o ../src/felica.c    
	
${OBJECTDIR}/_ext/1933841525/i2c_driver.o: ../../../../common/firmware/microchip/src/i2c_driver.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.ok ${OBJECTDIR}/_ext/1933841525/i2c_driver.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/i2c_driver.o.d" -o ${OBJECTDIR}/_ext/1933841525/i2c_driver.o ../../../../common/firmware/microchip/src/i2c_driver.c    
	
${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o: ../../../../common/firmware/microchip/src/weak_stream_functions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1933841525 
	@${RM} ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.ok ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DUSE_LOGGER=LOGGER_ON -DHAS_MIFARE -I"../../../../common/firmware/microchip/include" -I"../src" -I"../mifare_src" -O2 -MMD -MF "${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o.d" -o ${OBJECTDIR}/_ext/1933841525/weak_stream_functions.o ../../../../common/firmware/microchip/src/weak_stream_functions.c    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../src/p24FJ128GB108.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=0$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../src/p24FJ128GB108.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}         -Wl,--defsym=__MPLAB_BUILD=1,--heap=0$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}\\pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/AS3911_Firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
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
