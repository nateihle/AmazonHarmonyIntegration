/*******************************************************************************
  System Definitions

  File Name:
    system_definitions.h

  Summary:
    MPLAB Harmony project system definitions.

  Description:
    This file contains the system-wide prototypes and definitions for an MPLAB
    Harmony project.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END
<#include "/utilities/mhc/templates/freemarker_functions.ftl">
#ifndef _SYS_DEFINITIONS_H
#define _SYS_DEFINITIONS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "system/common/sys_common.h"
#include "system/common/sys_module.h"
<#if LIST_SYSTEM_DEFINITIONS_H_INCLUDES?has_content>
<@mhc_expand_list list=LIST_SYSTEM_DEFINITIONS_H_INCLUDES/>
</#if>
<#if CONFIG_USE_SYS_CLK == true>
#include "system/clk/sys_clk.h"
<#if CONFIG_HAVE_MPLL == true>
#include "peripheral/devcon/plib_devcon.h"
</#if>
</#if>
<#if CONFIG_USE_SYS_INT == true>
#include "system/int/sys_int.h"
</#if>
<#if CONFIG_USE_SYS_DMA == true>
#include "system/dma/sys_dma.h"
</#if>
<#if CONFIG_USE_SYS_FS == true>
#include "system/fs/sys_fs.h"
#include "system/fs/sys_fs_media_manager.h"
</#if>
<#if CONFIG_USE_SYS_CONSOLE == true>
#include "system/console/sys_console.h"
</#if>
<#if CONFIG_USE_SYS_RANDOM == true>
#include "system/random/sys_random.h"
</#if>
<#if CONFIG_SYS_FS_MPFS == true>
#include "system/fs/mpfs/mpfs.h"
</#if>
<#if CONFIG_SYS_FS_FAT == true>
#include "system/fs/fat_fs/src/file_system/ff.h"
#include "system/fs/fat_fs/src/file_system/ffconf.h"
#include "system/fs/fat_fs/src/hardware_access/diskio.h"
</#if>
<#if CONFIG_USE_SYS_TMR == true>
#include "system/tmr/sys_tmr.h"
</#if>
<#if CONFIG_USE_DRV_ADC == true>
<#if CONFIG_DRV_ADC_TYPE_ADCP == false>
#include "driver/adc/drv_adc_static.h"
<#if CONFIG_DRV_ADC_INTERRUPT_MODE == true>
#include "peripheral/int/plib_int.h"
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_CTR == true>
<#if CONFIG_DRV_CTR_DRIVER_MODE == "DYNAMIC">
#include "driver/ctr/drv_ctr.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_TMR == true>
<#if CONFIG_DRV_TMR_DRIVER_MODE == "DYNAMIC">
#include "driver/tmr/drv_tmr.h"
<#else>
#include "driver/tmr/drv_tmr_static.h"
<#if CONFIG_DRV_TMR_INTERRUPT_MODE == true>
#include "peripheral/int/plib_int.h"
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_PMP == true>
<#if CONFIG_DRV_PMP_IMPL == "DYNAMIC">
#include "driver/pmp/drv_pmp.h"
</#if>
<#if CONFIG_DRV_PMP_IMPL == "STATIC">
#include "framework/driver/pmp/drv_pmp_static.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_PTG == true>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
#include "driver/ptg/drv_ptg_static.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_CAN == true>
#include "driver/can/drv_can_static.h"
</#if>
<#if CONFIG_DRV_CVREF_ENABLE == true || CONFIG_USE_DRV_CMP ==true>
#include "driver/cmp/drv_cmp_static.h"
</#if>
<#if CONFIG_USE_BOOTLOADER_LIBRARY == true><#if CONFIG_CUSTOM_BOOTLOADER_PROGRAM_SPACE == false>
#include "bootloader/src/bootloader.h"
</#if></#if>
<#if CONFIG_USE_DRV_I2S == true>
#include "driver/i2s/drv_i2s.h"
</#if>
<#if CONFIG_USE_DRV_CODEC_AK4384?has_content><#if CONFIG_USE_DRV_CODEC_AK4384 == true>
#include "driver/codec/ak4384/drv_ak4384.h"
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK4953?has_content><#if CONFIG_USE_DRV_CODEC_AK4953 == true>
#include "driver/codec/ak4953/drv_ak4953.h"
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK4954?has_content><#if CONFIG_USE_DRV_CODEC_AK4954 == true>
#include "driver/codec/ak4954/drv_ak4954.h"
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK4642?has_content><#if CONFIG_USE_DRV_CODEC_AK4642 == true>
#include "driver/codec/ak4642/drv_ak4642.h"
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK7755?has_content><#if CONFIG_USE_DRV_CODEC_AK7755 == true>
#include "driver/codec/ak7755/drv_ak7755.h"
</#if></#if>
<#if CONFIG_USE_DRV_USART == true>
<#if CONFIG_DRV_USART_DRIVER_MODE == "DYNAMIC">
#include "driver/usart/drv_usart.h"
</#if>
<#if CONFIG_DRV_USART_DRIVER_MODE == "STATIC">
#include "driver/usart/drv_usart_static.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_EEPROM == true>
#include "driver/eeprom/drv_eeprom.h"
</#if>
<#if CONFIG_USE_DRV_NVM == true>
<#if CONFIG_DRV_NVM_DRIVER_TYPE == "BLOCK_DRIVER">
#include "driver/nvm/drv_nvm.h"
</#if>
<#if CONFIG_DRV_NVM_DRIVER_TYPE == "BETA">
#include "driver/nvm/beta_sw/drv_nvm.h"
<#if CONFIG_USE_DRV_NVM_MEDIA == true>
#include "driver/nvm/beta_sw/drv_nvm_media.h"
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25 == true>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "DYNAMIC">
#include "driver/spi_flash/sst25/drv_sst25.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25VF020B == true>
<#if CONFIG_DRV_SST25VF020B_DRIVER_MODE == "DYNAMIC">
#include "driver/spi_flash/sst25vf020b/drv_sst25vf020b.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25VF016B == true>
<#if CONFIG_DRV_SST25VF016B_DRIVER_MODE == "DYNAMIC">
#include "driver/spi_flash/sst25vf016b/drv_sst25vf016b.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25VF064C == true>
<#if CONFIG_DRV_SST25VF064C_DRIVER_MODE == "DYNAMIC">
#include "driver/spi_flash/sst25vf064c/drv_sst25vf064c.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_IPF == true>
<#if CONFIG_DRV_IPF_DRIVER_MODE == "DYNAMIC">
#include "driver/spi_flash/pic32wk_ipf/drv_ipf.h"
</#if>
</#if>
<#if CONFIG_USE_SYS_PORTS == true>
#include "system/ports/sys_ports.h"
</#if>
<#if CONFIG_USE_DRV_SRAM == true>
#include "driver/sram/drv_sram.h"
</#if>
<#if CONFIG_USE_DRV_SDCARD == true>
#include "driver/sdcard/drv_sdcard.h"
</#if>
<#if CONFIG_USE_DRV_SDHC == true>
#include "driver/sdhc/drv_sdhc.h"
<#else>
<#if CONFIG_HAVE_SDHC>
<#if CONFIG_PIC32MZ == true>
#include "peripheral/power/plib_power.h"
</#if>
</#if>
</#if>
<#if CONFIG_DRV_SPI_USE_DRIVER>
<#if CONFIG_DRV_SPI_DRIVER_MODE == "DYNAMIC">
#include "driver/spi/drv_spi.h"
<#else>
#include "driver/spi/static/drv_spi_static.h"
</#if>
</#if>
<#if CONFIG_USE_DRV_SQI == true>
#include "driver/sqi/drv_sqi.h"
#include "driver/sqi/drv_sqi_init.h"
</#if>
<#if CONFIG_USE_DRV_SST26 == true>
#include "driver/sqi_flash/sst26/drv_sst26.h"
</#if>
<#if CONFIG_DRV_ENCX24J600_USE_DRIVER == true>
#include "driver/encx24j600/drv_encx24j600.h"
</#if>
<#if CONFIG_DRV_ENC28J60_USE_DRIVER == true>
#include "driver/enc28j60/drv_enc28j60.h"
</#if>
<#if CONFIG_USE_SYS_DEBUG == true>
#include "system/debug/sys_debug.h"
</#if>
<#if CONFIG_USE_SYS_COMMAND == true>
#include "system/command/sys_command.h"
</#if>
<#if CONFIG_DRV_USB_DEVICE_SUPPORT == true>
<#if CONFIG_PIC32MZ == true>
#include "driver/usb/usbhs/drv_usbhs.h"
<#elseif ((CONFIG_PIC32MX == true) || (CONFIG_PIC32WK == true) || (CONFIG_PIC32MK == true))>
#include "driver/usb/usbfs/drv_usbfs.h"
</#if>
#include "usb/usb_device.h"
</#if>
<#if CONFIG_DRV_USB_HOST_SUPPORT == true>
<#if CONFIG_PIC32MZ == true>
#include "driver/usb/usbhs/drv_usbhs.h"
<#elseif ((CONFIG_PIC32MX == true) || (CONFIG_PIC32WK == true) || (CONFIG_PIC32MK == true))>
#include "driver/usb/usbfs/drv_usbfs.h"
</#if>
#include "usb/usb_host.h"
#include "usb/usb_host_hub.h"
#include "usb/usb_host_hub_interface.h"
</#if>
<#if CONFIG_USB_DEVICE_USE_MSD == true>
#include "usb/usb_device_msd.h"
</#if>
<#if CONFIG_USB_DEVICE_USE_CDC == true>
#include "usb/usb_device_cdc.h"
</#if>
<#if CONFIG_USB_DEVICE_USE_HID == true>
#include "usb/usb_device_hid.h"
</#if>
<#if CONFIG_USB_DEVICE_USE_AUDIO == true>
#include "usb/usb_device_audio_v1_0.h"
</#if>
<#if CONFIG_USB_HOST_USE_MSD == true>
#include "usb/usb_host_msd.h"
#include "usb/usb_host_scsi.h"
</#if>
<#if CONFIG_USB_HOST_USE_CDC == true>
#include "usb/usb_host_cdc.h"
</#if>
<#if CONFIG_USB_HOST_USE_HID == true>
#include "usb/usb_host_hid.h"
</#if>
<#if CONFIG_USB_HOST_USE_MOUSE == true>
#include "usb/usb_host_hid_mouse.h"
</#if>
<#if CONFIG_USB_HOST_USE_KEYBOARD == true>
#include "usb/usb_host_hid_keyboard.h"
</#if>
<#if CONFIG_USB_HOST_USE_AUDIO == true>
#include "usb/usb_host_audio_v1_0.h"
</#if>
<#if CONFIG_USE_DRV_RTCC == true>
<#if CONFIG_DRV_RTCC_DRIVER_MODE == "STATIC">
#include "driver/rtcc/drv_rtcc_static.h"
<#if CONFIG_DRV_RTCC_INTERRUPT_MODE == true>
#include "peripheral/int/plib_int.h"
</#if>
#include "peripheral/devcon/plib_devcon.h"
</#if>
</#if>
<#if CONFIG_USE_TCPIP_STACK == true>
#include "tcpip/tcpip.h"
#include "driver/ethmac/drv_ethmac.h"
</#if>
<#if CONFIG_DRV_MIIM_USE_DRIVER == true>
#include "driver/miim/drv_miim.h"
</#if>
<#if CONFIG_USE_DRV_WIFI == true>
 <#if CONFIG_DRV_WIFI_DEVICE == "MRF24WN">
#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_api.h"
 <#elseif CONFIG_DRV_WIFI_DEVICE == "WINC1500">
#include "driver/wifi/winc1500/include/wdrv_winc1500_api.h"
 <#elseif CONFIG_DRV_WIFI_DEVICE == "WILC1000">
#include "driver/wifi/wilc1000/include/wdrv_wilc1000_api.h"
 </#if>
</#if>
<#if CONFIG_USE_SYS_MSG == true>
#include "system/msg/sys_msg.h"
</#if>
<#if CONFIG_USE_SYS_INPUT == true>
#include "system/input/sys_input.h"
</#if>
<#if CONFIG_USE_SYS_TOUCH == true>
#include "system/touch/sys_touch.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_ADC_UPDATED == true>
#include "driver/touch/touch_adc/drv_touch_adc.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_ADC == true>
#include "driver/input/touch_adc/drv_touch_adc.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_ADC10BIT == true>
#include "driver/touch/adc10bit/drv_adc10bit.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_AR1021 == true>
#include "driver/touch/ar1021/drv_ar1021.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_MTCH6301 == true>
#include "driver/touch/mtch6301/drv_mtch6301.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_MTCH6303 == true>
#include "driver/touch/mtch6303/drv_mtch6303_static.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_MXT336T == true>
#include "driver/touch/mxt336t_legacy/drv_mxt336t.h"
#include "driver/touch/mxt336t_legacy/drv_mxt.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_MXT336T_UPDATED == true>
#include "driver/touch/mxt336t/drv_mxt336t.h"
#include "driver/touch/mxt336t/drv_mxt.h"
#include "framework/driver/touch/mxt336t/drv_mxt_prc_specific_static.h"
</#if>
<#if CONFIG_USE_DRV_TOUCH_GENERIC == true>
#include "driver/touch/generic/drv_touch_generic.h"
</#if>
<#if CONFIG_USE_DRV_INPUT_MXT336T == true>
#include "driver/input/touch/mxt336t/drv_mxt336t.h"
</#if>
<#if CONFIG_USE_DRV_OVM7690 == true>
#include "driver/camera/ovm7690/drv_camera_ovm7690.h"
</#if>
<#if (CONFIG_3RDPARTY_RTOS_USED == "FreeRTOS") || (CONFIG_3RDPARTY_RTOS_USED == "OpenRTOS_V8.x.x")>
#include "FreeRTOS.h"
#include "task.h"
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III">
#include "os.h"
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-II">
#include "os_cpu.h"
#include "ucos_ii.h"
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
#include "tx_api.h"
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
#include "RTOS.h"
</#if>
<#if CONFIG_USE_SEGGER_EMWIN_LIBRARY == true >
<#if CONFIG_DRV_GFX_CONTROLLER_TYPE == "Epson S1D13517">
#include "framework/gfx/driver/controller/s1d13517/drv_gfx_s1d13517.h"
</#if>
<#if CONFIG_SEGGER_EMWIN_TOUCH_WRAPPER == true>
#include "third_party/gfx/emwin/touch/emwin_touch_static.h"
</#if>
<#if CONFIG_SEGGER_EMWIN_GUI_WRAPPER == true>
#include "third_party/gfx/emwin/gui/emwin_gui_static.h"
</#if>
</#if>
<#if CONFIG_USE_SAMPLE_MODULE == true>
#include "sample/sample_module.h"
</#if>
<#if CONFIG_USE_TEST_HARNESS == true>
#include "test/test_harness.h"
</#if>
<#include "/framework/net/templates/system_definitions.h.include.ftl">
<#if CONFIG_USE_SAMPLE_FUNC_TEST == true>
#include "test/test_sample_functional.h"
</#if>
<#if CONFIG_USE_GFX_STACK == true>
#include "gfx/hal/gfx.h"
</#if>
<#if CONFIG_USE_LIBARIA == true>
#include "gfx/libaria/libaria_harmony.h"
</#if>
<#if CONFIG_USE_DSP == true>
#include "math/dsp/dsp.h"
</#if>
<#if CONFIG_USE_LIBQ == true>
#include "math/libq/libq.h"
</#if>
<#if CONFIG_USE_DECODER == true>
#include "decoder/audio_decoder.h"
</#if>
<#if CONFIG_APP_IDX_0?has_content>
#include "${CONFIG_APP_NAME_0?lower_case}.h"
<#else>
#include "app.h"
</#if>
<#if CONFIG_APP_IDX_1?has_content>
<#if CONFIG_APP_IDX_1 == true>
#include "${CONFIG_APP_NAME_1?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_2 == true>
#include "${CONFIG_APP_NAME_2?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_3 == true>
#include "${CONFIG_APP_NAME_3?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_4 == true>
#include "${CONFIG_APP_NAME_4?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_5 == true>
#include "${CONFIG_APP_NAME_5?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_6 == true>
#include "${CONFIG_APP_NAME_6?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_7 == true>
#include "${CONFIG_APP_NAME_7?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_8 == true>
#include "${CONFIG_APP_NAME_8?lower_case}.h"
</#if>
<#if CONFIG_APP_IDX_9 == true>
#include "${CONFIG_APP_NAME_9?lower_case}.h"
</#if>
</#if>


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* System Objects

  Summary:
    Structure holding the system's object handles

  Description:
    This structure contains the object handles for all objects in the
    MPLAB Harmony project's system configuration.

  Remarks:
    These handles are returned from the "Initialize" functions for each module
    and must be passed into the "Tasks" function for each module.
*/

typedef struct
{
<#if LIST_SYSTEM_DEFINITIONS_H_OBJECTS?has_content>
<@mhc_expand_list list=LIST_SYSTEM_DEFINITIONS_H_OBJECTS/>
</#if>
<#if CONFIG_USE_SYS_TMR = true >
    SYS_MODULE_OBJ  sysTmr;
</#if>
<#if CONFIG_USE_SYS_DMA = true >
    SYS_MODULE_OBJ  sysDma;
</#if>
<#if CONFIG_USE_DRV_TMR = true >
<#if CONFIG_DRV_TMR_INST_0 == true>
    SYS_MODULE_OBJ  drvTmr0;
</#if>
<#if CONFIG_DRV_TMR_INST_1 == true>
    SYS_MODULE_OBJ  drvTmr1;
</#if>
<#if CONFIG_DRV_TMR_INST_2 == true>
    SYS_MODULE_OBJ  drvTmr2;
</#if>
<#if CONFIG_DRV_TMR_INST_3 == true>
    SYS_MODULE_OBJ  drvTmr3;
</#if>
<#if CONFIG_DRV_TMR_INST_4 == true>
    SYS_MODULE_OBJ  drvTmr4;
</#if>
<#if CONFIG_DRV_TMR_INST_5 == true>
    SYS_MODULE_OBJ  drvTmr5;
</#if>
<#if CONFIG_DRV_TMR_INST_6 == true>
    SYS_MODULE_OBJ  drvTmr6;
</#if>
<#if CONFIG_DRV_TMR_INST_7 == true>
    SYS_MODULE_OBJ  drvTmr7;
</#if>
<#if CONFIG_DRV_TMR_INST_8 == true>
    SYS_MODULE_OBJ  drvTmr8;
</#if>
</#if>
<#if CONFIG_USE_DRV_CTR == true>
<#if CONFIG_DRV_CTR_DRIVER_MODE == "DYNAMIC">
    SYS_MODULE_OBJ  drvCtr0;
</#if>
</#if>
<#if CONFIG_USE_DRV_I2S = true >
<#if CONFIG_DRV_I2S_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvI2S0;
</#if>
<#if CONFIG_DRV_I2S_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvI2S1;
</#if>
<#if CONFIG_DRV_I2S_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvI2S2;
</#if>
<#if CONFIG_DRV_I2S_INST_IDX3 == true>
    SYS_MODULE_OBJ  drvI2S3;
</#if>
<#if CONFIG_DRV_I2S_INST_IDX4 == true>
    SYS_MODULE_OBJ  drvI2S4;
</#if>
<#if CONFIG_DRV_I2S_INST_IDX5 == true>
    SYS_MODULE_OBJ  drvI2S5;
</#if>
</#if>
<#if CONFIG_USE_DRV_CODEC_AK4384?has_content><#if CONFIG_USE_DRV_CODEC_AK4384 == true >
<#if CONFIG_DRV_CODEC_AK4384_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvak4384Codec0;
</#if>
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK4953?has_content><#if CONFIG_USE_DRV_CODEC_AK4953 == true >
<#if CONFIG_DRV_CODEC_AK4953_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvak4953Codec0;
</#if>
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK4954?has_content><#if CONFIG_USE_DRV_CODEC_AK4954 == true >
<#if CONFIG_DRV_CODEC_AK4954_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvak4954Codec0;
</#if>
</#if></#if>
<#if CONFIG_USE_DRV_CODEC_AK4642?has_content><#if CONFIG_USE_DRV_CODEC_AK4642 == true >
<#if CONFIG_DRV_CODEC_AK4642_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvak4642Codec0;
</#if>
</#if></#if>

<#if CONFIG_USE_DRV_CODEC_AK7755?has_content><#if CONFIG_USE_DRV_CODEC_AK7755 == true >
<#if CONFIG_DRV_CODEC_AK7755_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvak7755Codec0;
</#if>
</#if></#if>
<#if CONFIG_USE_DRV_USART == true>
<#if CONFIG_DRV_USART_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvUsart0;
</#if>
<#if CONFIG_DRV_USART_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvUsart1;
</#if>
<#if CONFIG_DRV_USART_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvUsart2;
</#if>
<#if CONFIG_DRV_USART_INST_IDX3 == true>
    SYS_MODULE_OBJ  drvUsart3;
</#if>
<#if CONFIG_DRV_USART_INST_IDX4 == true>
    SYS_MODULE_OBJ  drvUsart4;
</#if>
<#if CONFIG_DRV_USART_INST_IDX5 == true>
    SYS_MODULE_OBJ  drvUsart5;
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25 == true>
<#if CONFIG_DRV_SST25_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_SST25_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvSst25Obj0;
</#if>
<#if CONFIG_DRV_SST25_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvSst25Obj1;
</#if>
<#if CONFIG_DRV_SST25_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvSst25Obj2;
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25VF020B == true>
<#if CONFIG_DRV_SST25VF020B_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_SST25VF020B_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvSst25vf020b0;
</#if>
<#if CONFIG_DRV_SST25VF020B_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvSst25vf020b1;
</#if>
<#if CONFIG_DRV_SST25VF020B_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvSst25vf020b2;
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25VF016B == true>
<#if CONFIG_DRV_SST25VF016B_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_SST25VF016B_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvSst25vf016b0;
</#if>
<#if CONFIG_DRV_SST25VF016B_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvSst25vf016b1;
</#if>
<#if CONFIG_DRV_SST25VF016B_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvSst25vf016b2;
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_SST25VF064C == true>
<#if CONFIG_DRV_SST25VF064C_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_SST25VF064C_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvSst25vf064c0;
</#if>
<#if CONFIG_DRV_SST25VF064C_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvSst25vf064c1;
</#if>
<#if CONFIG_DRV_SST25VF064C_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvSst25vf064c2;
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_IPF == true>
<#if CONFIG_DRV_IPF_DRIVER_MODE == "DYNAMIC">
    SYS_MODULE_OBJ  drvIpf0;
</#if>
</#if>
<#if CONFIG_USE_DRV_I2C == true>
<#if CONFIG_PIC32MK == true || CONFIG_PIC32MX == true || CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true>
<#if CONFIG_DRV_I2C_DRIVER_MODE == "DYNAMIC">
<#if CONFIG_DRV_I2C_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvI2C0;
</#if>
<#if CONFIG_DRV_I2C_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvI2C1;
</#if>
<#if CONFIG_DRV_I2C_INST_IDX2 == true>
    SYS_MODULE_OBJ  drvI2C2;
</#if>
<#if CONFIG_DRV_I2C_INST_IDX3 == true>
    SYS_MODULE_OBJ  drvI2C3;
</#if>
<#if CONFIG_DRV_I2C_INST_IDX4 == true>
    SYS_MODULE_OBJ  drvI2C4;
</#if>
</#if>
</#if>
</#if>
<#if CONFIG_USE_DRV_EEPROM = true >
    SYS_MODULE_OBJ  drvEeprom;
</#if>
<#if CONFIG_USE_DRV_NVM = true >
    SYS_MODULE_OBJ  drvNvm;
</#if>
<#if CONFIG_USE_DRV_SQI = true >
    SYS_MODULE_OBJ  drvSqi;
</#if>
<#if CONFIG_DRV_SST26_INST_IDX0?has_content>
<#if CONFIG_DRV_SST26_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvSst26Obj0;
</#if>
</#if>
<#if CONFIG_DRV_SST26_INST_IDX1?has_content>
<#if CONFIG_DRV_SST26_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvSst26Obj1;
</#if>
</#if>
<#if CONFIG_DRV_S25FL_INST_IDX0?has_content>
<#if CONFIG_DRV_S25FL_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvS25flObj0;
</#if>
</#if>
<#if CONFIG_DRV_S25FL_INST_IDX1?has_content>
<#if CONFIG_DRV_S25FL_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvS25flObj1;
</#if>
</#if>
<#if CONFIG_USE_DRV_SDCARD = true >
    SYS_MODULE_OBJ  drvSDCard;
</#if>
<#if CONFIG_USE_DRV_SDHC = true >
    SYS_MODULE_OBJ  drvSDHC;
</#if>
<#if CONFIG_DRV_SRAM_INST_IDX0 == true>
    SYS_MODULE_OBJ  drvSramObj0;
</#if>
<#if CONFIG_DRV_SRAM_INST_IDX1 == true>
    SYS_MODULE_OBJ  drvSramObj1;
</#if>
<#if CONFIG_USE_DRV_PMP = true >
    SYS_MODULE_OBJ  drvPMP0;
</#if>
<#if CONFIG_USE_SYS_DEBUG == true>
    SYS_MODULE_OBJ  sysDebug;
</#if>
<#if CONFIG_SYS_MSG_INST_IDX0 == true>
    SYS_MODULE_OBJ  sysMsg0;
</#if>
<#if CONFIG_SYS_MSG_INST_IDX1 == true>
    SYS_MODULE_OBJ  sysMsg1;
</#if>
<#if CONFIG_SYS_MSG_INST_IDX2 == true>
    SYS_MODULE_OBJ  sysMsg2;
</#if>
<#if CONFIG_SYS_MSG_INST_IDX3 == true>
    SYS_MODULE_OBJ  sysMsg3;
</#if>
<#if CONFIG_SYS_MSG_INST_IDX4 == true>
    SYS_MODULE_OBJ  sysMsg4;
</#if>
<#if CONFIG_SYS_CONSOLE_INST_IDX0 == true>
    SYS_MODULE_OBJ  sysConsole0;
</#if>
<#if CONFIG_SYS_CONSOLE_INST_IDX1 == true>
    SYS_MODULE_OBJ  sysConsole1;
</#if>
<#if CONFIG_DRV_SPI_USE_DRIVER == true>
<#include "/framework/driver/spi/config/drv_spi_sys_defs.h.ftl">
</#if>
<#if CONFIG_USE_USB_STACK == true>
<#include "/framework/usb/templates/system_definitions.h.objects.ftl">
</#if>
<#if CONFIG_USE_SYS_TOUCH == true>
    SYS_MODULE_OBJ  sysTouchObject0;
</#if>
<#if CONFIG_USE_TCPIP_STACK == true>
    SYS_MODULE_OBJ  tcpip;
</#if>
<#if CONFIG_DRV_MIIM_USE_DRIVER == true>
    SYS_MODULE_OBJ  drvMiim;
</#if>
<#if CONFIG_USE_DRV_TOUCH_ADC == true>
    SYS_MODULE_OBJ  drvTouchAdc;
</#if>
<#if CONFIG_USE_DRV_TOUCH_ADC10BIT == true>
    SYS_MODULE_OBJ  drvAdc10bit;
</#if>
<#if CONFIG_USE_DRV_TOUCH_AR1021 == true>
    SYS_MODULE_OBJ  drvAr1021;
</#if>
<#if CONFIG_USE_DRV_TOUCH_MTCH6301 == true>
    SYS_MODULE_OBJ  drvMtch6301;
</#if>
<#if CONFIG_USE_DRV_TOUCH_MTCH6303 == true>
    SYS_MODULE_OBJ  drvMtch6303;
</#if>
<#if CONFIG_USE_DRV_TOUCH_MXT336T == true>
    SYS_MODULE_OBJ  drvMXT336T;
	SYS_MODULE_OBJ  drvMxt0;
</#if>
<#if CONFIG_USE_DRV_TOUCH_MXT336T_UPDATED == true>
    SYS_MODULE_OBJ  drvMXT336T;
	SYS_MODULE_OBJ  drvMxt0;
</#if>
<#if CONFIG_USE_DRV_INPUT_MXT336T == true>
    SYS_MODULE_OBJ  drvMXT336T;
</#if>
<#if CONFIG_USE_DRV_TOUCH_GENERIC == true>
    SYS_MODULE_OBJ  drvTouchgeneric;

</#if>
<#if CONFIG_USE_DRV_OVM7690 == true>
    SYS_MODULE_OBJ  drvOvm7690;
</#if>
<#include "/framework/net/templates/system_definitions.h.object.ftl">
<#if CONFIG_USE_SAMPLE_MODULE == true>
<#include "/framework/sample/templates/system_definitions.h.obj.ftl">
</#if>

} SYSTEM_OBJECTS;

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

extern SYSTEM_OBJECTS sysObj;

<#if LIST_SYSTEM_DEFINITIONS_H_EXTERNS?has_content>
<@mhc_expand_list list=LIST_SYSTEM_DEFINITIONS_H_EXTERNS/>
</#if>
<#if CONFIG_USE_USB_STACK == true>
<#include "/framework/usb/templates/system_definitions.h.externs.ftl">

</#if>
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _SYS_DEFINITIONS_H */
/*******************************************************************************
 End of File
*/

