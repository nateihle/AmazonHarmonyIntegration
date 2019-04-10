
<#if CONFIG_USE_BLUETOOTH_LIBRARIES == true>
// *****************************************************************************
// *****************************************************************************
// Section: Bluetooth Configuration
// *****************************************************************************
// *****************************************************************************
/* Bluetooth Device ID settings
 *
 * These macros define the bluetooth device ID.
 *
 * BT_DEVICE_DESIGN_ID         - sets the most significant 8 bytes
 * BT_DEVICE_ID_2LSB           - sets the least significant 4 bytes
 *
 * BT_DEVICE_ID_2LSB_RANDOMIZE - controls randomization of the 2LSB portion.
 *                               Note: 2LSB is randomized at power-on only.
 *
 *                               0 = disabled
 *                               1 = enabled
 *
 * device ID scheme:
 *        Design   2LSB
 *     -- -- -- -- ++ ++
 * ID :XX:XX:XX:XX:XX:XX
*/
//                                      Device
//                                      DesignID ID
//                                      --------++++
	
#define BT_CONNECTION_NAME 					    	"${CONFIG_SET_BROADCAST_NAME}"
#define BT_DEVICE_DESIGN_ID           				${CONFIG_SET_MAC_ADDRESS}
#define BT_MAX_PORTS_SPP 							${CONFIG_USE_MAX_BLUETOOTH_CONNECTIONS}
<#if CONFIG_SET_RANDOM_ADDRESSING == true>
#define BT_DEVICE_ID_2LSB_RANDOMIZE   				1 //enabled
#define BT_DEVICE_ID_2LSB             				0xFFFF
<#else>
#define BT_DEVICE_ID_2LSB_RANDOMIZE   				0 //disabled
#define BT_DEVICE_ID_2LSB             				${CONFIG_SET_BT_ADDRESS_LAST_4}
</#if>
#define BT_CONTROLLER 								${CONFIG_SELECT_RADIO_DRIVER_ACTUAL}

// *****************************************************************************
// *****************************************************************************
<#if CONFIG_APPLICATION_CONFIG_ONOFF?has_content> <#--has content-->
<#if CONFIG_APPLICATION_CONFIG_ONOFF == true> <#--Use demo settings-->

<#if CONFIG_BSP_BT_AUDIO_DK?has_content && CONFIG_BSP_BT_AUDIO_DK == true > <#--App Demo code-->
#define APP_BUTTON1_PIN 				(1<<PORTS_BIT_POS_0)
#define APP_BUTTON2_PIN 				(1<<PORTS_BIT_POS_1)
#define APP_BUTTON3_PIN 				(1<<PORTS_BIT_POS_10)
#define APP_BUTTON4_PIN 				(1<<PORTS_BIT_POS_12)
#define APP_BUTTON5_PIN 				(1<<PORTS_BIT_POS_13)
#define APP_BUTTON6_PIN 				(1<<PORTS_BIT_POS_14)
    
/* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

<#if CONFIG_USE_A2DP_AVRCP_PROFILE ==true><#-- a2dp-->

#define APP_LED_BT_READY1_ON()                          BSP_LEDOn(BSP_LED_6)
#define APP_LED_BT_READY1_OFF()                         BSP_LEDOff(BSP_LED_6)
#define APP_LED_BT_READY1_TOGGLE()                      BSP_LEDToggle(BSP_LED_6)
#define APP_LED_BT_READY2_ON()                          BSP_LEDOn(BSP_LED_7)
#define APP_LED_BT_READY2_OFF()                         BSP_LEDOff(BSP_LED_7)
#define APP_LED_BT_READY2_TOGGLE()                      BSP_LEDToggle(BSP_LED_7)
#define APP_LED_BT_STREAM_ON()                          BSP_LEDOn(BSP_LED_8)
#define APP_LED_BT_STREAM_OFF()                         BSP_LEDOff(BSP_LED_8)
#define APP_LED_BT_STREAM_TOGGLE()                      BSP_LEDToggle(BSP_LED_8)

/* Volume Potentiometer Specific defines. */
#define APP_ADC_VOLTAGEREFERENCESELECT    	PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS)
#define APP_ADC_SAMPLINGMODESELECT              PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA)
#define APP_ADC_RESULTBUFFERMODESELECT     	PLIB_ADC_ResultBufferModeSelect(ADC_ID_1, ADC_BUFFER_MODE_ONE_16WORD_BUFFER)
#define APP_ADC_SAMPLESPERINTERRUPTSELECT	PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_2SAMPLES_PER_INTERRUPT)
#define APP_ADC_MUXAINPUTSCANENABLE     	PLIB_ADC_MuxAInputScanEnable(ADC_ID_1)
#define APP_ADC_SAMPLEACQUISITIONTIMESET     	PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 0x1F)
#define APP_ADC_CONVERSIONCLOCKSET      	PLIB_ADC_ConversionClockSet(ADC_ID_1,APP_PBCLK_CLOCK_HZ, 5)
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT	PLIB_ADC_MuxChannel0InputNegativeSelect(ADC_ID_1,ADC_MUX_A,ADC_INPUT_NEGATIVE_VREF_MINUS)
#define APP_ADC_INPUTSCANMASKADD		PLIB_ADC_InputScanMaskAdd(ADC_ID_1, ADC_INPUT_SCAN_AN11)
#define APP_ADC_SAMPLEAUTOSTARTENABLE     	PLIB_ADC_SampleAutoStartEnable(ADC_ID_1)
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT   PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT)
#define APP_ADC_ENABLE    			PLIB_ADC_Enable(ADC_ID_1)
#define APP_ADC_RESULTGETBYINDEX                PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0)
#define APP_VOLUMEINIT()                        volumeInit()
#define APP_VOLUMETASK()                        volumeTask()
#define APP_GFX_MENU_DRAW()                     GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       DisplayTasks()

/* Peripheral Bus Clock specific defines */
#define APP_PBCLK_CLOCK_HZ                              (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1

#define APP_BT_BUTTON_REPEAT_INIT_HZ                    (2)
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD \
    (APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/APP_BT_BUTTON_REPEAT_INIT_HZ): \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)    /APP_BT_BUTTON_REPEAT_INIT_HZ)

#define APP_BT_BUTTON_REPEAT_REPEAT_HZ                  (100)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD \
	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/APP_BT_BUTTON_REPEAT_REPEAT_HZ): \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)    /APP_BT_BUTTON_REPEAT_REPEAT_HZ)



</#if><#-- a2dp end-->
/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN                           BSP_AK4384_PDNOn

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_B))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_1);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_3);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_4);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_5);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_6);
#define APP_VIRTUAL_LED_Y_OFFSET    0


<#elseif CONFIG_BSP_BT_AUDIO_DK_AK7755_BOARD?has_content && CONFIG_BSP_BT_AUDIO_DK_AK7755_BOARD == true > <#--App Demo code-->
#define APP_BUTTON1_PIN 				(1<<PORTS_BIT_POS_0)
#define APP_BUTTON2_PIN 				(1<<PORTS_BIT_POS_1)
#define APP_BUTTON3_PIN 				(1<<PORTS_BIT_POS_10)
#define APP_BUTTON4_PIN 				(1<<PORTS_BIT_POS_12)
#define APP_BUTTON5_PIN 				(1<<PORTS_BIT_POS_13)
#define APP_BUTTON6_PIN 				(1<<PORTS_BIT_POS_14)
    
/* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

<#if CONFIG_USE_A2DP_AVRCP_PROFILE ==true><#-- a2dp-->

#define APP_LED_BT_READY1_ON()                          BSP_LEDOn(BSP_LED_6)
#define APP_LED_BT_READY1_OFF()                         BSP_LEDOff(BSP_LED_6)
#define APP_LED_BT_READY1_TOGGLE()                      BSP_LEDToggle(BSP_LED_6)
#define APP_LED_BT_READY2_ON()                          BSP_LEDOn(BSP_LED_7)
#define APP_LED_BT_READY2_OFF()                         BSP_LEDOff(BSP_LED_7)
#define APP_LED_BT_READY2_TOGGLE()                      BSP_LEDToggle(BSP_LED_7)
#define APP_LED_BT_STREAM_ON()                          BSP_LEDOn(BSP_LED_8)
#define APP_LED_BT_STREAM_OFF()                         BSP_LEDOff(BSP_LED_8)
#define APP_LED_BT_STREAM_TOGGLE()                      BSP_LEDToggle(BSP_LED_8)

/* Volume Potentiometer Specific defines. */
#define APP_ADC_VOLTAGEREFERENCESELECT    	PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS)
#define APP_ADC_SAMPLINGMODESELECT              PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA)
#define APP_ADC_RESULTBUFFERMODESELECT     	PLIB_ADC_ResultBufferModeSelect(ADC_ID_1, ADC_BUFFER_MODE_ONE_16WORD_BUFFER)
#define APP_ADC_SAMPLESPERINTERRUPTSELECT	PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_2SAMPLES_PER_INTERRUPT)
#define APP_ADC_MUXAINPUTSCANENABLE     	PLIB_ADC_MuxAInputScanEnable(ADC_ID_1)
#define APP_ADC_SAMPLEACQUISITIONTIMESET     	PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 0x1F)
#define APP_ADC_CONVERSIONCLOCKSET      	PLIB_ADC_ConversionClockSet(ADC_ID_1,APP_PBCLK_CLOCK_HZ, 5)
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT	PLIB_ADC_MuxChannel0InputNegativeSelect(ADC_ID_1,ADC_MUX_A,ADC_INPUT_NEGATIVE_VREF_MINUS)
#define APP_ADC_INPUTSCANMASKADD		PLIB_ADC_InputScanMaskAdd(ADC_ID_1, ADC_INPUT_SCAN_AN11)
#define APP_ADC_SAMPLEAUTOSTARTENABLE     	PLIB_ADC_SampleAutoStartEnable(ADC_ID_1)
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT   PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT)
#define APP_ADC_ENABLE    			PLIB_ADC_Enable(ADC_ID_1)
#define APP_ADC_RESULTGETBYINDEX                PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0)
#define APP_VOLUMEINIT()                        volumeInit()
#define APP_VOLUMETASK()                        volumeTask()
#define APP_GFX_MENU_DRAW()                     GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       DisplayTasks()

/* Peripheral Bus Clock specific defines */
#define APP_PBCLK_CLOCK_HZ                              (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1

#define APP_BT_BUTTON_REPEAT_INIT_HZ                    (2)
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD \
    (APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/APP_BT_BUTTON_REPEAT_INIT_HZ): \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)    /APP_BT_BUTTON_REPEAT_INIT_HZ)

#define APP_BT_BUTTON_REPEAT_REPEAT_HZ                  (100)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD \
	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/APP_BT_BUTTON_REPEAT_REPEAT_HZ): \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)    /APP_BT_BUTTON_REPEAT_REPEAT_HZ)



</#if><#-- a2dp end-->
/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN                           BSP_AK7755_PDNOn

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_B))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_1);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_3);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_4);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_5);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_6);
#define APP_VIRTUAL_LED_Y_OFFSET    0

<#elseif CONFIG_BSP_BT_AUDIO_DK_AK4642_BOARD?has_content && CONFIG_BSP_BT_AUDIO_DK_AK4642_BOARD == true > <#--App Demo code-->
#define APP_BUTTON1_PIN 				(1<<PORTS_BIT_POS_0)
#define APP_BUTTON2_PIN 				(1<<PORTS_BIT_POS_1)
#define APP_BUTTON3_PIN 				(1<<PORTS_BIT_POS_10)
#define APP_BUTTON4_PIN 				(1<<PORTS_BIT_POS_12)
#define APP_BUTTON5_PIN 				(1<<PORTS_BIT_POS_13)
#define APP_BUTTON6_PIN 				(1<<PORTS_BIT_POS_14)
    
/* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

<#if CONFIG_USE_A2DP_AVRCP_PROFILE ==true><#-- a2dp-->

#define APP_LED_BT_READY1_ON()                          BSP_LEDOn(BSP_LED_6)
#define APP_LED_BT_READY1_OFF()                         BSP_LEDOff(BSP_LED_6)
#define APP_LED_BT_READY1_TOGGLE()                      BSP_LEDToggle(BSP_LED_6)
#define APP_LED_BT_READY2_ON()                          BSP_LEDOn(BSP_LED_7)
#define APP_LED_BT_READY2_OFF()                         BSP_LEDOff(BSP_LED_7)
#define APP_LED_BT_READY2_TOGGLE()                      BSP_LEDToggle(BSP_LED_7)
#define APP_LED_BT_STREAM_ON()                          BSP_LEDOn(BSP_LED_8)
#define APP_LED_BT_STREAM_OFF()                         BSP_LEDOff(BSP_LED_8)
#define APP_LED_BT_STREAM_TOGGLE()                      BSP_LEDToggle(BSP_LED_8)

/* Volume Potentiometer Specific defines. */
#define APP_ADC_VOLTAGEREFERENCESELECT    	PLIB_ADC_VoltageReferenceSelect(ADC_ID_1, ADC_REFERENCE_VDD_TO_AVSS)
#define APP_ADC_SAMPLINGMODESELECT              PLIB_ADC_SamplingModeSelect(ADC_ID_1, ADC_SAMPLING_MODE_MUXA)
#define APP_ADC_RESULTBUFFERMODESELECT     	PLIB_ADC_ResultBufferModeSelect(ADC_ID_1, ADC_BUFFER_MODE_ONE_16WORD_BUFFER)
#define APP_ADC_SAMPLESPERINTERRUPTSELECT	PLIB_ADC_SamplesPerInterruptSelect(ADC_ID_1, ADC_2SAMPLES_PER_INTERRUPT)
#define APP_ADC_MUXAINPUTSCANENABLE     	PLIB_ADC_MuxAInputScanEnable(ADC_ID_1)
#define APP_ADC_SAMPLEACQUISITIONTIMESET     	PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 0x1F)
#define APP_ADC_CONVERSIONCLOCKSET      	PLIB_ADC_ConversionClockSet(ADC_ID_1,APP_PBCLK_CLOCK_HZ, 5)
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT	PLIB_ADC_MuxChannel0InputNegativeSelect(ADC_ID_1,ADC_MUX_A,ADC_INPUT_NEGATIVE_VREF_MINUS)
#define APP_ADC_INPUTSCANMASKADD		PLIB_ADC_InputScanMaskAdd(ADC_ID_1, ADC_INPUT_SCAN_AN11)
#define APP_ADC_SAMPLEAUTOSTARTENABLE     	PLIB_ADC_SampleAutoStartEnable(ADC_ID_1)
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT   PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT)
#define APP_ADC_ENABLE    			PLIB_ADC_Enable(ADC_ID_1)
#define APP_ADC_RESULTGETBYINDEX                PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0)
#define APP_VOLUMEINIT()                        volumeInit()
#define APP_VOLUMETASK()                        volumeTask()
#define APP_GFX_MENU_DRAW()                     GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       DisplayTasks()

/* Peripheral Bus Clock specific defines */
#define APP_PBCLK_CLOCK_HZ                              (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_1
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1

#define APP_BT_BUTTON_REPEAT_INIT_HZ                    (2)
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD \
    (APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/APP_BT_BUTTON_REPEAT_INIT_HZ): \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)    /APP_BT_BUTTON_REPEAT_INIT_HZ)

#define APP_BT_BUTTON_REPEAT_REPEAT_HZ                  (100)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD \
	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/APP_BT_BUTTON_REPEAT_REPEAT_HZ): \
        (APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)    /APP_BT_BUTTON_REPEAT_REPEAT_HZ)
</#if><#-- a2dp end-->


/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN                           BSP_AK4384_PDNOn

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_B))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_1);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_3);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_4);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_5);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_6);
#define APP_VIRTUAL_LED_Y_OFFSET    0

<#elseif CONFIG_BSP_PIC32MX270F512L_PIM_BT_AUDIO_DK?has_content && CONFIG_BSP_PIC32MX270F512L_PIM_BT_AUDIO_DK == true>
#define APP_BUTTON1_PIN 				(1<<PORTS_BIT_POS_0)
#define APP_BUTTON2_PIN 				(1<<PORTS_BIT_POS_1)
#define APP_BUTTON3_PIN 				(1<<PORTS_BIT_POS_10)
#define APP_BUTTON4_PIN 				(1<<PORTS_BIT_POS_12)
#define APP_BUTTON5_PIN 				(1<<PORTS_BIT_POS_13)
#define APP_BUTTON6_PIN 				(1<<PORTS_BIT_POS_14)
    
/* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN    			BSP_AK4384_PDNOn

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_B))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_1);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_3);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_4);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_5);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_B, BSP_SWITCH_6);
#define APP_VIRTUAL_LED_Y_OFFSET    0

<#elseif CONFIG_BSP_PIC32MZ_EC_PIM_BT_AUDIO_DK?has_content && CONFIG_BSP_PIC32MZ_EC_PIM_BT_AUDIO_DK == true>

/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN    			BSP_AK4384_PDNOn

/* BT Buttons PORT Settings */
#define APP_BUTTON1_PIN                                 (1<<BSP_SWITCH_1) /*RK0*/
#define APP_BUTTON2_PIN                                 (1<<BSP_SWITCH_2) /*RA10*/
#define APP_BUTTON3_PIN                                 (1<<BSP_SWITCH_3) /*RJ14*/
#define APP_BUTTON4_PIN                                 (1<<BSP_SWITCH_4) /*RH0*/
#define APP_BUTTON5_PIN                                 (1<<BSP_SWITCH_5) /*RH1*/
#define APP_BUTTON6_PIN                                 (1<<BSP_SWITCH_6) /*RH2*/

/* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_K)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_J)|SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_H))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_K, BSP_SWITCH_1);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, BSP_SWITCH_2);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_J, BSP_SWITCH_3);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_H, BSP_SWITCH_4);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_H, BSP_SWITCH_5)

#define APP_VIRTUAL_LED_Y_OFFSET    0

 

<#elseif CONFIG_BSP_PIC32MZ_EF_PIM_BT_AUDIO_DK?has_content && CONFIG_BSP_PIC32MZ_EF_PIM_BT_AUDIO_DK == true>
/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_G
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_15
#define APP_SET_BLUETOOTH_PIN    			BSP_AK4384_PDNOn

/* BT Buttons PORT Settings */
#define APP_BUTTON1_PIN                                 (1<<BSP_SWITCH_1) /*RK0*/
#define APP_BUTTON2_PIN                                 (1<<BSP_SWITCH_2) /*RA10*/
#define APP_BUTTON3_PIN                                 (1<<BSP_SWITCH_3) /*RJ14*/
#define APP_BUTTON4_PIN                                 (1<<BSP_SWITCH_4) /*RH0*/
#define APP_BUTTON5_PIN                                 (1<<BSP_SWITCH_5) /*RH1*/
#define APP_BUTTON6_PIN                                 (1<<BSP_SWITCH_6) /*RH2*/

///* Interrupt Priorties of System Components */
//#define SYS_BT_BUTTON_CN_PRIO                           INT_PRIORITY_LEVEL2
//#define SYS_BT_BUTTON_CN_SUBPRIO                        INT_SUBPRIORITY_LEVEL0

/* Interrupt Vectors of System Components */
#define SYS_BT_BUTTON_CN_VECTOR                         INT_VECTOR_CHANGE_NOTICE_A

/* Interrupt Sources of System Components */
#define SYS_BT_BUTTON_CN_SOURCE_1                       INT_SOURCE_CHANGE_NOTICE_A
#define SYS_BT_BUTTON_CN_SOURCE_2                       INT_SOURCE_CHANGE_NOTICE_H
#define SYS_BT_BUTTON_CN_SOURCE_3                       INT_SOURCE_CHANGE_NOTICE_J
#define SYS_BT_BUTTON_CN_SOURCE_4                       INT_SOURCE_CHANGE_NOTICE_K

/* Interrupt ISR of System Components */
//#define SYS_BT_BUTTON_CN_ISR                            (_CHANGE_NOTICE_H_VECTOR )

/* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_5)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_5)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_5)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_6)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_6)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_6)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_7)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_7)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_7)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_8)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_8)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_8)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_9)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_9)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_9)

/* App Buttons */
#define APP_READ_BUTTON_PORTS()                         (SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_K)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_A)| SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_J)|SYS_PORTS_Read(PORTS_ID_0,PORT_CHANNEL_H))
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_K, PORTS_BIT_POS_0);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_10);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_14);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0);\
                                                        PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1)

#define APP_VIRTUAL_LED_Y_OFFSET    0


<#elseif CONFIG_BSP_PIC32MZ_EC_SK_MEB2?has_content && CONFIG_BSP_PIC32MZ_EC_SK_MEB2 == true>
/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_B
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_2

/* BT Buttons Settings */
#define APP_BUTTON1_PIN                                 (1<<PORTS_BIT_POS_0)   /* Only Button 1 Functionality Implemented in Demo */
#define APP_BUTTON2_PIN                                 (1<<PORTS_BIT_POS_12)  /* Button 2 Functionality not Implemented in Demo as the switch is on Starter KIT not on MEB-II */
#define APP_BUTTON3_PIN                                 (1<<PORTS_BIT_POS_13)  /* Button 3 Functionality not Implemented in Demo as the switch is on Starter KIT not on MEB-II */
#define APP_BUTTON4_PIN                                 (1<<PORTS_BIT_POS_14)  /* Button 4 Functionality not Implemented Pin Shared with UARTTX */
#define APP_BUTTON5_PIN                                 (1<<0)                 /* Button 5: Dummy not available on MEB-II and ESK. */
<#if CONFIG_USE_A2DP_AVRCP_PROFILE ==true><#--a2dp-->
#define APP_READ_BUTTONS1_PORT                         (BSP_SwitchStateGet(BSP_SWITCH_S1) << BSP_SWITCH_S1)
#define APP_READ_BUTTON_PORTS()                        (BSP_SwitchStateGet(BSP_SWITCH_S1) << BSP_SWITCH_S1)

/* Application LED's */
#define APP_LED_BT_READY1_ON()                          BSP_LEDOn(BSP_LED_D4)
#define APP_LED_BT_READY1_OFF()                         BSP_LEDOff(BSP_LED_D4)
#define APP_LED_BT_READY1_TOGGLE()                      BSP_LEDToggle(BSP_LED_D4)
#define APP_LED_BT_READY2_ON()                          BSP_LEDOn(BSP_LED_D5)
#define APP_LED_BT_READY2_OFF()                         BSP_LEDOff(BSP_LED_D5)
#define APP_LED_BT_READY2_TOGGLE()                      BSP_LEDToggle(BSP_LED_D5)
#define APP_LED_BT_STREAM_ON()                          BSP_LEDOn(BSP_LED_D6)
#define APP_LED_BT_STREAM_OFF()                         BSP_LEDOff(BSP_LED_D6)
#define APP_LED_BT_STREAM_TOGGLE()                      BSP_LEDToggle(BSP_LED_D6)

/* Peripheral Bus Clock frequency */
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_2
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_3
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD          (DRV_TMR_PRESCALE_IDX1 == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/2): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/2)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/128): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/128)
/* Volume Potentiometer Specific defines. Not needed in this configuration */
#define APP_ADC_VOLTAGEREFERENCESELECT
#define APP_ADC_SAMPLINGMODESELECT
#define APP_ADC_RESULTBUFFERMODESELECT
#define APP_ADC_SAMPLESPERINTERRUPTSELECT
#define APP_ADC_MUXAINPUTSCANENABLE
#define APP_ADC_SAMPLEACQUISITIONTIMESET
#define APP_ADC_CONVERSIONCLOCKSET
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT
#define APP_ADC_INPUTSCANMASKADD
#define APP_ADC_SAMPLEAUTOSTARTENABLE
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT
#define APP_ADC_ENABLE
#define APP_ADC_RESULTGETBYINDEX               120
#define APP_VOLUMEINIT()
#define APP_VOLUMETASK()
#define APP_GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       DisplayTasks()


</#if><#-- a2dp end-->

/* Interrupt ISR of System Components */
#define SYS_BT_TICK_TIMER_ISR                           _TIMER_1_VECTOR
#define SYS_BT_BUTTON_REPEAT_TIMER_ISR                  _TIMER_3_VECTOR
#define SYS_BT_BUTTON_CN_ISR                            _CHANGE_NOTICE_A_VECTOR
#define SYS_BT_USART_RX_DMA_CHANNEL_ISR                 _DMA3_VECTOR
#define SYS_BT_USART_TX_DMA_CHANNEL_ISR                 _DMA0_VECTOR
#define SYS_BT_USART_RX_ISR                             _UART2_RX_VECTOR
#define SYS_BT_USART_TX_ISR                             _UART2_TX_VECTOR
//
///* Application LED's */
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_D3)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_D3)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_D3)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_D4)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_D4)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_D4)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_D5)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_D5)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_D5)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_D6)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_D6)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_D6)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_D7)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_D7)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_D7)

<#if CONFIG_USE_A2DP_AVRCP_PROFILE == !true>
/* Button Read */
#define APP_READ_BUTTON_PORTS()                         SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_A)
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define APP_CHANGE_NOTICE_ON()                          PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_ID_0, PORT_CHANNEL_A)
#define APP_ENABLE_PULL_UP()                            PLIB_PORTS_ChangeNoticePullUpPerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
</#if>
<#if CONFIG_USE_A2DP_AVRCP_PROFILE == true>
/* Peripheral Bus Clock frequency */
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_2
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_3
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD          (DRV_TMR_PRESCALE_IDX1 == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/2): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/2)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/128): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/128)
/* Volume Potentiometer Specific defines. Not needed in this configuration */
#define APP_ADC_VOLTAGEREFERENCESELECT
#define APP_ADC_SAMPLINGMODESELECT
#define APP_ADC_RESULTBUFFERMODESELECT
#define APP_ADC_SAMPLESPERINTERRUPTSELECT
#define APP_ADC_MUXAINPUTSCANENABLE
#define APP_ADC_SAMPLEACQUISITIONTIMESET
#define APP_ADC_CONVERSIONCLOCKSET
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT
#define APP_ADC_INPUTSCANMASKADD
#define APP_ADC_SAMPLEAUTOSTARTENABLE
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT
#define APP_ADC_ENABLE
#define APP_ADC_RESULTGETBYINDEX               120
#define APP_VOLUMEINIT()
#define APP_VOLUMETASK()
#define APP_GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       DisplayTasks()


</#if><#-- a2dp end-->



#define APP_VIRTUAL_LED_Y_OFFSET    86
/* Peripheral Bus 2 Clock frequency (USART and SPI modules are on PBCLK2)*/
#define SYS_PBCLK2_CLOCK_HZ                             (uint32_t) 100000000UL
/* Peripheral Bus 3 Clock frequency (Timer modules are on PBCLK3)*/
#define SYS_PBCLK3_CLOCK_HZ                             (uint32_t) 50000000UL

<#elseif CONFIG_BSP_PIC32MZ_EF_SK_MEB2?has_content && CONFIG_BSP_PIC32MZ_EF_SK_MEB2 || CONFIG_BSP_PIC32MZ_EF_SK_MEB2_WVGA?has_content && CONFIG_BSP_PIC32MZ_EF_SK_MEB2_WVGA == true>
<#if CONFIG_USE_A2DP_AVRCP_PROFILE == true>
/* Peripheral Bus Clock frequency */
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_2
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_3
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD          (DRV_TMR_PRESCALE_IDX1 == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/2): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/2)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/128): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/128)
/* Volume Potentiometer Specific defines. Not needed in this configuration */
#define APP_ADC_VOLTAGEREFERENCESELECT
#define APP_ADC_SAMPLINGMODESELECT
#define APP_ADC_RESULTBUFFERMODESELECT
#define APP_ADC_SAMPLESPERINTERRUPTSELECT
#define APP_ADC_MUXAINPUTSCANENABLE
#define APP_ADC_SAMPLEACQUISITIONTIMESET
#define APP_ADC_CONVERSIONCLOCKSET
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT
#define APP_ADC_INPUTSCANMASKADD
#define APP_ADC_SAMPLEAUTOSTARTENABLE
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT
#define APP_ADC_ENABLE
#define APP_ADC_RESULTGETBYINDEX               120
#define APP_VOLUMEINIT()
#define APP_VOLUMETASK()
#define APP_GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       DisplayTasks()
</#if>
/* Bluetooth Reset */
#define APP_CLEAR_BLUETOOTH_PIN()                       BSP_BT_PINStateSet(BSP_BT_STATE_LOW)
#define APP_SET_BLUETOOTH_PIN()                         BSP_BT_PINStateSet(BSP_BT_STATE_HIGH)

/* BT Reset PORT settings */
#define APP_BT_RESET_PORT                               PORT_CHANNEL_B
#define APP_BT_RESET_BIT                                PORTS_BIT_POS_2

/* BT Buttons Settings */
#define APP_BUTTON1_PIN                                 (1<<PORTS_BIT_POS_0)   /* Only Button 1 Functionality Implemented in Demo */
#define APP_BUTTON2_PIN                                 (1<<PORTS_BIT_POS_12)  /* Button 2 Functionality not Implemented in Demo as the switch is on Starter KIT not on MEB-II */
#define APP_BUTTON3_PIN                                 (1<<PORTS_BIT_POS_13)  /* Button 3 Functionality not Implemented in Demo as the switch is on Starter KIT not on MEB-II */
#define APP_BUTTON4_PIN                                 (1<<PORTS_BIT_POS_14)  /* Button 4 Functionality not Implemented Pin Shared with UARTTX */
#define APP_BUTTON5_PIN                                 (1<<0)                 /* Button 5: Dummy not available on MEB-II and ESK. */
<#if CONFIG_USE_A2DP_AVRCP_PROFILE == true><#-- -->
#define APP_READ_BUTTONS1_PORT                         (BSP_SwitchStateGet(BSP_SWITCH_S1) << BSP_SWITCH_S1)
#define APP_READ_BUTTON_PORTS()                        (BSP_SwitchStateGet(BSP_SWITCH_S1) << BSP_SWITCH_S1)

</#if>
/* Interrupt ISR of System Components */
#define SYS_BT_TICK_TIMER_ISR                           _TIMER_1_VECTOR
#define SYS_BT_BUTTON_REPEAT_TIMER_ISR                  _TIMER_3_VECTOR
#define SYS_BT_BUTTON_CN_ISR                            _CHANGE_NOTICE_A_VECTOR
#define SYS_BT_USART_RX_DMA_CHANNEL_ISR                 _DMA3_VECTOR
#define SYS_BT_USART_TX_DMA_CHANNEL_ISR                 _DMA0_VECTOR
#define SYS_BT_USART_RX_ISR                             _UART2_RX_VECTOR
#define SYS_BT_USART_TX_ISR                             _UART2_TX_VECTOR

//* Application LED's */
<#if CONFIG_USE_A2DP_AVRCP_PROFILE == true>
#define APP_LED_BT_READY1_ON()                          BSP_LEDOn(BSP_LED_D4)
#define APP_LED_BT_READY1_OFF()                         BSP_LEDOff(BSP_LED_D4)
#define APP_LED_BT_READY1_TOGGLE()                      BSP_LEDToggle(BSP_LED_D4)
#define APP_LED_BT_READY2_ON()                          BSP_LEDOn(BSP_LED_D5)
#define APP_LED_BT_READY2_OFF()                         BSP_LEDOff(BSP_LED_D5)
#define APP_LED_BT_READY2_TOGGLE()                      BSP_LEDToggle(BSP_LED_D5)
#define APP_LED_BT_STREAM_ON()                          BSP_LEDOn(BSP_LED_D6)
#define APP_LED_BT_STREAM_OFF()                         BSP_LEDOff(BSP_LED_D6)
#define APP_LED_BT_STREAM_TOGGLE()                      BSP_LEDToggle(BSP_LED_D6)
</#if>
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_D3)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_D3)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_D3)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_D4)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_D4)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_D4)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_D5)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_D5)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_D5)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_D6)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_D6)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_D6)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_D7)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_D7)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_D7)

<#if CONFIG_USE_A2DP_AVRCP_PROFILE == !true>
/* Button Read */
#define APP_READ_BUTTON_PORTS()                         SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_A)
#define APP_ENABLE_BUTTON_CHANGE_NOTICE()               PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define APP_CHANGE_NOTICE_ON()                          PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_ID_0, PORT_CHANNEL_A)
#define APP_ENABLE_PULL_UP()                            PLIB_PORTS_ChangeNoticePullUpPerPortEnable(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
</#if>

#define APP_VIRTUAL_LED_Y_OFFSET    86
/* Peripheral Bus 2 Clock frequency (USART and SPI modules are on PBCLK2)*/
#define SYS_PBCLK2_CLOCK_HZ                             (uint32_t) 100000000UL
/* Peripheral Bus 3 Clock frequency (Timer modules are on PBCLK3)*/
#define SYS_PBCLK3_CLOCK_HZ                             (uint32_t) 50000000UL

<#elseif CONFIG_BSP_PIC32MZ_DA_SK_MEB2_WVGA?has_content && CONFIG_BSP_PIC32MZ_DA_SK_MEB2_WVGA == true>

#define APP_BUTTON1_PIN                                 (1<<0) /* Button 1 Functionality not Implemented in Demo  */
#define APP_BUTTON2_PIN                                 (1<<0) /* Button 2 Functionality not Implemented in Demo  */
#define APP_BUTTON3_PIN                                 (1<<0) /* Button 3 Functionality not Implemented in Demo  */
#define APP_BUTTON4_PIN                                 (1<<0) /* Button 4 Functionality not Implemented in Demo  */
#define APP_BUTTON5_PIN                                 (1<<0) /* Button 5 Functionality not Implemented in Demo  */
#define APP_READ_BUTTONS1_PORT                         (BSP_SwitchStateGet(BSP_SWITCH_S1) << BSP_SWITCH_S1)
#define APP_READ_BUTTON_PORTS()                        (BSP_SwitchStateGet(BSP_SWITCH_S1) << BSP_SWITCH_S1)



///* Application LED's */
#define APP_LED_BT_READY1_ON()                          BSP_LEDOn(BSP_LED_D4)
#define APP_LED_BT_READY1_OFF()                         BSP_LEDOff(BSP_LED_D4)
#define APP_LED_BT_READY1_TOGGLE()                      BSP_LEDToggle(BSP_LED_D4)
#define APP_LED_BT_READY2_ON()                          BSP_LEDOn(BSP_LED_D5)
#define APP_LED_BT_READY2_OFF()                         BSP_LEDOff(BSP_LED_D5)
#define APP_LED_BT_READY2_TOGGLE()                      BSP_LEDToggle(BSP_LED_D5)
#define APP_LED_BT_STREAM_ON()                          BSP_LEDOn(BSP_LED_D6)
#define APP_LED_BT_STREAM_OFF()                         BSP_LEDOff(BSP_LED_D6)
#define APP_LED_BT_STREAM_TOGGLE()                      BSP_LEDToggle(BSP_LED_D6)
#define APP_LED1_ON()                                   BSP_LEDOn(BSP_LED_D3)
#define APP_LED1_OFF()                                  BSP_LEDOff(BSP_LED_D3)
#define APP_LED1_TOGGLE()                               BSP_LEDToggle(BSP_LED_D3)
#define APP_LED2_ON()                                   BSP_LEDOn(BSP_LED_D4)
#define APP_LED2_OFF()                                  BSP_LEDOff(BSP_LED_D4)
#define APP_LED2_TOGGLE()                               BSP_LEDToggle(BSP_LED_D4)
#define APP_LED3_ON()                                   BSP_LEDOn(BSP_LED_D5)
#define APP_LED3_OFF()                                  BSP_LEDOff(BSP_LED_D5)
#define APP_LED3_TOGGLE()                               BSP_LEDToggle(BSP_LED_D5)
#define APP_LED4_ON()                                   BSP_LEDOn(BSP_LED_D6)
#define APP_LED4_OFF()                                  BSP_LEDOff(BSP_LED_D6)
#define APP_LED4_TOGGLE()                               BSP_LEDToggle(BSP_LED_D6)
#define APP_LED5_ON()                                   BSP_LEDOn(BSP_LED_D7)
#define APP_LED5_OFF()                                  BSP_LEDOff(BSP_LED_D7)
#define APP_LED5_TOGGLE()                               BSP_LEDToggle(BSP_LED_D7)

/* Interrupt ISR of System Components */
#define SYS_BT_TICK_TIMER_ISR                           _TIMER_1_VECTOR
#define SYS_BT_BUTTON_REPEAT_TIMER_ISR                  _TIMER_3_VECTOR
#define SYS_BT_BUTTON_CN_ISR                            _CHANGE_NOTICE_A_VECTOR
#define SYS_BT_USART_RX_DMA_CHANNEL_ISR                 _DMA3_VECTOR
#define SYS_BT_USART_TX_DMA_CHANNEL_ISR                 _DMA0_VECTOR
#define SYS_BT_USART_RX_ISR                             _UART2_RX_VECTOR
#define SYS_BT_USART_TX_ISR                             _UART2_TX_VECTOR

    
/* Peripheral Bus Clock frequency */
#define APP_BT_USART_BAUD_CLOCK                         (uint32_t) SYS_CLK_BUS_PERIPHERAL_2
#define APP_BT_REPEAT_TIMER_CLOCK_HZ                    (uint32_t) SYS_CLK_BUS_PERIPHERAL_3
#define APP_BT_USART_WORKING_BAUD_RATE                  4000000
#define APP_BT_TICK_TIMER_MS                            10
#define APP_BT_BUTTON_REPEAT_TIMER_PRESCALE		DRV_TMR_PRESCALE_IDX1
#define APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD          (DRV_TMR_PRESCALE_IDX1 == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/2): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/2)
#define APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD	(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE == 0x7)? \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<(APP_BT_BUTTON_REPEAT_TIMER_PRESCALE+1))/128): \
                                			(APP_BT_REPEAT_TIMER_CLOCK_HZ/(1<<APP_BT_BUTTON_REPEAT_TIMER_PRESCALE)/128)
/* Volume Potentiometer Specific defines. Not needed in this configuration */
#define APP_ADC_VOLTAGEREFERENCESELECT
#define APP_ADC_SAMPLINGMODESELECT
#define APP_ADC_RESULTBUFFERMODESELECT
#define APP_ADC_SAMPLESPERINTERRUPTSELECT
#define APP_ADC_MUXAINPUTSCANENABLE
#define APP_ADC_SAMPLEACQUISITIONTIMESET
#define APP_ADC_CONVERSIONCLOCKSET
#define APP_ADC_MUXCHANNEL0INPUTNEGATIVESELECT
#define APP_ADC_INPUTSCANMASKADD
#define APP_ADC_SAMPLEAUTOSTARTENABLE
#define APP_ADC_CONVERSIONTRIGGERSOURCESELECT
#define APP_ADC_ENABLE
#define APP_ADC_RESULTGETBYINDEX               120
#define APP_VOLUMEINIT()
#define APP_VOLUMETASK()
#define APP_GFX_MENU_DRAW()
#define APP_DISPLAYTASK()                       

#define APP_VIRTUAL_LED_Y_OFFSET    /*set this value for GFX. Value should be an int*/


<#elseif CONFIG_BSP_PIC32MX_BT_SK?has_content && CONFIG_BSP_PIC32MX_BT_SK == true>



<#else>

#warning User Implementation not defined. Please set User implementation below. then delete this line before rebuilding.
#define APP_BUTTON1_PIN                                 ()/* Button 1 Functionality not Implemented in Demo  */
#define APP_BUTTON2_PIN                                 () /* Button 2 Functionality not Implemented in Demo  */
#define APP_BUTTON3_PIN                                 () /* Button 3 Functionality not Implemented in Demo  */
#define APP_BUTTON4_PIN                                 () /* Button 4 Functionality not Implemented in Demo  */
#define APP_BUTTON5_PIN                                 () /* Button 5 Functionality not Implemented in Demo  */
/* Application LEDs */
#define APP_LED1_ON()                                   BSP_LEDOn() /*Functionality not Implemented in Demo  */
#define APP_LED1_OFF()                                  BSP_LEDOff() /*Functionality not Implemented in Demo  */
#define APP_LED1_TOGGLE()                               BSP_LEDToggle() /*Functionality not Implemented in Demo  */
#define APP_LED2_ON()                                   BSP_LEDOn() /*Functionality not Implemented in Demo  */
#define APP_LED2_OFF()                                  BSP_LEDOff() /*Functionality not Implemented in Demo  */
#define APP_LED2_TOGGLE()                               BSP_LEDToggle() /*Functionality not Implemented in Demo  */
#define APP_LED3_ON()                                   BSP_LEDOn() /*Functionality not Implemented in Demo  */
#define APP_LED3_OFF()                                  BSP_LEDOff() /*Functionality not Implemented in Demo  */
#define APP_LED3_TOGGLE()                               BSP_LEDToggle() /*Functionality not Implemented in Demo  */
#define APP_LED4_ON()                                   BSP_LEDOn() /*Functionality not Implemented in Demo  */
#define APP_LED4_OFF()                                  BSP_LEDOff() /*Functionality not Implemented in Demo  */
#define APP_LED4_TOGGLE()                               BSP_LEDToggle() /*Functionality not Implemented in Demo  */
#define APP_LED5_ON()                                   BSP_LEDOn() /*Functionality not Implemented in Demo  */
#define APP_LED5_OFF()                                  BSP_LEDOff() /*Functionality not Implemented in Demo  */
#define APP_LED5_TOGGLE()                               BSP_LEDToggle() /*Functionality not Implemented in Demo  */


#define APP_VIRTUAL_LED_Y_OFFSET    /*set this value for GFX. Value should be an int*/

</#if> <#--end of App Demo code-->
</#if><#--end of Use demo settings-->
</#if> <#--has content-->
</#if> <#--end of file-->
