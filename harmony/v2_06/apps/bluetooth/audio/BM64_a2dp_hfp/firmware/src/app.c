/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "driver/bluetooth/bm64/drv_bm64.h"     // only reference to BM64 (others use generic BT))
#if defined( ENABLE_SYS_LOG )
  #include "sys_log/sys_log.h"
  #include "sys_log/sys_log_define.h"
#endif

#include "audio.h"
extern AUDIO_DATA audioData;

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize()

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize()
{
    APP_LED1_ON();      // all LED's on during initialization    
    APP_LED2_ON();
    APP_LED3_ON();
    APP_LED4_ON();
    APP_LED5_ON();
        
    //Place the App state machine in its initial state.
    appData.state = APP_STATE_INIT;
    
    appData.volume = 0x40;
    
    appData.buttonState = BUTTON_STATE_IDLE;
    appData.buttonDelay = 0;        // used for button debounce
    appData.volumeDelay = 0;         // used for volume widget
    appData.linkStatus = -1;        // force mismatch 
    
    audioInitialize();
    
    APP_LED1_OFF();      // all LED's off
    APP_LED2_OFF();
    APP_LED3_OFF();
    APP_LED4_OFF();
    APP_LED5_OFF();    

 #if defined( ENABLE_SYS_LOG )    
    SYS_LOG_Init();
    SYS_LOG("----------------------------------------");
    SYS_LOG("- Starting:");
    SYS_LOG("----------------------------------------");
  #endif

}  //End APP_Initialize()

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
SYS_TMR_HANDLE tmrHandle;
uint16_t lastappData_state = 0xffff;

void APP_Tasks()
{    
#if defined( ENABLE_SYS_LOG )
    if (appData.state != lastappData_state)    
    {
        SYS_LOG1("APP_Tasks: appData.state=%d",appData.state);
        lastappData_state = appData.state;
    }
#endif
  
    audioTasks(); 
   
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            laWidget_SetVisible((laWidget*)GFX_CONNECTED, LA_FALSE);
            laWidget_SetVisible((laWidget*)GFX_PAIRED, LA_FALSE);
            laWidget_SetVisible((laWidget*)GFX_NOPAIR_NOCONNECTION, LA_TRUE);
            
            tmrHandle = SYS_TMR_CallbackPeriodic (1, 0, &Timer_1ms);
            if (tmrHandle == DRV_HANDLE_INVALID)
            {
#if defined( ENABLE_SYS_LOG )    
                SYS_LOG("----------------------------------------");
                SYS_LOG("- Invalid tmrHandle in app.c");
                SYS_LOG("----------------------------------------");
#endif 
                // sticks in this state if handle invalid
            }
            else
            {           
                appData.state = APP_STATE_WAIT_INIT;
            }       
            break;
        }            

        case APP_STATE_WAIT_INIT:
        {
            if (DRV_BT_STATUS_READY == DRV_BT_GetPowerStatus())
            {           
                appData.state=APP_STATE_IDLE;
                audioStart();
            }
            break;
        }
        
        // Initialized 
        case APP_STATE_IDLE:
        {
            buttonTasks();
            
            break;
        }        
    }
 
    if ((audioData.bt.handle != (DRV_HANDLE)NULL) && 
       (audioData.bt.handle != DRV_HANDLE_INVALID))
    {     
        DRV_BT_LINKSTATUS newLinkStatus = DRV_BT_GetLinkStatus(audioData.bt.handle);          
        // update connection status
        if (newLinkStatus & (DRV_BT_HFP_LINK_STATUS|DRV_BT_SCO_LINK_STATUS))
        {
            APP_LED4_ON();
        }
        else
        {
            APP_LED4_OFF();
        }
        if (newLinkStatus & DRV_BT_A2DP_LINK_STATUS)
        {
            APP_LED5_ON();
        }
        else
        {
            APP_LED5_OFF();
        }
        
        if (newLinkStatus != appData.linkStatus)
        {  
            if ((DRV_BT_GetPowerStatus() == DRV_BT_STATUS_READY))
            {
                if (newLinkStatus == DRV_BT_NO_LINK_STATUS)
                {    
                    laString tempStr;                                      
                    char buf [DRV_BT_MAXBDNAMESIZE+1];
                
                    laWidget_SetVisible((laWidget*)GFX_VOLUME_VALUE, LA_FALSE);                   
                    laWidget_SetVisible((laWidget*)GFX_BTNAME, LA_TRUE);
                    laWidget_SetVisible((laWidget*)GFX_BTADDRESS, LA_TRUE);

                    DRV_BT_GetBDName(audioData.bt.handle, buf, DRV_BT_MAXBDNAMESIZE+1);
                    tempStr = laString_CreateFromCharBuffer(buf, &LiberationSans12);
                    laLabelWidget_SetText(GFX_BTNAME_VALUE, tempStr);
                    laString_Destroy(&tempStr);

                    DRV_BT_GetBDAddress(audioData.bt.handle, buf);
                    tempStr = laString_CreateFromCharBuffer(buf, &LiberationSans12);
                    laLabelWidget_SetText(GFX_BTADDRESS_VALUE, tempStr);
                    laString_Destroy(&tempStr);
                    
                    laWidget_SetVisible((laWidget*)GFX_CONNECTED, LA_FALSE);
                    laWidget_SetVisible((laWidget*)GFX_PAIRED, LA_FALSE);
                    laWidget_SetVisible((laWidget*)GFX_NOPAIR_NOCONNECTION, LA_TRUE);                    
                }
                else
                {
                    laString tempStr;
                    
                    laWidget_SetVisible((laWidget*)GFX_BTNAME, LA_FALSE);
                    laWidget_SetVisible((laWidget*)GFX_BTADDRESS, LA_FALSE);

                    laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_FALSE);
                    laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_FALSE);                    

                    tempStr = laString_CreateFromCharBuffer(" ", &LiberationSans12);
                    laLabelWidget_SetText(GFX_BTNAME_VALUE, tempStr);
                    laLabelWidget_SetText(GFX_BTADDRESS_VALUE, tempStr);
                    laString_Destroy(&tempStr);
                    
                    laWidget_SetVisible((laWidget*)GFX_CONNECTED, LA_TRUE);
                    laWidget_SetVisible((laWidget*)GFX_PAIRED, LA_FALSE);
                    laWidget_SetVisible((laWidget*)GFX_NOPAIR_NOCONNECTION, LA_FALSE);                    
                }

                appData.linkStatus = newLinkStatus;
            }
        }
    }
}
     
void buttonTasks( )
{   
    //BUTTON PROCESSING
    /* Check the buttons' current state. */      
    switch ( appData.buttonState )
    {
        case BUTTON_STATE_IDLE:
        {
            if ( (appData.buttonDelay==0)&&
                 ((BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_4)==BSP_SWITCH_STATE_PRESSED)||
                  (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED)) )
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;       
                appData.buttonState=BUTTON_STATE_PRESSED;               
            }
            
            break;
        }
    
        case BUTTON_STATE_PRESSED:
        { 
            if (appData.buttonDelay>0)
            {
                break;      // still debouncing
            }
            
            if(BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED)       // SW1 volume up/pairing
            {                
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_UPPAIRING_PRESSED;                  
            }
            else if(BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED)        // SW2 volume down
            {
                appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                appData.buttonState=BUTTON_STATE_DOWNCONNECT_PRESSED;                 
            }                        
            
            else if (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED)        // SW3 -- next song)
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }
                    laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);                  
                    laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_FALSE);
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;
                }
                else
                {
                    appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                    appData.buttonState=BUTTON_STATE_NEXTFF_PRESSED; 
                    laWidget_SetVisible((laWidget*)GFX_NEXT, LA_TRUE);                 
                }
            }
            
            else if (BSP_SwitchStateGet(BSP_SWITCH_4)==BSP_SWITCH_STATE_PRESSED)       // SW4 -- play/pause
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }
                    laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);                  
                    laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_FALSE);
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;
                }
                else
                {
                    appData.buttonDelay=3*LONG_BUTTON_PRESS;          // 3 sec is long press
                    appData.buttonState=BUTTON_STATE_PLAYPAUSE_PRESSED;                   
                }
            }            
            else if (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED)       // SW5 -- previous song
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }
                    laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);                  
                    laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_FALSE);
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;
                }
                else
                {               
                    appData.buttonDelay=LONG_BUTTON_PRESS;          // 1 sec is long press
                    appData.buttonState=BUTTON_STATE_PREVREWIND_PRESSED;
                    laWidget_SetVisible((laWidget*)GFX_PREVIOUS, LA_TRUE);
                }
            }             
            else
            {
                appData.buttonState=BUTTON_STATE_IDLE;                 // button not pressed anymore, assume noise                
            }

            break;
        }
        
        case BUTTON_STATE_UPPAIRING_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_1)!=BSP_SWITCH_STATE_PRESSED))     // SW1 pressed and released < 1 sec -- Next
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {
                    DRV_BT_volumeUp(audioData.bt.handle);
                    //DRV_BT_volumeUpCurrentMode(audioData.bt.handle);
                    //DRV_BT_setVolCurrentMode();                
                }
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_1)==BSP_SWITCH_STATE_PRESSED))       // SW1 still pressed after 1 sec -- fast forward
            {
                DRV_BT_EnterBTPairingMode(audioData.bt.handle);               
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }           
            break;
        } 

        case BUTTON_STATE_DOWNCONNECT_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_2)!=BSP_SWITCH_STATE_PRESSED))     // SW2 pressed and released < 1 sec -- Next
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {                
                    DRV_BT_volumeDown(audioData.bt.handle);
                    //DRV_BT_volumeDownCurrentMode(audioData.bt.handle);
                    //DRV_BT_setVolCurrentMode();
                }
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_2)==BSP_SWITCH_STATE_PRESSED))       // SW2 still pressed after 1 sec -- fast forward
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle)!=0)
                {                 
                    DRV_BT_DisconnectAllLinks(audioData.bt.handle);
                }
                else
                {                 
                    DRV_BT_LinkLastDevice(audioData.bt.handle);
                }                
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        } 
        
        case BUTTON_STATE_NEXTFF_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_3)!=BSP_SWITCH_STATE_PRESSED))     // SW3 pressed and released < 1 sec -- Next
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {                 
                    DRV_BT_PlayNextSong(audioData.bt.handle);
                }
                laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);                  
                laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_FALSE);
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_3)==BSP_SWITCH_STATE_PRESSED))       // SW3 still pressed after 1 sec -- fast forward
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)                
                {                 
                    DRV_BT_FastForward(audioData.bt.handle);
                }
                laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);  
                laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_TRUE);                
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }                      
        
        case BUTTON_STATE_PLAYPAUSE_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_4)!=BSP_SWITCH_STATE_PRESSED))     // SW4 pressed and released < 1 sec -- play/pause
            {
                DRV_BT_PLAYINGSTATUS playingStatus = DRV_BT_GetPlayingStatus(audioData.bt.handle);
                if ((playingStatus==DRV_BT_PLAYING_FF)||(playingStatus==DRV_BT_PLAYING_FR))
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                 
                        DRV_BT_CancelForwardOrRewind(audioData.bt.handle);
                    }                                       
                }
                else
                {
                    if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                    {                                                             
                        if (playingStatus==DRV_BT_PLAYING_PLAYING)                       
                        {
                            laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_FALSE);     
                            laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_TRUE);
                        }
                        else
                        {
                            laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_TRUE);     
                            laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_FALSE);
                        }                           
                        DRV_BT_PlayPause(audioData.bt.handle);                            
                    }
                }
                laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);                  
                laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_FALSE);
                laWidget_SetVisible((laWidget*)GFX_PREVIOUS, LA_FALSE);                  
                laWidget_SetVisible((laWidget*)GFX_REWIND, LA_FALSE);                
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_4)==BSP_SWITCH_STATE_PRESSED))       // SW4 still pressed after 1 sec -- disconnect
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {                 
                    DRV_BT_ForgetAllLinks(audioData.bt.handle);               
                    appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE; 
                }                                                    
           
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }

        case BUTTON_STATE_PREVREWIND_PRESSED:
        {
            if ((appData.buttonDelay>0)&&
                (BSP_SwitchStateGet(BSP_SWITCH_5)!=BSP_SWITCH_STATE_PRESSED))     // SW3 pressed and released < 1 sec -- previous
            {
                if (DRV_BT_GetLinkStatus(audioData.bt.handle) & DRV_BT_AVRCP_LINK_STATUS)
                {                 
                    DRV_BT_PlayPreviousSong(audioData.bt.handle);
                }
                laWidget_SetVisible((laWidget*)GFX_PREVIOUS, LA_FALSE);                  
                laWidget_SetVisible((laWidget*)GFX_REWIND, LA_FALSE);                 
                appData.buttonDelay=BUTTON_DEBOUNCE;                
                appData.buttonState=BUTTON_STATE_IDLE;              
            }
            else if ((appData.buttonDelay==0)&&
                     (BSP_SwitchStateGet(BSP_SWITCH_5)==BSP_SWITCH_STATE_PRESSED))       // SW3 still pressed after 1 sec -- fast forward
            {
                DRV_BT_Rewind(audioData.bt.handle);
                laWidget_SetVisible((laWidget*)GFX_PREVIOUS, LA_FALSE);
                laWidget_SetVisible((laWidget*)GFX_REWIND, LA_TRUE);                
                appData.buttonState=BUTTON_STATE_WAIT_FOR_RELEASE;                
            }
            
            break;
        }  

        case BUTTON_STATE_WAIT_FOR_RELEASE:
        {
            if ((BSP_SwitchStateGet(BSP_SWITCH_1)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_2)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_3)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_4)!=BSP_SWITCH_STATE_PRESSED)&&
                (BSP_SwitchStateGet(BSP_SWITCH_5)!=BSP_SWITCH_STATE_PRESSED))
            {
                appData.buttonDelay=BUTTON_DEBOUNCE;
                appData.buttonState=BUTTON_STATE_IDLE;
            }
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
    
}

void Timer_1ms(uintptr_t context, uint32_t currTick)
{
    //Button Processing
    if(appData.buttonDelay)
        --appData.buttonDelay;
    
    if(appData.volumeDelay)
    {
        --appData.volumeDelay;    
        if (appData.volumeDelay==0)
        {
            laWidget_SetVisible((laWidget*)GFX_VOLUME_VALUE, LA_FALSE);                          
        }
    }
}

uint32_t coretime_i = 0;
uint32_t coretime_f = 0;
uint32_t coretime [MAX_CORETIME_I];
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void)
{
    uint32_t timer;
    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));
    return (timer);
}

void APP_SaveCoreTime(void)
{
    // to take measurement
    if (coretime_f && (coretime_i < (MAX_CORETIME_I-1)))
    {
        coretime [coretime_i++] = APP_ReadCoreTimer();
    }
}

void APP_StartCoreTime()
{
    coretime_f = 1;
}
/*******************************************************************************
 End of File
 */
