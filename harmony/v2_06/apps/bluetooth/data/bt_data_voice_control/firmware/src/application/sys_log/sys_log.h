/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    sys_log.h

  Summary:
    Provides messaging services for the application event logging.

  Description:
    Provides messaging services for the application event logging.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef __SYS_LOG_H_INCLUDED__
#define __SYS_LOG_H_INCLUDED__

// *****************************************************************************
/* Function:
    void SYS_LOG_Init(void)

  Summary:
    Initialize System Logging.

  Description:
    This function initializes the UART peripheral and System Log message queue.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).  This function must be
    called before any other System Logging functions are called.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
#if defined( ENABLE_SYS_LOG )
  #include "sys_log.h"
  #define SYS_LOG_TAG "app"
#endif
#include "sys_log_define.h"

void APP_Initialize (void)
{
    bluetoothInit(); // Bluetooth initialization

    buttonsInit();   // Buttons initialization

    audioInit();     // Audio intialization

    volumeInit();    // Audio volume initialization

  #if defined( ENABLE_SYS_LOG )
    SYS_LOG_Init();  // System Log initialization
    SYS_LOG_LongEntryWrite(SYS_LOG_TAG,"----------------------------------------");
    SYS_LOG_EntryWrite    (SYS_LOG_TAG,"- Starting:");
    SYS_LOG_LongEntryWrite(SYS_LOG_TAG,"----------------------------------------");
  #endif
}
  </code>

  Remarks:
  The following messages appear from this code:
  <sp>
      0.01481 ->app:----------------------------------------
      0.01015 ->app:- Starting:
      0.01742 ->app:----------------------------------------
  </sp>
  The time shown is the elapsed time, in milleseconds, from the previous log
  entry.  The first line indicates that 0.01481 milleseconds have elapsed
  since the call so SYS_LOG_Init().

  The symbol "->" indicates that no log messages have been skipped
  (failed to enter the queue.  The number of skipped log messages is indicated
  by "1>" to "F>" for 1 to 15 skipped messages.  "*>" indicate more than 15 log
  messages have been skipped (failed to be queued).
*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_Init(void);
#else
  #define SYS_LOG_Init()             ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void    SYS_LOG_Deinit(void)

  Summary:
    Deinitialize System Log Services.

  Description:
    Deinitialize System Log Services.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).   SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    None.
*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_Deinit(void);
#else
  #define SYS_LOG_Deinit()           ((void)0)
#endif//defined( ENABLE_SYS_LOG )

// *****************************************************************************
/* Function:
    void SYS_LOG_Start(void)

  Summary:
    Starts System Logging again by re-enabling the UART.

  Description:
    Starts System Logging again by re-enabling the UART.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).   SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    A call to SYS_LOG_START() is not needed after calling SYS_LOG_Init(), since
    SYS_LOG_INIT() starts the UART.

*/

#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_Start(void);
#else
  #define SYS_LOG_Start()            ((void)0)
#endif//defined( ENABLE_SYS_LOG )



// *****************************************************************************
/* Function:
    void SYS_LOG_Stop(void)

  Summary:
    Stops System Logging again by disabling the UART.

  Description:
    Stops System Logging again by disabling the UART.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).   SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    None.
*/

#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_Stop(void);
#else
  #define SYS_LOG_Stop()            ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_Task()

  Summary:
    Perform System Logging maintenance tasks.

  Description:
    Perform System Logging maintenance tasks as part of application tasking.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).  SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    None.

  Returns:
    None.

  Example:
  <code>
void APP_Tasks (void )
{
    switch(appData.state)
    {
        .
        .
        .
        case APP_STATE_GFX_TASKS:
        {
            APP_DISPLAYTASK();
          #if defined( ENABLE_SYS_LOG )
            appData.state = APP_STATE_LOG_TASKS;
          #else
            appData.state = APP_STATE_BT_TASK_RUN;
          #endif
        }
        break;

      #if defined( ENABLE_SYS_LOG )
        case APP_STATE_LOG_TASKS:
        {
            SYS_LOG_Task();
            appData.state = APP_STATE_BT_TASK_RUN;
        }
      #endif

      // Run the Bluetooth Task
      case APP_STATE_BT_TASK_RUN:
      {
          bluetoothTask();
          appData.state = APP_STATE_AUDIO_TASK_RUN;
      }

      break;
        default:
        {
        }
        break;
      }// end switch(appData.state)
      .
      .
      .
  </code>

  Remarks:
    This function checks if there are any system log messages in the queue
    and if messages are not being transmitted by the UART it will start the
    UART transmit of log messages.
*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_Task();
#else
  #define SYS_LOG_Task()        ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_EntryWrite(const char* tag, const char* message)

  Summary:

  Description:

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).   SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    tag - optional tag to identify file or service that generated the log message.
    message - character string message for log entry.

  Returns:
    None.

  Example:
  <code>
#if defined( ENABLE_SYS_LOG )
  #include "sys_log.h"
  #define SYS_LOG_TAG "buttons"
#endif
#include "sys_log_define.h"

static void clearRepeatTimer(void)
{
    DRV_TMR_Alarm32BitDeregister(appData.repeatTmrHandle); // Stops timer, too
    SYS_LOG("clearRepeatTimer");
  // OR:
  //SYS_LOG_EntryWrite(__SYS_LOG_TAG__,"clearRepeatTimer");
}
  </code>
  SYS_LOG is defined in sys_log_define.h as
  <code>
    #define SYS_LOG(msg)  SYS_LOG_EntryWrite(__SYS_LOG_TAG__,msg)
  </code>

  Remarks:
    None.
*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_EntryWrite(const char* tag, const char* message);
#else
  #define SYS_LOG_EntryWrite()       ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_FormatEntryWrite(const char* tag, const char* format, ...)

  Summary:

  Description:

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).   SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    tag - optional tag to identify file or service that generated the log message.
    message - character string message for log entry, including formatting
              characters for the additional parameters.  (See Example below.)

  Returns:
    None.

  Example:
  <code>
#if defined( ENABLE_SYS_LOG )
  #include "sys_log.h"
  #define SYS_LOG_TAG "buttons"
#endif
#include "sys_log_define.h"

void bttask_pal_handleButtonRepeatSignal(void)
{
    if (mRepeatButton)
    {
        mRepeatCount++;
        SYS_LOG2("ButtonRepeatSignal: b: %d, #: %d",mRepeatButton,mRepeatCount);
        setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD);
        btapp_OnButtonEvent(mRepeatButton,true,mRepeatCount);
    }
    else
    {
        SYS_LOG1("ButtonRepeatSignal: b: %d",mRepeatButton);
    }
}//bttask_pal_handleButtonRepeatSignal
  </code>
  Where SYS_LOG1 and SYS_LOG2 are defined in sys_log_define.h as
  <code>
    #define SYS_LOG1(msg,arg)       SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg)
    #define SYS_LOG2(msg,arg,arg2)  SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg,arg2)
  </code>
  Remarks:
    None.
*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_FormatEntryWrite(const char* tag, const char* format, ...);
#else
  #define SYS_LOG_FormatEntryWrite() ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_LongEntryWrite(const char* tag, const char* msg)

  Summary:

  Description:

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a #define ((void)0).   SYS_LOG_Init() has been
    called to initialize UART and message queue.

  Parameters:
    tag - optional tag to identify file or service that generated the log message.
    message - character string message for log entry.  The messasge is cut and
    broadcast as additional lines if needed.

  Returns:
    None.

  Example:
    See code example under SYS_LOG_Init.

  Remarks:
    None.
*/
#if defined( ENABLE_SYS_LOG )
  void    SYS_LOG_LongEntryWrite(const char* tag, const char* msg);
#else
  #define SYS_LOG_LongEntryWrite()   ((void)0)
#endif//defined( ENABLE_SYS_LOG )


// *****************************************************************************
/* Function:
    void SYS_LOG_LibEntryWrite(const char* msg)

  Summary:
    Provides System log entry write services for precompiled libraries.

  Description:
    Provides System log entry write services for precompiled libraries.

  Precondition:
    ENABLE_SYS_LOG must be defined by the active MPLAB.X configuration, otherwise
    this function is replaced by a empty function in sys_log.c. SYS_LOG_Init()
    has been called to initialize UART and message queue.

  Parameters:
    msg - log message

  Returns:
    None.

  Example:
  <code>
  </code>

  Remarks:
    None.
*/
// For use in pre-compiled libraries.
void SYS_LOG_LibEntryWrite(const char* msg);

#endif // __SYS_LOG_H_INCLUDED__
