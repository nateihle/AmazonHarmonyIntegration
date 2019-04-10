/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    sys_log_define.h

  Summary:
    Defines messaging services for the application event logging.

  Description:
    Defines messaging services for the application event logging.

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

#ifndef __SYS_LOG_DEFINE_H_INCLUDED__
#define __SYS_LOG_DEFINE_H_INCLUDED__

#if defined( ENABLE_SYS_LOG )

  #include "sys_log.h"

  #if defined(SYS_LOG_TAG)
    static const char __SYS_LOG_TAG__[] = SYS_LOG_TAG;
  #else
    #define __SYS_LOG_TAG__ 0
  #endif

  #define SYS_LOG(msg)                SYS_LOG_EntryWrite      (__SYS_LOG_TAG__,msg)
  #define SYS_LOG0(msg)               SYS_LOG_EntryWrite      (__SYS_LOG_TAG__,msg)
  #define SYS_LOG1(msg,arg)           SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg)
  #define SYS_LOG2(msg,arg,arg2)      SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg,arg2)
  #define SYS_LOG3(msg,arg,arg2,arg3) SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg,arg2,arg3)
  #define SYS_LOG4(msg,arg,arg2,arg3,arg4) \
                                      SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg,arg2,arg3,arg4)
  #define SYS_LOG5(msg,arg,arg2,arg3,arg4,arg5) \
                                      SYS_LOG_FormatEntryWrite(__SYS_LOG_TAG__,msg,arg,arg2,arg3,arg4,arg5)
#else

  #define SYS_LOG(msg)                ((void)0)
  #define SYS_LOG0(msg)               ((void)0)
  #define SYS_LOG1(msg,arg)           ((void)0)
  #define SYS_LOG2(msg,arg,arg2)      ((void)0)
  #define SYS_LOG3(msg,arg,arg2,arg3) ((void)0)
  #define SYS_LOG4(msg,arg,arg2,arg3,arg4) ((void)0)
  #define SYS_LOG5(msg,arg,arg2,arg3,arg4,arg5) ((void)0)

#endif//defined( ENABLE_SYS_LOG )


#endif // __SYS_LOG_DEFINE_H_INCLUDED__
