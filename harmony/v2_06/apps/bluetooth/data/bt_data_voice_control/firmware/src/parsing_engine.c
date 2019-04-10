/*******************************************************************************
 Parsing Engine

  Company:
    Microchip Technology Inc.

  File Name:
    parsing_engine.c

  Summary:
    Contains the functional implementation of data parsing functions

  Description:
    This file contains the functional implementation of data parsing functions
*******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app.h"

void TakeAction(char rcvdStr[])
{
//    int Temp =0;
    int Command, Size, Red, Green, Blue;

    if((strcmp(rcvdStr,"DISPLAY_ALL") == 0) || (strcmp(rcvdStr,"DISPLAY_ALL\r\n")== 0) ||
        (strcmp(rcvdStr,"DISPLAY_ALL\n")== 0) ||(strcmp(rcvdStr,"DAI")== 0) ||
        (strcmp(rcvdStr,"DAI\r\n")== 0) || (strcmp(rcvdStr,"DAI\n")== 0))
     {
         BT_DISPLAY_STATS.DISPLAY_ALL = 1;
     }
    if((strcmp(rcvdStr,"DISPLAY_ALL_STOP") == 0) || (strcmp(rcvdStr,"DISPLAY_ALL_STOP\r\n")== 0) || 
       (strcmp(rcvdStr,"DISPLAY_ALL_STOP\n")== 0) || (strcmp(rcvdStr,"DAS")== 0) ||
       (strcmp(rcvdStr,"DAS\r\n")== 0) || (strcmp(rcvdStr,"DAS\n")== 0))
    {
        BT_DISPLAY_STATS.DISPLAY_ALL = 0;
    }
    if((strcmp(rcvdStr,"led1on") == 0) || (strcmp(rcvdStr,"led1on\r\n")== 0) || (strcmp(rcvdStr,"led1on\n")== 0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.KNOWN_COMMAND = 1;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED1 = 1;
    }
    else if((strcmp(rcvdStr,"led1off") == 0) || (strcmp(rcvdStr,"led1off\r\n")== 0) || (strcmp(rcvdStr,"led1off\n")== 0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.KNOWN_COMMAND = 1;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED1 = 0;
    }
    else if((strcmp(rcvdStr,"led1toggle") == 0) || (strcmp(rcvdStr,"led1toggle\r\n")== 0) || (strcmp(rcvdStr,"led1toggle\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.KNOWN_COMMAND = 1;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        if (BT_DISPLAY_STATS.VLED1 == 1)
        {
            BT_DISPLAY_STATS.VLED1 = 0;
        }
        else
        {
            BT_DISPLAY_STATS.VLED1 = 1;
        }
    }

    if((strcmp(rcvdStr,"led2on") == 0 )|| (strcmp(rcvdStr,"led2on\r\n")== 0 )|| (strcmp(rcvdStr,"led2on\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
    }
    else if((strcmp(rcvdStr,"led2off" ) == 0)|| (strcmp(rcvdStr,"led2off\r\n")== 0) || (strcmp(rcvdStr,"led2off\n")== 0) )
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
    }
    else if((strcmp(rcvdStr,"led2toggle") == 0)|| (strcmp(rcvdStr,"led2toggle\r\n")== 0) || (strcmp(rcvdStr,"led2toggle\n")== 0) )
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        if (BT_DISPLAY_STATS.VLED2 == 1)
        {
            BT_DISPLAY_STATS.VLED2 = 0;
        }
        else
        {
            BT_DISPLAY_STATS.VLED2 = 1;
        }
    }

    if((strcmp(rcvdStr,"led3on")  == 0) || (strcmp(rcvdStr,"led3on\r\n")== 0) || (strcmp(rcvdStr,"led3on\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
//        BSP_LEDOn(BSP_LED_7);
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
    }
    else if((strcmp(rcvdStr,"led3off")  == 0) || (strcmp(rcvdStr,"led3off\r\n")== 0 )|| (strcmp(rcvdStr,"led3off\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
//         BSP_LEDOff(BSP_LED_7);
         BT_DISPLAY_STATS.DisplayUpdate = 1;
         BT_DISPLAY_STATS.VLED3 = 0;
    }
    else if((strcmp(rcvdStr,"led3toggle")== 0) || (strcmp(rcvdStr,"led3toggle\r\n")== 0) || (strcmp(rcvdStr,"led3toggle\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
//        BSP_LEDToggle(BSP_LED_7);
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        if (BT_DISPLAY_STATS.VLED3 == 1)
        {
            BT_DISPLAY_STATS.VLED3 = 0;
        }
        else
        {
            BT_DISPLAY_STATS.VLED3 = 1;
        }
    }

    if((strcmp(rcvdStr,"led4on") == 0) || (strcmp(rcvdStr,"led4on\r\n")== 0 )|| (strcmp(rcvdStr,"led4on\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
    }
    else if((strcmp(rcvdStr,"led4off") == 0)|| (strcmp(rcvdStr,"led4off\r\n")== 0) || (strcmp(rcvdStr,"led4off\n")== 0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
    }
    else if((strcmp(rcvdStr,"led4toggle") == 0)|| (strcmp(rcvdStr,"led4toggle\r\n")== 0 )|| (strcmp(rcvdStr,"led4toggle\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        if (BT_DISPLAY_STATS.VLED4 == 1)
        {
            BT_DISPLAY_STATS.VLED4 = 0;
        }
        else
        {
            BT_DISPLAY_STATS.VLED4 = 1;
        }
    }

    if((strcmp(rcvdStr,"led5on") == 0) || (strcmp(rcvdStr,"led5on\r\n")== 0) || (strcmp(rcvdStr,"led5on\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if((strcmp(rcvdStr,"led5off") == 0) || (strcmp(rcvdStr,"led5off\r\n")== 0) || (strcmp(rcvdStr,"led5off\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if((strcmp(rcvdStr,"led5toggle" )== 0) || (strcmp(rcvdStr,"led5toggle\r\n")== 0) || (strcmp(rcvdStr,"led5toggle\n")== 0 ))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        if (BT_DISPLAY_STATS.VLED5 == 1)
        {
            BT_DISPLAY_STATS.VLED5 = 0;
        }
        else
        {
            BT_DISPLAY_STATS.VLED5 = 1;
        }
    }

    if((strcmp(rcvdStr,"L0" ) ==0)|| (strcmp(rcvdStr,"L0\r" ) ==0) || (strcmp(rcvdStr,"L0\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if((strcmp(rcvdStr,"L1" ) ==0)|| (strcmp(rcvdStr,"L1\r" ) ==0) || (strcmp(rcvdStr,"L1\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L2" ) ==0) || (strcmp(rcvdStr,"L2\r" ) ==0) || (strcmp(rcvdStr,"L2\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L3" ) ==0) || (strcmp(rcvdStr,"L3\r" ) ==0) || (strcmp(rcvdStr,"L3\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L4" ) ==0)|| (strcmp(rcvdStr,"L4\r" ) ==0) || (strcmp(rcvdStr,"L4\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L5" ) ==0)|| (strcmp(rcvdStr,"L5\r" ) ==0) || (strcmp(rcvdStr,"L5\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L6" ) ==0)|| (strcmp(rcvdStr,"L6\r" ) ==0) || (strcmp(rcvdStr,"L6\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L7" ) ==0)|| (strcmp(rcvdStr,"L7\r" ) ==0) || (strcmp(rcvdStr,"L7\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L8" ) ==0)|| (strcmp(rcvdStr,"L8\r" ) ==0) || (strcmp(rcvdStr,"L8\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L9" ) ==0)|| (strcmp(rcvdStr,"L9\r" ) ==0) || (strcmp(rcvdStr,"L9\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L10" ) ==0)|| (strcmp(rcvdStr,"L10\r" ) ==0) || (strcmp(rcvdStr,"L10\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L11" ) ==0)|| (strcmp(rcvdStr,"L11\r" ) ==0) || (strcmp(rcvdStr,"L11\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L12" ) ==0)|| (strcmp(rcvdStr,"L12\r" ) ==0) || (strcmp(rcvdStr,"L12\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L13" ) ==0)|| (strcmp(rcvdStr,"L13\r" ) ==0) || (strcmp(rcvdStr,"L13\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L14" ) ==0)|| (strcmp(rcvdStr,"L14\r" ) ==0) || (strcmp(rcvdStr,"L14\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L15" ) ==0)|| (strcmp(rcvdStr,"L15\r" ) ==0) || (strcmp(rcvdStr,"L15\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 0;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L16" ) ==0)|| (strcmp(rcvdStr,"L16\r" ) ==0) || (strcmp(rcvdStr,"L16\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L17" ) ==0)|| (strcmp(rcvdStr,"L17\r" ) ==0) || (strcmp(rcvdStr,"L17\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L18" ) ==0)|| (strcmp(rcvdStr,"L18\r" ) ==0) || (strcmp(rcvdStr,"L18\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L19" ) ==0)|| (strcmp(rcvdStr,"L19\r" ) ==0) || (strcmp(rcvdStr,"L19\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L20" ) ==0)|| (strcmp(rcvdStr,"L20\r" ) ==0) || (strcmp(rcvdStr,"L20\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L21" ) ==0)|| (strcmp(rcvdStr,"L21\r" ) ==0) || (strcmp(rcvdStr,"L21\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L22" ) ==0)|| (strcmp(rcvdStr,"L22\r" ) ==0) || (strcmp(rcvdStr,"L22\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L23" ) ==0)|| (strcmp(rcvdStr,"L23\r" ) ==0) || (strcmp(rcvdStr,"L23\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 0;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L24" ) ==0)|| (strcmp(rcvdStr,"L24\r" ) ==0) || (strcmp(rcvdStr,"L24\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L25" ) ==0)|| (strcmp(rcvdStr,"L25\r" ) ==0) || (strcmp(rcvdStr,"L25\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L26" ) ==0)|| (strcmp(rcvdStr,"L26\r" ) ==0) || (strcmp(rcvdStr,"L26\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L27" ) ==0)|| (strcmp(rcvdStr,"L27\r" ) ==0) || (strcmp(rcvdStr,"L27\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 0;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L28" ) ==0)|| (strcmp(rcvdStr,"L28\r" ) ==0) || (strcmp(rcvdStr,"L28\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L29" ) ==0)|| (strcmp(rcvdStr,"L29\r" ) ==0) || (strcmp(rcvdStr,"L29\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 0;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    else if ((strcmp(rcvdStr,"L30" ) ==0)|| (strcmp(rcvdStr,"L30\r" ) ==0) || (strcmp(rcvdStr,"L30\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 0;
    }
    else if ((strcmp(rcvdStr,"L31" ) ==0)|| (strcmp(rcvdStr,"L31\r" ) ==0) || (strcmp(rcvdStr,"L31\r\n" ) ==0))
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.VLED1 = 1;
        BT_DISPLAY_STATS.VLED2 = 1;
        BT_DISPLAY_STATS.VLED3 = 1;
        BT_DISPLAY_STATS.VLED4 = 1;
        BT_DISPLAY_STATS.VLED5 = 1;
    }
    sscanf(rcvdStr, "%d,%d,%d,%d,%d", &Command, &Size, &Red, &Green, &Blue);
    if(Command == RGB_LED_COLOR)
    {
        BT_DISPLAY_STATS.PROCESS_TEXT = 0;
        BT_DISPLAY_STATS.DisplayUpdate = 1;
        BT_DISPLAY_STATS.VLED_Update = 1;
        BT_DISPLAY_STATS.VLED_R = Red;
        BT_DISPLAY_STATS.VLED_G = Green;
        BT_DISPLAY_STATS.VLED_B = Blue;
    }

}// end of function
