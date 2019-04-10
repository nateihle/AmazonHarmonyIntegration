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
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

/* TODO:  Add any necessary callback functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{

    DRV_TMR0_Start();

    /* Put the application into its initial state */
    appData.state = APP_STATE_LCD_INIT;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    switch(appData.state)
    {
        case APP_STATE_LCD_INIT:
        {
            /* Execute LCD initialization sequence */
            initializeLCD();
            appData.state = APP_STATE_LCD_COMMAND1;
            break;
        }

        case APP_STATE_LCD_COMMAND1:
        {
            /* Execute LCD initialization sequence */
            writeToLCD(CMDREG, 0x0F); /* Turn on blinking cursor */
            appData.state = APP_STATE_LCD_COMMAND2;
            break;
        }

        case APP_STATE_LCD_COMMAND2:
        {
            writeString((unsigned char *)"Hello!"); /* Write string to the LCD */
            appData.state = APP_STATE_LCD_COMMAND3;
            break;
        }

        case APP_STATE_LCD_COMMAND3:
        {
            newLine(); /* Set cursor to line two */
            appData.state = APP_STATE_LCD_COMMAND4;
            break;
        }

        case APP_STATE_LCD_COMMAND4:
        {
            writeString((unsigned char *)"Testing..."); /* Print another string */
            appData.state = APP_STATE_IDLE;
            break;
        }

        case APP_STATE_IDLE:
            /* Stuck in this loop */
            break;
    }
}


/*******************************************************************************

  Function:
    void Delay_ms (unsigned int x)

  Summary:
    Delay needed for the LCD to process data (in ms)
*/
void Delay_ms(unsigned int x)
{
    PLIB_TMR_Counter16BitClear(TMR_ID_2); /* Clear the timer */
    while (PLIB_TMR_Counter16BitGet(TMR_ID_2) < (40 * x)); /* 40 Timer 1 ticks = ~1ms */
}


/*******************************************************************************

  Function:
    void initializeLCD (void)

  Summary:
    LCD initialization sequence
*/
void initializeLCD(void)
{
    /* Configure general PMP options - enable read/write strobes, long waits, etc */
    PLIB_PMP_AddressSet(PMP_ID_0, PMP_PMA0_PORT);
    PLIB_PMP_AddressPortEnable(PMP_ID_0, PMP_PMA0_PORT);

    Delay_ms(60); /* LCD needs 60ms to power on and perform startup functions */

    PLIB_PMP_AddressSet(PMP_ID_0, CMDREG); /* Access the LCD command register */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x38); /* LCD Function Set - 8-bit interface, 2 lines, 5*7 Pixels */
    Delay_ms(1); /* Needs a 40us delay to complete */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x0C); /* Turn on display (with cursor hidden) */
    Delay_ms(1); /* Needs a 40us delay to complete */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x01); /* Clear the display */
    Delay_ms(2); /* Needs a 1.64ms delay to complete */

    PLIB_PMP_MasterSend(PMP_ID_0, 0x06); /* Set text entry mode - auto increment, no shift */
    Delay_ms(1); /* Needs a 40us delay to complete */
}


/*******************************************************************************

  Function:
    void writeToLCD(int reg, char c)

  Summary:
    Writes a byte of data to either of the two LCD registers (DATAREG, CMDREG)
*/
void writeToLCD(int reg, char c)
{
    PLIB_PMP_AddressSet(PMP_ID_0, reg); /* Either 'DATAREG' or 'CMDREG' */

    DRV_PMP0_Write(c);
    
    Delay_ms(100); /* 40us needed in between each write */
}


/*******************************************************************************

  Function:
    void writeString (unsigned char *string)

  Summary:
    Used to write text strings to the LCD
*/
void writeString(unsigned char *string)
{
    while (*string)
    {
        writeToLCD(DATAREG, *string++); /* Send characters one by one */
    }
}


/*******************************************************************************

  Function:
    void newLine(void)

  Summary:
    Sets the LCD cursor position to line two
*/
void newLine(void)
{
    writeToLCD(CMDREG, 0xC0); /* Cursor address 0x80 + 0x40 = 0xC0 */
}


/*******************************************************************************
 End of File
 */

