/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_sqi.c

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

#include <string.h>

#include "hexdecoder.h"
#include "drv_nvm_flash_sqi_sst26.h"

#include "gfx/libaria/libaria_init.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define USB_ID      1
#define SDCARD_ID   2

#define READ_BUFFER_SZ   1024

HexDecoder dec;

DRV_SQI_INIT_DATA sqiInitData;

SYS_FS_HANDLE fileHandle;
long          fileSize;
char          readChar;
uint8_t       readBuffer[READ_BUFFER_SZ];
uint8_t       writeBuffer[HEXDECODER_MAX_RECORD_SIZE] __attribute__((coherent, aligned(4)));
uint8_t       verificationBuffer[HEXDECODER_MAX_RECORD_SIZE] __attribute__((coherent, aligned(4)));

int32_t       usbDeviceConnected;
int32_t       sdcardDeviceConnected;

char number[12];

uint32_t i;
uint32_t recordCount;

laString str;

float percent;

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


USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event,
                                                 void* eventData,
                                                 uintptr_t context)
{
    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

static void deviceConnectionStateChanged()
{
    if(usbDeviceConnected != 0 || sdcardDeviceConnected != 0)
    {
        laWidget_SetVisible(ErrorMsgPanel, LA_FALSE);
    }
    else
    {
        laWidget_SetVisible(ErrorMsgPanel, LA_TRUE);
    }
    
    laWidget_SetVisible((laWidget*)USBButton, usbDeviceConnected != 0);
    laWidget_SetVisible((laWidget*)SDCardButton, sdcardDeviceConnected != 0);
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event,
                           void* eventData,
                           uintptr_t context)
{
    switch(event)
    {
        case SYS_FS_EVENT_MOUNT:
        {
            if(strcmp((const char *)eventData,"/mnt/usb") == 0)
            {
                usbDeviceConnected = 1;
            }
            else if(strcmp((const char *)eventData,"/mnt/sdcard") == 0)
            {
                sdcardDeviceConnected = 1;
            }
            
            deviceConnectionStateChanged();
            
            break;
        }    
        case SYS_FS_EVENT_UNMOUNT:
        {
            if(strcmp((const char *)eventData,"/mnt/usb") == 0)
            {
                usbDeviceConnected = 0;
            }
            else if(strcmp((const char *)eventData,"/mnt/sdcard") == 0)
            {
                sdcardDeviceConnected = 0;
            }
            
            deviceConnectionStateChanged();
            
            break;
        }
        default:
            break;
    }
}

void APP_SDCardButtonPressed(laButtonWidget* btn)
{
    SYS_FS_CurrentDriveSet("/mnt/sdcard");

    fileHandle = SYS_FS_FileOpen("SQI.hex", (SYS_FS_FILE_OPEN_READ));
    
    if(fileHandle != SYS_FS_HANDLE_INVALID)
    {
        appData.state = APP_VALIDATE_FILE;
    }
    else
    {
        appData.state = APP_FILE_NOT_FOUND;
    }
}

void APP_USBButtonPressed(laButtonWidget* btn)
{
    SYS_FS_CurrentDriveSet("/mnt/usb");

    fileHandle = SYS_FS_FileOpen("SQI.hex", (SYS_FS_FILE_OPEN_READ));
    
    if(fileHandle != SYS_FS_HANDLE_INVALID)
    {
        appData.state = APP_VALIDATE_FILE;
    }
    else
    {
        appData.state = APP_FILE_NOT_FOUND;
    }
}

void APP_OKButtonPressed(laButtonWidget* btn)
{
    laWidget_SetVisible(InfoPanel, LA_FALSE);
    laWidget_SetVisible(SelectMediumPanel, LA_TRUE);
    laWidget_SetVisible((laWidget*)InfoOKButton, LA_FALSE);
    laWidget_SetVisible((laWidget*)USBButton, LA_TRUE);
    laWidget_SetVisible((laWidget*)SDCardButton, LA_TRUE);
    deviceConnectionStateChanged();
}

int32_t recordReadCB(HexDecoder* dec,
                     uint32_t recordNum,
                     uint8_t record[HEXDECODER_MAX_RECORD_SIZE])
{
    uint32_t i = 0;
    uint32_t size;
    char csize[4];
    
    // unexpected end of file
    if(SYS_FS_FileEOF(fileHandle) == 1)
        return -1;
    
    readChar = 0;
    
    while(readChar != ':')
    {
        SYS_FS_FileRead(fileHandle, &readChar, 1);

        if(SYS_FS_FileEOF(fileHandle) == 1)
            return -1;
    }
    
    // colon
    record[i++] = readChar;
    
    if(SYS_FS_FileEOF(fileHandle) == 1)
        return -1;
    
    // size
    SYS_FS_FileRead(fileHandle, &record[i], 8);
    i += 8;
    
    if(SYS_FS_FileEOF(fileHandle) == 1)
        return -1;
    
    csize[0] = record[1];
    csize[1] = record[2];
    csize[2] = 0;
    csize[3] = 0;
    
    size = strtoul(csize, NULL, 16) * 2;
    
    // data block
    SYS_FS_FileRead(fileHandle, &record[i], size);
    i += size;
    
    if(SYS_FS_FileEOF(fileHandle) == 1)
        return -1;
    
    // checksum
    SYS_FS_FileRead(fileHandle, &record[i], 2);
    i += 2;
    
    return 0;
}

volatile int x;

int32_t dataWriteCB(HexDecoder* dec, 
                    uint32_t address,
                    uint8_t* buffer,
                    uint32_t size)
{
    if(address == 0x9D00)
    {
        x = 0;
    }
    
    SST26WriteArray(address, buffer, size);
    
    memset(verificationBuffer, 0x0, size);
    
    SST26ReadArray(address, verificationBuffer, size);
    
    if(memcmp(buffer, verificationBuffer, size) != 0)
        return -1;
    
    return 0;
}

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
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    usbDeviceConnected = 0;
    
    sqiInitData.clkDivider = CLK_DIV_2;
    sqiInitData.csPins = SQI_CS_OEN_1;
    sqiInitData.dataMode = SQI_DATA_MODE_3;
    SST26Init((DRV_SQI_INIT_DATA*)&sqiInitData);
    
    SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, USB_ID);
    USB_HOST_EventHandlerSet(&APP_USBHostEventHandler, 0);
    USB_HOST_BusEnable(0);
    
    laButtonWidget_SetReleasedEventCallback(SDCardButton,
                                            &APP_SDCardButtonPressed);
    
    laButtonWidget_SetReleasedEventCallback(USBButton,
                                            &APP_USBButtonPressed);
    
    laButtonWidget_SetReleasedEventCallback(InfoOKButton,
                                            &APP_OKButtonPressed);
    
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            appData.state = APP_INIT_WRITE_MEDIA;
            
            break;
        }
        
        case APP_INIT_WRITE_MEDIA:
        {
            if(SST26_SQI_FlashID_Read() == SQI_STATUS_SUCCESS)
                appData.state = APP_DONE;
            
            break;
        }
        
        case APP_OPEN_FILE:
        {
            fileHandle = SYS_FS_FileOpen("/SQI.hex", SYS_FS_FILE_OPEN_READ);
            
            if(fileHandle == SYS_FS_HANDLE_INVALID)
            {
                appData.state = APP_DONE;
            }
            else
            {
                appData.state = APP_VALIDATE_FILE;
            }
            break;
        }
        
        case APP_FILE_NOT_FOUND:
        {
            laWidget_SetVisible(SelectMediumPanel, LA_FALSE);
            laWidget_SetVisible(InfoPanel, LA_TRUE);
            
            str = laString_CreateFromID(string_FileNotFound1);
            laLabelWidget_SetText(InfoLabel1, str);
            
            str = laString_CreateFromID(string_FileNotFound2);
            laLabelWidget_SetText(InfoLabel2, str);
            
            laWidget_SetVisible((laWidget*)InfoLabel1, LA_TRUE);
            laWidget_SetVisible((laWidget*)InfoLabel2, LA_TRUE);
            
            appData.state = APP_DONE;
            break;
        }
        
        case APP_VALIDATE_FILE:
        {
            laWidget_SetVisible(SelectMediumPanel, LA_FALSE);
            laWidget_SetVisible(FlashingPanel, LA_TRUE);
            
            laWidget_SetVisible((laWidget*)CurrentRecordLabel, LA_FALSE);
            laWidget_SetVisible((laWidget*)OfLabel, LA_FALSE);
            laWidget_SetVisible((laWidget*)RecordsTotalLabel, LA_FALSE);
            
            str = laString_CreateFromID(string_RecordCount);
            laLabelWidget_SetText(FlashingLabel, str);
            
            fileSize = SYS_FS_FileSize(fileHandle);
            
            if(fileSize <= 0)
            {
                SYS_FS_FileClose(fileHandle);
                
                str = laString_CreateFromID(string_InvalidFile);
                laLabelWidget_SetText(InfoLabel1, str);

                laWidget_SetVisible((laWidget*)InfoLabel1, LA_TRUE);
                laWidget_SetVisible((laWidget*)InfoLabel2, LA_FALSE);
                
                appData.state = APP_DONE;
            }
            else
            {
                appData.state = APP_READ_RECORD_COUNT;
            }
        }
        
        case APP_READ_RECORD_COUNT:
        {
            recordCount = 0;
            
            str = laString_CreateFromID(string_RecordCount);
            laLabelWidget_SetText(FlashingLabel, str);
            
            for(i = 0; i < fileSize; i++)
            {
                SYS_FS_FileRead(fileHandle, &readChar, 1);
                
                if(readChar == ':')
                    recordCount++;
            }
            
            if(recordCount == 0)
            {
                SYS_FS_FileClose(fileHandle);
                
                str = laString_CreateFromID(string_InvalidFile);
                laLabelWidget_SetText(InfoLabel1, str);

                laWidget_SetVisible((laWidget*)InfoLabel1, LA_TRUE);
                laWidget_SetVisible((laWidget*)InfoLabel2, LA_FALSE);
                
                appData.state = APP_DONE;
            }
            else
            {
                appData.state = APP_START_DECODING;
            }
            
            break;
        }
        
        case APP_START_DECODING:
        {
            // update the UI
            str = laString_CreateFromID(string_Flashing);
            laLabelWidget_SetText(FlashingLabel, str);
            
            laProgressBarWidget_SetValue(FlashingProgressBar, 0);
            laWidget_SetVisible((laWidget*)USBButton, LA_FALSE);
            laWidget_SetVisible((laWidget*)SDCardButton, LA_FALSE);
            laWidget_SetVisible((laWidget*)CurrentRecordLabel, LA_TRUE);
            laWidget_SetVisible((laWidget*)OfLabel, LA_TRUE);
            laWidget_SetVisible((laWidget*)RecordsTotalLabel, LA_TRUE);
            
            itoa(number, recordCount, 10);
            
            str = laString_CreateFromCharBuffer(number, &Arial_sm);
            laLabelWidget_SetText(RecordsTotalLabel, str);
            laString_Destroy(&str);
            
            // initailize hex decoder
            HexDecoder_Initialize(&dec,
                                  recordCount,
                                  writeBuffer,
                                  &recordReadCB,
                                  &dataWriteCB);
            
            // reset file pointer to the start
            SYS_FS_FileSeek(fileHandle, 0, SYS_FS_SEEK_SET);
            
            // erase target space
            SST26ChipErase();
            
            appData.state = APP_PRE_DECODE;
            
            break;
        }
        
        case APP_PRE_DECODE:
        {
            itoa(number, dec.currentRecord + 1, 10);
            
            str = laString_CreateFromCharBuffer(number, &Arial_sm);
            laLabelWidget_SetText(CurrentRecordLabel, str);
            laString_Destroy(&str);
            
            percent = ((float)dec.currentRecord / ((float)dec.recordCount - 1)) * 100.0f;
            
            laProgressBarWidget_SetValue(FlashingProgressBar, (uint32_t)percent);
            
            appData.state = APP_DECODE_RECORD;
            
            break;
        }
        
        case APP_DECODE_RECORD:
        {
            if(dec.currentRecord == dec.recordCount)
            {
                SYS_FS_FileClose(fileHandle);
                
                laWidget_SetVisible(FlashingPanel, LA_FALSE);
                laWidget_SetVisible(InfoPanel, LA_TRUE);
                
                str = laString_CreateFromID(string_FlashingComplete);
                laLabelWidget_SetText(InfoLabel1, str);
                
                laWidget_SetVisible((laWidget*)USBButton, LA_TRUE);
                laWidget_SetVisible((laWidget*)SDCardButton, LA_TRUE);
                laWidget_SetVisible((laWidget*)InfoLabel1, LA_TRUE);
                laWidget_SetVisible((laWidget*)InfoLabel2, LA_FALSE);
                laWidget_SetVisible((laWidget*)InfoOKButton, LA_TRUE);
                
                // decode complete
                appData.state = APP_DONE;
            }
            else if(HexDecoder_Decode(&dec) == -1)
            {
                SYS_FS_FileClose(fileHandle);
                
                laWidget_SetVisible(FlashingPanel, LA_FALSE);
                laWidget_SetVisible(InfoPanel, LA_TRUE);
                
                str = laString_CreateFromID(string_UnknownError);
                laLabelWidget_SetText(InfoLabel1, str);
                
                laWidget_SetVisible((laWidget*)InfoLabel1, LA_TRUE);
                laWidget_SetVisible((laWidget*)InfoLabel2, LA_FALSE);
                
                appData.state = APP_DONE;
            }
            else
            {
                appData.state = APP_PRE_DECODE;
            }
            
            break;
        }
        
        case APP_DONE:
        /* The default state should never be executed. */
        default:
        {           
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
