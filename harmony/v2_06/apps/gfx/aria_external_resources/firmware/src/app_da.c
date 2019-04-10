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
#include "app_splash.h"
#include "drv_nvm_flash_sqi_sst26.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define DELAY_IN_TICKS       305000000
#define DELAY_FOR_LOAD_TIMES 305000000
//Longer time is taken by bmp's to load, so this longer delay ensures that one bmp draw finishes bfr the next one begins
//Also EF takes longer than DA, hence the two configurations have different values for the delay constants.
#define DELAY_FOR_LOAD_TIMES_BMP 595000000
#define GET_TICKS() __builtin_mfc0(9, 0)
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

#define USB_ID      1
#define SDCARD_ID   2

APP_DATA appData;

DRV_SQI_INIT_DATA sqiInitData;

#define SQI_CACHE_SIZE 256

uint8_t sqiCache[SQI_CACHE_SIZE];
uint32_t sqiCacheAddress = 0;
int32_t sqiCacheValid = 0;

int32_t       usbDeviceConnected;
int32_t       sdcardDeviceConnected;


SYS_FS_HANDLE fileHandle;
long          fileSize;
char          readChar;

const char* usbbinFile = "USBBin.bin";
//const char* jpegName = "AutumnGrass_half_usb.jpg";



GFXU_ImageAsset* leftImage;
GFXU_ImageAsset* rightImage;
laString leftString;
laString rightString;

int8_t idx_leftSelectedImage;
int8_t idx_rightSelectedImage;

int8_t idx_SelectedImageType = -1;


/*************************NewApp*****************************************************/


RESOURCE_SRC selectSource;
static uint32_t startTick;
static uint32_t endTick;
laString str;
char number[20];
extern uint8_t startCounting;
bool slides_on = 0;

#define NumOfImages 4
//uint8_t NumOfImages = 3;
char jpegUSBName[NumOfImages][50];
static uint8_t i_numImages = 1;

char timesList[3][20];

    static uint32_t tickerSlides;
    static uint32_t currentTickSlides;
    static uint32_t tickerPlayAll;
    static uint32_t DelayValue;
    
    
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
void Image_start(laImageWidget* image)
{
    if(image->image->format == GFXU_IMAGE_FORMAT_RAW)
    {
        laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_bmp));
        DelayValue = DELAY_FOR_LOAD_TIMES_BMP;
    }
    else if (image->image->format == GFXU_IMAGE_FORMAT_JPEG)
    {
        laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
        DelayValue = DELAY_FOR_LOAD_TIMES;
    }
    else
    {
    }
        
    startTick = GET_TICKS();
}

void Image_end(laImageWidget* image)
{
    uint32_t time = 0;
    endTick = GET_TICKS();
    
    //This sqiCache values are observed when the sqi is not flashed. But this is not a scientific way of confirming it is unflashed.
    //This needs to be fixed in the next revision.
    if((sqiCache[1] == 0xff )&& selectSource == SQI)
    {
        laButtonWidget_SetPressedImage(ButtonSQI, &sqiFlashrect_null_70);
        laButtonWidget_SetReleasedImage(ButtonSQI, &sqiFlashrect_null_70);
        return;
    }
    else
    {
        laButtonWidget_SetPressedImage(ButtonSQI, &sqiFlashrect_red_gimp_70);
        laButtonWidget_SetReleasedImage(ButtonSQI, &sqiFlashrect_red_gimp_70);
    }
    if(endTick > startTick)
    {
        time = (endTick - startTick)/100000;//ms
    }
    else if(endTick < startTick)
    {
        time = ((4294967296 - startTick) + endTick) / 100000; //ms
    }
    itoa(number, time, 10);
            
    if(appData.state != APP_PLAY_ALL_SOURCE && appData.prevState != APP_PLAY_ALL_SOURCE)
    {
        laWidget_SetVisible((laWidget*)LabelWidget_load_time, LA_TRUE);
        laWidget_SetVisible((laWidget*)LabelWidget_time, LA_TRUE);
        laWidget_SetVisible((laWidget*)LabelWidget_ImageType, LA_TRUE);
        str = laString_CreateFromCharBuffer(number, &Arial);
        laLabelWidget_SetText(LabelWidget_time, str);
        laString_Destroy(&str);
    }
    else if (appData.state == APP_STATE_PLAYALL_DONE)
    {
        appData.prevState = APP_STATE_SERVICE_TASKS;
        laWidget_SetVisible((laWidget*)LabelWidget_load_time, LA_FALSE);
        laWidget_SetVisible((laWidget*)LabelWidget_time, LA_FALSE);
        laWidget_SetVisible((laWidget*)LabelWidget_ImageType, LA_FALSE);
        
    }
    else if(appData.state == APP_PLAY_ALL_SOURCE || appData.prevState == APP_PLAY_ALL_SOURCE)
    {
        laWidget_SetVisible((laWidget*)LabelWidget_load_time, LA_FALSE);
        laWidget_SetVisible((laWidget*)LabelWidget_time, LA_FALSE);
        laWidget_SetVisible((laWidget*)LabelWidget_ImageType, LA_FALSE);
    }
}

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
                if(appData.state != APP_STATE_INIT && appData.state != APP_STATE_SPLASH )
                {
                    //appData.prevState = appData.state;
                    appData.state = APP_USB_STATUS;
                }
            }
            else if(strcmp((const char *)eventData,"/mnt/sdcard") == 0)
            {
                sdcardDeviceConnected = 1;
            }
            
            //laContext_RedrawAll();
            
            break;
        }    
        case SYS_FS_EVENT_UNMOUNT:
        {
            if(strcmp((const char *)eventData,"/mnt/usb") == 0)
            {
                usbDeviceConnected = 0;
                if(appData.state != APP_STATE_INIT && appData.state != APP_STATE_SPLASH )
                {
                    //appData.prevState = appData.state;
                    appData.state = APP_USB_STATUS;
                }
            }
            else if(strcmp((const char *)eventData,"/mnt/sdcard") == 0)
            {
                sdcardDeviceConnected = 0;
            }
            
            //laContext_RedrawAll();
            
            break;
        }
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

extern APP_SPLASH_DATA appSplashData;
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
    appData.prevState = APP_STATE_INIT;
    //for the splash screen
    appSplashData.state = 0;
    
    sqiInitData.clkDivider = CLK_DIV_2;
    sqiInitData.csPins = SQI_CS_OEN_1;
    sqiInitData.dataMode = SQI_DATA_MODE_3;
    SST26Init((DRV_SQI_INIT_DATA*)&sqiInitData);
    
    SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, USB_ID);
    USB_HOST_EventHandlerSet(&APP_USBHostEventHandler, 0);
    USB_HOST_BusEnable(0);
    
    createUSBNameArray();
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
     static laContext* context;
     context = laContext_GetActive();
     

     
     currentTickSlides = GET_TICKS();
     
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true; 
            
            if (appInitialized)
            {
                selectSource = INT_MEM;
                appData.state = APP_STATE_SPLASH;
                appData.prevState = APP_USB_STATUS;
            }
            break;
        }

        case APP_STATE_SPLASH:
        {            
            if (APP_IsSplashScreenComplete())
            {
                appData.state = APP_SET_NEW_MAINMENU_SCREEN;
                
                laContext_SetActiveScreen(New_appScreen_ID);
            }
        
            break;
        }
        case APP_SET_NEW_MAINMENU_SCREEN:
        {
            if (context->activeScreen->id == New_appScreen_ID && context->activeScreen->layers[0]->frameState != LA_LAYER_FRAME_IN_PROGRESS)
            {
                if(ImageWidget_newapp != NULL)
                {
                    laImageWidget_SetCallBackStart(ImageWidget_newapp, Image_start);
                    laImageWidget_SetCallBackEnd(ImageWidget_newapp, Image_end);
                    laImageWidget_SetImage(ImageWidget_newapp, &Image_int_1);
                    laWidget_SetVisible((laWidget*)LabelWidget_source, LA_TRUE);
                    laWidget_SetVisible((laWidget*)LabelWidget_load_time, LA_TRUE);
                    laWidget_SetVisible((laWidget*)LabelWidget_ImageType, LA_TRUE);
                    
                    laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_IntFlash));
                    laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_intmem));
                    laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
                }
                appData.prevState = APP_STATE_SERVICE_TASKS;
                appData.state = APP_USB_STATUS;
            }
            break;
        }
        
        case APP_USB_STATUS:
        {
            if(usbDeviceConnected == 0)
            {
                laWidget_SetVisible((laWidget*)ButtonUSB, LA_FALSE);
            }
            else //usbDeviceConnected == 1 
            {
                createUSBNameArray();
                laWidget_SetVisible((laWidget*)ButtonUSB, LA_TRUE);
            }
            
            appData.state = appData.prevState;
            break;
        }
        case APP_WAIT_TO_DRAW_IMAGE_TO_SET_TIME:
        {
            break;
        }
        case APP_SLIDE_MODE:
        {
            tickerSlides = currentTickSlides + DelayValue;
            appData.prevState = APP_SLIDE_MODE;
            appData.state = APP_STATE_SERVICE_TASKS;
            
            break;
        }
        case APP_PLAY_ALL_SOURCE:
        {
            tickerPlayAll = currentTickSlides + DelayValue;
            
            appData.state = APP_STATE_SERVICE_TASKS;
            appData.prevState = APP_PLAY_ALL_SOURCE;
            
            break;
        }
        case APP_STATE_PLAYALL_DONE:
        {
            break;
        }
        case APP_STATE_SERVICE_TASKS:
        {
            if(appData.prevState == APP_USB_STATUS)
            {
                appData.state = APP_USB_STATUS;
                appData.prevState = APP_STATE_SERVICE_TASKS;
            }
            
            if(appData.prevState == APP_SLIDE_MODE)
            {
                if(tickerSlides < currentTickSlides)
                {
                    //appData.prevState = APP_STATE_SERVICE_TASKS;
                    Slides();
                }
            }
            
            if(appData.prevState == APP_PLAY_ALL_SOURCE)
            {
                if(tickerPlayAll < currentTickSlides)
                {
                    //appData.prevState = APP_STATE_SERVICE_TASKS;
                    PlayAllSource();
                }
            }
            
            
            break;
        }


        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

GFX_Result app_externalMediaOpen(GFXU_AssetHeader* ast)
{

        
    if(ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBBin ||
       ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBFile)
    {
        if(usbDeviceConnected == 0)
            return GFX_FAILURE;
        
        SYS_FS_CurrentDriveSet("/mnt/usb");

        createUSBNameArray();
        
        if(ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBBin)
            fileHandle = SYS_FS_FileOpen(usbbinFile, (SYS_FS_FILE_OPEN_READ));
        else if(i_numImages == 1)
            fileHandle = SYS_FS_FileOpen(jpegUSBName[0], (SYS_FS_FILE_OPEN_READ));
        else if(i_numImages == 2)
            fileHandle = SYS_FS_FileOpen(jpegUSBName[1], (SYS_FS_FILE_OPEN_READ));
        else if(i_numImages == 3)
            fileHandle = SYS_FS_FileOpen(jpegUSBName[2], (SYS_FS_FILE_OPEN_READ));
        else if(i_numImages == 4)
            fileHandle = SYS_FS_FileOpen(jpegUSBName[3], (SYS_FS_FILE_OPEN_READ));
        if(fileHandle == SYS_FS_HANDLE_INVALID)
            return GFX_FAILURE;
    }
    
    return GFX_SUCCESS;
}

GFX_Result app_externalMediaRead(GFXU_ExternalAssetReader* reader,
                                 GFXU_AssetHeader* ast,
                                 void* address,
                                 uint32_t readSize,
                                 uint8_t* destBuffer,
                                 GFXU_MediaReadRequestCallback_FnPtr cb)
{
    

    
    if(ast->dataLocation == GFXU_ASSET_LOCATION_ID_SQI)
    {
        
        if(readSize > SQI_CACHE_SIZE)
        {
            // this driver is blocking
            SST26ReadArray((uint32_t)address, destBuffer, readSize);

            if(reader != NULL && cb != NULL)
                cb(reader); // indicate that the data buffer is ready

            return GFX_SUCCESS;
        }

        // read from cache if available
        if(sqiCacheValid == 1 &&
           sqiCacheAddress <= (uint32_t)address &&
           (uint32_t)address + readSize < sqiCacheAddress + SQI_CACHE_SIZE)
        {
            memcpy(destBuffer, &sqiCache[(uint32_t)address - sqiCacheAddress], readSize);

            if(reader != NULL && cb != NULL)
                cb(reader); // indicate that the data buffer is ready

            return GFX_SUCCESS; // success tells the decoder to keep going
        }
        else
        {
            sqiCacheAddress = (uint32_t)address;
            sqiCacheAddress -= sqiCacheAddress % 4;
                       
            // this driver is blocking
            SST26ReadArray(sqiCacheAddress, sqiCache, SQI_CACHE_SIZE);
            
            sqiCacheValid = 1;

            memcpy(destBuffer, &sqiCache[(uint32_t)address - sqiCacheAddress], readSize);

            //SST26ReadArray((uint32_t)address, destBuffer, readSize);

            // if reader or cb are NULL then the decoder expects to block
            // and be serviced immediately

            // if reader and cb are not NULL then the application can service
            // this data request as it is able using non-blocking drivers
            // store the function arguments and call the callback later
            // after the peripheral has completed the request

            if(reader != NULL && cb != NULL)
                cb(reader); // indicate that the data buffer is ready

            return GFX_SUCCESS; // success tells the decoder to keep going
        }
            
    }
    else if((ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBBin ||
             ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBFile) &&
             usbDeviceConnected == 1)
    {
        SYS_FS_FileSeek(fileHandle, (uint32_t)address, SYS_FS_SEEK_SET);

        SYS_FS_FileRead(fileHandle, destBuffer, readSize);
        
        if(cb != NULL && reader != NULL)
            cb(reader); // indicate that the data buffer is ready
        
        return GFX_SUCCESS;
    }
    
    return GFX_FAILURE; // failure tells the decoder to abort and move on
}

void app_externalMediaClose(GFXU_AssetHeader* ast)
{
    if((ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBBin ||
        ast->dataLocation == GFXU_ASSET_LOCATION_ID_USBFile) &&
        usbDeviceConnected == 1)
    {
        SYS_FS_FileClose(fileHandle);
        
        fileHandle = 0;
    }
}




/***********************************NEW APP******************************************/

void Load_image_from_usb(void)
{
    selectSource = USB;
    startCounting = 0;
    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_1, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_2, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_3, LA_FALSE);
    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE);        
    if(!slides_on)
    {
        laWidget_Invalidate((laWidget*)ImageWidget_newapp);
        laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_USBBin));
        laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_usb));
        laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
        if(i_numImages == 1)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_1);
        }
        else if(i_numImages == 2)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_2);
        }
        else if(i_numImages == 3)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_3);
        }
        else if(i_numImages == 4)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_4);
        }
    }
}

void Load_image_from_int(void)
{
    selectSource = INT_MEM;
    startCounting = 0;
    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_1, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_2, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_3, LA_FALSE);
    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE); 
    if(!slides_on)
    {
        laWidget_Invalidate((laWidget*)ImageWidget_newapp);
        laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_IntFlash));
        laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_intmem));
        laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
        if(i_numImages == 1)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_int_1);
        }
        else if(i_numImages == 2)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_int_2);
        }
        else if(i_numImages == 3)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_int_3);
        }
        else if(i_numImages == 4)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_int_4);
        }
    }
}

void Load_image_from_sqi(void)
{
    selectSource = SQI;
    startCounting = 0;
    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
        laWidget_SetVisible((laWidget*)label_ms_1, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_2, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_3, LA_FALSE);
    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE); 
    if(!slides_on)
    {
        laWidget_Invalidate((laWidget*)ImageWidget_newapp);
        laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_SQI));
        laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_sqi));
        laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
        if(i_numImages == 1)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_1);            
        }
        else if(i_numImages == 2)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_2);
        }
        else if(i_numImages == 3)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_3);
        }
        else if(i_numImages == 4)
        {
            laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_4);
        }
        
    }
    
    
}


void Slides(void)
{

    laWidget_SetVisible((laWidget*)LabelWidget1_IntTimeLabel, LA_FALSE);
    laWidget_SetVisible((laWidget*)LabelWidget_SqiTimeLabel, LA_FALSE);
    laWidget_SetVisible((laWidget*)LabelWidget_UsbTimeLabel, LA_FALSE);
    
    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
        laWidget_SetVisible((laWidget*)label_ms_1, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_2, LA_FALSE);
    laWidget_SetVisible((laWidget*)label_ms_3, LA_FALSE);
               
    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE); 
    
    if(slides_on)
    {
        if(i_numImages == 4)
        {
            i_numImages = 1;
        }
        else
        {
            i_numImages++;
        }
        if(selectSource == INT_MEM)
        {
            selectSource = INT_MEM;
            startCounting = 0;
            laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_IntFlash));
            laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_intmem));
            laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
            if(i_numImages == 1)
            {                    
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_1);
                
            }
            else if(i_numImages == 2)
            {   
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_2);
                
            }
            else if(i_numImages == 3)
            {   
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_3);
                
            }
            else if(i_numImages == 4)
            {   
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_4);
                
            }
            else 
            {
                //error condition handling
            }
        }
        else if(selectSource == USB)
        {
            selectSource = USB;
            startCounting = 0;
            laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_USBBin));
            laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_usb));
            laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
            if(i_numImages == 1)
            {                
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_1);
                
            }
            else if(i_numImages == 2)
            {
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_2);
                
            }
            else if(i_numImages == 3)
            {
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_3);
                
            }
            else if(i_numImages == 4)
            {
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_usb_4);
                
            }
            else 
            {
                //error condition handling
            }
        }
        else if(selectSource == SQI)
        {
            selectSource = SQI;
            startCounting = 0;
            laLabelWidget_SetText(LabelWidget_source, laString_CreateFromID(string_DisplayText_SQI));
            laLabelWidget_SetText(LabelWidget_load_time, laString_CreateFromID(string_Load_time_sqi));
            laLabelWidget_SetText(LabelWidget_ImageType, laString_CreateFromID(string_ImageType_jpeg));
            if(i_numImages == 1)
            {                
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_1);
                
            }
            else if(i_numImages == 2)
            {
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_2);
                
            }
            else if(i_numImages == 3)
            {
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_3);
                
            }
            else if(i_numImages == 4)
            {
                laWidget_Invalidate((laWidget*)ImageWidget_newapp);
                laImageWidget_SetImage(ImageWidget_newapp, &Image_sqi_4);
                
            }
            else 
            {
                //error condition handling
            }
        }
        
        appData.state = APP_SLIDE_MODE;
    }
    else
    {
        appData.state = APP_STATE_SERVICE_TASKS;
    }
}
    

void HitSlidesButton(void)
{
//       laWidget_SetVisible((laWidget*)LabelWidget1_IntTimeLabel, LA_FALSE);
//    laWidget_SetVisible((laWidget*)LabelWidget_SqiTimeLabel, LA_FALSE);
//    laWidget_SetVisible((laWidget*)LabelWidget_UsbTimeLabel, LA_FALSE);
//    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
//    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
//    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
//    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE); 
    slides_on = !slides_on;
    
    if(slides_on)
    {
        laButtonWidget_SetReleasedImage(ButtonSlides, &slides_pause_gimp_70);
        appData.state = APP_SLIDE_MODE;
    }
    else
    {
        laButtonWidget_SetReleasedImage(ButtonSlides, &slides_icon_gimp_70);
        appData.state = APP_STATE_SERVICE_TASKS;
    }
    
}
    

void createUSBNameArray(void)
{
    strcpy(jpegUSBName[0],"Image_usb_1.jpg");
    strcpy(jpegUSBName[1],"Image_usb_2.jpg");
    strcpy(jpegUSBName[2],"Image_usb_3.jpg");
    strcpy(jpegUSBName[3],"Image_usb_4.bmp");
}

void PlayAllSource(void)
{
    
    slides_on = 0;
    laButtonWidget_SetReleasedImage(ButtonSlides, &slides_icon_gimp_70);
    laString sourceAppend;
    
    laWidget_SetVisible((laWidget*)ButtonUSB, LA_FALSE);
    laWidget_SetVisible((laWidget*)ButtonSQI, LA_FALSE);
    laWidget_SetVisible((laWidget*)ButtonIntMem, LA_FALSE);
    laWidget_SetVisible((laWidget*)ButtonSlides, LA_FALSE);
    
    switch(selectSource)
    {
        case INT_MEM:
        {
            Load_image_from_int();
            selectSource = SQI;
             
            break;
        }
        case SQI:
        {
            strcpy(timesList[0], number);
            Load_image_from_sqi();
            
            if(usbDeviceConnected == 1)
                selectSource = USB;
            else
                selectSource = NONE;
                      
            break;
        }
        case USB:
        {
            strcpy(timesList[1], number); 
            if(usbDeviceConnected == 1)
            {
                Load_image_from_usb();
                
                selectSource = NONE;
            }
            break;
        }
        case NONE:
        {
            if(usbDeviceConnected == 0)
            {
                strcpy(timesList[1], number);
                laWidget_SetVisible((laWidget*)Label_Int_time, LA_TRUE);
                laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_TRUE);
                laWidget_SetVisible((laWidget*)label_ms_1, LA_TRUE);
                laWidget_SetVisible((laWidget*)label_ms_2, LA_TRUE);
   
                laWidget_SetVisible((laWidget*)LabelWidget1_IntTimeLabel, LA_TRUE);
                laWidget_SetVisible((laWidget*)LabelWidget_SqiTimeLabel, LA_TRUE);
  
            }
            else
            {
                strcpy(timesList[2], number);
                laWidget_SetVisible((laWidget*)Label_Int_time, LA_TRUE);
                laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_TRUE);
                laWidget_SetVisible((laWidget*)Label_Usb_time, LA_TRUE);
                laWidget_SetVisible((laWidget*)label_ms_1, LA_TRUE);
                laWidget_SetVisible((laWidget*)label_ms_2, LA_TRUE);
                laWidget_SetVisible((laWidget*)label_ms_3, LA_TRUE);
                
                laWidget_SetVisible((laWidget*)LabelWidget1_IntTimeLabel, LA_TRUE);
                laWidget_SetVisible((laWidget*)LabelWidget_SqiTimeLabel, LA_TRUE);
                laWidget_SetVisible((laWidget*)LabelWidget_UsbTimeLabel, LA_TRUE);
            }
            
        
            laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_TRUE); 
             
//            laWidget_SetVisible((laWidget*)LabelWidget_source, LA_FALSE);
//            laWidget_SetVisible((laWidget*)LabelWidget_time, LA_FALSE);
            
            str = laString_CreateFromCharBuffer(timesList[0], &Arial);
            sourceAppend = laString_CreateFromID(string_Int_Mem_load_time);
            laString_Append(&sourceAppend, &str);
            laLabelWidget_SetText(Label_Int_time, str);
            laString_Destroy(&str);
    
    
            str = laString_CreateFromCharBuffer(timesList[1], &Arial);
            //laLabelWidget_SetText(Label_Sqi_time, laString_CreateFromID(string_Sqi_load_time));
            laLabelWidget_SetText(Label_Sqi_time, str);
            
            str = laString_CreateFromCharBuffer(timesList[2], &Arial);
            //laLabelWidget_SetText(Label_Usb_time, laString_CreateFromID(string_USB_load_time));
            laLabelWidget_SetText(Label_Usb_time, str);
            
            if(i_numImages == 1)
            {        
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_1);               
            }
            else if(i_numImages == 2)
            {               
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_2);                
            }
            else if(i_numImages == 3)
            {                
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_3);                
            }
            else if(i_numImages == 4)
            {                
                laImageWidget_SetImage(ImageWidget_newapp, &Image_int_4);                
            }
            
            str = laString_CreateFromCharBuffer("", &Arial);
            laLabelWidget_SetText(LabelWidget_time, str);
            laWidget_SetVisible((laWidget*)LabelWidget_time, LA_FALSE);
            selectSource = INT_MEM;
            
            laWidget_SetVisible((laWidget*)ButtonUSB, LA_TRUE);
            laWidget_SetVisible((laWidget*)ButtonSQI, LA_TRUE);
            laWidget_SetVisible((laWidget*)ButtonIntMem, LA_TRUE);
            laWidget_SetVisible((laWidget*)ButtonSlides, LA_TRUE);
    
            appData.state = APP_STATE_PLAYALL_DONE;
            return;
        }
    
    }
    
    appData.state = APP_PLAY_ALL_SOURCE;    
}

void BackFromHelp(void)
{
    appData.state = APP_SET_NEW_MAINMENU_SCREEN;
    appData.prevState = APP_STATE_SERVICE_TASKS;
    startCounting = 0;
}

void InfoButton(void)
{
    appData.state = APP_STATE_SERVICE_TASKS;
    appData.prevState = APP_STATE_SERVICE_TASKS;
    startCounting = 0;
}
/*******************************************************************************
 End of File
 */