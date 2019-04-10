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
Software only when embedded on a Microchip microcontroller or Digital signal
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
#include "sound_effects/run_sound.h"
#include "sound_effects/blaze_sound.h"
#include "sound_effects/hurt_sound.h"
#include "sound_effects/dizzy_sound.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define APP_SPRITE_TIMER_DELAY 1
#define APP_RUN_SPRITE_DELAY 5  //This has to be greater than APP_SPEED_FACTOR_LIMIT 
#define APP_BLAZE_SPRITE_DELAY 5 //This has to be greater than APP_SPEED_FACTOR_LIMIT 
#define APP_IDLE_SPRITE_DELAY 480 
#define APP_HURT_SPRITE_DELAY 1  
#define APP_FALL_SPRITE_DELAY 5  
#define APP_DIZZY_SPRITE_DELAY 300
#define APP_NUM_IMAGE_PREPROCESS 53
#define APP_SPRITE_STEP_X APP_LAYER_MOVE_FACTOR2 / 2
#define APP_PRESS_COUNTER_DELAY 800
#define APP_UI_LAYER_MAX_ALPHA 155
#define APP_LAMB_STAMINA 100
#define APP_LAMB_BLAZE_ZONE 30
#define APP_LAMB_FATIGUE_LIMIT 10
#define APP_SCORE_MAX_COUNT 999999
#define APP_SCORE_RUN_INCREMENT 2
#define APP_SCORE_BLAZE_INCREMENT 144

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

int32_t miny = 0;
int32_t maxy = 0;

int32_t APP_SPRITE_ANCHOR_INIT = 0;
int32_t APP_SPRITE_ANCHOR_LEFT_LIMIT = 0;
int32_t APP_SPRITE_ANCHOR_RIGHT_LIMIT = 0;

int32_t APP_LAYER_MOVE_FACTOR0 = 1;
int32_t APP_LAYER_MOVE_FACTOR1 = 2;
int32_t APP_LAYER_MOVE_FACTOR2 = 4;
int32_t APP_BLAZE_SPEED_FACTOR_LIMIT = 6;
int32_t APP_SPEED_FACTOR_LIMIT = 3;

static uint32_t run_sound_size = 0;
static uint32_t blaze_sound_size = 0;
static uint32_t hurt_sound_size = 0;
static uint32_t dizzy_sound_size = 0;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
static void touchDown(laWidget* widget, laInput_TouchDownEvent* evt)
{
	evt->event.accepted = LA_TRUE;
}

static void touchUp(laWidget* widget, laInput_TouchUpEvent* evt)
{
	evt->event.accepted = LA_TRUE;
}

static void touchMovedUpDown(laWidget* widget, laInput_TouchMovedEvent* evt)
{
	int32_t dy;

	dy = evt->y - evt->prevY;

	dy *= 3;
	dy /= 2;

	if (InfoTextDragPanel->rect.y + dy < miny)
	{
		laWidget_SetPosition((laWidget*)InfoTextDragPanel, InfoTextDragPanel->rect.x, miny);
	}
	else if (InfoTextDragPanel->rect.y + dy > maxy)
	{
		laWidget_SetPosition((laWidget*)InfoTextDragPanel, InfoTextDragPanel->rect.x, maxy);
	}
	else
	{
		laWidget_Translate((laWidget*)InfoTextDragPanel, 0, dy);
	}

	evt->event.accepted = LA_TRUE;
}

static void playSoundEffect()
{
    switch(appData.spriteState)
    {
        
        case RUN:
            DRV_CODEC_BufferAddWrite(appData.codecClient.handle, 
                        &(appData.codecClient.bufHandle),
                        (void*)RUN_SOUND_EFFECT, run_sound_size);
            break;
        case BLAZE:
            DRV_CODEC_BufferAddWrite(appData.codecClient.handle, 
                                    &(appData.codecClient.bufHandle),
                                    (void*)BLAZE_SOUND_EFFECT, blaze_sound_size);
            break;
        case HURT:
            DRV_CODEC_BufferAddWrite(appData.codecClient.handle, 
                        &(appData.codecClient.bufHandle),
                        (void*)HURT_SOUND_EFFECT, hurt_sound_size);
            break;
        case FALL:
            DRV_CODEC_BufferAddWrite(appData.codecClient.handle, 
                        &(appData.codecClient.bufHandle),
                        (void*)HURT_SOUND_EFFECT, hurt_sound_size);
            break;
        case DIZZY:
            DRV_CODEC_BufferAddWrite(appData.codecClient.handle, 
                        &(appData.codecClient.bufHandle),
                        (void*)DIZZY_SOUND_EFFECT, dizzy_sound_size);
            break;
        case IDLE:
        default:
            break;
    }
                
}
static void audioCodecBufferWriteEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            
        }
        
        break;
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
            Nop();
        } break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
            Nop();
        } break;

    }
}

void APP_SetScoreScheme(laScheme* scheme)
{    
    laWidget_SetScheme((laWidget*)Digit0, scheme);
    laWidget_SetScheme((laWidget*)Digit1, scheme);
    laWidget_SetScheme((laWidget*)Digit2, scheme);
    laWidget_SetScheme((laWidget*)Digit3, scheme);
    laWidget_SetScheme((laWidget*)Digit4, scheme);
    laWidget_SetScheme((laWidget*)Digit5, scheme);
}

void APP_SetScore(uint32_t score, bool topScore)
{
    if (score > APP_SCORE_MAX_COUNT)
    {
        score = APP_SCORE_MAX_COUNT;
        return;
    }

    uint32_t scoreMod = score % 10;
    uint32_t scoreMod10 = (score / 10) % 10;
    uint32_t scoreMod100 = (score / 100) % 10;
    uint32_t scoreMod1000 = (score / 1000) % 10;
    uint32_t scoreMod10000 = (score / 10000) % 10;
    uint32_t scoreMod100000 = (score / 100000) % 10;    
    
    switch (scoreMod)
    {
        case 0:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Zero));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_One));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_One));            
            break;
        case 2:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Two));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Two));            
            break;
        case 3:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Three));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Three));            
            break;
        case 4:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Four));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Four));            
            break;
        case 5:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Five));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Five));            
            break;
        case 6:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Six));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Six));            
            break;
        case 7:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Seven));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Eight));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            if (topScore)
                laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Nine));                            
            else
                laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (scoreMod10)
    {
        case 0:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Zero));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_One));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_One));            
            break;
        case 2:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Two));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Two));            
            break;
        case 3:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Three));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Three));            
            break;
        case 4:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Four));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Four));            
            break;
        case 5:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Five));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Five));            
            break;
        case 6:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Six));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Six));            
            break;
        case 7:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Seven));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Eight));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            if (topScore)
                laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Nine));                            
            else
                laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (scoreMod100)
    {
        case 0:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Zero));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_One));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_One));            
            break;
        case 2:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Two));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Two));            
            break;
        case 3:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Three));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Three));            
            break;
        case 4:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Four));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Four));            
            break;
        case 5:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Five));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Five));            
            break;
        case 6:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Six));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Six));            
            break;
        case 7:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Seven));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Eight));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            if (topScore)
                laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Nine));                            
            else
                laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (scoreMod1000)
    {
        case 0:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Zero));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_One));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_One));            
            break;
        case 2:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Two));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Two));            
            break;
        case 3:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Three));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Three));            
            break;
        case 4:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Four));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Four));            
            break;
        case 5:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Five));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Five));            
            break;
        case 6:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Six));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Six));            
            break;
        case 7:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Seven));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Eight));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            if (topScore)
                laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Nine));                            
            else
                laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (scoreMod10000)
    {
        case 0:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Zero));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_One));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_One));            
            break;
        case 2:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Two));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Two));            
            break;
        case 3:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Three));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Three));            
            break;
        case 4:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Four));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Four));            
            break;
        case 5:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Five));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Five));            
            break;
        case 6:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Six));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Six));            
            break;
        case 7:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Seven));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Eight));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            if (topScore)
                laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_Nine));                            
            else
                laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }

    switch (scoreMod100000)
    {
        case 0:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Zero));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Zero));            
            break;
        case 1:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_One));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_One));            
            break;
        case 2:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Two));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Two));            
            break;
        case 3:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Three));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Three));            
            break;
        case 4:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Four));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Four));            
            break;
        case 5:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Five));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Five));            
            break;
        case 6:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Six));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Six));            
            break;
        case 7:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Seven));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Seven));            
            break;
        case 8:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Eight));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Eight));            
            break;
        case 9:
            if (topScore)
                laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Nine));                            
            else
                laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Nine));            
            break;
        default:
            break;
    }
}


void APP_HideAllSequences(void)
{
    laWidget_SetVisible((laWidget*)RunLeftSequence, LA_FALSE);
    laImageSequenceWidget_Stop(RunLeftSequence);
    
    laWidget_SetVisible((laWidget*)RunRightSequence, LA_FALSE);
    laImageSequenceWidget_Stop(RunRightSequence);

    laWidget_SetVisible((laWidget*)IdleLeftSequence, LA_FALSE);
    laImageSequenceWidget_Stop(IdleLeftSequence);

    laWidget_SetVisible((laWidget*)IdleRightSequence, LA_FALSE);
    laImageSequenceWidget_Stop(IdleRightSequence);

    laWidget_SetVisible((laWidget*)BlazeLeftSequence, LA_FALSE);
    laImageSequenceWidget_Stop(BlazeLeftSequence);
    
    laWidget_SetVisible((laWidget*)BlazeRightSequence, LA_FALSE);
    laImageSequenceWidget_Stop(BlazeRightSequence);
    
    laWidget_SetVisible((laWidget*)DizzySequence, LA_FALSE);
    laImageSequenceWidget_Stop(DizzySequence);
    
    laWidget_SetVisible((laWidget*)DizzyLeftSequence, LA_FALSE);
    laImageSequenceWidget_Stop(DizzyLeftSequence);
    
    laWidget_SetVisible((laWidget*)FallSequence, LA_FALSE);
    laImageSequenceWidget_Stop(FallSequence);
    
    laWidget_SetVisible((laWidget*)FallLeftSequence, LA_FALSE);
    laImageSequenceWidget_Stop(FallLeftSequence);

    laWidget_SetVisible((laWidget*)HurtSequence, LA_FALSE);
    laImageSequenceWidget_Stop(HurtSequence);
    
    laWidget_SetVisible((laWidget*)HurtLeftSequence, LA_FALSE);
    laImageSequenceWidget_Stop(HurtLeftSequence);
}

void APP_PressCounter_CallBack ( uintptr_t context, uint32_t currTick )
{
    if (appData.requestedDirection != USER_REQUESTED_STOP            
            && ((appData.requestedDirection == USER_REQUESTED_GO_RIGHT
                && appData.spriteFacingDirection == FACING_RIGHT)
                ||(appData.requestedDirection == USER_REQUESTED_GO_LEFT
                && appData.spriteFacingDirection == FACING_LEFT))) 
    {
        appData.speedFactor += 1;        
    }
    else
    {
        appData.speedFactor -= 1;
    }
    
    appData.speedFactor = GFX_Clampi(1, APP_SPEED_FACTOR_LIMIT, appData.speedFactor);
}

void APP_SpriteSequence_CallBack( uintptr_t context, uint32_t currTick )
{
    if (laContext_GetActiveScreen() && 
        laContext_GetActiveScreen()->id != 1)
         return;
    
    appData.uiLayerAlpha--;
    if (appData.uiLayerAlpha < 0)
    {
        appData.uiLayerAlpha = 0;
    }
    laLayer_SetAlphaAmount(laContext_GetActiveScreen()->layers[2], appData.uiLayerAlpha);
    
    laWidget_SetVisible((laWidget*)appData.currentSequence, LA_TRUE);                
    if(appData.spriteTriggerCount-- <= 0)
    {
        switch(appData.spriteState)
        {
            case IDLE:
                appData.spriteTriggerCount = APP_IDLE_SPRITE_DELAY;
                break;
            case BLAZE:
                appData.spriteTriggerCount = APP_BLAZE_SPRITE_DELAY;
                break;
            case RUN:
                appData.spriteTriggerCount = (int32_t)APP_RUN_SPRITE_DELAY - (int32_t)appData.speedFactor;
                break;
            case HURT:
                if (appData.spriteFrameLeft > 1)
                {
                    appData.spriteFrameLeft--;
                }
                else
                {
                    appData.spriteFrameLeft = 0;
                }
                appData.spriteTriggerCount = APP_HURT_SPRITE_DELAY;
                break;
            case FALL:
                if (appData.spriteFrameLeft > 1)
                {
                    appData.spriteFrameLeft--;
                }
                else
                {
                    appData.spriteFrameLeft = 0;
                }
                appData.spriteTriggerCount = APP_FALL_SPRITE_DELAY;
                break;
            case DIZZY:
                if (appData.spriteFrameLeft > 1)
                {
                    appData.spriteFrameLeft--;
                }
                else
                {
                    appData.currentScore = 0;
                    appData.spriteState = IDLE;
                    appData.spriteFrameLeft = 0;
                    APP_SetScore(appData.currentScore, false);
                }
                appData.spriteTriggerCount = APP_DIZZY_SPRITE_DELAY;
                break;                
            default:
                appData.spriteTriggerCount = APP_IDLE_SPRITE_DELAY;
                break;            
        }
        
        laImageSequenceWidget_ShowNextImage(appData.currentSequence);
    }

    if (appData.spriteState != HURT
            && appData.spriteState != DIZZY
            && appData.spriteState != FALL
            && appData.spriteStamina <= APP_LAMB_FATIGUE_LIMIT
            && appData.isEasterEggMode == false)
    {
        appData.spriteState = HURT;
        appData.spriteFrameLeft = 100;
    }
    else
    {
        if (appData.requestedDirection == USER_REQUESTED_STOP 
                || appData.spriteState == IDLE
                || appData.spriteState == DIZZY
                || appData.spriteState == FALL)
        {
            appData.spriteStamina += 1;
            
            if (appData.spriteState == BLAZE && appData.spriteStamina > APP_LAMB_BLAZE_ZONE)
            {
                appData.speedFactor = APP_SPEED_FACTOR_LIMIT;
            }
        }
        else if (appData.spriteTriggerCount % 6 == 0)
        {
            if (appData.spriteState == RUN)
            {
                appData.spriteStamina -= 1;
                appData.currentScore += APP_SCORE_RUN_INCREMENT; 

                if (appData.spriteStamina < APP_LAMB_BLAZE_ZONE)
                {
                    appData.spriteStamina = APP_LAMB_BLAZE_ZONE;
                }
            }
            else if (appData.spriteState == BLAZE)
            {
                appData.spriteStamina -= 2;       
                appData.currentScore += APP_SCORE_BLAZE_INCREMENT;                
            }        

            APP_SetScore(appData.currentScore, false);
            if (appData.currentScore > appData.topScore)
            {
                appData.topScore = appData.currentScore;
                APP_SetScore(appData.topScore, true);            
            }
        }
        
        if (appData.spriteState == BLAZE)
        {
            APP_SetScoreScheme(&yellowScheme);                
        }
        else if (appData.spriteState == RUN)
        {
            APP_SetScoreScheme(&greenScheme);                
        }
        else
        {
            APP_SetScoreScheme(&defaultScheme);
        }
        
        appData.spriteStamina = GFX_Clampi(0, APP_LAMB_STAMINA, appData.spriteStamina);
        laCircularGaugeWidget_SetValue(CircularGaugeWidget, appData.spriteStamina);        
    }        
}

void APP_Right(void)
{
    if (appData.spriteState == DIZZY 
            || appData.spriteState == FALL
            || appData.spriteState == HURT)
    {
        appData.requestedDirection = USER_REQUESTED_STOP;
        return;
    }
    
    appData.requestedDirection = USER_REQUESTED_GO_RIGHT;
    
    if (appData.spriteFacingDirection == FACING_LEFT)
    {
        appData.speedFactor = 1;        
    }
}

void APP_Left(void)
{
    if (appData.spriteState == DIZZY 
            || appData.spriteState == FALL
            || appData.spriteState == HURT)
    {
        appData.requestedDirection = USER_REQUESTED_STOP;
        return;
    }
    
    appData.requestedDirection = USER_REQUESTED_GO_LEFT;

    if (appData.spriteFacingDirection == FACING_RIGHT)
    {
        appData.speedFactor = 1;        
    }
}

void APP_Stop(void)
{
    appData.uiLayerAlpha = APP_UI_LAYER_MAX_ALPHA;
    appData.lastRequestedDirection = appData.requestedDirection;
    appData.requestedDirection = USER_REQUESTED_STOP;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************
bool APP_Preload_Images(void)
{
	static uint32_t* currentAddress = (uint32_t*)APP_PRELOAD_IMAGE_DDR_ADDRESS;
	GFXU_ImageAsset* imageAssets[APP_NUM_IMAGE_PREPROCESS];
    GFX_Bool needsPad = GFX_FALSE;
	uint32_t imageSize;
	int i = 0;
	static bool cached = false;

	if (cached == true)
	{
		return cached;
	}

    imageAssets[0] = &Bar;
	imageAssets[1] = &BackLayer;
	imageAssets[2] = &MiddleLayer;
	imageAssets[3] = &FrontLayer;

	imageAssets[4] = &Run0;
	imageAssets[5] = &Run1;
	imageAssets[6] = &Run2;

	imageAssets[7] = &RunToLeft0;
	imageAssets[8] = &RunToLeft1;
	imageAssets[9] = &RunToLeft2;

	imageAssets[10] = &Idle0;
	imageAssets[11] = &Idle1;
	imageAssets[12] = &Idle2;
	imageAssets[13] = &Idle3;

	imageAssets[14] = &IdleLeft0;
	imageAssets[15] = &IdleLeft1;
	imageAssets[16] = &IdleLeft2;
	imageAssets[17] = &IdleLeft3;

	imageAssets[18] = &Blaze0;
	imageAssets[19] = &Blaze1;
	imageAssets[20] = &Blaze2;

	imageAssets[21] = &BlazeLeft0;
	imageAssets[22] = &BlazeLeft1;
	imageAssets[23] = &BlazeLeft2;

	imageAssets[24] = &Dizzy0;
	imageAssets[25] = &Dizzy1;
	imageAssets[26] = &Dizzy2;

	imageAssets[27] = &DizzyLeft0;
	imageAssets[28] = &DizzyLeft1;
	imageAssets[29] = &DizzyLeft2;

	imageAssets[30] = &Fall0;
	imageAssets[31] = &Fall1;
	imageAssets[32] = &Fall2;
	imageAssets[33] = &Fall3;
	imageAssets[34] = &Fall4;

	imageAssets[35] = &FallLeft0;
	imageAssets[36] = &FallLeft1;
	imageAssets[37] = &FallLeft2;
	imageAssets[38] = &FallLeft3;
	imageAssets[39] = &FallLeft4;

	imageAssets[40] = &Hurt0;
	imageAssets[41] = &Hurt1;
	imageAssets[42] = &Hurt2;
	imageAssets[43] = &Hurt3;

	imageAssets[44] = &HurtLeft0;
	imageAssets[45] = &HurtLeft1;
	imageAssets[46] = &HurtLeft2;
	imageAssets[47] = &HurtLeft3;

	imageAssets[48] = &mchp_logo;
	imageAssets[49] = &right_bw;
	imageAssets[50] = &left_bw;

    imageAssets[51] = &HarmonyLogo;
    imageAssets[52] = &info_text;
    
	for (i = 0; i < APP_NUM_IMAGE_PREPROCESS; ++i)
	{
        imageSize = imageAssets[i]->width * imageAssets[i]->height * 4;
        needsPad = GFX_FALSE;
        
        if (imageAssets[i] == &mchp_logo
                || imageAssets[i] == &HarmonyLogo
                || imageAssets[i] == &info_text)
        {
            needsPad = GFX_TRUE;
        }
        
        GFXU_PreprocessImage(imageAssets[i], (uint32_t)currentAddress, GFX_COLOR_MODE_RGBA_8888, needsPad);

        currentAddress += imageSize;
	}

	cached = true;

	return cached;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
void APP_PrepareLayers(void)
{
    appData.backgroundLayerWidth = laContext_GetActiveScreen()->layers[0]->widget.rect.width;
    appData.backgroundLayerHeight = laContext_GetActiveScreen()->layers[0]->widget.rect.height;

    appData.middleLayerWidth = laContext_GetActiveScreen()->layers[1]->widget.rect.width;
    appData.middleLayerHeight = laContext_GetActiveScreen()->layers[1]->widget.rect.height;

    appData.foregroundLayerWidth = laContext_GetActiveScreen()->layers[2]->widget.rect.width;
    appData.foregroundLayerHeight = laContext_GetActiveScreen()->layers[2]->widget.rect.height;

    appData.layerLimit0 = -1 * BackLayer.width;
    appData.layerLimit1 = -1 * MiddleLayer.width;
    appData.layerLimit2 = -1 * FrontLayer.width;  
    
    APP_SPRITE_ANCHOR_LEFT_LIMIT = appData.middleLayerWidth / 25;
    APP_SPRITE_ANCHOR_RIGHT_LIMIT = appData.middleLayerWidth - APP_SPRITE_ANCHOR_LEFT_LIMIT - SpriteAnchor->rect.width;
    APP_SPRITE_ANCHOR_INIT = appData.middleLayerWidth / 32 * 10;  
    
    if (appData.backgroundLayerWidth > 480)
    {
        APP_SPEED_FACTOR_LIMIT = 4;
        APP_LAYER_MOVE_FACTOR0 = 2;
        APP_LAYER_MOVE_FACTOR1 = 4;
        APP_LAYER_MOVE_FACTOR2 = 8;
        APP_BLAZE_SPEED_FACTOR_LIMIT = 20;
    }
}

void APP_HandleSpriteAnimation(void)
{
    if (appData.spriteState != HURT 
            && appData.spriteState != DIZZY  
            && appData.spriteState != FALL)
    {
        if (appData.requestedDirection == USER_REQUESTED_GO_RIGHT)
        {
            if (appData.spriteStamina <= APP_LAMB_BLAZE_ZONE
                    && appData.speedFactor >= APP_SPEED_FACTOR_LIMIT)
            {
                if (laImageSequenceWidget_IsPlaying(BlazeRightSequence) == LA_FALSE)
                {
                    APP_HideAllSequences();

                    laImageSequenceWidget_Play(BlazeRightSequence);
                    appData.currentSequence = BlazeRightSequence;
                    laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                    appData.speedFactor = APP_BLAZE_SPEED_FACTOR_LIMIT;
                    appData.spriteTriggerCount = APP_BLAZE_SPRITE_DELAY;
                    appData.spriteState = BLAZE;
                }                                    
            }
            else
            {
                if (laImageSequenceWidget_IsPlaying(RunRightSequence) == LA_FALSE)
                {
                    APP_HideAllSequences();

                    laImageSequenceWidget_Play(RunRightSequence);
                    appData.currentSequence = RunRightSequence;
                    laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                    appData.spriteTriggerCount = (int32_t)APP_RUN_SPRITE_DELAY - (int32_t)appData.speedFactor;
                    appData.spriteState = RUN;
                }                    
            }
        }
        else if (appData.requestedDirection == USER_REQUESTED_GO_LEFT)
        {
            if (appData.spriteStamina <= APP_LAMB_BLAZE_ZONE
                    && appData.speedFactor >= APP_SPEED_FACTOR_LIMIT)
            {
                if (laImageSequenceWidget_IsPlaying(BlazeLeftSequence) == LA_FALSE)
                {
                    APP_HideAllSequences();

                    laImageSequenceWidget_Play(BlazeLeftSequence);
                    appData.currentSequence = BlazeLeftSequence;
                    laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                    appData.speedFactor = APP_BLAZE_SPEED_FACTOR_LIMIT;
                    appData.spriteTriggerCount = APP_BLAZE_SPRITE_DELAY;
                    appData.spriteState = BLAZE;
                }                                    
            }
            else
            {
                if (laImageSequenceWidget_IsPlaying(RunLeftSequence) == LA_FALSE)
                {
                    APP_HideAllSequences();

                    laImageSequenceWidget_Play(RunLeftSequence);
                    appData.currentSequence = RunLeftSequence;
                    laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                    appData.spriteTriggerCount = (int32_t)APP_RUN_SPRITE_DELAY - (int32_t)appData.speedFactor;
                    appData.spriteState = RUN;
                }
            }
        }
        else if (appData.requestedDirection == USER_REQUESTED_STOP)
        {
            if (appData.spriteFacingDirection == FACING_LEFT)
            {
                if (appData.spriteAnchorX != APP_SPRITE_ANCHOR_INIT)
                {
                    if (laImageSequenceWidget_IsPlaying(RunLeftSequence) == LA_FALSE)
                    {
                        APP_HideAllSequences();

                        laImageSequenceWidget_Play(RunLeftSequence);
                        appData.currentSequence = RunLeftSequence;
                        laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                        appData.spriteTriggerCount = (int32_t)APP_RUN_SPRITE_DELAY - (int32_t)appData.speedFactor;
                        appData.spriteState = RUN;
                    }                    
                }
                else if (laImageSequenceWidget_IsPlaying(IdleLeftSequence) == LA_FALSE)
                {
                    APP_HideAllSequences();

                    laImageSequenceWidget_Play(IdleLeftSequence);
                    appData.currentSequence = IdleLeftSequence;
                    laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                    appData.spriteTriggerCount = APP_IDLE_SPRITE_DELAY;
                    appData.spriteState = IDLE;
                }
            }
            else
            {
                if (appData.spriteAnchorX != APP_SPRITE_ANCHOR_INIT)
                {
                    if (laImageSequenceWidget_IsPlaying(RunRightSequence) == LA_FALSE)
                    {
                        APP_HideAllSequences();

                        laImageSequenceWidget_Play(RunRightSequence);
                        appData.currentSequence = RunRightSequence;
                        laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                        appData.spriteTriggerCount = (int32_t)APP_RUN_SPRITE_DELAY - (int32_t)appData.speedFactor;
                        appData.spriteState = RUN;
                    }                    
                }
                else if (laImageSequenceWidget_IsPlaying(IdleRightSequence) == LA_FALSE)
                {
                    APP_HideAllSequences();

                    laImageSequenceWidget_Play(IdleRightSequence);
                    appData.currentSequence = IdleRightSequence;
                    laImageSequenceWidget_ShowNextImage(appData.currentSequence);
                    appData.spriteTriggerCount = APP_IDLE_SPRITE_DELAY;
                    appData.spriteState = IDLE;
                }
            }
        }
    }

    if (appData.spriteState == HURT && appData.spriteFrameLeft == 100)
    {
        if (appData.spriteFacingDirection == FACING_LEFT)
        {
            APP_HideAllSequences();

            laImageSequenceWidget_Play(HurtLeftSequence);
            appData.currentSequence = HurtLeftSequence;
            laImageSequenceWidget_ShowNextImage(appData.currentSequence);
            appData.spriteTriggerCount = APP_HURT_SPRITE_DELAY;
            appData.spriteFrameLeft = laImageSequenceWidget_GetImageCount(appData.currentSequence) * 3;
            appData.spriteState = HURT;
        }
        else
        {
            APP_HideAllSequences();

            laImageSequenceWidget_Play(HurtSequence);
            appData.currentSequence = HurtSequence;
            laImageSequenceWidget_ShowNextImage(appData.currentSequence);
            appData.spriteFrameLeft = laImageSequenceWidget_GetImageCount(appData.currentSequence) * 3;
            appData.spriteTriggerCount = APP_HURT_SPRITE_DELAY;
            appData.spriteState = HURT;            
        }
    }
    else if (appData.spriteState == HURT && appData.spriteFrameLeft == 0)
    {
        if (appData.spriteFacingDirection == FACING_LEFT)
        {
            APP_HideAllSequences();

            laImageSequenceWidget_Rewind(FallLeftSequence);
            laImageSequenceWidget_Play(FallLeftSequence);
            appData.currentSequence = FallLeftSequence;
            laImageSequenceWidget_ShowNextImage(appData.currentSequence);
            appData.spriteFrameLeft = laImageSequenceWidget_GetImageCount(appData.currentSequence);
            appData.spriteTriggerCount = APP_FALL_SPRITE_DELAY;
            appData.spriteState = FALL;
        }
        else
        {
            APP_HideAllSequences();

            laImageSequenceWidget_Rewind(FallSequence);
            laImageSequenceWidget_Play(FallSequence);
            appData.currentSequence = FallSequence;
            laImageSequenceWidget_ShowNextImage(appData.currentSequence);
            appData.spriteFrameLeft = laImageSequenceWidget_GetImageCount(appData.currentSequence);
            appData.spriteTriggerCount = APP_FALL_SPRITE_DELAY;
            appData.spriteState = FALL;            
        }
    }
    else if (appData.spriteState == FALL 
            && appData.spriteFrameLeft == 0
            && appData.spriteAnchorX == APP_SPRITE_ANCHOR_INIT)
    {
        if (appData.spriteFacingDirection == FACING_LEFT)
        {
            APP_HideAllSequences();

            laImageSequenceWidget_Play(DizzyLeftSequence);
            appData.currentSequence = DizzyLeftSequence;
            laImageSequenceWidget_ShowNextImage(appData.currentSequence);
            appData.spriteFrameLeft = laImageSequenceWidget_GetImageCount(appData.currentSequence) * 2;
            appData.spriteTriggerCount = APP_DIZZY_SPRITE_DELAY;
            appData.spriteState = DIZZY;
        }
        else
        {
            APP_HideAllSequences();

            laImageSequenceWidget_Play(DizzySequence);
            appData.currentSequence = DizzySequence;
            laImageSequenceWidget_ShowNextImage(appData.currentSequence);
            appData.spriteFrameLeft = laImageSequenceWidget_GetImageCount(appData.currentSequence) * 2;
            appData.spriteTriggerCount = APP_DIZZY_SPRITE_DELAY;
            appData.spriteState = DIZZY;            
        }
    }
}

void APP_GoToInfoState(void)
{
	appData.state = APP_STATE_INFO;
}

void APP_GoToMainState(void)
{
    appData.state = APP_STATE_SETUP_IMAGE;                                
    appData.spriteState = IDLE;
    appData.layerX0 = 0;
    appData.currentScore = 0;
    appData.updateTopScore = true;
}

void APP_ToggleEasterEgg(void)
{
    appData.isEasterEggMode = !appData.isEasterEggMode;    
}


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
    appData.spriteState = IDLE;
    appData.spriteStamina = APP_LAMB_STAMINA;;
    appData.spriteAnchorX = APP_SPRITE_ANCHOR_INIT;
    appData.speedFactor = 1;
    appData.uiLayerAlpha = APP_UI_LAYER_MAX_ALPHA;
    appData.spriteFrameLeft = -1; //Max Frames

    appData.isEasterEggMode = false;
    appData.currentScore = 0;
    appData.topScore = 10000;
    appData.updateTopScore = false;
    
    appData.codecClient.bufferEventHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) audioCodecBufferWriteEventHandler;
    
    run_sound_size = sizeof(RUN_SOUND_EFFECT);
    blaze_sound_size = sizeof(BLAZE_SOUND_EFFECT);
    hurt_sound_size = sizeof(HURT_SOUND_EFFECT);
    dizzy_sound_size = sizeof(DIZZY_SOUND_EFFECT);
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Tasks ( void )
{
    SYS_STATUS codecStatus;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                appData.state = APP_STATE_PRELOAD_IMAGES;
            }
            break;
        }

        case APP_STATE_PRELOAD_IMAGES:
        {
            if (APP_Preload_Images() == true)
            {
                appData.state = APP_STATE_AUDIO_CODEC_OPEN;                                
                appData.layerX0 = 0;
            }        
            break;
        }
        case APP_STATE_AUDIO_CODEC_OPEN:
            codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
            if (SYS_STATUS_READY == codecStatus) 
            {
                appData.codecClient.handle =
                    DRV_CODEC_Open(DRV_CODEC_INDEX_0,
                    DRV_IO_INTENT_WRITE|DRV_IO_INTENT_EXCLUSIVE);
                if (appData.codecClient.handle != DRV_HANDLE_INVALID) {
                    appData.state = APP_STATE_SPLASH;
                    DRV_CODEC_BufferEventHandlerSet(appData.codecClient.handle,
                                            appData.codecClient.bufferEventHandler,
                                            appData.codecClient.context);
                   
                } 
            }

            break;
        case APP_STATE_SPLASH:
        {
            if (laContext_GetActiveScreen() && 
                laContext_GetActiveScreen()->id != 0)
                 break;

            if (APP_IsSplashScreenComplete() == true)
            {
                laContext_SetActiveScreen(1);
                appData.state = APP_STATE_SETUP_IMAGE;                                
                appData.spriteState = IDLE;
                appData.layerX0 = 0;
                appData.currentScore = 0;

                SYS_TMR_ObjectDelete(appData.pressCountTimer);
                appData.pressCountTimer = SYS_TMR_CallbackPeriodic(APP_PRESS_COUNTER_DELAY, 1, APP_PressCounter_CallBack);                                    

                SYS_TMR_ObjectDelete(appData.spriteTimer);
                appData.spriteTimer = SYS_TMR_CallbackPeriodic(APP_SPRITE_TIMER_DELAY, 1, APP_SpriteSequence_CallBack);
            }
        }
        
        case APP_STATE_SETUP_IMAGE:
        {
            if (laContext_GetActiveScreen() && 
                laContext_GetActiveScreen()->id != 1)
                 break;

            APP_PrepareLayers();
            
            laWidget_SetPosition(SpriteAnchor, appData.spriteAnchorX, SpriteAnchor->rect.y);
            laWidget_SetPosition(BackPanel, appData.layerX0, BackPanel->rect.y);
            laWidget_SetPosition(MiddlePanel, appData.layerX1, MiddlePanel->rect.y);
            laWidget_SetPosition(FrontPanel, appData.layerX2, FrontPanel->rect.y);

            APP_HandleSpriteAnimation();
            
            appData.state = APP_STATE_RUNNING;                                

            break;
        }
        
        case APP_STATE_RUNNING:
        {
            appData.state = APP_STATE_SETUP_IMAGE;                                

            if (appData.updateTopScore == true)
            {
                APP_SetScore(appData.topScore, true);
                appData.updateTopScore = false;
            }
                        
            playSoundEffect();
            
            if (appData.spriteState == IDLE 
                    || appData.spriteState == DIZZY)
            {
                break;
            }

            if (appData.requestedDirection == USER_REQUESTED_GO_RIGHT && appData.spriteState != FALL)
            {
                appData.spriteFacingDirection = FACING_RIGHT;
                appData.spriteAnchorX += APP_SPRITE_STEP_X;

                appData.spriteAnchorX = GFX_Clampi(APP_SPRITE_ANCHOR_INIT, APP_SPRITE_ANCHOR_RIGHT_LIMIT, appData.spriteAnchorX);
            }
            else if (appData.requestedDirection == USER_REQUESTED_GO_LEFT && appData.spriteState != FALL)
            {
                appData.spriteFacingDirection = FACING_LEFT;
                appData.spriteAnchorX -= APP_SPRITE_STEP_X;
                
                appData.spriteAnchorX = GFX_Clampi(APP_SPRITE_ANCHOR_LEFT_LIMIT, APP_SPRITE_ANCHOR_INIT, appData.spriteAnchorX);
            }
            else if (appData.spriteState != HURT)
            {
                if (appData.spriteFacingDirection == FACING_LEFT)
                {
                    appData.spriteAnchorX += APP_SPRITE_STEP_X;                    
                    appData.spriteAnchorX = GFX_Clampi(APP_SPRITE_ANCHOR_LEFT_LIMIT, APP_SPRITE_ANCHOR_INIT, appData.spriteAnchorX);
                }
                else
                {
                    appData.spriteAnchorX -= APP_SPRITE_STEP_X;                                        
                    appData.spriteAnchorX = GFX_Clampi(APP_SPRITE_ANCHOR_INIT, APP_SPRITE_ANCHOR_RIGHT_LIMIT, appData.spriteAnchorX);
                }
            }

            if (appData.spriteFacingDirection == FACING_RIGHT)
            {                
                appData.layerX0 -= APP_LAYER_MOVE_FACTOR0 * appData.speedFactor; 
                appData.layerX1 -= APP_LAYER_MOVE_FACTOR1 * appData.speedFactor; 
                appData.layerX2 -= APP_LAYER_MOVE_FACTOR2 * appData.speedFactor; 
                
                if (appData.layerX0 < appData.layerLimit0)
                {
                    appData.layerX0 = 0;
                }

                if (appData.layerX1 < appData.layerLimit1)
                {
                    appData.layerX1 = 0;
                }

                if (appData.layerX2 < appData.layerLimit2)
                {
                    appData.layerX2 = 0;
                }
            }
            else if (appData.spriteFacingDirection == FACING_LEFT)
            {
                appData.layerX0 += APP_LAYER_MOVE_FACTOR0 * appData.speedFactor; 
                appData.layerX1 += APP_LAYER_MOVE_FACTOR1 * appData.speedFactor; 
                appData.layerX2 += APP_LAYER_MOVE_FACTOR2 * appData.speedFactor; 

                if (appData.layerX0 > 0)
                {
                    appData.layerX0 = appData.layerLimit0;
                }
                
                if (appData.layerX1 > 0)
                {
                    appData.layerX1 = appData.layerLimit1;
                }

                if (appData.layerX2 > 0)
                {
                    appData.layerX2 = appData.layerLimit2;
                }
            }
                    
            break;
        }

        case APP_STATE_INFO:
		{
			// Do not continue to trigger any redraw if any layer hasn't been completely drawn
			if (laContext_IsDrawing())
				break;

			if (laContext_GetActive()->activeScreen->id != 2)
				break;

			if (InfoTextDragPanel != NULL)
			{
				miny = 0 - InfoTextDragPanel->rect.height;
				maxy = 0;

				laWidget_OverrideTouchDownEvent((laWidget*)InfoTextDragPanel, &touchDown);
				laWidget_OverrideTouchMovedEvent((laWidget*)InfoTextDragPanel, &touchMovedUpDown);
				laWidget_OverrideTouchUpEvent((laWidget*)InfoTextDragPanel, &touchUp);
			}
			break;
		}

        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
