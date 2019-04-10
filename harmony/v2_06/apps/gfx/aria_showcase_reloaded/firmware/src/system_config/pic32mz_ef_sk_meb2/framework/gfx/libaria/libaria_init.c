/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_init.c

  Summary:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version 2.06
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

#include "gfx/libaria/libaria_init.h"

laScheme DarkRedBaseScheme;
laScheme DarkBlueBaseScheme;
laScheme MainMenuScheme;
laScheme BlueForegroundScheme;
laScheme BarGraphWidgetScheme;
laScheme defaultScheme;
laScheme CircularSliderScheme;
laScheme DataSeriesA;
laScheme GrayForegroundScheme;
laScheme DataSeriesB;
laScheme BlackBaseScheme;
laScheme RedlineScheme;
laScheme DarkGrayForegroundScheme;
laScheme LightBlueForegroundScheme;
laScheme PieChartDemoScheme;
laScheme MidGrayForegroundScheme;
laScheme BrightPinkBaseScheme;
laScheme CircularSliderDemoScheme;
laScheme WhiteForegroundScheme;
laScheme BrightPurpleBaseScheme;
laScheme BarGraphDemoScheme;
laScheme BlackBackgroundScheme;
laScheme CircularGaugeScheme;
laScheme DarkBlueForeGroundScheme;
laScheme BrightYellowBaseScheme;
laScheme CircularProgressBarScheme;
laScheme WhiteBackgroundScheme;
laScheme ArcDemoScheme;
laScheme LineGraphDemoScheme;
laScheme NewScheme;
laScheme BrightOrangeBaseScheme;
laScheme CircularGaugeDemoScheme;
laScheme WhiteBaseScheme;
laScheme BrightBlueBaseScheme;
laScheme LineGraphScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget2;
laWidget* PanelWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget4;
laWidget* PanelWidget3;
laWidget* PanelWidgetBackQ2;
laWidget* PanelWidget12;
laWidget* PanelWidget10;
laWidget* PanelWidget8;
laWidget* PanelWidget7;
laWidget* PanelWidget6;
laWidget* PanelWidget5;
laButtonWidget* ArcWidgetButton;
laButtonWidget* CircularSliderButton;
laButtonWidget* CircularGaugeButton;
laButtonWidget* PieChartButton;
laButtonWidget* BarGraphDemoButton;
laButtonWidget* LineGraphDemoButton;
laLabelWidget* LabelWidget28;
laLabelWidget* LabelWidget;
laLabelWidget* LabelWidget18;
laButtonWidget* StartArcDemoButtonWidget;
laArcWidget* ArcWidgetIn;
laArcWidget* ArcWidgetInMid;
laArcWidget* ArcWidgetMid;
laArcWidget* ArcWidgetOut;
laArcWidget* ArcWidgetPerimeter;
laButtonWidget* NextButton_ArcDemo;
laWidget* PanelWidget;
laWidget* PanelWidget9;
laImageWidget* ImageWidget10;
laWidget* ArcDemoMenuPanel;
laButtonWidget* HomeButton_ArcDemo;
laWidget* ArcHomeButtonBackPanel;
laWidget* ArcHomeButtonPanelWidget;
laImageWidget* ArcHomeButtonImage;
laImageWidget* TouchImage_ArcDemo;
laLabelWidget* LabelWidget2;
laLabelWidget* LabelWidget31;
laLabelWidget* CircularSliderValueLabel;
laCircularSliderWidget* CircularProgressBar;
laCircularSliderWidget* CircularSliderWidgetControl;
laButtonWidget* CircSlider_NextButton;
laWidget* PanelWidget17;
laWidget* PanelWidget18;
laImageWidget* ImageWidget19;
laButtonWidget* CircSlider_HomeButton;
laWidget* PanelWidget13;
laWidget* PanelWidget14;
laImageWidget* ImageWidget15;
laImageWidget* TouchImage_CircSliderDemo;
laLabelWidget* LabelWidget5;
laLabelWidget* LabelWidget41;
laButtonWidget* CircGauge_NextButton;
laWidget* PanelWidget25;
laWidget* PanelWidget26;
laImageWidget* ImageWidget27;
laLabelWidget* GaugeValueLabelWidget;
laLabelWidget* MphLabel;
laCircularGaugeWidget* CircularGaugeWidget43;
laCircularGaugeWidget* SpeedoCircularGaugeWidget;
laLabelWidget* LabelWidget49;
laLabelWidget* LabelWidget51;
laLabelWidget* LabelWidget53;
laLabelWidget* LabelWidget50;
laLabelWidget* LabelWidget54;
laLabelWidget* LabelWidget55;
laLabelWidget* LabelWidget56;
laLabelWidget* LabelWidget57;
laLabelWidget* LabelWidget59;
laLabelWidget* LabelWidget60;
laLabelWidget* LabelWidget58;
laButtonWidget* CircGauge_HomeButton;
laWidget* PanelWidget21;
laWidget* PanelWidget22;
laImageWidget* ImageWidget23;
laLabelWidget* LabelWidget1;
laButtonWidget* GasButton;
laImageWidget* TouchImage_CircGaugeDemo;
laLabelWidget* LabelWidget7;
laButtonWidget* PieChart_NextButton;
laWidget* PanelWidget33;
laWidget* PanelWidget34;
laImageWidget* ImageWidget35;
laLabelWidget* LabelWidget39;
laPieChartWidget* PieChartWidget2;
laButtonWidget* PieChart_HomeButton;
laWidget* PanelWidget29;
laWidget* PanelWidget30;
laImageWidget* ImageWidget31;
laImageWidget* TouchImage_PieChartDemo;
laLabelWidget* LabelWidget9;
laLabelWidget* LabelWidget4;
laBarGraphWidget* BarGraphWidget8;
laButtonWidget* BarGraphTouchedButton;
laImageWidget* TouchImage_BarGraphDemo;
laLabelWidget* LabelWidget12;
laButtonWidget* BarGraph_NextButton;
laWidget* PanelWidget41;
laWidget* PanelWidget42;
laImageWidget* ImageWidget43;
laButtonWidget* BarGraph_HomeButton;
laWidget* PanelWidget37;
laWidget* PanelWidget38;
laImageWidget* ImageWidget39;
laWidget* PanelWidget;
laLabelWidget* LabelWidget;
laWidget* PanelWidget4;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget11;
laLineGraphWidget* LineGraphWidget13;
laButtonWidget* LineGraphTouchedButton;
laImageWidget* TouchImage_LineGraphDemo;
laLabelWidget* LabelWidget14;
laButtonWidget* LineGraph_HomeButton;
laWidget* PanelWidget46;
laWidget* PanelWidget47;
laImageWidget* ImageWidget45;
laButtonWidget* LineGraph_NextButton;
laWidget* PanelWidget54;
laWidget* PanelWidget55;
laImageWidget* ImageWidget56;
laWidget* PanelWidget11;
laLabelWidget* LabelWidget13;
laWidget* PanelWidget15;
laLabelWidget* LabelWidget16;
laButtonWidget* CheckBoxPhantomButton;
laCheckBoxWidget* CheckBoxWidget17;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_MainMenu(laScreen* screen);
static void ScreenCreate_ArcWidgetDemo(laScreen* screen);
static void ScreenCreate_CircularSliderDemo(laScreen* screen);
static void ScreenCreate_CircularGaugeDemo(laScreen* screen);
static void ScreenCreate_PieChartDemo(laScreen* screen);
static void ScreenCreate_BarGraphDemo(laScreen* screen);
static void ScreenCreate_LineGraphScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&DarkRedBaseScheme, GFX_COLOR_MODE_RGB_565);
    DarkRedBaseScheme.base = 0x8000;
    DarkRedBaseScheme.highlight = 0xC67A;
    DarkRedBaseScheme.highlightLight = 0xFFFF;
    DarkRedBaseScheme.shadow = 0x8410;
    DarkRedBaseScheme.shadowDark = 0x4208;
    DarkRedBaseScheme.foreground = 0x0;
    DarkRedBaseScheme.foregroundInactive = 0xD71C;
    DarkRedBaseScheme.foregroundDisabled = 0x8410;
    DarkRedBaseScheme.background = 0xFFFF;
    DarkRedBaseScheme.backgroundInactive = 0xD71C;
    DarkRedBaseScheme.backgroundDisabled = 0xC67A;
    DarkRedBaseScheme.text = 0x0;
    DarkRedBaseScheme.textHighlight = 0x1F;
    DarkRedBaseScheme.textHighlightText = 0xFFFF;
    DarkRedBaseScheme.textInactive = 0xD71C;
    DarkRedBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&DarkBlueBaseScheme, GFX_COLOR_MODE_RGB_565);
    DarkBlueBaseScheme.base = 0x10;
    DarkBlueBaseScheme.highlight = 0xC67A;
    DarkBlueBaseScheme.highlightLight = 0xFFFF;
    DarkBlueBaseScheme.shadow = 0x8410;
    DarkBlueBaseScheme.shadowDark = 0x4208;
    DarkBlueBaseScheme.foreground = 0x0;
    DarkBlueBaseScheme.foregroundInactive = 0xD71C;
    DarkBlueBaseScheme.foregroundDisabled = 0x8410;
    DarkBlueBaseScheme.background = 0xFFFF;
    DarkBlueBaseScheme.backgroundInactive = 0xD71C;
    DarkBlueBaseScheme.backgroundDisabled = 0xC67A;
    DarkBlueBaseScheme.text = 0x0;
    DarkBlueBaseScheme.textHighlight = 0x1F;
    DarkBlueBaseScheme.textHighlightText = 0xFFFF;
    DarkBlueBaseScheme.textInactive = 0xD71C;
    DarkBlueBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&MainMenuScheme, GFX_COLOR_MODE_RGB_565);
    MainMenuScheme.base = 0x4B73;
    MainMenuScheme.highlight = 0xC67A;
    MainMenuScheme.highlightLight = 0xFFFF;
    MainMenuScheme.shadow = 0x8410;
    MainMenuScheme.shadowDark = 0x4208;
    MainMenuScheme.foreground = 0x0;
    MainMenuScheme.foregroundInactive = 0xD71C;
    MainMenuScheme.foregroundDisabled = 0x8410;
    MainMenuScheme.background = 0x95DB;
    MainMenuScheme.backgroundInactive = 0xD71C;
    MainMenuScheme.backgroundDisabled = 0xC67A;
    MainMenuScheme.text = 0xFFFF;
    MainMenuScheme.textHighlight = 0x1F;
    MainMenuScheme.textHighlightText = 0xFFFF;
    MainMenuScheme.textInactive = 0xD71C;
    MainMenuScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BlueForegroundScheme, GFX_COLOR_MODE_RGB_565);
    BlueForegroundScheme.base = 0xC67A;
    BlueForegroundScheme.highlight = 0xC67A;
    BlueForegroundScheme.highlightLight = 0xFFFF;
    BlueForegroundScheme.shadow = 0x8410;
    BlueForegroundScheme.shadowDark = 0x4208;
    BlueForegroundScheme.foreground = 0x4D9F;
    BlueForegroundScheme.foregroundInactive = 0xD71C;
    BlueForegroundScheme.foregroundDisabled = 0x8410;
    BlueForegroundScheme.background = 0xFFFF;
    BlueForegroundScheme.backgroundInactive = 0xD71C;
    BlueForegroundScheme.backgroundDisabled = 0xC67A;
    BlueForegroundScheme.text = 0x0;
    BlueForegroundScheme.textHighlight = 0x1F;
    BlueForegroundScheme.textHighlightText = 0xFFFF;
    BlueForegroundScheme.textInactive = 0xD71C;
    BlueForegroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BarGraphWidgetScheme, GFX_COLOR_MODE_RGB_565);
    BarGraphWidgetScheme.base = 0xDFDF;
    BarGraphWidgetScheme.highlight = 0xC67A;
    BarGraphWidgetScheme.highlightLight = 0xFFFF;
    BarGraphWidgetScheme.shadow = 0x8410;
    BarGraphWidgetScheme.shadowDark = 0x4208;
    BarGraphWidgetScheme.foreground = 0x17;
    BarGraphWidgetScheme.foregroundInactive = 0xD71C;
    BarGraphWidgetScheme.foregroundDisabled = 0x8410;
    BarGraphWidgetScheme.background = 0xFFFF;
    BarGraphWidgetScheme.backgroundInactive = 0xD71C;
    BarGraphWidgetScheme.backgroundDisabled = 0xC67A;
    BarGraphWidgetScheme.text = 0x10;
    BarGraphWidgetScheme.textHighlight = 0x1F;
    BarGraphWidgetScheme.textHighlightText = 0xFFFF;
    BarGraphWidgetScheme.textInactive = 0xD71C;
    BarGraphWidgetScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGB_565);
    defaultScheme.base = 0xC67A;
    defaultScheme.highlight = 0xC67A;
    defaultScheme.highlightLight = 0xFFFF;
    defaultScheme.shadow = 0x8410;
    defaultScheme.shadowDark = 0x4208;
    defaultScheme.foreground = 0x0;
    defaultScheme.foregroundInactive = 0xD71C;
    defaultScheme.foregroundDisabled = 0x8410;
    defaultScheme.background = 0xFFFF;
    defaultScheme.backgroundInactive = 0xD71C;
    defaultScheme.backgroundDisabled = 0xC67A;
    defaultScheme.text = 0x0;
    defaultScheme.textHighlight = 0x1F;
    defaultScheme.textHighlightText = 0xFFFF;
    defaultScheme.textInactive = 0xD71C;
    defaultScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&CircularSliderScheme, GFX_COLOR_MODE_RGB_565);
    CircularSliderScheme.base = 0x8200;
    CircularSliderScheme.highlight = 0xC67A;
    CircularSliderScheme.highlightLight = 0xFFFF;
    CircularSliderScheme.shadow = 0x8410;
    CircularSliderScheme.shadowDark = 0x4208;
    CircularSliderScheme.foreground = 0xFC00;
    CircularSliderScheme.foregroundInactive = 0xFEF7;
    CircularSliderScheme.foregroundDisabled = 0x8410;
    CircularSliderScheme.background = 0xFFFF;
    CircularSliderScheme.backgroundInactive = 0xD71C;
    CircularSliderScheme.backgroundDisabled = 0xC67A;
    CircularSliderScheme.text = 0x0;
    CircularSliderScheme.textHighlight = 0x1F;
    CircularSliderScheme.textHighlightText = 0xFFFF;
    CircularSliderScheme.textInactive = 0xD71C;
    CircularSliderScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&DataSeriesA, GFX_COLOR_MODE_RGB_565);
    DataSeriesA.base = 0xFC10;
    DataSeriesA.highlight = 0xC67A;
    DataSeriesA.highlightLight = 0xFFFF;
    DataSeriesA.shadow = 0x8410;
    DataSeriesA.shadowDark = 0x4208;
    DataSeriesA.foreground = 0x8000;
    DataSeriesA.foregroundInactive = 0xD71C;
    DataSeriesA.foregroundDisabled = 0x8410;
    DataSeriesA.background = 0xFFFF;
    DataSeriesA.backgroundInactive = 0xD71C;
    DataSeriesA.backgroundDisabled = 0xC67A;
    DataSeriesA.text = 0x0;
    DataSeriesA.textHighlight = 0x1F;
    DataSeriesA.textHighlightText = 0xFFFF;
    DataSeriesA.textInactive = 0xD71C;
    DataSeriesA.textDisabled = 0x8C92;

    laScheme_Initialize(&GrayForegroundScheme, GFX_COLOR_MODE_RGB_565);
    GrayForegroundScheme.base = 0xC67A;
    GrayForegroundScheme.highlight = 0xC67A;
    GrayForegroundScheme.highlightLight = 0xFFFF;
    GrayForegroundScheme.shadow = 0x8410;
    GrayForegroundScheme.shadowDark = 0x4208;
    GrayForegroundScheme.foreground = 0xB5B6;
    GrayForegroundScheme.foregroundInactive = 0xD71C;
    GrayForegroundScheme.foregroundDisabled = 0x8410;
    GrayForegroundScheme.background = 0xFFFF;
    GrayForegroundScheme.backgroundInactive = 0xD71C;
    GrayForegroundScheme.backgroundDisabled = 0xC67A;
    GrayForegroundScheme.text = 0x0;
    GrayForegroundScheme.textHighlight = 0x1F;
    GrayForegroundScheme.textHighlightText = 0xFFFF;
    GrayForegroundScheme.textInactive = 0xD71C;
    GrayForegroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&DataSeriesB, GFX_COLOR_MODE_RGB_565);
    DataSeriesB.base = 0xBDFF;
    DataSeriesB.highlight = 0xC67A;
    DataSeriesB.highlightLight = 0xFFFF;
    DataSeriesB.shadow = 0x8410;
    DataSeriesB.shadowDark = 0x4208;
    DataSeriesB.foreground = 0x10;
    DataSeriesB.foregroundInactive = 0xD71C;
    DataSeriesB.foregroundDisabled = 0x8410;
    DataSeriesB.background = 0xFFFF;
    DataSeriesB.backgroundInactive = 0xD71C;
    DataSeriesB.backgroundDisabled = 0xC67A;
    DataSeriesB.text = 0x0;
    DataSeriesB.textHighlight = 0x1F;
    DataSeriesB.textHighlightText = 0xFFFF;
    DataSeriesB.textInactive = 0xD71C;
    DataSeriesB.textDisabled = 0x8C92;

    laScheme_Initialize(&BlackBaseScheme, GFX_COLOR_MODE_RGB_565);
    BlackBaseScheme.base = 0x0;
    BlackBaseScheme.highlight = 0xC67A;
    BlackBaseScheme.highlightLight = 0xFFFF;
    BlackBaseScheme.shadow = 0x8410;
    BlackBaseScheme.shadowDark = 0x4208;
    BlackBaseScheme.foreground = 0x0;
    BlackBaseScheme.foregroundInactive = 0xD71C;
    BlackBaseScheme.foregroundDisabled = 0x8410;
    BlackBaseScheme.background = 0xFFFF;
    BlackBaseScheme.backgroundInactive = 0xD71C;
    BlackBaseScheme.backgroundDisabled = 0xC67A;
    BlackBaseScheme.text = 0x0;
    BlackBaseScheme.textHighlight = 0x1F;
    BlackBaseScheme.textHighlightText = 0xFFFF;
    BlackBaseScheme.textInactive = 0xD71C;
    BlackBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&RedlineScheme, GFX_COLOR_MODE_RGB_565);
    RedlineScheme.base = 0xC67A;
    RedlineScheme.highlight = 0xC67A;
    RedlineScheme.highlightLight = 0xFFFF;
    RedlineScheme.shadow = 0x8410;
    RedlineScheme.shadowDark = 0x4208;
    RedlineScheme.foreground = 0xBA00;
    RedlineScheme.foregroundInactive = 0xD71C;
    RedlineScheme.foregroundDisabled = 0x8410;
    RedlineScheme.background = 0xFFFF;
    RedlineScheme.backgroundInactive = 0xD71C;
    RedlineScheme.backgroundDisabled = 0xC67A;
    RedlineScheme.text = 0x0;
    RedlineScheme.textHighlight = 0x1F;
    RedlineScheme.textHighlightText = 0xFFFF;
    RedlineScheme.textInactive = 0xD71C;
    RedlineScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&DarkGrayForegroundScheme, GFX_COLOR_MODE_RGB_565);
    DarkGrayForegroundScheme.base = 0xC67A;
    DarkGrayForegroundScheme.highlight = 0xC67A;
    DarkGrayForegroundScheme.highlightLight = 0xFFFF;
    DarkGrayForegroundScheme.shadow = 0x8410;
    DarkGrayForegroundScheme.shadowDark = 0x4208;
    DarkGrayForegroundScheme.foreground = 0x5ACB;
    DarkGrayForegroundScheme.foregroundInactive = 0xD71C;
    DarkGrayForegroundScheme.foregroundDisabled = 0x8410;
    DarkGrayForegroundScheme.background = 0xFFFF;
    DarkGrayForegroundScheme.backgroundInactive = 0xD71C;
    DarkGrayForegroundScheme.backgroundDisabled = 0xC67A;
    DarkGrayForegroundScheme.text = 0x0;
    DarkGrayForegroundScheme.textHighlight = 0x1F;
    DarkGrayForegroundScheme.textHighlightText = 0xFFFF;
    DarkGrayForegroundScheme.textInactive = 0xD71C;
    DarkGrayForegroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&LightBlueForegroundScheme, GFX_COLOR_MODE_RGB_565);
    LightBlueForegroundScheme.base = 0xA6DF;
    LightBlueForegroundScheme.highlight = 0xC67A;
    LightBlueForegroundScheme.highlightLight = 0xFFFF;
    LightBlueForegroundScheme.shadow = 0x8410;
    LightBlueForegroundScheme.shadowDark = 0x4208;
    LightBlueForegroundScheme.foreground = 0xA6DF;
    LightBlueForegroundScheme.foregroundInactive = 0xD71C;
    LightBlueForegroundScheme.foregroundDisabled = 0x8410;
    LightBlueForegroundScheme.background = 0xFFFF;
    LightBlueForegroundScheme.backgroundInactive = 0xD71C;
    LightBlueForegroundScheme.backgroundDisabled = 0xC67A;
    LightBlueForegroundScheme.text = 0x0;
    LightBlueForegroundScheme.textHighlight = 0x1F;
    LightBlueForegroundScheme.textHighlightText = 0xFFFF;
    LightBlueForegroundScheme.textInactive = 0xD71C;
    LightBlueForegroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&PieChartDemoScheme, GFX_COLOR_MODE_RGB_565);
    PieChartDemoScheme.base = 0xF81F;
    PieChartDemoScheme.highlight = 0xC67A;
    PieChartDemoScheme.highlightLight = 0xFFFF;
    PieChartDemoScheme.shadow = 0x8410;
    PieChartDemoScheme.shadowDark = 0x4208;
    PieChartDemoScheme.foreground = 0x0;
    PieChartDemoScheme.foregroundInactive = 0xD71C;
    PieChartDemoScheme.foregroundDisabled = 0x8410;
    PieChartDemoScheme.background = 0xFDFF;
    PieChartDemoScheme.backgroundInactive = 0xD71C;
    PieChartDemoScheme.backgroundDisabled = 0xC67A;
    PieChartDemoScheme.text = 0x0;
    PieChartDemoScheme.textHighlight = 0x1F;
    PieChartDemoScheme.textHighlightText = 0xFFFF;
    PieChartDemoScheme.textInactive = 0xD71C;
    PieChartDemoScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&MidGrayForegroundScheme, GFX_COLOR_MODE_RGB_565);
    MidGrayForegroundScheme.base = 0xC67A;
    MidGrayForegroundScheme.highlight = 0xC67A;
    MidGrayForegroundScheme.highlightLight = 0xFFFF;
    MidGrayForegroundScheme.shadow = 0x8410;
    MidGrayForegroundScheme.shadowDark = 0x4208;
    MidGrayForegroundScheme.foreground = 0xDEFB;
    MidGrayForegroundScheme.foregroundInactive = 0xD71C;
    MidGrayForegroundScheme.foregroundDisabled = 0x8410;
    MidGrayForegroundScheme.background = 0xFFFF;
    MidGrayForegroundScheme.backgroundInactive = 0xD71C;
    MidGrayForegroundScheme.backgroundDisabled = 0xC67A;
    MidGrayForegroundScheme.text = 0x0;
    MidGrayForegroundScheme.textHighlight = 0x1F;
    MidGrayForegroundScheme.textHighlightText = 0xFFFF;
    MidGrayForegroundScheme.textInactive = 0xD71C;
    MidGrayForegroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BrightPinkBaseScheme, GFX_COLOR_MODE_RGB_565);
    BrightPinkBaseScheme.base = 0xF81F;
    BrightPinkBaseScheme.highlight = 0xC67A;
    BrightPinkBaseScheme.highlightLight = 0xFFFF;
    BrightPinkBaseScheme.shadow = 0x8410;
    BrightPinkBaseScheme.shadowDark = 0x4208;
    BrightPinkBaseScheme.foreground = 0xF81F;
    BrightPinkBaseScheme.foregroundInactive = 0xD71C;
    BrightPinkBaseScheme.foregroundDisabled = 0x8410;
    BrightPinkBaseScheme.background = 0xFFFF;
    BrightPinkBaseScheme.backgroundInactive = 0xD71C;
    BrightPinkBaseScheme.backgroundDisabled = 0xC67A;
    BrightPinkBaseScheme.text = 0x0;
    BrightPinkBaseScheme.textHighlight = 0x1F;
    BrightPinkBaseScheme.textHighlightText = 0xFFFF;
    BrightPinkBaseScheme.textInactive = 0xD71C;
    BrightPinkBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&CircularSliderDemoScheme, GFX_COLOR_MODE_RGB_565);
    CircularSliderDemoScheme.base = 0x841F;
    CircularSliderDemoScheme.highlight = 0xC67A;
    CircularSliderDemoScheme.highlightLight = 0xFFFF;
    CircularSliderDemoScheme.shadow = 0x8410;
    CircularSliderDemoScheme.shadowDark = 0x4208;
    CircularSliderDemoScheme.foreground = 0x0;
    CircularSliderDemoScheme.foregroundInactive = 0xD71C;
    CircularSliderDemoScheme.foregroundDisabled = 0x8410;
    CircularSliderDemoScheme.background = 0xBDFF;
    CircularSliderDemoScheme.backgroundInactive = 0xD71C;
    CircularSliderDemoScheme.backgroundDisabled = 0xC67A;
    CircularSliderDemoScheme.text = 0x0;
    CircularSliderDemoScheme.textHighlight = 0x1F;
    CircularSliderDemoScheme.textHighlightText = 0xFFFF;
    CircularSliderDemoScheme.textInactive = 0xD71C;
    CircularSliderDemoScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteForegroundScheme, GFX_COLOR_MODE_RGB_565);
    WhiteForegroundScheme.base = 0x0;
    WhiteForegroundScheme.highlight = 0xC67A;
    WhiteForegroundScheme.highlightLight = 0xFFFF;
    WhiteForegroundScheme.shadow = 0x8410;
    WhiteForegroundScheme.shadowDark = 0x4208;
    WhiteForegroundScheme.foreground = 0xFFFF;
    WhiteForegroundScheme.foregroundInactive = 0xD71C;
    WhiteForegroundScheme.foregroundDisabled = 0x8410;
    WhiteForegroundScheme.background = 0xFFFF;
    WhiteForegroundScheme.backgroundInactive = 0xD71C;
    WhiteForegroundScheme.backgroundDisabled = 0xC67A;
    WhiteForegroundScheme.text = 0x0;
    WhiteForegroundScheme.textHighlight = 0x1F;
    WhiteForegroundScheme.textHighlightText = 0xFFFF;
    WhiteForegroundScheme.textInactive = 0xD71C;
    WhiteForegroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BrightPurpleBaseScheme, GFX_COLOR_MODE_RGB_565);
    BrightPurpleBaseScheme.base = 0xA27F;
    BrightPurpleBaseScheme.highlight = 0xC67A;
    BrightPurpleBaseScheme.highlightLight = 0xFFFF;
    BrightPurpleBaseScheme.shadow = 0x8410;
    BrightPurpleBaseScheme.shadowDark = 0x4208;
    BrightPurpleBaseScheme.foreground = 0xB17F;
    BrightPurpleBaseScheme.foregroundInactive = 0xD71C;
    BrightPurpleBaseScheme.foregroundDisabled = 0x8410;
    BrightPurpleBaseScheme.background = 0xFFFF;
    BrightPurpleBaseScheme.backgroundInactive = 0xD71C;
    BrightPurpleBaseScheme.backgroundDisabled = 0xC67A;
    BrightPurpleBaseScheme.text = 0x0;
    BrightPurpleBaseScheme.textHighlight = 0x1F;
    BrightPurpleBaseScheme.textHighlightText = 0xFFFF;
    BrightPurpleBaseScheme.textInactive = 0xD71C;
    BrightPurpleBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BarGraphDemoScheme, GFX_COLOR_MODE_RGB_565);
    BarGraphDemoScheme.base = 0xFC00;
    BarGraphDemoScheme.highlight = 0xC67A;
    BarGraphDemoScheme.highlightLight = 0xFFFF;
    BarGraphDemoScheme.shadow = 0x8410;
    BarGraphDemoScheme.shadowDark = 0x4208;
    BarGraphDemoScheme.foreground = 0x0;
    BarGraphDemoScheme.foregroundInactive = 0xD71C;
    BarGraphDemoScheme.foregroundDisabled = 0x8410;
    BarGraphDemoScheme.background = 0xFEF7;
    BarGraphDemoScheme.backgroundInactive = 0xD71C;
    BarGraphDemoScheme.backgroundDisabled = 0xC67A;
    BarGraphDemoScheme.text = 0x0;
    BarGraphDemoScheme.textHighlight = 0x1F;
    BarGraphDemoScheme.textHighlightText = 0xFFFF;
    BarGraphDemoScheme.textInactive = 0xD71C;
    BarGraphDemoScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BlackBackgroundScheme, GFX_COLOR_MODE_RGB_565);
    BlackBackgroundScheme.base = 0xC67A;
    BlackBackgroundScheme.highlight = 0xC67A;
    BlackBackgroundScheme.highlightLight = 0xFFFF;
    BlackBackgroundScheme.shadow = 0x8410;
    BlackBackgroundScheme.shadowDark = 0x4208;
    BlackBackgroundScheme.foreground = 0x0;
    BlackBackgroundScheme.foregroundInactive = 0xD71C;
    BlackBackgroundScheme.foregroundDisabled = 0x8410;
    BlackBackgroundScheme.background = 0x0;
    BlackBackgroundScheme.backgroundInactive = 0xD71C;
    BlackBackgroundScheme.backgroundDisabled = 0xC67A;
    BlackBackgroundScheme.text = 0x0;
    BlackBackgroundScheme.textHighlight = 0x1F;
    BlackBackgroundScheme.textHighlightText = 0xFFFF;
    BlackBackgroundScheme.textInactive = 0xD71C;
    BlackBackgroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&CircularGaugeScheme, GFX_COLOR_MODE_RGB_565);
    CircularGaugeScheme.base = 0xC67A;
    CircularGaugeScheme.highlight = 0xC67A;
    CircularGaugeScheme.highlightLight = 0xFFFF;
    CircularGaugeScheme.shadow = 0x8410;
    CircularGaugeScheme.shadowDark = 0x4208;
    CircularGaugeScheme.foreground = 0xB800;
    CircularGaugeScheme.foregroundInactive = 0x0;
    CircularGaugeScheme.foregroundDisabled = 0x8410;
    CircularGaugeScheme.background = 0x0;
    CircularGaugeScheme.backgroundInactive = 0xD71C;
    CircularGaugeScheme.backgroundDisabled = 0xC67A;
    CircularGaugeScheme.text = 0x0;
    CircularGaugeScheme.textHighlight = 0x1F;
    CircularGaugeScheme.textHighlightText = 0xFFFF;
    CircularGaugeScheme.textInactive = 0xD71C;
    CircularGaugeScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&DarkBlueForeGroundScheme, GFX_COLOR_MODE_RGB_565);
    DarkBlueForeGroundScheme.base = 0x227F;
    DarkBlueForeGroundScheme.highlight = 0xC67A;
    DarkBlueForeGroundScheme.highlightLight = 0xFFFF;
    DarkBlueForeGroundScheme.shadow = 0x8410;
    DarkBlueForeGroundScheme.shadowDark = 0x4208;
    DarkBlueForeGroundScheme.foreground = 0x227F;
    DarkBlueForeGroundScheme.foregroundInactive = 0xD71C;
    DarkBlueForeGroundScheme.foregroundDisabled = 0x8410;
    DarkBlueForeGroundScheme.background = 0xFFFF;
    DarkBlueForeGroundScheme.backgroundInactive = 0xD71C;
    DarkBlueForeGroundScheme.backgroundDisabled = 0xC67A;
    DarkBlueForeGroundScheme.text = 0x0;
    DarkBlueForeGroundScheme.textHighlight = 0x1F;
    DarkBlueForeGroundScheme.textHighlightText = 0xFFFF;
    DarkBlueForeGroundScheme.textInactive = 0xD71C;
    DarkBlueForeGroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BrightYellowBaseScheme, GFX_COLOR_MODE_RGB_565);
    BrightYellowBaseScheme.base = 0xFFE0;
    BrightYellowBaseScheme.highlight = 0xC67A;
    BrightYellowBaseScheme.highlightLight = 0xFFFF;
    BrightYellowBaseScheme.shadow = 0x8410;
    BrightYellowBaseScheme.shadowDark = 0x4208;
    BrightYellowBaseScheme.foreground = 0xFFE0;
    BrightYellowBaseScheme.foregroundInactive = 0xD71C;
    BrightYellowBaseScheme.foregroundDisabled = 0x8410;
    BrightYellowBaseScheme.background = 0xFFFF;
    BrightYellowBaseScheme.backgroundInactive = 0xD71C;
    BrightYellowBaseScheme.backgroundDisabled = 0xC67A;
    BrightYellowBaseScheme.text = 0x0;
    BrightYellowBaseScheme.textHighlight = 0x1F;
    BrightYellowBaseScheme.textHighlightText = 0xFFFF;
    BrightYellowBaseScheme.textInactive = 0xD71C;
    BrightYellowBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&CircularProgressBarScheme, GFX_COLOR_MODE_RGB_565);
    CircularProgressBarScheme.base = 0x410;
    CircularProgressBarScheme.highlight = 0xC67A;
    CircularProgressBarScheme.highlightLight = 0xFFFF;
    CircularProgressBarScheme.shadow = 0x8410;
    CircularProgressBarScheme.shadowDark = 0x4208;
    CircularProgressBarScheme.foreground = 0x5F7;
    CircularProgressBarScheme.foregroundInactive = 0xBFFF;
    CircularProgressBarScheme.foregroundDisabled = 0x8410;
    CircularProgressBarScheme.background = 0xFFFF;
    CircularProgressBarScheme.backgroundInactive = 0xD71C;
    CircularProgressBarScheme.backgroundDisabled = 0xC67A;
    CircularProgressBarScheme.text = 0x0;
    CircularProgressBarScheme.textHighlight = 0x1F;
    CircularProgressBarScheme.textHighlightText = 0xFFFF;
    CircularProgressBarScheme.textInactive = 0xD71C;
    CircularProgressBarScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteBackgroundScheme, GFX_COLOR_MODE_RGB_565);
    WhiteBackgroundScheme.base = 0xFFFF;
    WhiteBackgroundScheme.highlight = 0xC67A;
    WhiteBackgroundScheme.highlightLight = 0xFFFF;
    WhiteBackgroundScheme.shadow = 0x8410;
    WhiteBackgroundScheme.shadowDark = 0x4208;
    WhiteBackgroundScheme.foreground = 0x0;
    WhiteBackgroundScheme.foregroundInactive = 0xD71C;
    WhiteBackgroundScheme.foregroundDisabled = 0x8410;
    WhiteBackgroundScheme.background = 0xFFFF;
    WhiteBackgroundScheme.backgroundInactive = 0xD71C;
    WhiteBackgroundScheme.backgroundDisabled = 0xC67A;
    WhiteBackgroundScheme.text = 0x0;
    WhiteBackgroundScheme.textHighlight = 0x1F;
    WhiteBackgroundScheme.textHighlightText = 0xFFFF;
    WhiteBackgroundScheme.textInactive = 0xD71C;
    WhiteBackgroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&ArcDemoScheme, GFX_COLOR_MODE_RGB_565);
    ArcDemoScheme.base = 0x5E0;
    ArcDemoScheme.highlight = 0xC67A;
    ArcDemoScheme.highlightLight = 0xFFFF;
    ArcDemoScheme.shadow = 0x8410;
    ArcDemoScheme.shadowDark = 0x4208;
    ArcDemoScheme.foreground = 0x0;
    ArcDemoScheme.foregroundInactive = 0xD71C;
    ArcDemoScheme.foregroundDisabled = 0x8410;
    ArcDemoScheme.background = 0xBFF7;
    ArcDemoScheme.backgroundInactive = 0xD71C;
    ArcDemoScheme.backgroundDisabled = 0xC67A;
    ArcDemoScheme.text = 0x0;
    ArcDemoScheme.textHighlight = 0x1F;
    ArcDemoScheme.textHighlightText = 0xFFFF;
    ArcDemoScheme.textInactive = 0xD71C;
    ArcDemoScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&LineGraphDemoScheme, GFX_COLOR_MODE_RGB_565);
    LineGraphDemoScheme.base = 0xB2DF;
    LineGraphDemoScheme.highlight = 0xC67A;
    LineGraphDemoScheme.highlightLight = 0xFFFF;
    LineGraphDemoScheme.shadow = 0x8410;
    LineGraphDemoScheme.shadowDark = 0x4208;
    LineGraphDemoScheme.foreground = 0x0;
    LineGraphDemoScheme.foregroundInactive = 0xD71C;
    LineGraphDemoScheme.foregroundDisabled = 0x8410;
    LineGraphDemoScheme.background = 0xD65F;
    LineGraphDemoScheme.backgroundInactive = 0xD71C;
    LineGraphDemoScheme.backgroundDisabled = 0xC67A;
    LineGraphDemoScheme.text = 0x0;
    LineGraphDemoScheme.textHighlight = 0x1F;
    LineGraphDemoScheme.textHighlightText = 0xFFFF;
    LineGraphDemoScheme.textInactive = 0xD71C;
    LineGraphDemoScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&NewScheme, GFX_COLOR_MODE_RGB_565);
    NewScheme.base = 0xF81F;
    NewScheme.highlight = 0xC67A;
    NewScheme.highlightLight = 0xFFFF;
    NewScheme.shadow = 0x8410;
    NewScheme.shadowDark = 0x4208;
    NewScheme.foreground = 0xFDFF;
    NewScheme.foregroundInactive = 0xD71C;
    NewScheme.foregroundDisabled = 0x8410;
    NewScheme.background = 0xFFFF;
    NewScheme.backgroundInactive = 0xD71C;
    NewScheme.backgroundDisabled = 0xC67A;
    NewScheme.text = 0x0;
    NewScheme.textHighlight = 0x1F;
    NewScheme.textHighlightText = 0xFFFF;
    NewScheme.textInactive = 0xD71C;
    NewScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BrightOrangeBaseScheme, GFX_COLOR_MODE_RGB_565);
    BrightOrangeBaseScheme.base = 0xFC00;
    BrightOrangeBaseScheme.highlight = 0xC67A;
    BrightOrangeBaseScheme.highlightLight = 0xFFFF;
    BrightOrangeBaseScheme.shadow = 0x8410;
    BrightOrangeBaseScheme.shadowDark = 0x4208;
    BrightOrangeBaseScheme.foreground = 0xFC00;
    BrightOrangeBaseScheme.foregroundInactive = 0xD71C;
    BrightOrangeBaseScheme.foregroundDisabled = 0x8410;
    BrightOrangeBaseScheme.background = 0xFFFF;
    BrightOrangeBaseScheme.backgroundInactive = 0xD71C;
    BrightOrangeBaseScheme.backgroundDisabled = 0xC67A;
    BrightOrangeBaseScheme.text = 0x0;
    BrightOrangeBaseScheme.textHighlight = 0x1F;
    BrightOrangeBaseScheme.textHighlightText = 0xFFFF;
    BrightOrangeBaseScheme.textInactive = 0xD71C;
    BrightOrangeBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&CircularGaugeDemoScheme, GFX_COLOR_MODE_RGB_565);
    CircularGaugeDemoScheme.base = 0xFFE0;
    CircularGaugeDemoScheme.highlight = 0xC67A;
    CircularGaugeDemoScheme.highlightLight = 0xFFFF;
    CircularGaugeDemoScheme.shadow = 0x8410;
    CircularGaugeDemoScheme.shadowDark = 0x4208;
    CircularGaugeDemoScheme.foreground = 0x0;
    CircularGaugeDemoScheme.foregroundInactive = 0xD71C;
    CircularGaugeDemoScheme.foregroundDisabled = 0x8410;
    CircularGaugeDemoScheme.background = 0xFFF7;
    CircularGaugeDemoScheme.backgroundInactive = 0xD71C;
    CircularGaugeDemoScheme.backgroundDisabled = 0xC67A;
    CircularGaugeDemoScheme.text = 0x0;
    CircularGaugeDemoScheme.textHighlight = 0x1F;
    CircularGaugeDemoScheme.textHighlightText = 0xFFFF;
    CircularGaugeDemoScheme.textInactive = 0xD71C;
    CircularGaugeDemoScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteBaseScheme, GFX_COLOR_MODE_RGB_565);
    WhiteBaseScheme.base = 0xFFFF;
    WhiteBaseScheme.highlight = 0xC67A;
    WhiteBaseScheme.highlightLight = 0xFFFF;
    WhiteBaseScheme.shadow = 0x8410;
    WhiteBaseScheme.shadowDark = 0x4208;
    WhiteBaseScheme.foreground = 0x0;
    WhiteBaseScheme.foregroundInactive = 0xD71C;
    WhiteBaseScheme.foregroundDisabled = 0x8410;
    WhiteBaseScheme.background = 0xFFFF;
    WhiteBaseScheme.backgroundInactive = 0xD71C;
    WhiteBaseScheme.backgroundDisabled = 0xC67A;
    WhiteBaseScheme.text = 0x0;
    WhiteBaseScheme.textHighlight = 0x1F;
    WhiteBaseScheme.textHighlightText = 0xFFFF;
    WhiteBaseScheme.textInactive = 0xD71C;
    WhiteBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BrightBlueBaseScheme, GFX_COLOR_MODE_RGB_565);
    BrightBlueBaseScheme.base = 0x7FF;
    BrightBlueBaseScheme.highlight = 0xC67A;
    BrightBlueBaseScheme.highlightLight = 0xFFFF;
    BrightBlueBaseScheme.shadow = 0x8410;
    BrightBlueBaseScheme.shadowDark = 0x4208;
    BrightBlueBaseScheme.foreground = 0x7FF;
    BrightBlueBaseScheme.foregroundInactive = 0xD71C;
    BrightBlueBaseScheme.foregroundDisabled = 0x8410;
    BrightBlueBaseScheme.background = 0xFFFF;
    BrightBlueBaseScheme.backgroundInactive = 0xD71C;
    BrightBlueBaseScheme.backgroundDisabled = 0xC67A;
    BrightBlueBaseScheme.text = 0x0;
    BrightBlueBaseScheme.textHighlight = 0x1F;
    BrightBlueBaseScheme.textHighlightText = 0xFFFF;
    BrightBlueBaseScheme.textInactive = 0xD71C;
    BrightBlueBaseScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&LineGraphScheme, GFX_COLOR_MODE_RGB_565);
    LineGraphScheme.base = 0xD598;
    LineGraphScheme.highlight = 0xC67A;
    LineGraphScheme.highlightLight = 0xFFFF;
    LineGraphScheme.shadow = 0x8410;
    LineGraphScheme.shadowDark = 0x4208;
    LineGraphScheme.foreground = 0x8000;
    LineGraphScheme.foregroundInactive = 0xD71C;
    LineGraphScheme.foregroundDisabled = 0x8410;
    LineGraphScheme.background = 0xFFFF;
    LineGraphScheme.backgroundInactive = 0xD71C;
    LineGraphScheme.backgroundDisabled = 0xC67A;
    LineGraphScheme.text = 0x4000;
    LineGraphScheme.textHighlight = 0x1F;
    LineGraphScheme.textHighlightText = 0xFFFF;
    LineGraphScheme.textInactive = 0xD71C;
    LineGraphScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainMenu);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_ArcWidgetDemo);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_CircularSliderDemo);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_CircularGaugeDemo);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_PieChartDemo);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_BarGraphDemo);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_LineGraphScreen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_SplashScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget2, 240, 139);
    laWidget_SetVisible((laWidget*)ImageWidget2, LA_FALSE);
    laWidget_SetScheme((laWidget*)ImageWidget2, &WhiteBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget2);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 0, 207);
    laWidget_SetSize((laWidget*)PanelWidget1, 480, 65);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 480, 0);
    laWidget_SetSize((laWidget*)ImageWidget3, 480, 65);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)ImageWidget3);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 17, 23);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
    laWidget_SetVisible((laWidget*)ImageWidget4, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)ImageWidget4);

}

static void ScreenCreate_MainMenu(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &MainMenuScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    PanelWidget3 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget3, 240, 136);
    laWidget_SetSize((laWidget*)PanelWidget3, 240, 136);
    laWidget_SetScheme((laWidget*)PanelWidget3, &MainMenuScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget3, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget3);

    PanelWidgetBackQ2 = laWidget_New();
    laWidget_SetSize((laWidget*)PanelWidgetBackQ2, 240, 136);
    laWidget_SetScheme((laWidget*)PanelWidgetBackQ2, &MainMenuScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidgetBackQ2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidgetBackQ2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidgetBackQ2);

    PanelWidget12 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget12, 135, 154);
    laWidget_SetSize((laWidget*)PanelWidget12, 100, 100);
    laWidget_SetScheme((laWidget*)PanelWidget12, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget12, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget12, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget12);

    PanelWidget10 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget10, 24, 154);
    laWidget_SetSize((laWidget*)PanelWidget10, 100, 100);
    laWidget_SetScheme((laWidget*)PanelWidget10, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget10, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget10, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget10);

    PanelWidget8 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget8, 365, 24);
    laWidget_SetSize((laWidget*)PanelWidget8, 100, 100);
    laWidget_SetScheme((laWidget*)PanelWidget8, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget8, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget8);

    PanelWidget7 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget7, 254, 25);
    laWidget_SetSize((laWidget*)PanelWidget7, 100, 100);
    laWidget_SetScheme((laWidget*)PanelWidget7, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget7, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget7);

    PanelWidget6 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget6, 135, 24);
    laWidget_SetSize((laWidget*)PanelWidget6, 100, 100);
    laWidget_SetScheme((laWidget*)PanelWidget6, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget6, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget6);

    PanelWidget5 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget5, 24, 24);
    laWidget_SetSize((laWidget*)PanelWidget5, 100, 100);
    laWidget_SetScheme((laWidget*)PanelWidget5, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget5, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget5);

    ArcWidgetButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ArcWidgetButton, 20, 20);
    laWidget_SetSize((laWidget*)ArcWidgetButton, 100, 100);
    laWidget_SetScheme((laWidget*)ArcWidgetButton, &ArcDemoScheme);
    laWidget_SetBackgroundType((laWidget*)ArcWidgetButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ArcWidgetButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ArcWidgetButton, &ArcDemoIconDrop);
    laButtonWidget_SetReleasedImage(ArcWidgetButton, &ArcDemoIconDrop);
    laButtonWidget_SetReleasedEventCallback(ArcWidgetButton, &ArcWidgetButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ArcWidgetButton);

    CircularSliderButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CircularSliderButton, 131, 20);
    laWidget_SetSize((laWidget*)CircularSliderButton, 100, 100);
    laWidget_SetScheme((laWidget*)CircularSliderButton, &CircularSliderDemoScheme);
    laWidget_SetBackgroundType((laWidget*)CircularSliderButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CircularSliderButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(CircularSliderButton, &CircularSliderDrop);
    laButtonWidget_SetReleasedImage(CircularSliderButton, &CircularSliderDrop);
    laButtonWidget_SetReleasedEventCallback(CircularSliderButton, &CircularSliderButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularSliderButton);

    CircularGaugeButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CircularGaugeButton, 250, 20);
    laWidget_SetSize((laWidget*)CircularGaugeButton, 100, 100);
    laWidget_SetScheme((laWidget*)CircularGaugeButton, &CircularGaugeDemoScheme);
    laWidget_SetBackgroundType((laWidget*)CircularGaugeButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CircularGaugeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(CircularGaugeButton, &CircularGaugeDrop);
    laButtonWidget_SetReleasedImage(CircularGaugeButton, &CircularGaugeDrop);
    laButtonWidget_SetReleasedEventCallback(CircularGaugeButton, &CircularGaugeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularGaugeButton);

    PieChartButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PieChartButton, 361, 20);
    laWidget_SetSize((laWidget*)PieChartButton, 100, 100);
    laWidget_SetScheme((laWidget*)PieChartButton, &PieChartDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PieChartButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PieChartButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(PieChartButton, &PieChartDrop);
    laButtonWidget_SetReleasedImage(PieChartButton, &PieChartDrop);
    laButtonWidget_SetReleasedEventCallback(PieChartButton, &PieChartButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PieChartButton);

    BarGraphDemoButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)BarGraphDemoButton, 20, 150);
    laWidget_SetSize((laWidget*)BarGraphDemoButton, 100, 100);
    laWidget_SetScheme((laWidget*)BarGraphDemoButton, &BarGraphDemoScheme);
    laWidget_SetBackgroundType((laWidget*)BarGraphDemoButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BarGraphDemoButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(BarGraphDemoButton, &BarGraphDrop);
    laButtonWidget_SetReleasedImage(BarGraphDemoButton, &BarGraphDrop);
    laButtonWidget_SetReleasedEventCallback(BarGraphDemoButton, &BarGraphDemoButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)BarGraphDemoButton);

    LineGraphDemoButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)LineGraphDemoButton, 131, 150);
    laWidget_SetSize((laWidget*)LineGraphDemoButton, 100, 100);
    laWidget_SetScheme((laWidget*)LineGraphDemoButton, &LineGraphDemoScheme);
    laWidget_SetBackgroundType((laWidget*)LineGraphDemoButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LineGraphDemoButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(LineGraphDemoButton, &LineGraphDrop);
    laButtonWidget_SetReleasedImage(LineGraphDemoButton, &LineGraphDrop);
    laButtonWidget_SetReleasedEventCallback(LineGraphDemoButton, &LineGraphDemoButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineGraphDemoButton);

    LabelWidget28 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget28, 339, 188);
    laWidget_SetSize((laWidget*)LabelWidget28, 130, 73);
    laWidget_SetScheme((laWidget*)LabelWidget28, &MainMenuScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget28, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget28, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget28, laString_CreateFromID(string_AriaShowcase));
    laLabelWidget_SetHAlignment(LabelWidget28, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget28);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 368, 226);
    laWidget_SetSize((laWidget*)LabelWidget, 100, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_Reloaded));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laLabelWidget_SetVAlignment(LabelWidget, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

}

static void ScreenCreate_ArcWidgetDemo(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &ArcDemoScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget18 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget18, 112, 15);
    laWidget_SetSize((laWidget*)LabelWidget18, 256, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget18, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget18, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget18, laString_CreateFromID(string_ArcPrimitiveDemo));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget18);

    StartArcDemoButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StartArcDemoButtonWidget, 0, 50);
    laWidget_SetSize((laWidget*)StartArcDemoButtonWidget, 479, 220);
    laWidget_SetBackgroundType((laWidget*)StartArcDemoButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)StartArcDemoButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedEventCallback(StartArcDemoButtonWidget, &StartArcDemoButtonWidget_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(StartArcDemoButtonWidget, &StartArcDemoButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)StartArcDemoButtonWidget);

    ArcWidgetIn = laArcWidget_New();
    laWidget_SetPosition((laWidget*)ArcWidgetIn, 170, 100);
    laWidget_SetSize((laWidget*)ArcWidgetIn, 120, 120);
    laWidget_SetEnabled((laWidget*)ArcWidgetIn, LA_FALSE);
    laWidget_SetScheme((laWidget*)ArcWidgetIn, &GrayForegroundScheme);
    laWidget_SetBackgroundType((laWidget*)ArcWidgetIn, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcWidgetIn, LA_WIDGET_BORDER_NONE);
    laArcWidget_SetRadius(ArcWidgetIn, 60);
    laArcWidget_SetStartAngle(ArcWidgetIn, 225);
    laArcWidget_SetCenterAngle(ArcWidgetIn, 270);
    laArcWidget_SetThickness(ArcWidgetIn, 5);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ArcWidgetIn);

    ArcWidgetInMid = laArcWidget_New();
    laWidget_SetPosition((laWidget*)ArcWidgetInMid, 160, 90);
    laWidget_SetSize((laWidget*)ArcWidgetInMid, 140, 140);
    laWidget_SetEnabled((laWidget*)ArcWidgetInMid, LA_FALSE);
    laWidget_SetScheme((laWidget*)ArcWidgetInMid, &BrightOrangeBaseScheme);
    laWidget_SetBackgroundType((laWidget*)ArcWidgetInMid, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcWidgetInMid, LA_WIDGET_BORDER_NONE);
    laArcWidget_SetRadius(ArcWidgetInMid, 65);
    laArcWidget_SetStartAngle(ArcWidgetInMid, 145);
    laArcWidget_SetCenterAngle(ArcWidgetInMid, 70);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ArcWidgetInMid);

    ArcWidgetMid = laArcWidget_New();
    laWidget_SetPosition((laWidget*)ArcWidgetMid, 140, 70);
    laWidget_SetSize((laWidget*)ArcWidgetMid, 180, 180);
    laWidget_SetEnabled((laWidget*)ArcWidgetMid, LA_FALSE);
    laWidget_SetScheme((laWidget*)ArcWidgetMid, &MidGrayForegroundScheme);
    laWidget_SetBackgroundType((laWidget*)ArcWidgetMid, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcWidgetMid, LA_WIDGET_BORDER_NONE);
    laArcWidget_SetRadius(ArcWidgetMid, 85);
    laArcWidget_SetStartAngle(ArcWidgetMid, 45);
    laArcWidget_SetCenterAngle(ArcWidgetMid, 270);
    laArcWidget_SetThickness(ArcWidgetMid, 3);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ArcWidgetMid);

    ArcWidgetOut = laArcWidget_New();
    laWidget_SetPosition((laWidget*)ArcWidgetOut, 130, 60);
    laWidget_SetSize((laWidget*)ArcWidgetOut, 200, 200);
    laWidget_SetEnabled((laWidget*)ArcWidgetOut, LA_FALSE);
    laWidget_SetScheme((laWidget*)ArcWidgetOut, &BrightOrangeBaseScheme);
    laWidget_SetBackgroundType((laWidget*)ArcWidgetOut, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcWidgetOut, LA_WIDGET_BORDER_NONE);
    laArcWidget_SetRadius(ArcWidgetOut, 92);
    laArcWidget_SetStartAngle(ArcWidgetOut, 45);
    laArcWidget_SetCenterAngle(ArcWidgetOut, -90);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ArcWidgetOut);

    ArcWidgetPerimeter = laArcWidget_New();
    laWidget_SetPosition((laWidget*)ArcWidgetPerimeter, 120, 50);
    laWidget_SetSize((laWidget*)ArcWidgetPerimeter, 220, 220);
    laWidget_SetEnabled((laWidget*)ArcWidgetPerimeter, LA_FALSE);
    laWidget_SetScheme((laWidget*)ArcWidgetPerimeter, &BlueForegroundScheme);
    laWidget_SetBackgroundType((laWidget*)ArcWidgetPerimeter, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcWidgetPerimeter, LA_WIDGET_BORDER_NONE);
    laArcWidget_SetRadius(ArcWidgetPerimeter, 105);
    laArcWidget_SetStartAngle(ArcWidgetPerimeter, 90);
    laArcWidget_SetCenterAngle(ArcWidgetPerimeter, 360);
    laArcWidget_SetThickness(ArcWidgetPerimeter, 5);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ArcWidgetPerimeter);

    NextButton_ArcDemo = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)NextButton_ArcDemo, 420, 0);
    laWidget_SetSize((laWidget*)NextButton_ArcDemo, 60, 60);
    laWidget_SetBackgroundType((laWidget*)NextButton_ArcDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)NextButton_ArcDemo, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(NextButton_ArcDemo, &NextButton_ArcDemo_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)NextButton_ArcDemo);

    PanelWidget = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget, 15, 19);
    laWidget_SetSize((laWidget*)PanelWidget, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)NextButton_ArcDemo, PanelWidget);

    PanelWidget9 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget9, 19, 15);
    laWidget_SetSize((laWidget*)PanelWidget9, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget9, &ArcDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget9, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget9, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)NextButton_ArcDemo, PanelWidget9);

    ImageWidget10 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget10, 19, 15);
    laWidget_SetSize((laWidget*)ImageWidget10, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget10, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget10, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget10, &NextButton30x30Glow);
    laWidget_AddChild((laWidget*)NextButton_ArcDemo, (laWidget*)ImageWidget10);

    ArcDemoMenuPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)ArcDemoMenuPanel, 0, 61);
    laWidget_SetSize((laWidget*)ArcDemoMenuPanel, 58, 210);
    laWidget_SetBackgroundType((laWidget*)ArcDemoMenuPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcDemoMenuPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, ArcDemoMenuPanel);

    HomeButton_ArcDemo = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)HomeButton_ArcDemo, 1, 0);
    laWidget_SetSize((laWidget*)HomeButton_ArcDemo, 60, 60);
    laWidget_SetBackgroundType((laWidget*)HomeButton_ArcDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)HomeButton_ArcDemo, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(HomeButton_ArcDemo, LA_HALIGN_LEFT);
    laButtonWidget_SetVAlignment(HomeButton_ArcDemo, LA_VALIGN_TOP);
    laButtonWidget_SetPressedEventCallback(HomeButton_ArcDemo, &HomeButton_ArcDemo_PressedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)HomeButton_ArcDemo);

    ArcHomeButtonBackPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)ArcHomeButtonBackPanel, 19, 19);
    laWidget_SetSize((laWidget*)ArcHomeButtonBackPanel, 30, 30);
    laWidget_SetScheme((laWidget*)ArcHomeButtonBackPanel, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)ArcHomeButtonBackPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ArcHomeButtonBackPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)HomeButton_ArcDemo, ArcHomeButtonBackPanel);

    ArcHomeButtonPanelWidget = laWidget_New();
    laWidget_SetPosition((laWidget*)ArcHomeButtonPanelWidget, 15, 15);
    laWidget_SetSize((laWidget*)ArcHomeButtonPanelWidget, 30, 30);
    laWidget_SetScheme((laWidget*)ArcHomeButtonPanelWidget, &ArcDemoScheme);
    laWidget_SetBackgroundType((laWidget*)ArcHomeButtonPanelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ArcHomeButtonPanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)HomeButton_ArcDemo, ArcHomeButtonPanelWidget);

    ArcHomeButtonImage = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ArcHomeButtonImage, 15, 15);
    laWidget_SetSize((laWidget*)ArcHomeButtonImage, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ArcHomeButtonImage, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ArcHomeButtonImage, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ArcHomeButtonImage, &HomeButton30x30Glow);
    laWidget_AddChild((laWidget*)HomeButton_ArcDemo, (laWidget*)ArcHomeButtonImage);

    TouchImage_ArcDemo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchImage_ArcDemo, 190, 120);
    laWidget_SetSize((laWidget*)TouchImage_ArcDemo, 84, 83);
    laWidget_SetEnabled((laWidget*)TouchImage_ArcDemo, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TouchImage_ArcDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchImage_ArcDemo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(TouchImage_ArcDemo, &TouchScreen);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchImage_ArcDemo);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 10, 66);
    laWidget_SetSize((laWidget*)LabelWidget2, 69, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_TouchHere));
    laWidget_AddChild((laWidget*)TouchImage_ArcDemo, (laWidget*)LabelWidget2);

}

static void ScreenCreate_CircularSliderDemo(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &CircularSliderDemoScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget31 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget31, 103, 16);
    laWidget_SetSize((laWidget*)LabelWidget31, 278, 25);
    laWidget_SetScheme((laWidget*)LabelWidget31, &CircularSliderDemoScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget31, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget31, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget31, laString_CreateFromID(string_CircularSliderDemo));
    laLabelWidget_SetHAlignment(LabelWidget31, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget31);

    CircularSliderValueLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CircularSliderValueLabel, 126, 132);
    laWidget_SetSize((laWidget*)CircularSliderValueLabel, 59, 37);
    laWidget_SetBackgroundType((laWidget*)CircularSliderValueLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircularSliderValueLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CircularSliderValueLabel, laString_CreateFromID(string_DefaultValueBig));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularSliderValueLabel);

    CircularProgressBar = laCircularSliderWidget_New();
    laWidget_SetPosition((laWidget*)CircularProgressBar, 80, 79);
    laWidget_SetSize((laWidget*)CircularProgressBar, 147, 146);
    laWidget_SetScheme((laWidget*)CircularProgressBar, &CircularProgressBarScheme);
    laWidget_SetBackgroundType((laWidget*)CircularProgressBar, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircularProgressBar, LA_WIDGET_BORDER_NONE);
    laCircularSliderWidget_SetRadius(CircularProgressBar, 70);
    laCircularSliderWidget_SetStartAngle(CircularProgressBar, 270);
    laCircularSliderWidget_SetStickyButton(CircularProgressBar, LA_FALSE);
    laCircularSliderWidget_SetTouchOnButtonOnly(CircularProgressBar, LA_FALSE);
    laCircularSliderWidget_SetDirection(CircularProgressBar, CIRCULAR_SLIDER_DIR_CLOCKWISE);
    laCircularSliderWidget_SetArcThickness(CircularProgressBar, INSIDE_CIRCLE_BORDER, 2);
    laCircularSliderWidget_SetArcThickness(CircularProgressBar, ACTIVE_AREA, 15);
    laCircularSliderWidget_SetArcVisible(CircularProgressBar, CIRCLE_BUTTON, LA_FALSE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularProgressBar);

    CircularSliderWidgetControl = laCircularSliderWidget_New();
    laWidget_SetPosition((laWidget*)CircularSliderWidgetControl, 240, 70);
    laWidget_SetSize((laWidget*)CircularSliderWidgetControl, 170, 171);
    laWidget_SetScheme((laWidget*)CircularSliderWidgetControl, &CircularSliderScheme);
    laWidget_SetBackgroundType((laWidget*)CircularSliderWidgetControl, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircularSliderWidgetControl, LA_WIDGET_BORDER_NONE);
    laCircularSliderWidget_SetRadius(CircularSliderWidgetControl, 70);
    laCircularSliderWidget_SetStartAngle(CircularSliderWidgetControl, 270);
    laCircularSliderWidget_SetTouchOnButtonOnly(CircularSliderWidgetControl, LA_FALSE);
    laCircularSliderWidget_SetArcVisible(CircularSliderWidgetControl, OUTSIDE_CIRCLE_BORDER, LA_FALSE);
    laCircularSliderWidget_SetArcVisible(CircularSliderWidgetControl, INSIDE_CIRCLE_BORDER, LA_FALSE);
    laCircularSliderWidget_SetArcThickness(CircularSliderWidgetControl, ACTIVE_AREA, 10);
    laCircularSliderWidget_SetArcRadius(CircularSliderWidgetControl, CIRCLE_BUTTON, 25);
    laCircularSliderWidget_SetArcScheme(CircularSliderWidgetControl, CIRCLE_BUTTON, &CircularSliderScheme);
    laCircularSliderWidget_SetValueChangedEventCallback(CircularSliderWidgetControl, &CircularSliderWidgetControl_ValueChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularSliderWidgetControl);

    CircSlider_NextButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CircSlider_NextButton, 420, 0);
    laWidget_SetSize((laWidget*)CircSlider_NextButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)CircSlider_NextButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircSlider_NextButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(CircSlider_NextButton, &CircSlider_NextButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircSlider_NextButton);

    PanelWidget17 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget17, 15, 19);
    laWidget_SetSize((laWidget*)PanelWidget17, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget17, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget17, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget17, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircSlider_NextButton, PanelWidget17);

    PanelWidget18 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget18, 19, 15);
    laWidget_SetSize((laWidget*)PanelWidget18, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget18, &CircularSliderDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget18, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget18, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircSlider_NextButton, PanelWidget18);

    ImageWidget19 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget19, 19, 15);
    laWidget_SetSize((laWidget*)ImageWidget19, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget19, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget19, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget19, &NextButton30x30Glow);
    laWidget_AddChild((laWidget*)CircSlider_NextButton, (laWidget*)ImageWidget19);

    CircSlider_HomeButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)CircSlider_HomeButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)CircSlider_HomeButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircSlider_HomeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(CircSlider_HomeButton, &CircSlider_HomeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircSlider_HomeButton);

    PanelWidget13 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget13, 19, 19);
    laWidget_SetSize((laWidget*)PanelWidget13, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget13, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget13, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget13, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircSlider_HomeButton, PanelWidget13);

    PanelWidget14 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget14, 15, 15);
    laWidget_SetSize((laWidget*)PanelWidget14, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget14, &CircularSliderDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget14, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget14, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircSlider_HomeButton, PanelWidget14);

    ImageWidget15 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget15, 15, 15);
    laWidget_SetSize((laWidget*)ImageWidget15, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget15, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget15, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget15, &HomeButton30x30Glow);
    laWidget_AddChild((laWidget*)CircSlider_HomeButton, (laWidget*)ImageWidget15);

    TouchImage_CircSliderDemo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchImage_CircSliderDemo, 286, 81);
    laWidget_SetSize((laWidget*)TouchImage_CircSliderDemo, 84, 83);
    laWidget_SetEnabled((laWidget*)TouchImage_CircSliderDemo, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TouchImage_CircSliderDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchImage_CircSliderDemo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(TouchImage_CircSliderDemo, &TouchScreen);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchImage_CircSliderDemo);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 10, 66);
    laWidget_SetSize((laWidget*)LabelWidget5, 69, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_TouchHere));
    laWidget_AddChild((laWidget*)TouchImage_CircSliderDemo, (laWidget*)LabelWidget5);

}

static void ScreenCreate_CircularGaugeDemo(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &CircularGaugeDemoScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget41 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget41, 103, 11);
    laWidget_SetSize((laWidget*)LabelWidget41, 278, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget41, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget41, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget41, laString_CreateFromID(string_CircularGaugeDemo));
    laLabelWidget_SetHAlignment(LabelWidget41, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget41);

    CircGauge_NextButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CircGauge_NextButton, 420, 0);
    laWidget_SetSize((laWidget*)CircGauge_NextButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)CircGauge_NextButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircGauge_NextButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(CircGauge_NextButton, &CircGauge_NextButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircGauge_NextButton);

    PanelWidget25 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget25, 15, 19);
    laWidget_SetSize((laWidget*)PanelWidget25, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget25, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget25, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget25, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircGauge_NextButton, PanelWidget25);

    PanelWidget26 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget26, 19, 15);
    laWidget_SetSize((laWidget*)PanelWidget26, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget26, &CircularGaugeDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget26, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget26, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircGauge_NextButton, PanelWidget26);

    ImageWidget27 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget27, 19, 15);
    laWidget_SetSize((laWidget*)ImageWidget27, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget27, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget27, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget27, &NextButton30x30Glow);
    laWidget_AddChild((laWidget*)CircGauge_NextButton, (laWidget*)ImageWidget27);

    GaugeValueLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GaugeValueLabelWidget, 242, 153);
    laWidget_SetSize((laWidget*)GaugeValueLabelWidget, 44, 30);
    laWidget_SetBackgroundType((laWidget*)GaugeValueLabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GaugeValueLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GaugeValueLabelWidget, laString_CreateFromID(string_DefaultGaugeValue));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GaugeValueLabelWidget);

    MphLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MphLabel, 251, 175);
    laWidget_SetSize((laWidget*)MphLabel, 29, 25);
    laWidget_SetBackgroundType((laWidget*)MphLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MphLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(MphLabel, laString_CreateFromID(string_mph));
    laLabelWidget_SetHAlignment(MphLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MphLabel);

    CircularGaugeWidget43 = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)CircularGaugeWidget43, 102, 59);
    laWidget_SetSize((laWidget*)CircularGaugeWidget43, 220, 204);
    laWidget_SetScheme((laWidget*)CircularGaugeWidget43, &CircularGaugeScheme);
    laWidget_SetBackgroundType((laWidget*)CircularGaugeWidget43, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircularGaugeWidget43, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(CircularGaugeWidget43, 100);
    laCircularGaugeWidget_SetStartAngle(CircularGaugeWidget43, 270);
    laCircularGaugeWidget_SetCenterAngle(CircularGaugeWidget43, -230);
    laCircularGaugeWidget_SetValue(CircularGaugeWidget43, 0);
    laCircularGaugeWidget_SetEndValue(CircularGaugeWidget43, 100);
    laCircularGaugeWidget_SetTickLength(CircularGaugeWidget43, 15);
    laCircularGaugeWidget_SetTickLabelsVisible(CircularGaugeWidget43, LA_FALSE);
    laCircularGaugeWidget_SetTicksLabelsStringID(CircularGaugeWidget43, string_NumsSmall);
    laCircularGaugeWidget_SetStringTable(CircularGaugeWidget43, &stringTable);
    laCircularGaugeWidget_SetHandRadius(CircularGaugeWidget43, 80);
    laCircularGaugeWidget_SetCenterCircleVisible(CircularGaugeWidget43, LA_FALSE);
    laCircularGaugeWidget_SetCenterCircleRadius(CircularGaugeWidget43, 10);
    laCircularGaugeWidget_SetCenterCircleThickness(CircularGaugeWidget43, 10);
    laCircularGaugeWidget_AddValueArc(CircularGaugeWidget43, 0, 100, 100, 5, &BlackBaseScheme);
    laCircularGaugeWidget_AddValueArc(CircularGaugeWidget43, 0, 75, 95, 20, &GrayForegroundScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget43, 0, 360, 10, 10, &BlackBaseScheme);
    laCircularGaugeWidget_AddValueArc(CircularGaugeWidget43, 75, 100, 95, 20, NULL);
    laCircularGaugeWidget_AddMinorTicks(CircularGaugeWidget43, 2, 100, 100, 10, 2, &BlackBackgroundScheme);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularGaugeWidget43);

    SpeedoCircularGaugeWidget = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)SpeedoCircularGaugeWidget, 220, 120);
    laWidget_SetSize((laWidget*)SpeedoCircularGaugeWidget, 148, 150);
    laWidget_SetScheme((laWidget*)SpeedoCircularGaugeWidget, &CircularGaugeScheme);
    laWidget_SetBackgroundType((laWidget*)SpeedoCircularGaugeWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SpeedoCircularGaugeWidget, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(SpeedoCircularGaugeWidget, 55);
    laCircularGaugeWidget_SetStartAngle(SpeedoCircularGaugeWidget, 210);
    laCircularGaugeWidget_SetCenterAngle(SpeedoCircularGaugeWidget, 230);
    laCircularGaugeWidget_SetValue(SpeedoCircularGaugeWidget, 0);
    laCircularGaugeWidget_SetEndValue(SpeedoCircularGaugeWidget, 160);
    laCircularGaugeWidget_SetTickValue(SpeedoCircularGaugeWidget, 40);
    laCircularGaugeWidget_SetTicksLabelsStringID(SpeedoCircularGaugeWidget, string_NumsSmall);
    laCircularGaugeWidget_SetStringTable(SpeedoCircularGaugeWidget, &stringTable);
    laCircularGaugeWidget_SetCenterCircleThickness(SpeedoCircularGaugeWidget, 5);
    laCircularGaugeWidget_AddValueArc(SpeedoCircularGaugeWidget, 0, 80, 48, 6, &GrayForegroundScheme);
    laCircularGaugeWidget_AddValueArc(SpeedoCircularGaugeWidget, 0, 160, 50, 2, &defaultScheme);
    laCircularGaugeWidget_AddValueArc(SpeedoCircularGaugeWidget, 80, 160, 48, 6, &RedlineScheme);
    laCircularGaugeWidget_AddMinorTicks(SpeedoCircularGaugeWidget, 10, 150, 55, 3, 10, &GrayForegroundScheme);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)SpeedoCircularGaugeWidget);

    LabelWidget49 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget49, 215, 253);
    laWidget_SetSize((laWidget*)LabelWidget49, 15, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget49, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget49, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget49, laString_CreateFromID(string_NumGauge0));
    laLabelWidget_SetHAlignment(LabelWidget49, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget49);

    LabelWidget51 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget51, 115, 231);
    laWidget_SetSize((laWidget*)LabelWidget51, 16, 20);
    laWidget_SetBackgroundType((laWidget*)LabelWidget51, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget51, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget51, laString_CreateFromID(string_NumGauge2));
    laLabelWidget_SetHAlignment(LabelWidget51, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget51);

    LabelWidget53 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget53, 95, 192);
    laWidget_SetSize((laWidget*)LabelWidget53, 16, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget53, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget53, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget53, laString_CreateFromID(string_NumGauge3));
    laLabelWidget_SetHAlignment(LabelWidget53, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget53);

    LabelWidget50 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget50, 161, 253);
    laWidget_SetSize((laWidget*)LabelWidget50, 14, 21);
    laWidget_SetBackgroundType((laWidget*)LabelWidget50, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget50, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget50, laString_CreateFromID(string_NumGauge1));
    laLabelWidget_SetHAlignment(LabelWidget50, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget50);

    LabelWidget54 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget54, 90, 152);
    laWidget_SetSize((laWidget*)LabelWidget54, 13, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget54, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget54, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget54, laString_CreateFromID(string_NumGauge4));
    laLabelWidget_SetHAlignment(LabelWidget54, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget54);

    LabelWidget55 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget55, 96, 106);
    laWidget_SetSize((laWidget*)LabelWidget55, 15, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget55, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget55, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget55, laString_CreateFromID(string_NumGauge5));
    laLabelWidget_SetHAlignment(LabelWidget55, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget55);

    LabelWidget56 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget56, 126, 70);
    laWidget_SetSize((laWidget*)LabelWidget56, 15, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget56, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget56, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget56, laString_CreateFromID(string_NumGauge6));
    laLabelWidget_SetHAlignment(LabelWidget56, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget56);

    LabelWidget57 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget57, 166, 42);
    laWidget_SetSize((laWidget*)LabelWidget57, 15, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget57, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget57, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget57, laString_CreateFromID(string_NumGauge7));
    laLabelWidget_SetHAlignment(LabelWidget57, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget57);

    LabelWidget59 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget59, 252, 52);
    laWidget_SetSize((laWidget*)LabelWidget59, 15, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget59, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget59, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget59, laString_CreateFromID(string_NumGauge9));
    laLabelWidget_SetHAlignment(LabelWidget59, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget59);

    LabelWidget60 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget60, 290, 80);
    laWidget_SetSize((laWidget*)LabelWidget60, 25, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget60, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget60, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget60, laString_CreateFromID(string_NumGauge10));
    laLabelWidget_SetHAlignment(LabelWidget60, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget60);

    LabelWidget58 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget58, 210, 37);
    laWidget_SetSize((laWidget*)LabelWidget58, 15, 19);
    laWidget_SetBackgroundType((laWidget*)LabelWidget58, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget58, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget58, laString_CreateFromID(string_NumGauge8));
    laLabelWidget_SetHAlignment(LabelWidget58, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget58);

    CircGauge_HomeButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)CircGauge_HomeButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)CircGauge_HomeButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircGauge_HomeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(CircGauge_HomeButton, &CircGauge_HomeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircGauge_HomeButton);

    PanelWidget21 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget21, 19, 19);
    laWidget_SetSize((laWidget*)PanelWidget21, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget21, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget21, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget21, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircGauge_HomeButton, PanelWidget21);

    PanelWidget22 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget22, 15, 15);
    laWidget_SetSize((laWidget*)PanelWidget22, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget22, &CircularGaugeDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget22, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget22, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)CircGauge_HomeButton, PanelWidget22);

    ImageWidget23 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget23, 15, 15);
    laWidget_SetSize((laWidget*)ImageWidget23, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget23, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget23, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget23, &HomeButton30x30Glow);
    laWidget_AddChild((laWidget*)CircGauge_HomeButton, (laWidget*)ImageWidget23);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 184, 114);
    laWidget_SetSize((laWidget*)LabelWidget1, 57, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_rpm));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

    GasButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)GasButton, 0, 54);
    laWidget_SetSize((laWidget*)GasButton, 480, 216);
    laWidget_SetBackgroundType((laWidget*)GasButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GasButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedEventCallback(GasButton, &GasButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(GasButton, &GasButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)GasButton);

    TouchImage_CircGaugeDemo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchImage_CircGaugeDemo, 367, 66);
    laWidget_SetSize((laWidget*)TouchImage_CircGaugeDemo, 84, 90);
    laWidget_SetEnabled((laWidget*)TouchImage_CircGaugeDemo, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TouchImage_CircGaugeDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchImage_CircGaugeDemo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(TouchImage_CircGaugeDemo, &TouchScreen);
    laWidget_AddChild((laWidget*)GasButton, (laWidget*)TouchImage_CircGaugeDemo);

    LabelWidget7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget7, 14, 66);
    laWidget_SetSize((laWidget*)LabelWidget7, 69, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget7, laString_CreateFromID(string_TouchHere));
    laWidget_AddChild((laWidget*)TouchImage_CircGaugeDemo, (laWidget*)LabelWidget7);

}

static void ScreenCreate_PieChartDemo(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &PieChartDemoScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    PieChart_NextButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PieChart_NextButton, 420, 0);
    laWidget_SetSize((laWidget*)PieChart_NextButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)PieChart_NextButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PieChart_NextButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(PieChart_NextButton, &PieChart_NextButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PieChart_NextButton);

    PanelWidget33 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget33, 15, 19);
    laWidget_SetSize((laWidget*)PanelWidget33, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget33, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget33, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget33, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PieChart_NextButton, PanelWidget33);

    PanelWidget34 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget34, 19, 15);
    laWidget_SetSize((laWidget*)PanelWidget34, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget34, &PieChartDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget34, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget34, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PieChart_NextButton, PanelWidget34);

    ImageWidget35 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget35, 19, 15);
    laWidget_SetSize((laWidget*)ImageWidget35, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget35, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget35, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget35, &NextButton30x30Glow);
    laWidget_AddChild((laWidget*)PieChart_NextButton, (laWidget*)ImageWidget35);

    LabelWidget39 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget39, 123, 21);
    laWidget_SetSize((laWidget*)LabelWidget39, 238, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget39, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget39, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget39, laString_CreateFromID(string_PieChartDemo));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget39);

    PieChartWidget2 = laPieChartWidget_New();
    laWidget_SetPosition((laWidget*)PieChartWidget2, 130, 50);
    laWidget_SetSize((laWidget*)PieChartWidget2, 220, 220);
    laWidget_SetBackgroundType((laWidget*)PieChartWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PieChartWidget2, LA_WIDGET_BORDER_NONE);
    laPieChartWidget_SetCenterAngle(PieChartWidget2, 360);
    laPieChartWidget_SetLabelsStringID(PieChartWidget2, string_NumsSmall);
    laPieChartWidget_SetStringTable(PieChartWidget2, &stringTable);
    laPieChartWidget_AddEntry(PieChartWidget2, NULL);
    laPieChartWidget_SetEntryValue(PieChartWidget2, 0, 30);
    laPieChartWidget_SetEntryRadius(PieChartWidget2, 0, 90);
    laPieChartWidget_SetEntryOffset(PieChartWidget2, 0, 0);
    laPieChartWidget_SetEntryScheme(PieChartWidget2, 0, &DarkBlueForeGroundScheme);
    laPieChartWidget_AddEntry(PieChartWidget2, NULL);
    laPieChartWidget_SetEntryValue(PieChartWidget2, 1, 40);
    laPieChartWidget_SetEntryRadius(PieChartWidget2, 1, 90);
    laPieChartWidget_SetEntryOffset(PieChartWidget2, 1, 0);
    laPieChartWidget_SetEntryScheme(PieChartWidget2, 1, &LightBlueForegroundScheme);
    laPieChartWidget_AddEntry(PieChartWidget2, NULL);
    laPieChartWidget_SetEntryValue(PieChartWidget2, 2, 25);
    laPieChartWidget_SetEntryRadius(PieChartWidget2, 2, 90);
    laPieChartWidget_SetEntryOffset(PieChartWidget2, 2, 0);
    laPieChartWidget_SetEntryScheme(PieChartWidget2, 2, &BrightPinkBaseScheme);
    laPieChartWidget_AddEntry(PieChartWidget2, NULL);
    laPieChartWidget_SetEntryValue(PieChartWidget2, 3, 45);
    laPieChartWidget_SetEntryRadius(PieChartWidget2, 3, 90);
    laPieChartWidget_SetEntryOffset(PieChartWidget2, 3, 0);
    laPieChartWidget_SetEntryScheme(PieChartWidget2, 3, &BrightPurpleBaseScheme);
    laPieChartWidget_AddEntry(PieChartWidget2, NULL);
    laPieChartWidget_SetEntryValue(PieChartWidget2, 4, 35);
    laPieChartWidget_SetEntryRadius(PieChartWidget2, 4, 90);
    laPieChartWidget_SetEntryOffset(PieChartWidget2, 4, 0);
    laPieChartWidget_SetEntryScheme(PieChartWidget2, 4, &BrightYellowBaseScheme);
    laPieChartWidget_SetPressedEventCallback(PieChartWidget2, &PieChartWidget2_PressedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PieChartWidget2);

    PieChart_HomeButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)PieChart_HomeButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)PieChart_HomeButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PieChart_HomeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(PieChart_HomeButton, &PieChart_HomeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PieChart_HomeButton);

    PanelWidget29 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget29, 19, 19);
    laWidget_SetSize((laWidget*)PanelWidget29, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget29, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget29, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget29, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PieChart_HomeButton, PanelWidget29);

    PanelWidget30 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget30, 15, 15);
    laWidget_SetSize((laWidget*)PanelWidget30, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget30, &PieChartDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget30, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget30, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PieChart_HomeButton, PanelWidget30);

    ImageWidget31 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget31, 15, 15);
    laWidget_SetSize((laWidget*)ImageWidget31, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget31, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget31, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget31, &HomeButton30x30Glow);
    laWidget_AddChild((laWidget*)PieChart_HomeButton, (laWidget*)ImageWidget31);

    TouchImage_PieChartDemo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchImage_PieChartDemo, 206, 183);
    laWidget_SetSize((laWidget*)TouchImage_PieChartDemo, 84, 87);
    laWidget_SetEnabled((laWidget*)TouchImage_PieChartDemo, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TouchImage_PieChartDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchImage_PieChartDemo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(TouchImage_PieChartDemo, &TouchScreen);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchImage_PieChartDemo);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 14, 66);
    laWidget_SetSize((laWidget*)LabelWidget9, 69, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_TouchHere));
    laWidget_AddChild((laWidget*)TouchImage_PieChartDemo, (laWidget*)LabelWidget9);

}

static void ScreenCreate_BarGraphDemo(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &BarGraphDemoScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget4, 119, 20);
    laWidget_SetSize((laWidget*)LabelWidget4, 250, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget4, laString_CreateFromID(string_BarGraphDemo));
    laLabelWidget_SetHAlignment(LabelWidget4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget4);

    BarGraphWidget8 = laBarGraphWidget_New();
    laWidget_SetPosition((laWidget*)BarGraphWidget8, 0, 66);
    laWidget_SetSize((laWidget*)BarGraphWidget8, 470, 207);
    laWidget_SetScheme((laWidget*)BarGraphWidget8, &BarGraphWidgetScheme);
    laWidget_SetBackgroundType((laWidget*)BarGraphWidget8, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BarGraphWidget8, LA_WIDGET_BORDER_NONE);
    laBarGraphWidget_SetStacked(BarGraphWidget8, LA_FALSE);
    laBarGraphWidget_SetMaxValue(BarGraphWidget8, 0, 500);
    laBarGraphWidget_SetValueAxisTickInterval(BarGraphWidget8, 0, 100);
    laBarGraphWidget_SetValueAxisSubtickInterval(BarGraphWidget8, 0, 25);
    laBarGraphWidget_SetTicksLabelsStringID(BarGraphWidget8, string_NumsSmall);
    laBarGraphWidget_SetStringTable(BarGraphWidget8, &stringTable);
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 0, laString_CreateFromID(string_pt0));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 1, laString_CreateFromID(string_pt1));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 2, laString_CreateFromID(string_pt2));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 3, laString_CreateFromID(string_pt3));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 4, laString_CreateFromID(string_pt4));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 5, laString_CreateFromID(string_pt5));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 6, laString_CreateFromID(string_pt6));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 7, laString_CreateFromID(string_pt7));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 8, laString_CreateFromID(string_pt8));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 9, laString_CreateFromID(string_pt9));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 10, laString_CreateFromID(string_pt10));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 11, laString_CreateFromID(string_pt11));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 12, laString_CreateFromID(string_pt12));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 13, laString_CreateFromID(string_pt13));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 14, laString_CreateFromID(string_pt14));
    laBarGraphWidget_AddCategory(BarGraphWidget8, NULL);
    laBarGraphWidget_SetCategoryText(BarGraphWidget8, 15, laString_CreateFromID(string_pt15));
    laBarGraphWidget_AddSeries(BarGraphWidget8, NULL);
    laBarGraphWidget_SetSeriesScheme(BarGraphWidget8, 0, &DarkBlueForeGroundScheme);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 0, 0, NULL);
    laBarGraphWidget_AddSeries(BarGraphWidget8, NULL);
    laBarGraphWidget_SetSeriesScheme(BarGraphWidget8, 1, &LightBlueForegroundScheme);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laBarGraphWidget_AddDataToSeries(BarGraphWidget8, 1, 0, NULL);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)BarGraphWidget8);

    BarGraphTouchedButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)BarGraphTouchedButton, 480, 270);
    laWidget_SetBackgroundType((laWidget*)BarGraphTouchedButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BarGraphTouchedButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedEventCallback(BarGraphTouchedButton, &BarGraphTouchedButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(BarGraphTouchedButton, &BarGraphTouchedButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)BarGraphTouchedButton);

    TouchImage_BarGraphDemo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchImage_BarGraphDemo, 200, 110);
    laWidget_SetSize((laWidget*)TouchImage_BarGraphDemo, 84, 90);
    laWidget_SetEnabled((laWidget*)TouchImage_BarGraphDemo, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TouchImage_BarGraphDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchImage_BarGraphDemo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(TouchImage_BarGraphDemo, &TouchScreen);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchImage_BarGraphDemo);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 12, 66);
    laWidget_SetSize((laWidget*)LabelWidget12, 69, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_TouchHere));
    laWidget_AddChild((laWidget*)TouchImage_BarGraphDemo, (laWidget*)LabelWidget12);

    BarGraph_NextButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)BarGraph_NextButton, 420, 0);
    laWidget_SetSize((laWidget*)BarGraph_NextButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)BarGraph_NextButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BarGraph_NextButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(BarGraph_NextButton, &BarGraph_NextButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)BarGraph_NextButton);

    PanelWidget41 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget41, 15, 19);
    laWidget_SetSize((laWidget*)PanelWidget41, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget41, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget41, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget41, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)BarGraph_NextButton, PanelWidget41);

    PanelWidget42 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget42, 19, 15);
    laWidget_SetSize((laWidget*)PanelWidget42, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget42, &BarGraphDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget42, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget42, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)BarGraph_NextButton, PanelWidget42);

    ImageWidget43 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget43, 19, 15);
    laWidget_SetSize((laWidget*)ImageWidget43, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget43, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget43, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget43, &NextButton30x30Glow);
    laWidget_AddChild((laWidget*)BarGraph_NextButton, (laWidget*)ImageWidget43);

    BarGraph_HomeButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)BarGraph_HomeButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)BarGraph_HomeButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BarGraph_HomeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(BarGraph_HomeButton, &BarGraph_HomeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)BarGraph_HomeButton);

    PanelWidget37 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget37, 19, 19);
    laWidget_SetSize((laWidget*)PanelWidget37, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget37, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget37, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget37, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)BarGraph_HomeButton, PanelWidget37);

    PanelWidget38 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget38, 15, 15);
    laWidget_SetSize((laWidget*)PanelWidget38, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget38, &BarGraphDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget38, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget38, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)BarGraph_HomeButton, PanelWidget38);

    ImageWidget39 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget39, 15, 15);
    laWidget_SetSize((laWidget*)ImageWidget39, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget39, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget39, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget39, &HomeButton30x30Glow);
    laWidget_AddChild((laWidget*)BarGraph_HomeButton, (laWidget*)ImageWidget39);

    PanelWidget = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget, 157, 60);
    laWidget_SetSize((laWidget*)PanelWidget, 10, 10);
    laWidget_SetScheme((laWidget*)PanelWidget, &DarkBlueForeGroundScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 173, 56);
    laWidget_SetSize((laWidget*)LabelWidget, 15, 15);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_x));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

    PanelWidget4 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget4, 237, 60);
    laWidget_SetSize((laWidget*)PanelWidget4, 10, 10);
    laWidget_SetScheme((laWidget*)PanelWidget4, &LightBlueForegroundScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget4, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget4);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 255, 51);
    laWidget_SetSize((laWidget*)LabelWidget6, 15, 24);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_y));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget6);

}

static void ScreenCreate_LineGraphScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &LineGraphDemoScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget11, 111, 17);
    laWidget_SetSize((laWidget*)LabelWidget11, 263, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget11, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget11, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget11, laString_CreateFromID(string_LineGraphDemo));
    laLabelWidget_SetHAlignment(LabelWidget11, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget11);

    LineGraphWidget13 = laLineGraphWidget_New();
    laWidget_SetPosition((laWidget*)LineGraphWidget13, 0, 60);
    laWidget_SetSize((laWidget*)LineGraphWidget13, 480, 212);
    laWidget_SetScheme((laWidget*)LineGraphWidget13, &LineGraphScheme);
    laWidget_SetBackgroundType((laWidget*)LineGraphWidget13, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LineGraphWidget13, LA_WIDGET_BORDER_NONE);
    laLineGraphWidget_SetMaxValue(LineGraphWidget13, 0, 500);
    laLineGraphWidget_SetValueAxisTickInterval(LineGraphWidget13, 0, 100);
    laLineGraphWidget_SetValueAxisSubtickInterval(LineGraphWidget13, 0, 25);
    laLineGraphWidget_SetTicksLabelsStringID(LineGraphWidget13, string_NumsSmall);
    laLineGraphWidget_SetStringTable(LineGraphWidget13, &stringTable);
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 0, laString_CreateFromID(string_pt0));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 1, laString_CreateFromID(string_pt1));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 2, laString_CreateFromID(string_pt2));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 3, laString_CreateFromID(string_pt3));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 4, laString_CreateFromID(string_pt4));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 5, laString_CreateFromID(string_pt5));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 6, laString_CreateFromID(string_pt6));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 7, laString_CreateFromID(string_pt7));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 8, laString_CreateFromID(string_pt8));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 9, laString_CreateFromID(string_pt9));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 10, laString_CreateFromID(string_pt10));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 11, laString_CreateFromID(string_pt11));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 12, laString_CreateFromID(string_pt12));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 13, laString_CreateFromID(string_pt13));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 14, laString_CreateFromID(string_pt14));
    laLineGraphWidget_AddCategory(LineGraphWidget13, NULL);
    laLineGraphWidget_SetCategoryText(LineGraphWidget13, 15, laString_CreateFromID(string_pt15));
    laLineGraphWidget_AddSeries(LineGraphWidget13, NULL);
    laLineGraphWidget_SetSeriesScheme(LineGraphWidget13, 0, &DataSeriesA);
    laLineGraphWidget_SetSeriesPointType(LineGraphWidget13, 0, LINE_GRAPH_DATA_POINT_CIRCLE);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 0, 0, NULL);
    laLineGraphWidget_AddSeries(LineGraphWidget13, NULL);
    laLineGraphWidget_SetSeriesScheme(LineGraphWidget13, 1, &DataSeriesB);
    laLineGraphWidget_SetSeriesPointType(LineGraphWidget13, 1, LINE_GRAPH_DATA_POINT_SQUARE);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laLineGraphWidget_AddDataToSeries(LineGraphWidget13, 1, 0, NULL);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineGraphWidget13);

    LineGraphTouchedButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)LineGraphTouchedButton, 480, 272);
    laWidget_SetBackgroundType((laWidget*)LineGraphTouchedButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LineGraphTouchedButton, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineGraphTouchedButton);

    TouchImage_LineGraphDemo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchImage_LineGraphDemo, 207, 141);
    laWidget_SetSize((laWidget*)TouchImage_LineGraphDemo, 84, 90);
    laWidget_SetEnabled((laWidget*)TouchImage_LineGraphDemo, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TouchImage_LineGraphDemo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchImage_LineGraphDemo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(TouchImage_LineGraphDemo, &TouchScreen);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchImage_LineGraphDemo);

    LabelWidget14 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget14, 13, 66);
    laWidget_SetSize((laWidget*)LabelWidget14, 69, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget14, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget14, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget14, laString_CreateFromID(string_TouchHere));
    laWidget_AddChild((laWidget*)TouchImage_LineGraphDemo, (laWidget*)LabelWidget14);

    LineGraph_HomeButton = laButtonWidget_New();
    laWidget_SetSize((laWidget*)LineGraph_HomeButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)LineGraph_HomeButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LineGraph_HomeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(LineGraph_HomeButton, &LineGraph_HomeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineGraph_HomeButton);

    PanelWidget46 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget46, 19, 19);
    laWidget_SetSize((laWidget*)PanelWidget46, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget46, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget46, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget46, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)LineGraph_HomeButton, PanelWidget46);

    PanelWidget47 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget47, 15, 15);
    laWidget_SetSize((laWidget*)PanelWidget47, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget47, &LineGraphDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget47, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget47, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)LineGraph_HomeButton, PanelWidget47);

    ImageWidget45 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget45, 15, 15);
    laWidget_SetSize((laWidget*)ImageWidget45, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget45, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget45, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget45, &HomeButton30x30Glow);
    laWidget_AddChild((laWidget*)LineGraph_HomeButton, (laWidget*)ImageWidget45);

    LineGraph_NextButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)LineGraph_NextButton, 420, 0);
    laWidget_SetSize((laWidget*)LineGraph_NextButton, 60, 60);
    laWidget_SetBackgroundType((laWidget*)LineGraph_NextButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LineGraph_NextButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(LineGraph_NextButton, &LineGraph_NextButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineGraph_NextButton);

    PanelWidget54 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget54, 15, 19);
    laWidget_SetSize((laWidget*)PanelWidget54, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget54, &BlackBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget54, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget54, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)LineGraph_NextButton, PanelWidget54);

    PanelWidget55 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget55, 19, 15);
    laWidget_SetSize((laWidget*)PanelWidget55, 30, 30);
    laWidget_SetScheme((laWidget*)PanelWidget55, &LineGraphDemoScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget55, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget55, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)LineGraph_NextButton, PanelWidget55);

    ImageWidget56 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget56, 19, 15);
    laWidget_SetSize((laWidget*)ImageWidget56, 30, 30);
    laWidget_SetBackgroundType((laWidget*)ImageWidget56, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget56, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget56, &NextButton30x30Glow);
    laWidget_AddChild((laWidget*)LineGraph_NextButton, (laWidget*)ImageWidget56);

    PanelWidget11 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget11, 157, 58);
    laWidget_SetSize((laWidget*)PanelWidget11, 10, 10);
    laWidget_SetScheme((laWidget*)PanelWidget11, &DarkBlueBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget11, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget11, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget11);

    LabelWidget13 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget13, 173, 54);
    laWidget_SetSize((laWidget*)LabelWidget13, 15, 15);
    laWidget_SetBackgroundType((laWidget*)LabelWidget13, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget13, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget13, laString_CreateFromID(string_x));
    laLabelWidget_SetHAlignment(LabelWidget13, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget13);

    PanelWidget15 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget15, 217, 58);
    laWidget_SetSize((laWidget*)PanelWidget15, 10, 10);
    laWidget_SetScheme((laWidget*)PanelWidget15, &DarkRedBaseScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget15, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget15, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget15);

    LabelWidget16 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget16, 235, 49);
    laWidget_SetSize((laWidget*)LabelWidget16, 15, 24);
    laWidget_SetBackgroundType((laWidget*)LabelWidget16, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget16, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget16, laString_CreateFromID(string_y));
    laLabelWidget_SetHAlignment(LabelWidget16, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget16);

    CheckBoxPhantomButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CheckBoxPhantomButton, 270, 49);
    laWidget_SetSize((laWidget*)CheckBoxPhantomButton, 79, 29);
    laWidget_SetBackgroundType((laWidget*)CheckBoxPhantomButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CheckBoxPhantomButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetToggleable(CheckBoxPhantomButton, LA_TRUE);
    laButtonWidget_SetPressed(CheckBoxPhantomButton, LA_TRUE);
    laButtonWidget_SetPressedEventCallback(CheckBoxPhantomButton, &CheckBoxPhantomButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(CheckBoxPhantomButton, &CheckBoxPhantomButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)CheckBoxPhantomButton);

    CheckBoxWidget17 = laCheckBoxWidget_New();
    laWidget_SetPosition((laWidget*)CheckBoxWidget17, 7, 3);
    laWidget_SetSize((laWidget*)CheckBoxWidget17, 63, 25);
    laWidget_SetBackgroundType((laWidget*)CheckBoxWidget17, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CheckBoxWidget17, LA_WIDGET_BORDER_NONE);
    laCheckBoxWidget_SetChecked(CheckBoxWidget17, LA_TRUE);
    laCheckBoxWidget_SetText(CheckBoxWidget17, laString_CreateFromID(string_Fill));
    laCheckBoxWidget_SetCheckedEventCallback(CheckBoxWidget17, &CheckBoxWidget17_CheckedEvent);
    laCheckBoxWidget_SetUncheckedEventCallback(CheckBoxWidget17, &CheckBoxWidget17_UncheckedEvent);

    laWidget_AddChild((laWidget*)CheckBoxPhantomButton, (laWidget*)CheckBoxWidget17);

}



