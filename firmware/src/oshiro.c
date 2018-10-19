/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    oshiro.c

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

#include "oshiro.h"
#include "spi_oledrgb.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define SCREEN_WIDTH                                    (75)
#define SCREEN_HEIGHT                                   (32)

#define SCREEN_HEIGHT_BYTE                              (SCREEN_HEIGHT/8)

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

OSHIRO_DATA oshiroData;

static uint8_t screenData[SCREEN_WIDTH][SCREEN_HEIGHT_BYTE] = {0};


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
void timerCallback( uintptr_t context, uint32_t currTick )
{
    oshiroData.oledState = OLED_STATE_MEASURE;
    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
static void setScreenAdcData( void )
{
    uint32_t data;
    
    data = oshiroData.adcData;
    
    if( data < 0x400){
        
        screenData[SCREEN_WIDTH-1][0] = 0x00;
        screenData[SCREEN_WIDTH-1][1] = 0x00;
        screenData[SCREEN_WIDTH-1][2] = 0x00;
        screenData[SCREEN_WIDTH-1][3] = (0x01 << (data%1024)/128);
        
    } else if ( data >= 0x400 && data < 0x800 ) {
        
        screenData[SCREEN_WIDTH-1][0] = 0x00;
        screenData[SCREEN_WIDTH-1][1] = 0x00;
        screenData[SCREEN_WIDTH-1][2] = (0x01 << (data%1024)/128);
        screenData[SCREEN_WIDTH-1][3] = 0x00;
        
    } else if ( data >= 0x800 && data < 0xC00 ) {
        screenData[SCREEN_WIDTH-1][0] = 0x00;
        screenData[SCREEN_WIDTH-1][1] = (0x01 << (data%1024)/128);
        screenData[SCREEN_WIDTH-1][2] = 0x00;
        screenData[SCREEN_WIDTH-1][3] = 0x00;
        
    } else if ( data >= 0xC00 ) {
        screenData[SCREEN_WIDTH-1][0] = (0x01 << (data%1024)/128);
        screenData[SCREEN_WIDTH-1][1] = 0x00;
        screenData[SCREEN_WIDTH-1][2] = 0x00;
        screenData[SCREEN_WIDTH-1][3] = 0x00;
        
    } 
}

static void OLED_Tasks ( void )
{
    SPI_OLEDRGB_COLOR color;
    uint8_t loop;
    
    switch(oshiroData.oledState)
    {
        case OLED_STATE_INIT:
        {
            if ( SPI_OLEDRGB_DevInit() )
            {
                oshiroData.oledState = OLED_STATE_FRAME;
            }
            
            break;
        }
        case OLED_STATE_FRAME:
        {
            
            // Gray Line
            color.red = 0xCC;
            color.green = 0xCC;
            color.blue = 0xCC;
            
            if ( !SPI_OLEDRGB_DrawLine( 20, 15, 95, 15, &color) )break;
            if ( !SPI_OLEDRGB_DrawLine( 20, 48, 95, 48, &color) )break;
            
            // string
            color.red = 0xFF;
            color.green = 0xFF;
            color.blue = 0xFF;
            
            if ( !SPI_OLEDRGB_DrawAsciiString( 0, 0, "oshiro", &color) )break;
            
            if ( !SPI_OLEDRGB_SetRemapDataFormat( 0x73 ))break;
            
            oshiroData.oledState = OLED_STATE_STARTMEASURE;
            break;
        }
        case OLED_STATE_STARTMEASURE:
        {
            oshiroData.tmrHandle = SYS_TMR_ObjectCreate( 100, 0, timerCallback, SYS_TMR_FLAG_PERIODIC );
            if( oshiroData.tmrHandle != SYS_TMR_HANDLE_INVALID ){
                oshiroData.oledState = OLED_STATE_WAITMEASURE;
            }
            break;
        }
        case OLED_STATE_MEASURE:
        {
            for(loop = 0; loop < SCREEN_WIDTH-1; loop++){
               screenData[loop][0] = screenData[loop+1][0];
               screenData[loop][1] = screenData[loop+1][1];
               screenData[loop][2] = screenData[loop+1][2];
               screenData[loop][3] = screenData[loop+1][3]; 
            }
            // ADC Data の設定
            setScreenAdcData();
            
            color.red = 0x00;
            color.green = 0x00;
            color.blue = 0xFF;
            if ( !SPI_OLEDRGB_DrawPixel( 20, 16, 94, 47, (uint8_t*)screenData, &color) )break;
            
            oshiroData.oledState = OLED_STATE_WAITMEASURE;
            
            break;
        }
        default:
            break;
    }
    return;
}

static void ADC_Tasks ( void )
{
    switch( oshiroData.adcState )
    {
        case ADC_STATE_INIT:
        {
            // ADC オープン
            DRV_ADC0_Open();
            oshiroData.adcState = ADC_STATE_START;
            break;
        }
        case ADC_STATE_START:
        {
            DRV_ADC_Start(); // ADC 変換開始
            oshiroData.adcState = ADC_STATE_GET;
            break;
        }
        case ADC_STATE_GET:
        {
            if(DRV_ADC_SamplesAvailable(ADCHS_AN1)){
              // ADC 変換終了
              oshiroData.adcData = DRV_ADC_SamplesRead(ADCHS_AN1);   // AN0 からADCデータ取得
              oshiroData.adcState = ADC_STATE_START;
            }
            break;
        }
    }
    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void OSHIRO_Initialize ( void )

  Remarks:
    See prototype in oshiro.h.
 */

void OSHIRO_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    oshiroData.state = OSHIRO_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    oshiroData.oledState = OLED_STATE_INIT;
    oshiroData.adcState = ADC_STATE_INIT;
    
    oshiroData.tmrHandle = SYS_TMR_HANDLE_INVALID;
}


/******************************************************************************
  Function:
    void OSHIRO_Tasks ( void )

  Remarks:
    See prototype in oshiro.h.
 */

void OSHIRO_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( oshiroData.state )
    {
        /* Application's initial state. */
        case OSHIRO_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                oshiroData.state = OSHIRO_STATE_SERVICE_TASKS;
            }
            break;
        }

        case OSHIRO_STATE_SERVICE_TASKS:
        {
            OLED_Tasks();
            ADC_Tasks();
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

 

/*******************************************************************************
 End of File
 */
