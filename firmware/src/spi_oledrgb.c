/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    spi_oledrgb.c

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

#include "spi_oledrgb.h"
#include "fontascii57.h"

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

SPI_OLEDRGB_DATA spi_oledrgbData;

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
static void delay(uint16_t ms)
{
    uint32_t time;
    
    time = _CP0_GET_COUNT();
    time += (SYS_CLK_FREQ / 2 / 1000) * ms;
    while((int32_t)(time-_CP0_GET_COUNT()) > 0){};
    return;
}

static bool spiWrite( void *txBuffer, size_t size )
{
    DRV_SPI_BUFFER_EVENT event;
    
    CSOff();
    
    DRV_SPI_BufferAddWrite2( spi_oledrgbData.spiHandle, txBuffer, size, NULL, NULL, &spi_oledrgbData.spiBufferHandle );
    if( spi_oledrgbData.spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID){
        return false;
    }
    
    while(1){
        event = DRV_SPI_BufferStatus(spi_oledrgbData.spiBufferHandle);
        
        switch(event){
            case DRV_SPI_BUFFER_EVENT_COMPLETE:
            {
                return true;
            }
            case DRV_SPI_BUFFER_EVENT_ERROR:
            {
                return false;
            }
        }
    }
    
    CSOn();
    
    return true;
}

static uint16_t getColor16( SPI_OLEDRGB_COLOR* color ){
    uint16_t ret = 0x0000;
    
    ret |= ((uint16_t)(color->red)<<11 & 0xF800 );
    ret |= ((uint16_t)(color->green)<<5 & 0x07E0 );
    ret |= ((uint16_t)(color->blue) & 0x001F );
    
    return ret;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SPI_OLEDRGB_Initialize ( void )

  Remarks:
    See prototype in spi_oledrgb.h.
 */

void SPI_OLEDRGB_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    spi_oledrgbData.state = SPI_OLEDRGB_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    /*** ADD ***/
    spi_oledrgbData.spiHandle = DRV_HANDLE_INVALID;
    spi_oledrgbData.spiBufferHandle = DRV_SPI_BUFFER_HANDLE_INVALID;
    /*** ADD ***/
}


/******************************************************************************
  Function:
    void SPI_OLEDRGB_Tasks ( void )

  Remarks:
    See prototype in spi_oledrgb.h.
 */

void SPI_OLEDRGB_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( spi_oledrgbData.state )
    {
        /* Application's initial state. */
        case SPI_OLEDRGB_STATE_INIT:
        {
            bool appInitialized = true;
       
            /*** ADD ***/
            if (spi_oledrgbData.spiHandle == DRV_HANDLE_INVALID)
            {
                spi_oledrgbData.spiHandle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_WRITE | DRV_IO_INTENT_NONBLOCKING );
                appInitialized &= ( DRV_HANDLE_INVALID != spi_oledrgbData.spiHandle );
            }
            /*** ADD ***/
            
            if (appInitialized)
            {
            
                spi_oledrgbData.state = SPI_OLEDRGB_STATE_SERVICE_TASKS;
            }
            break;
        }

        case SPI_OLEDRGB_STATE_SERVICE_TASKS:
        {
        
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

bool SPI_OLEDRGB_Clear( void )
{
    uint8_t writeBuf[10];
    
    writeBuf[0] = 0x25;
    writeBuf[1] = 0x00;
    writeBuf[2] = 0x00;
    writeBuf[3] = 95;
    writeBuf[4] = 63;
    
    if(!spiWrite(writeBuf,5))return false;
    
    return true;
}
bool SPI_OLEDRGB_DrawLine( uint8_t sCol, uint8_t sRow, uint8_t fCol, uint8_t fRow, SPI_OLEDRGB_COLOR* color)
{
    uint8_t writeBuf[10];
    
    writeBuf[0] = 0x21;
    writeBuf[1] = sCol;
    writeBuf[2] = sRow;
    writeBuf[3] = fCol;
    writeBuf[4] = fRow;
    writeBuf[5] = ((color->red >> 3 ) & 0x1F);
    writeBuf[6] = ((color->green >> 2 ) & 0x3F);
    writeBuf[7] = ((color->blue >> 3 ) & 0x1F);
    
    if(!spiWrite(writeBuf,8))return false;
    
    return true;
    
}

bool SPI_OLEDRGB_DrawPixel( uint8_t sCol, uint8_t sRow, uint8_t fCol, uint8_t fRow, uint8_t* data, SPI_OLEDRGB_COLOR* color)
{
    uint8_t writeBuf[10];
    uint16_t loop;
    uint16_t size;
    uint16_t color16;
    
    uint16_t bufIdx;
    uint8_t bufPos;
    
    writeBuf[0] = 0x15;
    writeBuf[1] = sCol;
    writeBuf[2] = fCol;
    writeBuf[3] = 0x75;
    writeBuf[4] = sRow;
    writeBuf[5] = fRow;
    
    if(!spiWrite(writeBuf, 6))return false;
    
    DCOn();
    size = (fRow-sRow+1)*(fCol-sCol+1);
    for( loop = 0; loop<size; loop++){
        
        bufIdx = loop/8;
        bufPos = loop%8;
        
        if(( data[bufIdx] >> (7 - bufPos)) & 0x01){
            color16 = getColor16(color);
        } else {
            color16 = 0x0000;
        }
        writeBuf[0] = (uint8_t)(color16>>8);
        writeBuf[1] = (uint8_t)(color16);
        spiWrite(writeBuf,2);
    }
    DCOff();
    
    return true;
}

bool SPI_OLEDRGB_DrawAscii( uint8_t col, uint8_t row, char ascii, SPI_OLEDRGB_COLOR* color )
{
    uint8_t writeBuf[10];
    uint8_t i;
    uint8_t j;
    
    uint16_t color16;
    
    if (( col + 4 > MAX_COL) || ( row + 6 > MAX_ROW))return false;
    
    CSOff();
    
    writeBuf[0] = 0x15;
    writeBuf[1] = col;
    writeBuf[2] = col+4;
    writeBuf[3] = 0x75;
    writeBuf[4] = row;
    writeBuf[5] = row+6;
    
    if(!spiWrite(writeBuf, 6))return false;
    
    DCOn();
    for( i = 0; i < 7; i++){
        for( j = 0; j < 5; j++){
            if(( FontAscii57[ascii][i] >> (7 - j)) & 0x01){
                color16 = getColor16(color);
            } else {
                color16 = 0x0000;
            }
            writeBuf[0] = (uint8_t)(color16>>8);
            writeBuf[1] = (uint8_t)(color16);
            spiWrite(writeBuf,2);
        }
    }
    
    DCOff();
    
    CSOn();
    
    return true;
}

bool SPI_OLEDRGB_DrawAsciiString( uint8_t col, uint8_t row, char* str, SPI_OLEDRGB_COLOR* color )
{
    char* p = str;
    uint8_t cnt = 0;
    
    if ((( col + strlen(str)*6 ) > MAX_COL ) || ((row + 8) > MAX_ROW )) return false;
    
    while( *p != NULL )
    {
        if(!SPI_OLEDRGB_DrawAscii(col+cnt*6, row, *p, color))return false;
        p++;
        cnt++;
    }
    return true;
}

bool SPI_OLEDRGB_SetRemapDataFormat( uint8_t value )
{
    uint8_t writeBuf[2];
    writeBuf[0] = 0xA0;
    writeBuf[1] = value;
    return spiWrite(&writeBuf, 2);
}

bool SPI_OLEDRGB_DevInit( void )
{
    uint8_t writeBuf[10];
    PMODENOn();
    
    delay(1000);
    
    RESOff();
    delay(1000);
    RESOn();
    
    // unlock
    writeBuf[0] = 0xFD;
    writeBuf[1] = 0x12;
    if(!spiWrite(writeBuf, 2))return false;
    
    // Display Off
    writeBuf[0] = 0xAE;
    if(!spiWrite(writeBuf, 1))return false;
    
    // Set Remap and Data Format
    writeBuf[0] = 0xA0;
    writeBuf[1] = 0x72;
    if(!spiWrite(writeBuf, 2))return false;
    
    // Set Display Start Line
    writeBuf[0] = 0xA1;
    writeBuf[1] = 0x00;
    if(!spiWrite(writeBuf, 2))return false;
    
    // Set Display Start Line
    writeBuf[0] = 0xA2;
    writeBuf[1] = 0x00;
    if(!spiWrite(writeBuf, 2))return false;
    
    // Set Multiplex Ratio
    writeBuf[0] = 0xA8;
    writeBuf[1] = 0x3F;
    if(!spiWrite(writeBuf, 2))return false;
    
    // Set Master Configuration
    writeBuf[0] = 0xAD;
    writeBuf[1] = 0x8E;
    if(!spiWrite(writeBuf, 2))return false;

    // Set Power Saving Mode
    writeBuf[0] = 0xB0;
    writeBuf[1] = 0x0B;
    if(!spiWrite(writeBuf, 2))return false;

	// Set Phase Length
    writeBuf[0] = 0xB1;
    writeBuf[1] = 0x31; //phase 2 = 14 DCLKs, phase 1 = 15 DCLKS     
    if(!spiWrite(writeBuf, 2))return false;


	// Send Clock Divide Ratio and Oscillator Frequency
    writeBuf[0] = 0xB3;
    writeBuf[1] = 0xF0; //mid high oscillator frequency, DCLK = FpbCllk/2
    if(!spiWrite(writeBuf, 2))return false;

	// Set Second Pre-charge Speed of Color A
    writeBuf[0] = 0x8A;
    writeBuf[1] = 0x64; //Set Second Pre-change Speed For ColorA
    if(!spiWrite(writeBuf, 2))return false;

	// Set Set Second Pre-charge Speed of Color B
    writeBuf[0] = 0x8B;
    writeBuf[1] = 0x78;//Set Second Pre-change Speed For ColorB
    if(!spiWrite(writeBuf, 2))return false;

	// Set Second Pre-charge Speed of Color C
    writeBuf[0] = 0x8C;
    writeBuf[1] = 0x64; //Set Second Pre-change Speed For ColorC
    if(!spiWrite(writeBuf, 2))return false;

	// Set Pre-Charge Voltage
    writeBuf[0] = 0xBB;
    writeBuf[1] = 0x3A; // Pre-charge voltage =...Vcc    
    if(!spiWrite(writeBuf, 2))return false;

	// Set VCOMH Deselect Level
    writeBuf[0] = 0xBE;
    writeBuf[1] = 0x3E; // Vcomh = ...*Vcc
    if(!spiWrite(writeBuf, 2))return false;

	// Set Master Current
    writeBuf[0] = 0x87;
    writeBuf[1] = 0x06;
    if(!spiWrite(writeBuf, 2))return false;

	// Set Contrast for Color A
    writeBuf[0] = 0x81;
    writeBuf[1] = 0x91; // Set contrast for color A
	if(!spiWrite(writeBuf, 2))return false;

	// Set Contrast for Color B
    writeBuf[0] = 0x82;
    writeBuf[1] = 0x50; // Set contrast for color B
	if(!spiWrite(writeBuf, 2))return false; 

	// Set Contrast for Color C
    writeBuf[0] = 0x83;
    writeBuf[1] = 0x7D; // Set contrast for color C
	if(!spiWrite(writeBuf, 2))return false;

    writeBuf[0] = 0x2E;   //disable scrolling
	if(!spiWrite(writeBuf, 1))return false;
    
    SPI_OLEDRGB_Clear();
    
    VCCENOn();
    delay(1000);
    
    // Set display ON
    writeBuf[0] = 0xAF;   //disable scrolling
	if(!spiWrite(writeBuf, 1))return false;
    
    delay(300);
    
    return true;
}

/*******************************************************************************
 End of File
 */
