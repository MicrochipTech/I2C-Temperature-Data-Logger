/*******************************************************************************
    Filename:       i2c_init.c
    Dependencies:   See INCLUDES section
    Processor:      PIC18F13K50
    Hardware:       This demo implements an I2C temperature data logger. It is 
                    designed to teach the basics of I2C.
    Complier:       Microchip XC8 V2.35
    Company:        Microchip Technology Inc.
*******************************************************************************/
/*******************************************************************************
    (c) 2022 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip
    software and any derivatives exclusively with Microchip products. It is
    your responsibility to comply with third party license terms applicable to
    your use of third party software (including open source software) that may
    accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*******************************************************************************/
/*******************************************************************************
    File Description:
 
    Change History:
        Rev   Description
        ----  -----------------------------------------
        1.0   Initial release
        2.0   General code cleanup, no functional change
*******************************************************************************/

/** INCLUDES *******************************************************/
#include <xc.h>
#include "HardwareProfile.h"
#include "i2c_lowlevel.h"
#include "i2c_module.h"

/********************************************************************
 * Function:        void I2C_Init(UINT8 baud)
 *
 * PreCondition:    None
 *
 * Input:           baud - The I2C baud rate to use
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Opens the I2C channel for communication by
 *                  configuring the necessary I/O pins and the
 *                  I2C1 module.
 *
 * Note:            None
 *******************************************************************/
void I2C_Init(uint8_t baud)
{
    // Set default pin states and directions
    I2C_SCL_LAT = 0;            // Set I2C SCL latch low
    I2C_SDA_LAT = 0;            // Set I2C SDA latch low
    I2C_SCL_TRIS = 1;           // Configure I2C SCL pin as an input
    I2C_SDA_TRIS = 1;           // Configure I2C SDA pin as an input

    // Configure I2C1 module
    SSPCON1bits.SSPEN = 0;      // Ensure I2C module is disabled before configuring
    SSPSTAT = 0b00000000;       // Enable slew rate control, disable SMBus inputs
    SSPCON1 = 0b00001000;       // Select I2C master mode
    SSPADD = baud;              // Set baud rate
    SSPCON1bits.SSPEN = 1;      // Enable I2C module
}

/********************************************************************
 * Function:        void I2C_SetBaudRate(UINT8 baud)
 *
 * PreCondition:    None
 *
 * Input:           baud - The I2C baud rate to use
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets the I2C baud rate to the specified value.
 *
 * Note:            None
 *******************************************************************/
void I2C_SetBaudRate(uint8_t baud)
{
    SSPCON1bits.SSPEN = 0;      // Disable I2C module
    SSPADD = baud;              // Set baud rate
    SSPCON1bits.SSPEN = 1;      // Enable I2C module
}
