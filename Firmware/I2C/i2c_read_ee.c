/*******************************************************************************
    Filename:       i2c_read_ee.c
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
 * Function:        void I2C_ReadEEBlock(UINT8 addrHigh, UINT8 addrLow
 *                                       UINT16 numBytes, UINT8 *data)
 *
 * PreCondition:    None
 *
 * Input:           addrHigh - Array address MSB from which to start
 *                             reading
 *                  addrLow - Array address LSB from which to start
 *                            reading
 *                  numBytes - Number of bytes to read
 *                  data - Pointer to memory block in which to store
 *                         data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Reads a block of data from the EEPROM.
 *
 * Note:            None
 *******************************************************************/
void I2C_ReadEEBlock(uint8_t addrHigh, uint8_t addrLow, uint16_t numBytes, uint8_t *data)
{
    uint16_t i;                   // Loop counter

    // Set address pointer
    I2C_StartCond();            // Generate start condition
    I2C_OutputByte(I2C_EEPROM_WRITE_ADDR); // Output EEPROM write control byte
    I2C_OutputByte(addrHigh);   // Output address byte MSB
    I2C_OutputByte(addrLow);    // Output address byte LSB

    // Start read operation
    I2C_RestartCond();          // Generate restart condition
    I2C_OutputByte(I2C_EEPROM_READ_ADDR); // Output EEPROM read control byte

    // Loop through number of data bytes to read
    for (i = 0; i < numBytes; i++)
    {
        data[i] = I2C_InputByte(); // Read and store next data byte

        // If this is not the last iteration of the loop, then send an ACK bit
        //   to request more data. Otherwise, send a NoACK bit.
        if (i < (numBytes - 1))
            I2C_SendACK();      // Send an ACK to request more data
        else
            I2C_SendNoACK();    // Send a NoACK to end the operation
    }

    // End read operation
    I2C_StopCond();             // Generate stop condition
}
