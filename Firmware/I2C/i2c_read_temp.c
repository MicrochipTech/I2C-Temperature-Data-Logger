/*******************************************************************************
    Filename:        i2c_read_temp.c
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
 * Function:        void I2C_ReadTempSensorBlock(UINT8 address,
 *                                               UINT8 numBytes,
 *                                               UINT8 *data)
 *
 * PreCondition:    None
 *
 * Input:           address - Register address from which to start
 *                            reading
 *                  numBytes - Number of bytes to read
 *                  data - Pointer to memory block in which to store
 *                         data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Reads a block of data from the temperature
 *                  sensor.
 *
 * Note:            None
 *******************************************************************/
void I2C_ReadTempSensorBlock(uint8_t address, uint8_t numBytes, uint8_t *data)
{
    uint8_t i;                    // Loop counter

    I2C_StartCond();            // ### 3.1 Generate start condition
    I2C_OutputByte(I2C_TEMP_WRITE_ADDR); // ### 3.2 Output temp sensor write control byte
    I2C_OutputByte(address);    // ### 3.3 Output register address byte

    // Start read operation
    I2C_RestartCond();          // ### 3.4 Generate restart condition
    I2C_OutputByte(I2C_TEMP_READ_ADDR); // ### 3.5 Output temp sensor read control byte

    // Loop through number of data bytes to read
    for (i = 0; i < numBytes; i++)
    {
        data[i] = I2C_InputByte(); // ### 3.6 Read and store next data byte

        // If this is not the last iteration of the loop, then send an ACK bit
        //   to request more data. Otherwise, send a NoACK bit.
        if (i < (numBytes - 1))
            I2C_SendACK();      // ### 3.7 Send an ACK to request more data
        else
            I2C_SendNoACK();    // ### 3.7 Send a NoACK to end the operation
    }

    // End read operation
    I2C_StopCond();             // ### 3.8 Generate stop condition
}
