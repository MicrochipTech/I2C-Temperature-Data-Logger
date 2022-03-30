/*******************************************************************************
    Filename:        i2c_write_ee.c
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
 * Function:        void I2C_WriteEEBlock(UINT8 addrHigh, UINT8 addrLow
 *                                        UINT8 numBytes, UINT8 *data)
 *
 * PreCondition:    None
 *
 * Input:           addrHigh - Array address MSB from which to start
 *                             reading
 *                  addrLow - Array address LSB from which to start
 *                            reading
 *                  numBytes - Number of bytes to write
 *                  data - Pointer to memory block of data to write
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Write a block of data to the EEPROM.
 *
 * Note:            This function does not prevent crossing page
 *                  boundaries. It is the responsibility of the
 *                  calling function to avoid this.
 *
 *                  This function also does not delay for the
 *                  EEPROM's internal write cycle to complete. Any
 *                  attempted read or write operations during the
 *                  write cycle will fail.
 *******************************************************************/
void I2C_WriteEEBlock(uint8_t addrHigh, uint8_t addrLow, uint8_t numBytes, uint8_t *data)
{
    uint8_t i;                    // Loop counter

    // Start write operation
    I2C_StartCond();            // Generate start condition
    I2C_OutputByte(I2C_EEPROM_WRITE_ADDR); // Output EEPROM write control byte
    I2C_OutputByte(addrHigh);   // Output address byte MSB
    I2C_OutputByte(addrLow);    // Output address byte LSB

    // Loop through number of data bytes to write
    for (i = 0; i < numBytes; i++)
    {
        I2C_OutputByte(data[i]); // Output next data byte
    }

    // End write operation
    I2C_StopCond();             // Generate stop condition
}
