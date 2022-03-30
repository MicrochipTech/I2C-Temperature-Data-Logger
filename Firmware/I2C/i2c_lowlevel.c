/*******************************************************************************
    Filename:        i2c_lowlevel.c
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

/********************************************************************
 * Function:        void I2C_StartCond(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Generates a Start condition.
 *
 * Note:            None
 *******************************************************************/
void I2C_StartCond(void)
{
    SSPCON2bits.SEN = 1;        // Initiate start condition
    while (SSPCON2bits.SEN);    // Wait for start condition to complete
}

/********************************************************************
 * Function:        void I2C_RestartCond(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Generates a Restart condition.
 *
 * Note:            None
 *******************************************************************/
void I2C_RestartCond(void)
{
    SSPCON2bits.RSEN = 1;       // Initiate restart condition
    while (SSPCON2bits.RSEN);   // Wait for restart condition to complete
}

/********************************************************************
 * Function:        void I2C_StopCond(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Generates a Stop condition.
 *
 * Note:            None
 *******************************************************************/
void I2C_StopCond(void)
{
    SSPCON2bits.PEN = 1;        // Initiate stop condition
    while (SSPCON2bits.PEN);    // Wait for stop condition to complete
}

/********************************************************************
 * Function:        void I2C_SendACK(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sends an ACK bit.
 *
 * Note:            This function should only be used after inputting
 *                  a byte. When outputting a byte, the ACK sequence
 *                  is automatically performed.
 *******************************************************************/
void I2C_SendACK(void)
{
    // Perform ACK sequence
    SSPCON2bits.ACKDT = 0;      // Set value for ACK
    SSPCON2bits.ACKEN = 1;      // Initiate ACK sequence
    while (SSPCON2bits.ACKEN);  // Wait for ACK sequence to complete
}

/********************************************************************
 * Function:        void I2C_SendNoACK(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sends a NoACK bit.
 *
 * Note:            This function should only be used after inputting
 *                  a byte. When outputting a byte, the ACK sequence
 *                  is automatically performed.
 *******************************************************************/
void I2C_SendNoACK(void)
{
    // Perform ACK sequence
    SSPCON2bits.ACKDT = 1;      // Set value for NoACK
    SSPCON2bits.ACKEN = 1;      // Initiate ACK sequence
    while (SSPCON2bits.ACKEN);  // Wait for ACK sequence to complete
}

/********************************************************************
 * Function:        UINT8 I2C_OutputByte(UINT8 data)
 *
 * PreCondition:    None
 *
 * Input:           data - Data byte to output
 *
 * Output:          Value of ACK bit received
 *
 * Side Effects:    None
 *
 * Overview:        Outputs a byte on the I2C bus and checks for an
 *                  ACK.
 *
 * Note:            None
 *******************************************************************/
uint8_t I2C_OutputByte(uint8_t data)
{
    // Output data byte
    PIR1bits.SSPIF = 0;         // Clear SSPIF flag initially
    SSPBUF = data;              // Initiate byte transmission
    while (!PIR1bits.SSPIF);    // Wait for transmission to complete

    // Return value of ACK bit
    if (SSPCON2bits.ACKSTAT)
        return 1;
    else
        return 0;
}

/********************************************************************
 * Function:        UINT8 I2C_InputByte(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Data byte received
 *
 * Side Effects:    None
 *
 * Overview:        Inputs a byte on the I2C bus.
 *
 * Note:            This function does not automatically send an ACK
 *                  or NoACK bit following reception of the data
 *                  byte. The I2C_AckSeq() function should be used
 *                  to do this after calling this function.
 *******************************************************************/
uint8_t I2C_InputByte(void)
{
    // Initiate byte reception
    PIR1bits.SSPIF = 0;         // Clear SSPIF flag initially
    SSPCON2bits.RCEN = 1;       // Start receiving data byte
    while (!PIR1bits.SSPIF);    // Wait for receive to complete

    return SSPBUF;              // Return byte read
}
