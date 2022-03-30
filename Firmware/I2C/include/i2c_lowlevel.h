/*******************************************************************************
    Filename:       i2c_lowlevel.h
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

#ifndef __I2C_LOWLEVEL_H
#define __I2C_LOWLEVEL_H

#include "stdint.h"

/** G L O B A L   P R O T O T Y P E S ******************************/

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
void I2C_StartCond(void);

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
void I2C_RestartCond(void);

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
void I2C_StopCond(void);

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
void I2C_SendACK(void);

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
void I2C_SendNoACK(void);

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
uint8_t I2C_OutputByte(uint8_t data);

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
uint8_t I2C_InputByte(void);

#endif
