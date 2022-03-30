/*******************************************************************************
    Filename:       HardwareProfile.h
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

#ifndef __HARDWARE_PROFILE_H
#define __HARDWARE_PROFILE_H

/*******************************************************************/
/******** USB stack hardware selection options *********************/
/*******************************************************************/
//This section is the set of definitions required by the MCHPFSUSB
//  framework.  These definitions tell the firmware what mode it is
//  running in, and where it can find the results to some information
//  that the stack needs.
//These definitions are required by every application developed with
//  this revision of the MCHPFSUSB framework.  Please review each
//  option carefully and determine which options are desired/required
//  for your application.

#define self_power              0

#define USE_USB_BUS_SENSE_IO
#define tris_usb_bus_sense      TRISCbits.TRISC2
#define USB_BUS_SENSE           PORTCbits.RC2

/*******************************************************************/
/** Application specific definitions *******************************/
/*******************************************************************/

// Clock frequency constants - when in logging mode, HF internal oscillator
//   divided down to 2 MHz is selected. When in USB mode, primary oscillator
//   boosted to 48 MHz via PLL then divided down to 12 MHz is selected.
#define LOG_SYS_CLOCK_FREQ      (2000000ul)
#define GetLogSystemClock()     (LOG_SYS_CLOCK_FREQ)
#define GetLogInstClock()       (GetLogSystemClock()/4)
#define USB_SYS_CLOCK_FREQ      (12000000ul)
#define GetUSBSystemClock()     (USB_SYS_CLOCK_FREQ)
#define GetUSBInstClock()       (GetUSBSystemClock()/4)

// Datalogging timebase frequency (in Hz) - this determines how often the system
//   runs through the main loop when in datalogging mode
#define GetLoggingLoopClock()   (50)
// READING_INTERVAL_COUNT defines how many times the main system loop must
//   iterate before a temperature reading is taken.
//   Examples:
//   (GetLoggingLoopClock() * 5) = 1 reading every 5 seconds
//   (GetLoggingLoopClock() / 1) = 1 reading per second
//   (GetLoggingLoopClock() / 2) = 2 readings per second
//   (GetLoggingLoopClock() / 5) = 5 readings per second
#define READING_INTERVAL_COUNT  (GetLoggingLoopClock() / 1)

// I2C clock frequencies - in logging mode, the MCU is not running fast enough
//   to support 400 kHz
#define GetI2CLogClock()        (250000ul)
#define GetI2CUSBClock()        (400000ul)

/** I 2 C   P E R I P H E R A L   D E F I N I T I O N S ************/
#define I2C_EEPROM_PAGE_SIZE    (32)
#define I2C_EEPROM_WRITE_ADDR   (0xA0)
#define I2C_EEPROM_READ_ADDR    (I2C_EEPROM_WRITE_ADDR | 0x01)
#define I2C_EEPROM_DENSITY      (0x2000)
#define I2C_EEPROM_TWC_MS       (5)

#define I2C_TEMP_WRITE_ADDR     (0x30)
#define I2C_TEMP_READ_ADDR      (I2C_TEMP_WRITE_ADDR | 0x01)
#define I2C_TEMPSENSOR_TEMPREG  (0x05)

// A temp sensor value of 0x1000 equates to -256C, which will never occur, so
//   we can use that to mark which EEPROM data is not a valid reading
#define INVALID_EEPROM_DATA     (0x1000)

/** I / O   P I N   D E F I N I T I O N S **************************/
#define INPUT_PIN               1
#define OUTPUT_PIN              0

#define I2C_SCL_LAT             LATBbits.LATB6
#define I2C_SCL_TRIS            TRISBbits.TRISB6
#define I2C_SDA_LAT             LATBbits.LATB4
#define I2C_SDA_TRIS            TRISBbits.TRISB4

#define STATUS_LED_LAT          LATCbits.LATC7
#define STATUS_LED_TRIS         TRISCbits.TRISC7
#define STATUS_LED_ON           1
#define STATUS_LED_OFF          0

#endif  //HARDWARE_PROFILE_H
