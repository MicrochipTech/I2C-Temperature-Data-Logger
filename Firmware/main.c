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

/** INCLUDES *******************************************************/
#include <xc.h>
#include "system.h"
#include "usb_config.h"
#include "usb.h"
#include "usb_device_hid.h"
#include "HardwareProfile.h"
#include "i2c_module.h"
#include "stdbool.h"

/** C O N S T A N T S **********************************************/
#define USB_EE_READ_CMD         0x80
#define USB_TEMP_READ_CMD       0x81
#define USB_EE_RESET_CMD        0x82

#define USB_EE_RESET_UNLOCK1    0x55
#define USB_EE_RESET_UNLOCK2    0xAA


/** T Y P E D E F S ************************************************/
typedef enum
{
    LOG_MODE,                   // Temperature logging mode
    USB_MODE                    // USB processing mode
} APP_STATES;

/** V A R I A B L E S **********************************************/

unsigned char ReceivedDataBuffer[HID_INT_OUT_EP_SIZE] @ HID_OUT_DATA_BUFFER_ADDRESS;
unsigned char ToSendDataBuffer[HID_INT_IN_EP_SIZE] @ HID_IN_DATA_BUFFER_ADDRESS;

volatile USB_HANDLE USBOutHandle = 0;
volatile USB_HANDLE USBInHandle = 0;

APP_STATES appState = LOG_MODE;

// To save power by reducing the number of EEPROM writes performed, temperature
//   temperature readings are buffered until a full EEPROM page of data is ready.
//   Then the entire buffer is written in a single operation.
uint8_t logBuffer[I2C_EEPROM_PAGE_SIZE]; // Datalogging buffer
uint8_t logBufferIndex;           // Index of current position in datalog buffer
uint16_union logEEAddress;        // Array address for next EEPROM page write

uint8_t logIntervalCount;         // Temperature reading interval counter

extern volatile USB_HANDLE USBOutHandle;
extern volatile USB_HANDLE USBInHandle;

/** P R I V A T E   P R O T O T Y P E S ****************************/
void HighPriorityISRCode(void);
void LowPriorityISRCode(void);
void InitializeSystem(void);
void UserSystemInit(void);
void LogModeInit(void);
void USBModeInit(void);
void ProcessLogIO(void);
void ProcessUSBIO(void);
void FindLastLogEntry(void);
void ResetEEPROMData(void);
void USBCBSendResume(void);
void USBCB_SOF_Handler(void);
void USBCBSuspend(void);
void USBCBWakeFromSuspend(void);
void USBCBInitEP(void);
void USBCBStdSetDscHandler(void);
void USBCBCheckOtherReq(void);
void USBCBErrorHandler(void);


MAIN_RETURN main(void)
{
    // Initialize the general system resources
    InitializeSystem();

    // Determine which state to enter initially and enter it
    if (USB_BUS_SENSE == 1)
        USBModeInit();
    else
        LogModeInit();

    while(1)
    {
        // Ensure status LED is off after any previous temperature readings
        STATUS_LED_LAT = STATUS_LED_OFF;

        // If USB cable is plugged in, then temperature readings are suspended
        //   and we're operating in data transfer mode
        if (USB_BUS_SENSE == 1)
        {
            // If we're not already in USB mode, switch to it now
            if (appState != USB_MODE)
                USBModeInit();

            // Process application-related USB tasks
            ProcessUSBIO();
        }
        // If USB cable is not plugged in, then we need to periodically read
        //   and log the current temperature
        else
        {
            // If we're not already in logging mode, switch to it now
            if (appState != LOG_MODE)
                LogModeInit();

            // Process datalogging tasks
            ProcessLogIO();

            // Datalogging-related tasks are now complete, enter idle mode until
            //   ECCP1 triggers to save power
            OSCCONbits.IDLEN = 1; // Ensure idle enable bit is set
            Sleep();            // Enter idle mode
            PIR1bits.CCP1IF = 0; // Clear CCP1IF flag for next loop
        }
    }//end while

}//end main


bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch((int)event)
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:            
            USBCB_SOF_Handler();
            break;

        case EVENT_SUSPEND:
            USBCBSuspend();
            break;

        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;

        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;

        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;

        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;

        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}



/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.
 *
 * Note:            None
 *******************************************************************/
void InitializeSystem(void)
{
    // Switch from primary oscillator to internal oscillator, and disable
    //   primary oscillator and PLL to save power
    OSCCONbits.IRCF = 0x04;     // Set internal oscillator to 2 MHz
    OSCCONbits.SCS1 = 1;        // Select internal oscillator
    while (OSCCONbits.OSTS);    // Wait for clock switch to complete
    OSCCON2bits.PRI_SD = 0;     // Disable primary oscillator
    OSCTUNEbits.SPLLEN = 0;     // Disable PLL

    // Disable all analog inputs
    ANSEL = 0;
    ANSELH = 0;

//  The USB specifications require that USB peripheral devices must never source
//  current onto the Vbus pin.  Additionally, USB peripherals should not source
//  current on D+ or D- when the host/hub is not actively powering the Vbus line.
//  When designing a self powered (as opposed to bus powered) USB peripheral
//  device, the firmware should make sure not to turn on the USB module and D+
//  or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//  firmware needs some means to detect when Vbus is being powered by the host.
//  A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
//  can be used to detect when Vbus is high (host actively powering), or low
//  (host is shut down or otherwise not supplying power).  The USB firmware
//  can then periodically poll this I/O pin to know when it is okay to turn on
//  the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//  peripheral device, it is not possible to source current on D+ or D- when the
//  host is not actively providing power on Vbus. Therefore, implementing this
//  bus sense feature is optional.  This firmware can be made to use this bus
//  sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//  HardwareProfile.h file.
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif

//  If the host PC sends a GetStatus (device) request, the firmware must respond
//  and let the host know if the USB peripheral device is currently bus powered
//  or self powered.  See chapter 9 in the official USB specifications for details
//  regarding this request.  If the peripheral device is capable of being both
//  self and bus powered, it should not return a hard coded value for this request.
//  Instead, firmware should check if it is currently self or bus powered, and
//  respond accordingly.  If the hardware has been configured like demonstrated
//  on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//  currently selected power source.  On the PICDEM FS USB Demo Board, "RA2"
//  is used for this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//  has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//  to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
    #endif

    USBDeviceInit();            //usb_device.c.  Initializes USB module SFRs and firmware
                                //variables to known states.

    UserSystemInit();           // Perform app-related system initialization
}//end InitializeSystem

/******************************************************************************
 * Function:        void UserSystemInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine takes care of all of the application-related
 *                  system initialization.
 *
 * Note:            None
 *****************************************************************************/
void UserSystemInit(void)
{
    // Configure status LED pin
    STATUS_LED_LAT = STATUS_LED_OFF; // Turn LED off by default
    STATUS_LED_TRIS = OUTPUT_PIN; // Configure pin as an output

    // Initialize the I2C module - calculate initial baud rate based on the
    //   logging mode system clock - it will get reconfigured later
    I2C_Init((GetLogInstClock() / GetI2CLogClock()) - 1);
}//end UserSystemInit

/******************************************************************************
 * Function:        void LogModeInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine switches the system over to logging mode.
 *
 * Note:            None
 *****************************************************************************/
void LogModeInit(void)
{
    // Make sure USB module is disabled (safe to call every loop)
    USBDeviceDetach();

    // USB operations are suspended until cable is plugged in, so make sure
    //   internal oscillator is selected to save power
    if (OSCCONbits.OSTS)
    {
        OSCCONbits.SCS1 = 1;    // Select internal oscillator
        while (OSCCONbits.OSTS); // Wait for clock switch to complete
        OSCCON2bits.PRI_SD = 0; // Disable primary oscillator
        OSCTUNEbits.SPLLEN = 0; // Disable PLL
    }

    // Configure Timer1/ECCP1 modules for special event trigger compare mode to
    //   maintain datalogging time base
    CCPR1L = (uint8_t)(GetLogInstClock() / GetLoggingLoopClock()); // Set timebase
    CCPR1H = (uint8_t)((GetLogInstClock() / GetLoggingLoopClock()) >> 8);
    T3CONbits.T3CCP1 = 0;       // Assign Timer1 to ECCP1
    CCP1CON = 0b00001011;       // Select special event trigger compare mode
    TMR1H = 0;                  // Clear Timer1 count MSB buffer
    TMR1L = 0;                  // Clear Timer1 count LSB and latch MSB buffer
    T1CON = 0b10000000;         // Select system clock source, 1:1 prescaler, 16-bit mode
    PIR1bits.CCP1IF = 0;        // Ensure CCP1IF flag is cleared
    PIE1bits.CCP1IE = 1;        // Enable CCP1 interrupt flag to exit idle mode
    T1CONbits.TMR1ON = 1;       // Enable Timer1
    RCONbits.IPEN = 0;          // Disable interrupt priorities
    INTCONbits.PEIE = 1;        // Ensure peripheral interrupts are enabled
    INTCONbits.GIE = 0;         // Ensure global interrupts are disabled

    // Set the I2C baud rate based on the logging mode system clock
    I2C_SetBaudRate((GetLogInstClock() / GetI2CLogClock()) - 1);

    // Switch application state to logging mode
    appState = LOG_MODE;

    // Reset datalog interval counter (used to measure time between log intervals)
    logIntervalCount = 0;

    // Find last datalog entry - this automatically initializes logBuffer,
    //   logBufferIndex, and logEEAddress
    FindLastLogEntry();
}//end LogModeInit

/******************************************************************************
 * Function:        void USBModeInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine switches the system over to USB mode.
 *
 * Note:            None
 *****************************************************************************/
void USBModeInit(void)
{
    // Initialize the handles for the last USB transmissions
    USBOutHandle = 0;
    USBInHandle = 0;

    // Disable Timer1 and ECCP1 modules
    T1CONbits.TMR1ON = 0;       // Disable Timer1
    CCP1CON = 0;                // Disable ECCP1

    // Before proceeding with USB operation, make sure primary oscillator
    //   and PLL are enabled, stable, and selected
    if (!OSCCONbits.OSTS)
    {
        OSCCON2bits.PRI_SD = 1; // Enable primary oscillator
        OSCTUNEbits.SPLLEN = 1; // Enable PLL
        OSCCONbits.SCS1 = 0; // Select primary oscillator
        while (!OSCCONbits.OSTS); // Wait for clock switch to complete
    }

    // Set the I2C baud rate based on the USB mode system clock
    I2C_SetBaudRate((GetUSBInstClock() / GetI2CUSBClock()) - 1);

    // Make sure USB module is enabled (safe to call every loop)
    USBDeviceAttach();

    // Switch application state to USB mode
    appState = USB_MODE;
}//end USBModeInit

/********************************************************************
 * Function:        void ProcessLogIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function handles temperature datalogging
 *                  tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessLogIO(void)
{
    // Make sure we don't go past the end of the EEPROM array
    if (logEEAddress.value >= I2C_EEPROM_DENSITY)
        return;

    // Check if next temperature reading needs to be performed
    if (++logIntervalCount >= READING_INTERVAL_COUNT)
    {
        // Turn on status LED to indicate a reading is being taken - the LED
        //   will automatically be turned off at the beginning of the next
        //   system loop, allowing it to remain on for nearly 1 loop interval
        STATUS_LED_LAT = STATUS_LED_ON;

        // Read and store temperature from sensor
        I2C_ReadTempSensorBlock(I2C_TEMPSENSOR_TEMPREG, 2, &logBuffer[logBufferIndex]);

        // Increment buffer index
        logBufferIndex += 2;

        // Increment buffer index for next reading and check if buffer is full
        if (logBufferIndex >= I2C_EEPROM_PAGE_SIZE)
        {
            // Buffer is full, dump to EEPROM
            I2C_WriteEEBlock(logEEAddress.byte.HB, logEEAddress.byte.LB,
                             I2C_EEPROM_PAGE_SIZE, logBuffer);

            // Increment EEPROM array address
            logEEAddress.value += I2C_EEPROM_PAGE_SIZE;

            // Reset buffer index
            logBufferIndex = 0;
        }

        // Reset log interval counter
        logIntervalCount = 0;
    }
}//end ProcessLogIO

/********************************************************************
 * Function:        void ProcessUSBIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function handles application-related USB
 *                  tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessUSBIO(void)
{
    // If USB module is not yet in a configured state or if it's suspended, then
    //   do not proceed
    if ((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1))
        return;

    if (!HIDRxHandleBusy(USBOutHandle))             // Check if data was received from the host
    {
        // USB data was received from host - process it here
        switch (ReceivedDataBuffer[0])              // Check byte 0 for command value
        {
            case USB_EE_READ_CMD:                   // EEPROM read command received
                // Ensure HIDTxHandle is not busy
                while (HIDTxHandleBusy(USBInHandle));

                // Read HID_INT_IN_EP_SIZE bytes from EEPROM starting at address
                //   specified by command, storing result in USB data buffer
                I2C_ReadEEBlock(ReceivedDataBuffer[2], ReceivedDataBuffer[1],
                                HID_INT_IN_EP_SIZE, ToSendDataBuffer);

                // Initiate the USB transfer back to the host
                USBInHandle = HIDTxPacket(HID_EP,(uint8_t*)&ToSendDataBuffer,HID_INT_IN_EP_SIZE);
                break;

            case USB_TEMP_READ_CMD:                 // Temp sensor read command received
                // Ensure HIDTxHandle is not busy
                while (HIDTxHandleBusy(USBInHandle));

                // Read current temperature from sensor - note that temp reading
                //   only requires 2 bytes in the buffer, so the remaining
                //   (HID_INT_IN_EP_SIZE - 2) bytes are undefined and should be
                //   ignored by the host
                I2C_ReadTempSensorBlock(I2C_TEMPSENSOR_TEMPREG, 2, ToSendDataBuffer);

                // Initiate the USB transfer back to the host
                USBInHandle = HIDTxPacket(HID_EP,(uint8_t*)&ToSendDataBuffer,HID_INT_IN_EP_SIZE);
                break;

            case USB_EE_RESET_CMD:                  // EEPROM reset command received
                // Verify unlock values
                if (ReceivedDataBuffer[1] == USB_EE_RESET_UNLOCK1 &&
                    ReceivedDataBuffer[2] == USB_EE_RESET_UNLOCK2)
                {
                    // Unlock values are valid, reset EEPROM by filling the entire
                    //   EEPROM array with a known pattern to mark data as invalid
                    ResetEEPROMData();

                    // Indicate to host that sequence was valid
                    ToSendDataBuffer[0] = USB_EE_RESET_CMD;
                    ToSendDataBuffer[1] = 1;
                }
                else
                {
                    // Indicate to host that sequence was not valid
                    ToSendDataBuffer[0] = USB_EE_RESET_CMD;
                    ToSendDataBuffer[1] = 0;
                }

                // Initiate the USB transfer back to the host
                USBInHandle = HIDTxPacket(HID_EP,(uint8_t*)&ToSendDataBuffer,HID_INT_IN_EP_SIZE);
                break;
        }

        // Re-arm the OUT endpoint for the next packet
        USBOutHandle = HIDRxPacket(HID_EP,(uint8_t*)&ReceivedDataBuffer,HID_INT_OUT_EP_SIZE);
    }
}//end ProcessUSBIO

/******************************************************************************
 * Function:        void FindLastLogEntry(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    logBufferIndex, logBuffer, and logEEAddress are overwritten
 *
 * Overview:        This routine loops through the EEPROM one page at a time,
 *                  looking for the last valid temperature log entry. Afterwards,
 *                  the global variables logBufferIndex, logBuffer, and
 *                  logEEAddress will be configured appropriately to continue
 *                  logging.
 *
 * Note:            After calling this function, logBuffer is left populated with
 *                  data from the last EEPROM page read, so that logging can be
 *                  restarted anywhere within a page without losing data.
 *****************************************************************************/
void FindLastLogEntry(void)
{
    // Turn LED off initially - it will be toggled while searching the EEPROM
    STATUS_LED_LAT = STATUS_LED_OFF;

    // Loop through EEPROM array, reading 1 page at a time and checking if the
    //   last log entry is in that page (i.e. there are both valid and invalid
    //   entries in the same page)
    for (logEEAddress.value = 0; logEEAddress.value < I2C_EEPROM_DENSITY; logEEAddress.value += I2C_EEPROM_PAGE_SIZE)
    {
        // Toggle status LED
        STATUS_LED_LAT = ~STATUS_LED_LAT;

        // Read page of data from EEPROM
        I2C_ReadEEBlock(logEEAddress.byte.HB, logEEAddress.byte.LB,
                         I2C_EEPROM_PAGE_SIZE, logBuffer);

        // To save time in case the page is full of valid data, check the last
        //   entry first, keeping in mind data is stored big-endian
        if ((logBuffer[I2C_EEPROM_PAGE_SIZE - 2] != (unsigned char)(INVALID_EEPROM_DATA >> 8)) ||
            (logBuffer[I2C_EEPROM_PAGE_SIZE - 1] != (unsigned char)INVALID_EEPROM_DATA))
            continue;

        // Last entry must have been invalid, so loop through buffer looking for
        //   first invalid entry
        for (logBufferIndex = 0; logBufferIndex < I2C_EEPROM_PAGE_SIZE; logBufferIndex += 2)
        {
            if ((logBuffer[logBufferIndex] == (unsigned char)(INVALID_EEPROM_DATA >> 8)) &&
                (logBuffer[logBufferIndex + 1] == (unsigned char)INVALID_EEPROM_DATA))
            {
                // The first invalid entry was found, make sure status LED is off
                //   and return
                STATUS_LED_LAT = STATUS_LED_OFF;
                return;
            }
        }
    }

    // If no invalid entries were found (indicating the EEPROM is full of data),
    //   then logEEAddress would be past the end of the array. This would prevent
    //   additional logging automatically, so we don't need to perform any extra
    //   checks here.

    // Turn LED off
    STATUS_LED_LAT = STATUS_LED_OFF;
}

/******************************************************************************
 * Function:        void ResetEEPROMData(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    logBufferIndex, logBuffer, and logEEAddress are overwritten
 *
 * Overview:        This routine writes INVALID_EEPROM_DATA to the entire
 *                  EEPROM array, thereby marking all values as invalid data.
 *
 * Note:            To avoid wasting memory, this function uses the global
 *                  variables logBufferIndex, logBuffer, and logEEAddress.
 *****************************************************************************/
void ResetEEPROMData(void)
{
    // Turn LED on to indicate EEPROM is being reset
    STATUS_LED_LAT = STATUS_LED_ON;

    // Fill the log buffer with the invalid data value
    for (logBufferIndex = 0; logBufferIndex < I2C_EEPROM_PAGE_SIZE;)
    {
        // Data from the temperature sensor is stored big-endian, so we must also
        //   store the INVALID_EEPROM_DATA values big-endian
        logBuffer[logBufferIndex++] = (unsigned char)(INVALID_EEPROM_DATA >> 8);
        logBuffer[logBufferIndex++] = (unsigned char)INVALID_EEPROM_DATA;
    }

    // Loop through EEPROM array, writing buffer to every page in EEPROM
    for (logEEAddress.value = 0; logEEAddress.value < I2C_EEPROM_DENSITY; logEEAddress.value += I2C_EEPROM_PAGE_SIZE)
    {
        // Write page of data to EEPROM
        I2C_WriteEEBlock(logEEAddress.byte.HB, logEEAddress.byte.LB,
                         I2C_EEPROM_PAGE_SIZE, logBuffer);

        // Determine which oscillator is running in order to calculate write
        //   cycle delay correctly
        if (OSCCONbits.OSTS)
        {
            // OSTS is high, running off of primary oscillator used for USB

            // Delay for write cycle time before proceeding
            //   (the "(GetUSBInstClock() / 1000)" equation calculates the
            //   number of clocks in 1 ms)
            _delay((GetUSBInstClock() / 1000) * I2C_EEPROM_TWC_MS);

        }
        else
        {
            // OSTS is low, running off of internal oscillator used for logging

            // Delay for write cycle time before proceeding
            //   (the "(GetLogInstClock() / 1000)" equation calculates the
            //   number of clocks in 1 ms)
            _delay((GetLogInstClock() / 1000) * I2C_EEPROM_TWC_MS);

        }
    }

    // Reset EEPROM array address for logging, datalog buffer index, and interval counter
    logEEAddress.value = 0;       // Reset EEPROM array address
    logBufferIndex = 0;         // Reset datalog buffer index
    logIntervalCount = 0;       // Reset temperature reading interval count

    // Turn LED off to indicate EEPROM reset is complete
    STATUS_LED_LAT = STATUS_LED_OFF;
}


/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:

    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();   //should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();    //Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();                                  //Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
    //things to not work as intended.


    #if defined(__C30__)
        //This function requires that the _IPL level be something other than 0.
        //  We can set it here to something other than
        #ifndef DSPIC33E_USB_STARTER_KIT
        _IPL = 1;
        USBSleepOnSuspend();
        #endif
    #endif
}


/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *                  suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *                  mode, the host may wake the device back up by sending non-
 *                  idle state signalling.
 *
 *                  This call back is invoked when a wakeup from USB suspend
 *                  is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
    // If clock switching or other power savings measures were taken when
    // executing the USBCBSuspend() function, now would be a good time to
    // switch back to normal full power run mode conditions.  The host allows
    // a few milliseconds of wakeup time, after which the device must be
    // fully back to normal, and capable of receiving and processing USB
    // packets.  In order to do this, the USB module must receive proper
    // clocking (IE: 48MHz clock must be available to SIE for full speed USB
    // operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

    // Typically, user firmware does not need to do anything special
    // if a USB error occurs.  For example, if the host sends an OUT
    // packet to your device, but the packet gets corrupted (ex:
    // because of a bad connection, or the user unplugs the
    // USB cable during the transmission) this will typically set
    // one or more USB error interrupt flags.  Nothing specific
    // needs to be done however, since the SIE will automatically
    // send a "NAK" packet to the host.  In response to this, the
    // host will normally retry to send the packet again, and no
    // data loss occurs.  The system will typically recover
    // automatically, without the need for application firmware
    // intervention.

    // Nevertheless, this callback function is provided, such as
    // for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *                  firmware must process the request and respond
 *                  appropriately to fulfill the request.  Some of
 *                  the SETUP packets will be for standard
 *                  USB "chapter 9" (as in, fulfilling chapter 9 of
 *                  the official USB specifications) requests, while
 *                  others may be specific to the USB device class
 *                  that is being implemented.  For example, a HID
 *                  class device needs to be able to respond to
 *                  "GET REPORT" type of requests.  This
 *                  is not a standard USB chapter 9 request, and
 *                  therefore not handled by usb_device.c.  Instead
 *                  this request should be handled by class specific
 *                  firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *                  called when a SETUP, bRequest: SET_DESCRIPTOR request
 *                  arrives.  Typically SET_DESCRIPTOR requests are
 *                  not used in most applications, and it is
 *                  optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *                  SET_CONFIGURATION (wValue not = 0) request.  This
 *                  callback function should initialize the endpoints
 *                  for the device's usage according to the current
 *                  configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = HIDRxPacket(HID_EP,(uint8_t*)&ReceivedDataBuffer,HID_INT_OUT_EP_SIZE);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 *                  peripheral devices to wake up a host PC (such
 *                  as if it is in a low power suspend to RAM state).
 *                  This can be a very useful feature in some
 *                  USB applications, such as an Infrared remote
 *                  control receiver.  If a user presses the "power"
 *                  button on a remote control, it is nice that the
 *                  IR receiver can detect this signalling, and then
 *                  send a USB "command" to the PC to wake up.
 *
 *                  The USBCBSendResume() "callback" function is used
 *                  to send this special USB signalling which wakes
 *                  up the PC.  This function may be called by
 *                  application firmware to wake up the PC.  This
 *                  function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *                  1.  The USB driver used on the host PC supports
 *                      the remote wakeup capability.
 *                  2.  The USB configuration descriptor indicates
 *                      the device is remote wakeup capable in the
 *                      bmAttributes field.
 *                  3.  The USB host PC is currently sleeping,
 *                      and has previously sent your device a SET
 *                      FEATURE setup packet which "armed" the
 *                      remote wakeup capability.
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior,
 *                  as a USB device that has not been armed to perform remote
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *
 *                  This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex:
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup.
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static uint16_t delay_count;

    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager
    //properties page for the USB device, power management tab, the
    //"Allow this device to bring the computer out of standby." checkbox
    //should be checked).
    if(USBGetRemoteWakeupStatus() == true)
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == true)
        {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = false;  //So we don't execute this code again,
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;
            do
            {
                delay_count--;
            }while(delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************************
 End of File
*/

