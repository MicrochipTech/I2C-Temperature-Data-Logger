//DOM-IGNORE-BEGIN
/*********************************************************************
 * The following lines are used by VDI.
 * GUID=E537A0C0-6FEE-4afd-89B9-0C35BF72A80B
 * GUIInterfaceVersion=1.00
 * LibraryVersion=2.4
 *********************************************************************/
//DOM-IGNORE-END
/*******************************************************************************

    USB Header File

Summary:
    This file aggregates all necessary header files for the Microchip USB Host,
    Device, and OTG libraries.  It provides a single-file can be included in
    application code.  The USB libraries simplify the implementation of USB
    applications by providing an abstraction of the USB module and its registers
    and bits such that the source code for the can be the same across various
    hardware platforms.

Description:
    This file aggregates all necessary header files for the Microchip USB Host,
    Device, and OTG libraries.  It provides a single-file can be included in
    application code.  The USB libraries simplify the implementation of USB
    applications by providing an abstraction of the USB module and its registers
    and bits such that the source code for the can be the same across various
    hardware platforms.
    
    Note that this file does not include the header files for any client or
    function drivers.
    
    This file is located in the "\<Install Directory\>\\Microchip\\Include\\USB"
    directory.
    
    When including this file in a new project, this file can either be
    referenced from the directory in which it was installed or copied
    directly into the user application folder. If the first method is
    chosen to keep the file located in the folder in which it is installed
    then include paths need to be added so that the library and the
    application both know where to reference each others files. If the
    application folder is located in the same folder as the Microchip
    folder (like the current demo folders), then the following include
    paths need to be added to the application's project:
    
    .    

    ..\\..\\Microchip\\Include
    
    If a different directory structure is used, modify the paths as
    required. An example using absolute paths instead of relative paths
    would be the following:
    
    C:\\Microchip Solutions\\Microchip\\Include
    
    C:\\Microchip Solutions\\My Demo Application 

******************************************************************************/
/*******************************************************************************
    Filename:       usb.h
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

#ifndef _USB_H_
#define _USB_H_
//DOM-IGNORE-END

#if __XC16_VERSION__ == 1020
#error XC16 v1.20 is not compatible with this firmware, please use a later version of the XC16 compiler.
#endif


// *****************************************************************************
// *****************************************************************************
// Section: All necessary USB Library headers
// *****************************************************************************
// *****************************************************************************

#include "system.h"
#include "usb_config.h"

#include "usb_common.h"         // Common USB library definitions
#include "usb_ch9.h"            // USB device framework definitions

#if defined( USB_SUPPORT_DEVICE )
    #include "usb_device.h"     // USB Device abstraction layer interface
#endif

#if defined( USB_SUPPORT_HOST )
    #include "usb_host.h"       // USB Host abstraction layer interface
#endif

#if defined ( USB_SUPPORT_OTG )
    #include "usb_otg.h"
#endif

#include "usb_hal.h"            // Hardware Abstraction Layer interface

// *****************************************************************************
// *****************************************************************************
// Section: MCHPFSUSB Firmware Version
// *****************************************************************************
// *****************************************************************************

#define USB_MAJOR_VER   2       // Firmware version, major release number.
#define USB_MINOR_VER   10      // Firmware version, minor release number.
#define USB_DOT_VER     0       // Firmware version, dot release number.

#endif // _USB_H_
/*************************************************************************
 * EOF
 */

