# I<sup>2</sup>C Temperature Data Logger

## ABOUT
This demo implements an I<sup>2</sup>C temperature data logger. It is designed to teach the basics of I<sup>2</sup>C. The hardware uses the [**PIC18F13K50**](https://www.microchip.com/wwwproducts/en/PIC18F13K50) to communicate with the [**MCP9804**](https://www.microchip.com/wwwproducts/en/MCP9804) for the current temperature and logs that temperature using the [**24LC64**](https://www.microchip.com/wwwproducts/en/24LC64) 64-Kbit I<sup>2</sup>C Serial EEPROM. A AAA 1.5V battery is used to power the hardware and the [**MCP1640**](https://www.microchip.com/wwwproducts/en/MCP1640) then boosts the 1.5V to 3.3V to power the I<sup>2</sup>C Host and Client devices. The software communicates  with the firmware using a USB Mini cable and the firmware can be programmed by using one of Microchip's many PICkit™ Programmers and Debuggers using the PICkit header.

### Hardware
![Demo](/Docs/Media/Temp-Logger.png)
- [**Schematic**](/Docs/Temperature%20Logger%20Schematic.pdf)
- [**BOM**](/Docs/Temperature%20Logger%20%20Bill%20of%20Materials.xls)
- [**PCB**](/PCB/I2C%20Temperature%20Logger.pdf)
- [**Gerbers**](/PCB/Gerber%20Files/05-07893-R1%20Gerbers.zip)

### Software
![GUI](/Docs/Media/GUI.png)
- [**Graphical User Interface**](/Software/I2C%20Temp%20Datalogger.exe)

### Firmware
- [**Firmware**](/Firmware/dist/Default/Production/Firmware.production.hex)

