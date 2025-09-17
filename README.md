# I2C-Inter-Integrated-Circuit-Protocol-Programs-in-PIC
I2C (Inter-Integrated Circuit) Programs in PIC

## I2C Protocol
I2C (Inter-Integrated Circuit) is a two-wire serial communication protocol consisting of:
SDA (Serial Data Line)
SCL (Serial Clock Line)
It supports multiple masters and multiple slaves on the same bus and uses 7-bit or 10-bit addressing.

Common I2C Interfacing Programs
1️⃣ EEPROM (24Cxx Series)
Store and retrieve data permanently using external EEPROM.
Program demonstrates byte write, page write, and sequential read.
Useful for saving settings, logs, or calibration data.

2️⃣ RTC DS1307 (Real-Time Clock)
Communicate with DS1307 to read/write time and date.
RTC keeps track of time using a 32.768 kHz crystal and backup battery.
Example program displays current time/date on an LCD via I2C.

3️⃣ I2C LCD (PCF8574/PCF8571 Interface IC)
PCF8574/PCF8571 provides an I2C-to-parallel expander, reducing LCD pin usage from 8/4 pins → only 2 pins (SDA, SCL).
PIC sends commands and data to LCD through the I2C expander.
Makes wiring simpler and saves microcontroller pins.
