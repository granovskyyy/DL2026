# DL2026
Design Lab classes repository 
Project - IoT system based on ATMega328PB with all connectivity interfaces (I2C, SPI, UART etc) 
### FEATURES
The projects support various connectivity protocols such as: 
- UART
- I2C
- SPI
- 1-WIRE
- Outputs (7seg display)  
Software functions:
- UART connectivity and displaying counter of 7-seg display as well as data obtained from the sensors.
### HARDWARE
- ATMega328PB - microcontroller  
- BMP280 - Pressure sensor 
- DS18B20 – temperature probe 
- PL2303 - UART connectivity (in schematic is FT232RL used due to KiCad avaibality) 
- 3.3V voltage regulators 
- Capacitors (100n ceramic and 10uF electrolytic) 
- Resistors (4,7K to pullup, 220 for 7seg)
### SCHEMATIC
<img width="947" height="663" alt="image" src="https://github.com/user-attachments/assets/bb8fc186-8201-4604-8b40-7928a3d82ddf" />

### PCB LAYOUT
<img width="1207" height="610" alt="image" src="https://github.com/user-attachments/assets/c2b9c92f-9903-4330-9501-307c1d5c5596" />

### PCB MODEL
<img width="747" height="430" alt="image" src="https://github.com/user-attachments/assets/225b35e7-a1b3-40f7-98b7-c49bb92d9148" />

### PROJECT STRUCTURES  
- /datasheets/-  Components datasheets 
- schematics/  -  Schematics and PCB designs 
- I2C-BMP280/  -  I2C examples with BMP280 
- Final Project – complete DL code with support for all protocols 
- README.md

### SOFTWARE
- KiCad
- AVR Microchip Studio
#### COMPILATION 
avr-gcc -mmcu=atmega328pb -Os -o main.elf source_file.c
avr-objcopy -O ihex main.elf main.hex
#### FLASH
avrdude -c usbasp -p m328pb -U flash:w:main.hex

NOTE- you can compile code via Makefile 


  
