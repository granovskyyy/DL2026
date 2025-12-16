KICad is required to open schematics

This custom circuit was designed to measure properties of the fluids or gases using sensors, which could be obtained by connecting to the board via USB to uart connection or serial port.

Overall structure of the project is:
    -> datasheets folder - contains datasheets of all the elements used in the project.
    It is: 
        sensors: 
            BMP280, 
            BS18B20, 
        usb to uart: 
            PL2303GS USB to Full UART IC, 
        ATmega328PB Xplained Mini board.
    -> I2C-BMP280 folder - containing base library of communicating with BMP280 sensors via I2C interface, which was helpful in process of writing custom firmware for the project case.
    -> schematics folder - containing Electric Schematic and PCB design of the circuit.
    -> Final Project folder - containing final versions of the software used to read and display the data from sensors.

Firmware was written in C to maintain high readability, modularity and speed of the code, as well as future development of the project.