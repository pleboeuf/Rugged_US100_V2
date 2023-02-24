# Rugged_US100_V2
Arduino software for US100 sensor box use in outside reservoirs

The V2.0.0 code use NeoSWSerial while V1.3 use SoftwareSerial library
This enclosure use an Arduino Pro Mini to control the heating of the box and tilt detection. 
Those sensor boxes can be removed to clean the reservoirs. The tilt detection indicate invalid readings when the box is not in operating position
The US100 sensor is seen from outside as a normal US100. The data is copied between the UART and a software serial port transparently except when
the box is tilted.

The temperature in the box is maintained by a very crude ON/OFF software thermostat.
