Arduino sketches
================

joystick
--------

This sketch reads the gameport interface of a standard 4-axes joystick and prints on serial interface 5 space separated values: value of currently pressed button (0 = no button), plus the values of the 4 axes, in range [-100, 100]. The axes output have a 5% dead zone around zero.

Joystick buttons 5 (f1 on Saitek Cyborg 3D) and 6 (f2 on Saitek Cyborg 3D) have special functions: 5 enters the calibration procedure (and lights on the LED connected on pin 13), and button 6 exits (and lights off that LED). At the very first entering in calibration mode, the current axes positions are saved as zero values, then just scramble all the axes along full travel to acquire the max and min values for each axis. When done, press button 6. Updated values are sent over serial even during the calibration procedure.

Serial communication runs at 57600 baud and values are updated roughly every 20 ms.

Gameport pinout is available [here](http://pinouts.ru/Inputs/GameportPC_pinout.shtml). In order to connect pins to the Arduino you have to proceed like that:

1. Power the joystick: pin 1 goes to +5V, pin 12 goes to ground.
2. Connect the button pins to digital pins 2-5 (2 to 2, 8 to 3, 10 to 4, and 14 to 5).
3. Connect the analog output pins to ground through a 10k resistor, and read pins through arduino's analog pins 0-3.