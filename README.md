# Rotary_Encoder_Basics
Rotary encoder and button use for zoom functions in the GIMP program.

## Purpose
This sketch contains code for GIMP to:
- zoom in - (rotate right),
- zoom out - (rotate left) 
- fit to window - (push button)

## Hardware
This has been tested on Arduino Nano ESP32. This can use all ditital pins as interrupts.
On other Arduino's one would have to code the pushbutton stuff in the main loop()

## Software
The (very clever) way the interrupts are handled is derived from the work of Marko Pinteric, see his Technical paper explaining this code: https://www.pinteric.com/rotary.html 

