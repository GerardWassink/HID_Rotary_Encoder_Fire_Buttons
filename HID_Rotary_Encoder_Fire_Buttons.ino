/* ------------------------------------------------------------------------- *
 * Name   : HID_Rotary_Encoder_Fire_Buttons
 * Author : Gerard Wassink
 * Date   : January 2024
 * Purpose: Fire keyboard buttons at the GIMP program
 * Versions:
 *   0.1  : Initial code base
 *   1.0  : Working
* ------------------------------------------------------------------------- */
#define progVersion "1.0"                     // Program version definition
/* ------------------------------------------------------------------------- *
 *             GNU LICENSE CONDITIONS
 * ------------------------------------------------------------------------- *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * ------------------------------------------------------------------------- *
 *       Copyright (C) Januari 2024 Gerard Wassink
 * ------------------------------------------------------------------------- */


#if ARDUINO_USB_MODE

#warning This sketch should be used when USB is in OTG mode
void setup(){}
void loop(){}

#else

/* ------------------------------------------------------------------------- *
 *       Compiler directives to switch debugging on / off
 *       Do not enable debug when not needed, Serial takes space and time!
 * ------------------------------------------------------------------------- */
#define DEBUG 0

#if DEBUG == 1
  #define debugstart(x) Serial.begin(x)
  #define debug(x) Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debugstart(x)
  #define debug(x)
  #define debugln(x)
#endif


/* ------------------------------------------------------------------------- *
 *                                                         Include libraries
 * ------------------------------------------------------------------------- */
#include <USB.h>                        // USB library
#include <USBHIDKeyboard.h>             // Keyboard library

/* ------------------------------------------------------------------------- *
 *                                                            Create objects
 * ------------------------------------------------------------------------- */
USBHIDKeyboard Keyboard;                // Instantiate keyboard object

/* ------------------------------------------------------------------------- *
 *                                                      Rotary encoder stuff
 * ------------------------------------------------------------------------- */
#define PIN_A D2
#define PIN_B D3
#define buttonPin D4

int rotationCounter = 0;                // Turn counter for the rotary encoder
                                        //   (negative = anti-clockwise)

volatile bool rotaryEncoder = false;    // Interrupt busy flag
volatile int8_t rotationValue = 0;      // Keep score
int8_t previousRotationValue = 1;       // previous value for comparison
int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};

/* ------------------------------------------------------------------------- *
 *                                           Variable to set based on rotary
 * ------------------------------------------------------------------------- */
int setValue = 0;

/* ------------------------------------------------------------------------- *
 *                                        Variables for rotary switch button 
 * ------------------------------------------------------------------------- */
int buttonStateCurrent = HIGH;
int buttonStatePrevious = HIGH;
bool buttonPressed = false;

/* ------------------------------------------------------------------------- *
 *                                                           Key definitions
 * ------------------------------------------------------------------------- */
char ctrlKey  = KEY_LEFT_CTRL;           // for Windows & Linux
char shiftKey = KEY_LEFT_SHIFT;


/* ------------------------------------------------------------------------- *
 *       ISR routine, determines:                       checkRotaryEncoder()
 *          validity of movement
 *          set output rotationValue: 0, 1, -1 
 * ------------------------------------------------------------------------- */
void checkRotaryEncoder()
{
  if (rotaryEncoder == false)           // Allowed to enter?
  {
    rotaryEncoder = true;               // DO NOT DISTURB, interrupt in progress...

    static uint8_t lrmem = 3;
    static int lrsum = 0;

    int8_t l = digitalRead(PIN_A);      // Read BOTH pin states to deterimine validity 
    int8_t r = digitalRead(PIN_B);      //   of rotation

    lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;  // Move previous value 2 bits 
                                                // to the left and add in our new values
    
    lrsum += TRANS[lrmem];              // Convert the bit pattern to a movement indicator
                                        //    (14 = impossible, ie switch bounce)

    if (lrsum % 4 != 0)                 // encoder not in the neutral (detent) state
    {
        rotationValue = 0;
    }
    else if (lrsum == 4)                // encoder in the neutral state - clockwise rotation
    {
        lrsum = 0;
        rotationValue = 1;
    } else if (lrsum == -4)             // encoder in the neutral state - anti-clockwise rotation
    {
        lrsum = 0;
        rotationValue = -1;
    } else
    {                                   // An impossible rotation has been detected - ignore the movement
      lrsum = 0;
      rotationValue = 0;
    }

    rotaryEncoder = false;              // Ready for next interrupt
  }
}


/* ------------------------------------------------------------------------- *
 *                                                    ISR routine - button()
 * ------------------------------------------------------------------------- */
void button()
{
  static unsigned long lastInterruptTime = 0; // persistent between calls
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5)  // Measure time for debounce
  {
    buttonStateCurrent = digitalRead(buttonPin);
    if ((buttonStateCurrent != buttonStatePrevious) 
        && (buttonStateCurrent == LOW)) 
    {
      buttonPressed = true;                   // indicate button pressed
    }
    buttonStatePrevious = buttonStateCurrent;
  }
  lastInterruptTime = interruptTime;
}


/* ------------------------------------------------------------------------- *
 *                                 Setup routing, do initial stuff - setup()
 * ------------------------------------------------------------------------- */
void setup()
{
  debugstart(115200);

  pinMode(PIN_A, INPUT_PULLUP);         // mode for 
  pinMode(PIN_B, INPUT_PULLUP);         //  rotary pins

  pinMode(buttonPin, INPUT_PULLUP);     // mode for push-button pin

  Keyboard.begin();                     // Start keayboard
  USB.begin();                          // Start USB

  //
  // We need to monitor both pins, rising and falling for all states
  //
  attachInterrupt(digitalPinToInterrupt(PIN_A), checkRotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), checkRotaryEncoder, CHANGE);

  //
  // Monitor pushbutton pin
  //
  attachInterrupt(digitalPinToInterrupt(buttonPin), button, CHANGE);

  debugln("Setup completed");
}


/* ------------------------------------------------------------------------- *
 *                                        Do actual repetitive work - loop()
 * ------------------------------------------------------------------------- */
void loop()
{
  int delayTime = 75;
  if (rotationValue != previousRotationValue)   // Has rotary moved?
  {
    switch (rotationValue) {
      case 1:
        debugln("Zoom in  - ");
        Keyboard.press('+');            // GIMP value
        delay(delayTime);
        Keyboard.releaseAll();
        delay(delayTime);
        break;

      case -1:
        debugln("Zoom out - ");
        Keyboard.press('-');            // GIMP value
        delay(delayTime);
        Keyboard.releaseAll();
        delay(delayTime);
        break;
        
      default:
        break;

    }
    previousRotationValue = rotationValue; // avoid entering this code every time
  }

  if (buttonPressed == true)            // If the switch is pressed
  {
    debugln("Fit in window");
    Keyboard.press(shiftKey);
    Keyboard.press(ctrlKey);
    Keyboard.press('J');                // GIMP value
    delay(delayTime);
    Keyboard.releaseAll();
    delay(delayTime);
    buttonPressed = false;              // avoid entering this code every time
   }
}
#endif
