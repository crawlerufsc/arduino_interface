/*
  Mega analogWrite() test

  This sketch fades LEDs up and down one at a time on digital pins 2 through 13.
  This sketch was written for the Arduino Mega, and will not work on other boards.

  The circuit:
  - LEDs attached from pins 2 through 13 to ground.

  created 8 Feb 2009
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogWriteMega
*/

void setup() {
    pinMode(3, OUTPUT);
}

void loop() {
  // iterate over the pins:
    for (int brightness = 0; brightness < 255; brightness++) {
      analogWrite(3, brightness);
      delay(2);
    }
    // fade the LED on thisPin from brightest to off:
    for (int brightness = 255; brightness >= 0; brightness--) {
      analogWrite(3, brightness);
      delay(2);
    }
  }
