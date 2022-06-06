#include <Servo.h>

/* Test script to open and close desktop arm gripper
 *  
 * Pinout should match the RAMPS 1.4 board
 * 
 */

#define SERVO0_PIN 4
int pos;

Servo gripper; // servo object
void setup() {
  gripper.attach(SERVO0_PIN);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    gripper.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}
