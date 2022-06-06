
 /* (c) Jonathan Shulgach - Cite and Notice license:
   All modifications to this code or use of it must include this notice and give credit for use.

   Credit requirements:
    All publications using this code must cite all contributors to this code.
    A list must be updated below indicating the contributors alongside the original or modified code appropriately.
    All code built on this code must retain this notice. All projects incorporating this code must retain this license text alongside the original or modified code.
    All projects incorporating this code must retain the existing citation and license text in each code file and modify it to include all contributors.
    Web, video, or other presentation materials must give credit for the contributors to this code, if it contributes to the subject presented.
    All modifications to this code or other associated documentation must retain this notice or a modified version which may only involve updating the contributor list.

   Primary Authors:
    - Jonathan Shulgach, PhD Student - Neuromechatronics Lab, Carnegie Mellon University

   Contributor List:
   
   Other than the above, this code may be used for any purpose and no financial or other compensation is required.
   Contributors do not relinquish their copyright(s) to this software by virtue of offering this license.
   Any modifications to the license require permission of the authors.

   More info on Qwiic here: https://www.sparkfun.com/qwiic

   AVR-Based Serial Enabled LCDs Hookup Guide
   https://learn.sparkfun.com/tutorials/avr-based-serial-enabled-lcds-hookup-guide

*/
#include <Servo.h>
#include <Wire.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
#include <AccelStepper.h>
//#include <TMC2208Stepper.h>
//#include <TMC2208Stepper_REGDEFS.h>
#include "Configuration.h"
#include "pins.h"

AccelStepper joint0Stepper = AccelStepper(motorInterfaceType, JOINT0_STEP_PIN, JOINT0_DIR_PIN);
AccelStepper joint1Stepper = AccelStepper(motorInterfaceType, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2Stepper = AccelStepper(motorInterfaceType, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3Stepper = AccelStepper(motorInterfaceType, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4Stepper = AccelStepper(motorInterfaceType, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
Servo gripper;


SerLCD lcd; // Initialize the library with default I2C address 0x72


#define MIN_RELAY_OPEN_TIME 0.1  // milliseconds
#define BUFFER_SIZE 16
#define BUTTON_PIN_IN 2
#define REWARD_LED_PIN_OUT 13
#define CONNECT_LED_PIN_OUT 4
#define RELAY_PIN_OUT 5

// Global state-tracking variables
volatile bool do_dispense;
volatile double dispense_duration_ms;
char route;
int n_repeat, t_start, x;
int pos;
int joint0_pos;
int joint1_pos;
int joint2_pos;
int joint3_pos;
int joint4_pos;

void setup()
{
  Wire.begin();


  // ===== Initialize stepper motors =====
  //zero_all_joints();
  //send_home_state();


 
  // ===== LCD Setup =====
  lcd.begin(Wire); //Set up the LCD for I2C communication
  lcd.setBacklight(5, 205, 205); //Set backlight to bright white
  lcd.setContrast(0); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  lcd.print("Hello, World!");


  t_start = millis();
  n_repeat = 1;
  do_dispense = false;
  
  setupPins();
  setupJoints();
  
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial.flush();
  // Commented out since not sure if client needs to clear received bytes before sending new commands
  //Serial.begin("NML-NHP Reward"); //Bluetooth device name
}

void loop(){ 

  // Update counter on screen (testing)
  lcd.setCursor(0, 1);
  // Print the number of seconds since reset:
  lcd.print(millis() / 1000);


  // Indicate if connected via USB
  if (Serial) {
    digitalWrite(CONNECT_LED_PIN_OUT, HIGH);
  } else {
    digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  }

  // If interrupt tells us to dispense, then we use a blocking dispense to handle it.
  // ============ Uncomment lines below ============
  //if (do_dispense) {
  //  _handle_blocking_dispense();
  //}
  // ===============================================
  
  if (Serial.available()) {
    // read the incoming byte:
    route = Serial.read();    // The first byte is a char that determines how the following bytes are parsed.
    switch (route) {
      case 'M': // Move command
          parse_motor_command();
        //_start_dispensing();
        break;
      //case 'q': // quit "continuous" request
      //  _stop_dispensing();
      //  break;
      //case 'd': // "duration" request
      //  x = Serial.parseInt();
      //  //Serial.print("got duration");
      //  _handle_duration_request(x);
      //  break;
      //case 'r': // "reset" request
      //  Serial.flush();
      //  setup();
      //  return;
      //case 'v': // "volume" request
      //  x = Serial.parseInt();
      //  _handle_volume_request(x);
      //  break;
      default:  // unrecognized request
        route = Serial.read();
        break;
    }

    //if (Serial.available()) {
    //  if (Serial.find("x")) {
    //    n_repeat = Serial.parseInt();
    //  } else {
    //    n_repeat = 1;
    //  }
    //  while (Serial.available()) {
    //    Serial.read();   // "Flush" serial buffer
    //  }
    //}
  }
}

void parse_motor_command() {
  // Assumes that the resulting data incoming will be 5 joint positions, separated by commas
  if (Serial.available()) {
    if (Serial.find(":")) {
      joint0_pos = Serial.parseInt();
      move_joint(joint0Stepper, joint0_pos);
    }
  }
  if (Serial.available()) {
     if (Serial.find(",")) {
      joint1_pos = Serial.parseInt();
      move_joint(joint1Stepper, joint1_pos);
    }
  }
  if (Serial.available()) {
     if (Serial.find(",")) {
      joint2_pos = Serial.parseInt();
      move_joint(joint2Stepper, joint2_pos);
    }
  }
  if (Serial.available()) {
    if (Serial.find(",")) {
      joint3_pos = Serial.parseInt();
      move_joint(joint3Stepper, joint3_pos);
    }
  }
  if (Serial.available()) {
    if (Serial.find(",")) {
      joint4_pos = Serial.parseInt();
      move_joint(joint4Stepper, joint4_pos);
    }
  }
  
}

void move_joint(AccelStepper stepper, int pos) {
  while(stepper.currentPosition() != pos)
  {
    stepper.setSpeed(500);
    stepper.runSpeed();
  }
  
}

// BEGIN: SETUP FUNCTIONS
void setupPins() { // Setup input and output pins
  pinMode(JOINT0_ENABLE_PIN, OUTPUT);   
  pinMode(JOINT1_ENABLE_PIN, OUTPUT);   
  pinMode(JOINT2_ENABLE_PIN, OUTPUT);   
  pinMode(JOINT3_ENABLE_PIN, OUTPUT);   
  pinMode(JOINT4_ENABLE_PIN, OUTPUT);   
  
  digitalWrite(JOINT0_ENABLE_PIN, LOW);
  digitalWrite(JOINT1_ENABLE_PIN, LOW);
  digitalWrite(JOINT2_ENABLE_PIN, LOW);
  digitalWrite(JOINT3_ENABLE_PIN, LOW);
  digitalWrite(JOINT4_ENABLE_PIN, LOW);

  //pinMode(RELAY_PIN_OUT, OUTPUT);  
  //pinMode(CONNECT_LED_PIN_OUT, OUTPUT);
  //pinMode(REWARD_LED_PIN_OUT, OUTPUT);
  //pinMode(BUTTON_PIN_IN, INPUT_PULLUP);
  //digitalWrite(RELAY_PIN_OUT, LOW);
  //digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  //digitalWrite(REWARD_LED_PIN_OUT, LOW);
  //attachInterrupt(digitalPinToInterrupt(BUTTON_PIN_IN), _handle_manual_dispense, CHANGE);
}

void setupJoints() {
  // Rotating base joint
  joint0Stepper.setMaxSpeed(500);
  joint0Stepper.setCurrentPosition(0); // Set the current position to 0

  // Joint 1 (shoulder rotation)
  joint1Stepper.setMaxSpeed(500);
  joint1Stepper.setCurrentPosition(0); // Set the current position to 0

  // Joint 2 ( elbow rotation)
  joint2Stepper.setMaxSpeed(500);
  joint2Stepper.setCurrentPosition(0); // Set the current position to 0

  // Joint 3 (rotating wrist joint)
  joint3Stepper.setMaxSpeed(500);
  joint3Stepper.setCurrentPosition(0); // Set the current position to 0

  // Joint 4 (moves the gripper)
  joint4Stepper.setMaxSpeed(500);
  joint4Stepper.setCurrentPosition(0); // Set the current position to 0

}

// END: SETUP FUNCTIONS

// BEGIN: DISPENSING FUNCTIONS

void _handle_blocking_dispense() {
  //_HANDLE_BLOCKING_DISPENSE This handles running the blocking dispense routine, with notifications to Serial monitor.
  for (int i = 0; i < n_repeat; i++) {
    _blocking_dispense(); 
    delay(100);
  }
  do_dispense = false;
}

void _start_dispensing() {
  //_START_DISPENSING Set the correct pin(s) to HIGH related to circuit configuration.
  digitalWrite(RELAY_PIN_OUT, HIGH);
  digitalWrite(REWARD_LED_PIN_OUT, HIGH);
  //Serial.print("dispensing...");
}

void _stop_dispensing() {
  //_STOP_DISPENSING Set the correct pin(s) to LOW related to circuit configuration.
  digitalWrite(RELAY_PIN_OUT, LOW);
  digitalWrite(REWARD_LED_PIN_OUT, LOW);
  //Serial.println("done");
}

void _handle_manual_dispense() {
  if (digitalRead(BUTTON_PIN_IN) == LOW) {
    _start_dispensing();
  } else {
    _stop_dispensing();
  }
}

void _blocking_dispense() { // Turn the correct pin on, wait, then turn it off
  _start_dispensing();
  delay(dispense_duration_ms);
  _stop_dispensing();
}

// END: DISPENSING FUNCTIONS

// BEGIN: HANDLE FUNCTIONS
void _handle_duration_request(double x) { // Handles requests led with 'd'
  dispense_duration_ms = max(x, MIN_RELAY_OPEN_TIME);
  do_dispense = true; 
}

void _handle_volume_request(double x) { // Handles requests led with 'v'
  x = x / 1000.0;
  dispense_duration_ms = max(x * 777.0 - 93.0, MIN_RELAY_OPEN_TIME); // Uses linear regression from empirical calibration.
  do_dispense = true; 
}
// END: HANDLE FUNCTIONS
