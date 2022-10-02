
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
#include <MultiStepper.h>
#include <ezButton.h>
//#include <TMC2208Stepper.h>
//#include <TMC2208Stepper_REGDEFS.h>
#include "Configuration.h"
#include "pins.h"

// Desktop arm controlled by 6 steppers 
AccelStepper joint0Stepper = AccelStepper(motorInterfaceType, JOINT0_STEP_PIN, JOINT0_DIR_PIN);
AccelStepper joint1Stepper = AccelStepper(motorInterfaceType, JOINT1_STEP_PIN, JOINT1_DIR_PIN); // This joint is controlled by 2 steppers linked together
AccelStepper joint2Stepper = AccelStepper(motorInterfaceType, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3Stepper = AccelStepper(motorInterfaceType, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4Stepper = AccelStepper(motorInterfaceType, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

//
Servo gripper;

ezButton joint0Switch(JOINT0_MIN_PIN);  // create ezButton object that attach to pin 7;
ezButton joint1Switch(JOINT1_MIN_PIN);  // create ezButton object that attach to pin 7;
ezButton joint2Switch(JOINT2_MIN_PIN);  // create ezButton object that attach to pin 7;
ezButton joint3Switch(JOINT3_MIN_PIN);  // create ezButton object that attach to pin 7;
ezButton joint4Switch(JOINT4_MIN_PIN);  // create ezButton object that attach to pin 7;


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
int gripper_state;
int joint0_pos;
int joint1_pos;
int joint2_pos;
int joint3_pos;
int joint4_pos;
int joint0_prev_pos;
int joint1_prev_pos;
int joint2_prev_pos;
int joint3_prev_pos;
int joint4_prev_pos;
int JOINT0_STATE;
int JOINT1_STATE;
int JOINT2_STATE;
int JOINT3_STATE;
int JOINT4_STATE;

bool joint0SwitchPressed=false;
bool joint1SwitchPressed=false;
bool joint2SwitchPressed=false;
bool joint3SwitchPressed=false;
bool joint4SwitchPressed=false;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial.flush();


  Wire.begin();
  setupPins();
  setupSwitches(1);

  // ===== Initialize stepper motors  and servos =====
  setupJoints();
  setupServos();
  // === Select one or the other for zero positioning ===
  //zero_all_joints();
  set_current_joints_pos_to_zero();
  
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
  
  
  // Commented out since not sure if client needs to clear received bytes before sending new commands
  //Serial.begin("NML-NHP Reward"); //Bluetooth device name
}

void loop(){ 

  //joint4Stepper.moveTo(1000);
  //joint4Stepper.run();
  // Update counter on screen (testing)
  //lcd.setCursor(0, 1);
  // Print the number of seconds since reset:
  //lcd.print(millis() / 1000);


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

  // Want to constently monitor the system, this gets executed at the beginning
  update_system();
  
  if (Serial.available()) {
    // read the incoming byte:
    route = Serial.read();    // The first byte is a char that determines how the following bytes are parsed.
    switch (route) {
      case 'M': // Move command
        Serial.println("Received motor command!");
        parse_multistepper_command();
        //parse_motor_command();
        //_start_dispensing();
        break;
      //case 'S': // sensor command
      //  break;
      case 'G': //gripper command
          parse_gripper_command();
      case 'Z':
          set_current_joints_pos_to_zero();
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

void update_system(){

  // Check limit switches for all joints 
  joint0Switch.loop(); // MUST call the loop() function first for switches
  if (joint0Switch.getState() == PRESSED_STATE){ 
    joint0Stepper.stop();  
    Serial.println("Limit switch for Joint 0 pressed");
  } else {
    joint0Stepper.run();
  }

  joint1Switch.loop();
  if (joint1Switch.getState() == PRESSED_STATE){
    joint1Stepper.stop(); 
    Serial.println("Limit switch for Joint 1 pressed");
  } else {
    joint1Stepper.run();
  }  
  joint2Switch.loop();
  if (joint2Switch.getState() == PRESSED_STATE){
    joint2Stepper.stop();
    Serial.println("Limit switch for Joint 2 pressed");
  } else {
    joint2Stepper.run();
  }  
  
  joint3Switch.loop();
  if (joint3Switch.getState() == PRESSED_STATE){
    joint3Stepper.stop();
    Serial.println("Limit switch for Joint 3 pressed");
  } else {
    joint3Stepper.run();
  }  

  joint4Switch.loop();
  if (joint4Switch.getState() == PRESSED_STATE){
    joint4Stepper.stop();
    Serial.println("Limit switch for Joint 4 pressed");           
  } else {
    joint4Stepper.run();
  }  
}

void zero_joint2() {
  // Spins the joints until the limit switches are activated to establish the joint limits
  
  //Right now just have joint 2 set up
  while (joint2SwitchPressed==false) {
    joint2Switch.loop(); // MUST call the loop() function first
    if (joint2SwitchPressed==false){
      int state = joint2Switch.getState();
      if(state == HIGH)  {
        //joint2Stepper.setSpeed(1000);
        joint2Stepper.runSpeed();
      } else {
        joint2SwitchPressed = true;
        joint2Stepper.stop();
      }
    }    
  }
}

void zero_all_joints() {
  // Spins the joints until the limit switches are activated to establish the joint limits
  
  
  if (joint2SwitchPressed==false) {
    joint2Switch.loop(); // MUST call the loop() function first
    if (joint2SwitchPressed==false){
      int state = joint2Switch.getState();
      if(state == HIGH)  {
        //joint2Stepper.setSpeed(1000);
        joint2Stepper.runSpeed();
      } else {
        joint2SwitchPressed = true;
        joint2Stepper.stop();
      }
    }    
  }
}

void parse_gripper_command() {
  // For now simple open and close
  if (Serial.available()) {
    if (Serial.find(":")) {
      gripper_state = Serial.parseInt();
      if (gripper_state == 1) {
        Serial.println("Gripper open");
        gripper.write(GRIPPER_OPEN_POS);
      }
      if (gripper_state == 0) {
        Serial.println("Gripper close");
        gripper.write(GRIPPER_CLOSE_POS);
      }
    }
  }  
}

void parse_multistepper_command() {
  // Utilizing MultiSpetter library, we can parse all the motor commands and update all the joints at once

  if (Serial.available()) {
    if (Serial.find(":")) {
      joint0_pos = update_joint(&JOINT0_STATE, JOINT0_MIN_POS, JOINT0_MAX_POS, JOINT0_DISABLED);
    }
    if (Serial.find(",")) {
      joint1_pos = update_joint(&JOINT1_STATE, JOINT1_MIN_POS, JOINT1_MAX_POS, JOINT1_DISABLED);
    }
    if (Serial.find(",")) {
      joint2_pos = update_joint(&JOINT2_STATE, JOINT2_MIN_POS, JOINT2_MAX_POS, JOINT2_DISABLED);
    }
    if (Serial.find(",")) {
      joint3_pos = update_joint(&JOINT3_STATE, JOINT3_MIN_POS, JOINT3_MAX_POS, JOINT3_DISABLED);
    }
    if (Serial.find(",")) {
      joint4_pos = update_joint(&JOINT4_STATE, JOINT4_MIN_POS, JOINT4_MAX_POS, JOINT4_DISABLED);
    }
    long positions[5]; //Array of desired stepper positions
    positions[0] = joint0_pos;
    positions[1] = joint1_pos;
    positions[2] = joint2_pos;
    positions[3] = joint3_pos;
    positions[4] = joint4_pos;

    if (enable_verbose) { 
      Serial.print("NEW JOINT_STATE: ");
      Serial.print(positions[0]);
      Serial.print(",");
      Serial.print(positions[1]);
      Serial.print(",");
      Serial.print(positions[2]);
      Serial.print(",");
      Serial.print(positions[3]);
      Serial.print(",");
      Serial.println(positions[4]);
    }  

    joint0Stepper.moveTo(positions[0]);
    joint1Stepper.moveTo(positions[1]);
    joint2Stepper.moveTo(positions[2]);
    joint3Stepper.moveTo(positions[3]);
    joint4Stepper.moveTo(positions[4]);
    //joint4Stepper.moveTo(1000);

    //joint0Stepper.runSpeed();
    //joint1Stepper.runSpeed();
    //joint2Stepper.runSpeed();
    //joint3Stepper.runSpeed();
    //joint4Stepper.runSpeed();
    //delay(1000);
    //steppers.moveTo(positions);
    //steppers.run();
    //steppers.runSpeedToPosition(); // Blocks until all are in position
  }
}

int update_joint(int *JOINT_STATE, int JOINT_MIN_POS, int JOINT_MAX_POS, bool JOINT_DISABLED){
  // Update a particular joint 
  int joint_pos = 0;
  if (Serial.available()) {
    joint_pos = convert_deg_to_joint(Serial.parseInt());
    Serial.print("value received: ");
    Serial.println(joint_pos);
    if (JOINT_DISABLED) {
      Serial.println("JOINT DISABLED");
      joint_pos = 0;      
    } else {
      // Assume the starting position is halfway between min and max position.
      // Check that the desired value doesn't go past the min limit
      //if (JOINT_STATE + joint_pos < JOINT_MIN_POS) { // Check that the desired value doesn't go past the min limit
      //  joint_pos = JOINT_MIN_POS - JOINT_STATE;
      //}
      //else if (JOINT_STATE + joint_pos > JOINT_MAX_POS) { // Check that the desired value doesn't go past the max limit
      //  joint_pos = JOINT_MAX_POS - JOINT_STATE;
      //}
      // Update internal joint state
      JOINT_STATE = JOINT_STATE + joint_pos;
    }
  }
  return joint_pos;
}

int convert_deg_to_joint(int joint_pos) {
  // Convert the input value to the robot readable stepper position values
  //TO-DO
  return joint_pos;
}

void parse_motor_command() {
  // Assumes that the resulting data incoming will be 5 joint positions, separated by commas
  if (Serial.available()) {
    if (Serial.find(":")) {
      joint0_pos = Serial.parseInt();
      if (!JOINT0_DISABLED) {
        Serial.println("M0");
        move_joint(joint0Stepper, joint0_pos, JOINT0_MIN_POS, JOINT0_MAX_POS);
      }
    }
  }
  if (Serial.available()) {
     if (Serial.find(",")) {
      joint1_pos = Serial.parseInt();
      if (!JOINT1_DISABLED) {
        Serial.println("M1");
        move_joint(joint1Stepper, joint1_pos, JOINT1_MIN_POS, JOINT1_MAX_POS);
      }
    }
  }
  if (Serial.available()) {
     if (Serial.find(",")) {
      joint2_pos = Serial.parseInt();
      if (!JOINT2_DISABLED) {
        Serial.println("M2");
        move_joint(joint2Stepper, joint2_pos, JOINT2_MIN_POS, JOINT2_MAX_POS);
      }
    }
  }
  if (Serial.available()) {
    if (Serial.find(",")) {
      joint3_pos = Serial.parseInt();
      if (!JOINT3_DISABLED) {
        Serial.println("M3");
        move_joint(joint3Stepper, joint3_pos, JOINT3_MIN_POS, JOINT3_MAX_POS);
      }
    }
  }
  if (Serial.available()) {
    if (Serial.find(",")) {
      joint4_pos = Serial.parseInt();
      if (!JOINT4_DISABLED) {
        Serial.println("M4");
        move_joint(joint4Stepper, joint4_pos, JOINT4_MIN_POS, JOINT4_MAX_POS);
      }
    }
  }
}

void move_joint(AccelStepper stepper, int pos, int min_pos, int max_pos) {
  
  if (pos > max_pos) { pos = max_pos;};
  if (pos < min_pos) { pos = min_pos;};
  
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

void set_current_joints_pos_to_zero(){
  joint0Stepper.setCurrentPosition(0); // Set the current position to 0
  joint1Stepper.setCurrentPosition(0); // Set the current position to 0
  joint2Stepper.setCurrentPosition(0); // Set the current position to 0
  joint3Stepper.setCurrentPosition(0); // Set the current position to 0
  joint4Stepper.setCurrentPosition(0); // Set the current position to 0

  // Ideally the motors will move until the limit switches are hit, setting the new min limit, 
  // then the joints adjust to the zero position for the joints, currently defined as the midpoint between
  // the minimum and maximum position
  
  //JOINT3_STATE = (JOINT3_MAX_POS - JOINT3_MIN_POS)/2;
  //Serial.print("JOINT3_STATE: ");
  //Serial.println(JOINT3_STATE);
  //JOINT4_STATE = 1000;
  //Serial.print("JOINT4_STATE: ");
  //Serial.println(JOINT4_STATE);
  
}

void setupSwitches(int debounce_time){
  // Set up limit switch debounce time (ex: 1 millisecond, quick response)
  joint0Switch.setDebounceTime(debounce_time);
  joint1Switch.setDebounceTime(debounce_time);
  joint2Switch.setDebounceTime(debounce_time);
  joint3Switch.setDebounceTime(debounce_time);
  joint4Switch.setDebounceTime(debounce_time);  
}

void setupJoints() {
  // Rotating base joint
  joint0Stepper.setAcceleration(1000.0);
  joint0Stepper.setMaxSpeed(1000.0);
  //joint0Stepper.setSpeed(1000);
  Serial.println("Joint_0 Max Accel:1000, Max Speed:1000");

  // Joint 1 (shoulder rotation)
  joint1Stepper.setAcceleration(1000.0);
  joint1Stepper.setMaxSpeed(1000);
  //joint1Stepper.setSpeed(1000);
  Serial.println("Joint_1 Max Accel:1000, Max Speed:1000");

  // Joint 2 ( elbow rotation)
  joint2Stepper.setAcceleration(500.0);
  joint2Stepper.setMaxSpeed(100.0);
  //joint2Stepper.setSpeed(1000);
  Serial.println("Joint_2 Max Accel:1000, Max Speed:1000");

  // Joint 3 (rotating wrist joint)
  joint3Stepper.setAcceleration(1000.0);
  joint3Stepper.setMaxSpeed(500.0);
  //joint3Stepper.setSpeed(1000);
  Serial.println("Joint_3 Max Accel:1000, Max Speed:1000");

  // Joint 4 (moves the gripper)
  joint4Stepper.setAcceleration(10000.0);
  joint4Stepper.setMaxSpeed(1000.0);
  joint4Stepper.moveTo(1000);
  //joint4Stepper.setSpeed(1000);
  Serial.println("Joint_4 Max Accel:1000, Max Speed:1000");

  //steppers.addStepper(joint0Stepper);
  //steppers.addStepper(joint1Stepper);
  //steppers.addStepper(joint2Stepper);
  //steppers.addStepper(joint3Stepper);
  //steppers.addStepper(joint4Stepper);
}

void setupServos() {
  // Just one servo controlling gripper
  gripper.attach(SERVO0_PIN);
}

// END: SETUP FUNCTIONS

// BEGIN: DISPENSING FUNCTIONS


// END: DISPENSING FUNCTIONS

// BEGIN: HANDLE FUNCTIONS
//void _handle_duration_request(double x) { // Handles requests led with 'd'
//  dispense_duration_ms = max(x, MIN_RELAY_OPEN_TIME);
//  do_dispense = true; 
//}

//void _handle_volume_request(double x) { // Handles requests led with 'v'
//  x = x / 1000.0;
//  dispense_duration_ms = max(x * 777.0 - 93.0, MIN_RELAY_OPEN_TIME); // Uses linear regression from empirical calibration.
//  do_dispense = true; 
//}
// END: HANDLE FUNCTIONS
