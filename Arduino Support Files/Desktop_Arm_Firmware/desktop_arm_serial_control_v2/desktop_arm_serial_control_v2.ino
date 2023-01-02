
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

   Reference for multi-stepper serial command parsing from JustinJKwok's Github
   https://github.com/JustinJKwok/Arduino-multi-stepper/blob/master/multi_stepper.ino

*/
#include <Servo.h>
//#include <Wire.h>
//#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
#include <AccelStepper.h>
//#include <ezButton.h>
//#include <TMC2208Stepper.h>
//#include <TMC2208Stepper_REGDEFS.h>
#include "Configuration.h"
#include "pins.h"

const int numSteppers = 5;
AccelStepper joint0Stepper = AccelStepper(motorInterfaceType, JOINT0_STEP_PIN, JOINT0_DIR_PIN);  // Rotating base joint
AccelStepper joint1Stepper = AccelStepper(motorInterfaceType, JOINT1_STEP_PIN, JOINT1_DIR_PIN);  // Joint 1 (shoulder rotation)
AccelStepper joint2Stepper = AccelStepper(motorInterfaceType, JOINT2_STEP_PIN, JOINT2_DIR_PIN);  // Joint 2 ( elbow rotation)
AccelStepper joint3Stepper = AccelStepper(motorInterfaceType, JOINT3_STEP_PIN, JOINT3_DIR_PIN);  // Joint 3 (rotating wrist joint)
AccelStepper joint4Stepper = AccelStepper(motorInterfaceType, JOINT4_STEP_PIN, JOINT4_DIR_PIN);  // Joint 4 (moves the gripper)
AccelStepper* steppers[numSteppers] = {&joint0Stepper, &joint1Stepper, &joint2Stepper, &joint3Stepper, &joint4Stepper};
// Up to 10 steppers can be handled as a group by AccelStepper
Servo gripper;

//ezButton joint0Switch(JOINT0_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint1Switch(JOINT1_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint2Switch(JOINT2_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint3Switch(JOINT3_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint4Switch(JOINT4_MIN_PIN);  // create ezButton object that attach to pin 7;

//SerLCD lcd; // Initialize the library with default I2C address 0x72

// Stepper motor and servo specific variables including limit switches
int homeSwitchPin[] = {JOINT0_MIN_PIN, JOINT1_MIN_PIN, JOINT2_MIN_PIN, JOINT3_MIN_PIN, JOINT4_MIN_PIN};
int jointEnablePin[] = {JOINT0_ENABLE_PIN, JOINT1_ENABLE_PIN, JOINT2_ENABLE_PIN, JOINT3_ENABLE_PIN, JOINT4_ENABLE_PIN};
float minPosition[] = {JOINT0_MIN_POS, JOINT1_MIN_POS, JOINT2_MIN_POS, JOINT3_MIN_POS, JOINT4_MIN_POS};
float maxPosition[] = {JOINT0_MAX_POS, JOINT1_MAX_POS, JOINT2_MAX_POS, JOINT3_MAX_POS, JOINT4_MAX_POS};
float maxMotorSpeed[] = {JOINT0_MAX_SPEED, JOINT1_MAX_SPEED, JOINT2_MAX_SPEED, JOINT3_MAX_SPEED, JOINT4_MAX_SPEED};
float maxMotorAcceleration[] = {JOINT0_MAX_ACCELERATION, JOINT1_MAX_ACCELERATION, JOINT2_MAX_ACCELERATION, JOINT3_MAX_ACCELERATION, JOINT4_MAX_ACCELERATION};
float homingBuffer[] = {JOINT0_HOMING_BUFFER, JOINT1_HOMING_BUFFER, JOINT2_HOMING_BUFFER, JOINT3_HOMING_BUFFER, JOINT4_HOMING_BUFFER}; 
float stepsPerRev[] = {JOINT0_STEPS, JOINT1_STEPS, JOINT2_STEPS, JOINT3_STEPS, JOINT4_STEPS}; 
float distPerRev[] = {JOINT0_REV_DISTANCE, JOINT1_REV_DISTANCE, JOINT2_REV_DISTANCE, JOINT3_REV_DISTANCE, JOINT4_REV_DISTANCE}; 
bool isDisabled[] = {JOINT0_DISABLED, JOINT1_DISABLED, JOINT2_DISABLED, JOINT3_DISABLED, JOINT4_DISABLED};
bool hasHomed[] = {false, false, false, false, false};
bool isHoming[] = {false, false, false, false, false};
bool hasReported[] = {true, true, true, true, true};
float stepsPerDist[numSteppers];
int gripper_state;

//bool joint0SwitchPressed=false;
//bool joint1SwitchPressed=false;
//bool joint2SwitchPressed=false;
//bool joint3SwitchPressed=false;
//bool joint4SwitchPressed=false;
//int joint0_prev_pos;
//int joint1_prev_pos;
//int joint2_prev_pos;
//int joint3_prev_pos;
//int joint4_prev_pos;
//int JOINT0_STATE;
//int JOINT1_STATE;
//int JOINT2_STATE;
//int JOINT3_STATE;
//int JOINT4_STATE;

// Global state-tracking variables
//int n_repeat, t_start, x;

// Serial communication handling for commands 
// see Serial Input Basics by Robin2: https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
bool VERBOSE = true;
char *DELIMITER = " ";
char endChar = '\n';
const int MAXCHARS = 32;
const int MAXTOKENS = 5;
char incomingChars[MAXCHARS];
char incomingParams[MAXTOKENS][MAXCHARS]; //incoming chars parsed into cstrings
char tempChars[MAXCHARS];
bool newSerialData = false;
int numTokens = 0;


void setup()
{
  
  // ===== Initialize stepper motors  and servos =====
  setupPins();
  //setupSwitches(1);
  setupJoints();
  setupServos();
  
  // === Select one or the other for zero positioning ===
  //zero_all_joints();
  set_current_pos_to_zero();
  //send_home_state();
 
  // ===== LCD Setup =====
  //Wire.begin();
  //lcd.begin(Wire); //Set up the LCD for I2C communication
  //lcd.setBacklight(5, 205, 205); //Set backlight to bright white
  //lcd.setContrast(0); //Set contrast. Lower to 0 for higher contrast.
  //lcd.clear(); //Clear the display - this moves the cursor to home position as well
  //lcd.print("Hello, World!");

    
  //t_start = millis();  
  
  Serial.begin(BAUDRATE);
  Serial.println("Ready");
}

void loop(){ 

    checkConnection(); // Indicate if connected via USB
    readNextSerial(); // Check serial input stream if any data is available
    if (newSerialData == true)
    {
        if (VERBOSE)
        { 
            Serial.print("Received: ");
            Serial.println(incomingChars);
        }
        strcpy(tempChars, incomingChars);
        parseTokens(); // Breaks up and evaluates incoming string message
        parseCommand(); // Handles parsed command snd underlying subtasks
    }

    updateJoints(); // Check the state of the robot
    //updateLCD(); // Update the front display

    // anything else...

    newSerialData = false; //reset new data flag
    for (int i = 0; i < numTokens; ++i) //reset the parsed tokens
    {
        incomingParams[i][0] = '\0';
    }
    numTokens = 0; //reset the token counter
}

void checkConnection()
{
  if (Serial) {
    digitalWrite(CONNECT_LED_PIN_OUT, HIGH);
  } else {
    digitalWrite(CONNECT_LED_PIN_OUT, LOW);
  }
}

void readNextSerial()
{
    //read serial buffer until newline, read into incomingChars
    static int ndx = 0;
    char nextChar;
    while (Serial.available() > 0 && newSerialData == false)
    {
        nextChar = Serial.read();
        //keep adding into cstring until newline
        if (nextChar != endChar) 
        {
            incomingChars[ndx] = nextChar;
            ++ndx;
            if (ndx >= MAXCHARS) 
            {
                ndx = MAXCHARS - 1;
            }
        } 
        else 
        {
            //if reading newline then terminate cstring and indicate there is new data to process
            incomingChars[ndx] = '\0';
            ndx = 0;
            newSerialData = true;
        }
    }
}

void parseTokens()
{
    char* token;
    numTokens = 0;

    token = strtok(tempChars, DELIMITER);

    while (token != NULL)
    {
        //save into cstring array and read next portion
        strcpy(incomingParams[numTokens], token);
        ++numTokens;

        if (numTokens >= MAXTOKENS)
        {
            numTokens = MAXTOKENS - 1;
        }
        token = strtok(NULL, DELIMITER);
    }
}

void parseCommand()
{
    //(For some reason using the switch command broke for me, later learned cannot 
    // use switch/case for cstring comparisons, actually better with if-else
    if (strcmp(incomingParams[0], "enablemotor") == 0)
    {
        enableMotor();
    } 
    else if (strcmp(incomingParams[0], "disablemotor") == 0)
    {
        disableMotor();
    } 
    else if (strcmp(incomingParams[0], "motorinfo") == 0)
    {
        whereMotor();
    }
    else if (strcmp(incomingParams[0], "gripperinfo") == 0)
    {
        printGripperInfo();
    }
    else if (strcmp(incomingParams[0], "robotinfo") == 0)
    {
      printRobotInfoCmd();
    }
    else if (strcmp(incomingParams[0], "movemotor") == 0)
    {
        moveMotor();
    }
    else if (strcmp(incomingParams[0], "gripper") == 0)
    {
        parse_gripper_command();
    }
    else if (strcmp(incomingParams[0], "help")==0)
    {
        printHelp(); // Cool help command
    }
    else if (strcmp(incomingParams[0], "debugon") == 0)
    {
        setDebugOn();
    }
    else if (strcmp(incomingParams[0], "debugoff") == 0)
    {
        setDebugOff();
    }
    else if (strcmp(incomingParams[0], "enablefan") == 0)
    {
        enableFan();
    }
    else if (strcmp(incomingParams[0], "disablefan") == 0)
    {
        disableFan();
    }
    else
    {
        //no command exists
        Serial.print("No command \"");
        Serial.print(incomingParams[0]);
        Serial.println("\" exists. Type \"help\' for more info.");
    }  
}

void updateJoints()
{
    //run stepper motors
    for (int i = 0; i < numSteppers; ++i)
    {
        // if currently homing and the limit switch is depressed
        if (isHoming[i] && !digitalRead(homeSwitchPin[i]))
        {
            steppers[i]->enableOutputs();
            steppers[i]->move(homingBuffer[i] * stepsPerDist[i]);
            hasReported[i] = false;   
        }
        if (steppers[i]->distanceToGo() != 0)
        {
            // can this cause intervals to skip? because not EXACTLY a multiple of 100 on evaluation
            // might depend on how slow/fast the main loop is, if too fast then it will repeat, if to slow it may skip
            // not too critical since only used for printing 
            if (millis() % 100 == 0 && VERBOSE)
            {
                //Serial.print("Moving, distance to go for stepper ");
                //Serial.print(i+1);
                //Serial.print(" is: ");
                //Serial.print(steppers[i]->distanceToGo() / stepsPerDist[i]);
                //Serial.print(" at ");
                //Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);
            }
            steppers[i]->run();
        }
    }
}

void moveMotor()
{
  //Check the number of tokens
    if (numTokens != 3)
    {
        Serial.println("Invalid number of parameters");
        return;
    }

    //Check that the first number is an integer and is within the range of valid stepper numbers
    long motorNum;
    if (!isValidMotorNum(incomingParams[1], motorNum))
    {
        return;
    }
    
    //Check that the second number is an actual number and is within the range of valid positons
    double targetPos;
    if (!isValidTargetPos(incomingParams[2], motorNum, targetPos))
    {
        return;
    }

    // Check limit switch
    //if (!hasHomed[motorNum-1])
    //{
    //    Serial.print("You must home stepper ");
    //    Serial.print(motorNum);
    //    Serial.println(" before you can move it");
    //    return;
    //}
    steppers[motorNum-1]->enableOutputs();
    steppers[motorNum-1]->moveTo(targetPos * stepsPerDist[motorNum-1]);
    hasReported[motorNum-1] = false;
}

void enableMotor()
{
    if (numTokens == 1)
    {
        for (int i=0; i<numSteppers; i++)
        {
            steppers[i]->stop();
            steppers[i]->enableOutputs();
        }
        if (VERBOSE)
        {
            Serial.println("Stepper motors have been enabled");
        }
    }
    if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }
        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->enableOutputs();
        if (VERBOSE)
        {
            Serial.print("Stepper ");
            Serial.print(motorNum);
            Serial.println(" has been enabled");
        }
    }
    else
    {
        Serial.println("Invalid number of parameters");
    }    
}

void enableFan()
{
    digitalWrite(FAN_0_PIN, HIGH);  
}

void disableFan()
{
    digitalWrite(FAN_0_PIN, LOW);  
}

void disableMotor()
{
    if (numTokens == 1)
    {
        for (int i = 0; i < numSteppers; ++i)
        {
            steppers[i]->stop();
            steppers[i]->disableOutputs();
        }
        if (VERBOSE)
        {
            Serial.println(F("Steppers have been disabled"));
        }
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }
        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->disableOutputs();
        if (VERBOSE)
        {
            Serial.print("Stepper ");
            Serial.print(motorNum);
            Serial.println(" has been disabled");
        }
    }
    else
    {
        Serial.println("Invalid number of parameters");
    }
}

void whereMotor()
{
    if (numTokens == 1)
    {
      for (int i = 0; i < numSteppers; ++i)
      {
         Serial.print("Stepper ");
         Serial.print(i+1);
         Serial.print(" is at = ");
         Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);
      }
    }
    else if (numTokens == 2)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }
        Serial.print("Stepper ");
        Serial.print(motorNum);
        Serial.print(" is at = ");
        Serial.println(steppers[motorNum-1]->currentPosition() / stepsPerDist[motorNum-1]);
    }
    else
    {
        Serial.println("Invalid number of parameters");
    }
}

bool isValidMotorNum(char* str, long& num)
{
    char* endptr;
    num = strtol(str, &endptr, 10);
    if (endptr == str || *endptr != '\0')
    {
        Serial.print("Parameter ");
        Serial.print(str);
        Serial.println(" is not an integer");
        return false;
    }
    else
    {
        if (num <= 0 || num > numSteppers)
        {
            Serial.println("Stepper number is invalid or out of bounds");
            return false;
        }
        else
        {
            return true;
        }
    }
}

bool isValidTargetPos(char* str, long motorNum, double& pos)
{
    char* endptr;
    pos = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0')
    {
        Serial.print("Parameter ");
        Serial.print(incomingParams[2]);
        Serial.println(" is not a number");
        return false;
    }
    else
    {
        if (pos < minPosition[motorNum-1] || pos > maxPosition[motorNum-1])
        {
            Serial.print("Position ");
            Serial.print(pos);
            Serial.println(" is out of range");
            return false;
        }
        else
        {
          return true;
        }
    }
}

void printRobotInfoCmd()
{
    if (numTokens != 1 && numTokens != 2)
    {
        Serial.println("Invalid number of parameters");
        return;
    }
    else if (numTokens == 1)
    {   
        Serial.print("Total number of steppers: ");
        Serial.println(numSteppers);
        for (int i = 0; i < numSteppers; ++i)
        {
            printMotorInfo(i);
        }
    }
    else
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        printMotorInfo(motorNum-1);
    }
    printGripperInfo();
}

void printGripperInfo()
{
    Serial.print("\n");
    Serial.println(F("===== Gripper ====="));
    Serial.println(F(" Info ====="));
    Serial.print(F("Gripper state: "));
    Serial.println(gripper_state);
}

void printMotorInfo(int i)
{
    Serial.print("\n");
    Serial.print(F("===== Stepper "));
    Serial.print(i+1);
    Serial.println(F(" Info ====="));

    Serial.print(F("Has been homed?:                  "));
    Serial.println(hasHomed[i]);
    
    Serial.print(F("Current position (mm):            "));
    Serial.println(steppers[i]->currentPosition() / stepsPerDist[i]);

    Serial.print(F("Lower position limit (mm):        "));
    Serial.println(minPosition[i]);

    Serial.print(F("Upper position limit (mm):        "));
    Serial.println(maxPosition[i]);

    Serial.print(F("Homing buffer gap (mm):           "));
    Serial.println(homingBuffer[i]);

    Serial.print(F("Disable when not moving?:         "));
    Serial.println(isDisabled[i]);

    //Serial.print(F("Is forward CCW?:                  "));
    //Serial.println(isForwardCCW[i]);

    Serial.print(F("Max speed (mm/s [rev/s]):         "));
    Serial.print(maxMotorSpeed[i] * distPerRev[i]);
    Serial.print(" [");
    Serial.print(maxMotorSpeed[i]);
    Serial.println("]");

    Serial.print(F("Acceleration (mm/s^2 [rev/s^2]):  "));
    Serial.print(maxMotorAcceleration[i] * distPerRev[i]);
    Serial.print(" [");
    Serial.print(maxMotorAcceleration[i]);
    Serial.println("]");

    Serial.print(F("Steps per distance (steps/mm):    "));
    Serial.println(stepsPerDist[i]);

    Serial.print(F("Steps per revolution:             "));
    Serial.println(stepsPerRev[i]);

    Serial.print(F("Distance per revolution (mm/rev): "));
    Serial.println(distPerRev[i]);
}

void printHelp()
{
    Serial.println(F("=================================== List of commands ==================================="));
    Serial.println(F("\tA = motor number from 1 to total number of motors"));
    Serial.println(F("\tB = distance of rotation in deg"));
    Serial.println(F(" "));
    Serial.println(F("\tinfo A           // Prints information about stepper motor A. Leave A blank for info on all motors."));
    Serial.println(F("\tmovemotor A B    // Moves motor A to absolute position B (deg)."));
    Serial.println(F("\tenablemotor A    // Enables motor A's stepper driver. Stepper holds position when not moving. Leave A blank to enable all motors."));
    Serial.println(F("\tdisablemotor A   // Disables motor A's stepper driver. Stepper may be rotated by external force when not moving. Leave A blank to disable all motors."));
    Serial.println(F("\tdebugon          // Enables the verbose output to get system information on commands"));
    Serial.println(F("\tdebugoff         // Disables the verbose output to get system information on commands"));
    Serial.println(F("\tgripper #        // Gripper command to send the state opened (1) or closed (0)"));
    Serial.println(F("========================================================================================"));
}

void setDebugOn()
{
    if (numTokens == 1)
    {
        VERBOSE = true;
        Serial.println(F("Debug messages turned ON"));
    }
    else
    {
        if (VERBOSE)  
        {
            Serial.println("Invalid number of parameters");
        }
    }
}

void setDebugOff()
{
    if (numTokens == 1)
    {
        VERBOSE = false;
        //should not be able to print below if false
        Serial.println("Debug messages turned OFF");
    }
    else
    {
        if (VERBOSE)  
        {
            Serial.println("Invalid number of parameters");
        }
    }
}

int get_gripper_state()
{
    return gripper_state;
}

void parse_gripper_command() 
{
    // For now simple open and close
    gripper_state = atoi(incomingParams[1]);
    if (gripper_state == 1) 
    {
        if (VERBOSE)
        {
            Serial.println("Gripper open");
        }
        gripper.write(GRIPPER_OPEN_POS);
    }
    if (gripper_state == 0) {
        if (VERBOSE)
        {
            Serial.println("Gripper close");
        }
        gripper.write(GRIPPER_CLOSE_POS);
    }
}

void updateLCD()
{
  // Makes a delay in the code, not using yet
  // Update counter on screen (testing)
  //lcd.setCursor(0, 1);
  // Print the number of seconds since reset:
  //lcd.print(millis() / 1000);
}

void set_current_pos_to_zero()
{
    for (int i = 0; i < numSteppers; ++i)
    {
        steppers[i]->setCurrentPosition(0); // Set the current position to 0
    }  
    if (VERBOSE)
    {
        Serial.println(F("Current stepper positions have been set to zero"));
    }
}

void setupPins() 
{ 
    // Setup enable pinds for stepper motors
    for (int i = 0; i < numSteppers; ++i)
    {
        stepsPerDist[i] = stepsPerRev[i] / distPerRev[i];
        steppers[i]->setEnablePin(jointEnablePin[i]);
        pinMode(homeSwitchPin[i], INPUT); // limit switches
    }
    pinMode(FAN_0_PIN, OUTPUT);    // Internal fan control
    //pinMode(JOINT1_ENABLE_PIN, OUTPUT);   
    //pinMode(JOINT2_ENABLE_PIN, OUTPUT);   
    //pinMode(JOINT3_ENABLE_PIN, OUTPUT);   
    //pinMode(JOINT4_ENABLE_PIN, OUTPUT);   
  
    //digitalWrite(JOINT0_ENABLE_PIN, LOW);
    //digitalWrite(JOINT1_ENABLE_PIN, LOW);
    //digitalWrite(JOINT2_ENABLE_PIN, LOW);
    //digitalWrite(JOINT3_ENABLE_PIN, LOW);
    //digitalWrite(JOINT4_ENABLE_PIN, LOW);

    pinMode(CONNECT_LED_PIN_OUT, OUTPUT); // Serial connection status
    
}

void setupSwitches(int debounce_time)
{
  // Set up limit switch debounce time (ex: 1 millisecond, quick response)
  //joint0Switch.setDebounceTime(debounce_time);
  //joint1Switch.setDebounceTime(debounce_time);
  //joint2Switch.setDebounceTime(debounce_time);
  //joint3Switch.setDebounceTime(debounce_time);
  //joint4Switch.setDebounceTime(debounce_time);  
}

void setupJoints() 
{
    for (int i = 0; i < numSteppers; ++i)
    {
        steppers[i]->setPinsInverted(false, false, true); 
        steppers[i]->setAcceleration(maxMotorAcceleration[i]);
        steppers[i]->setMaxSpeed(maxMotorSpeed[i]);
        steppers[i]->setSpeed(maxMotorSpeed[i]);
        if (isDisabled[i])
        {
            steppers[i]->disableOutputs();  
        }
        else
        {
            steppers[i]->enableOutputs();
        }
    }
    if (VERBOSE)
    {
        Serial.println(F("Steppers have been enabled"));
    }
}

void setupServos() {
  // Just one servo controlling gripper
  gripper.attach(SERVO0_PIN);
}
