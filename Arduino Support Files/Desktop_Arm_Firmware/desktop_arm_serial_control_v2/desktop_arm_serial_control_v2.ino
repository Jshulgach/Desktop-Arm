
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

   Description:
      This Arduino code controls the Desktop Arm robot. it uses microstepping of steppermotors for precise joint states,
      serial parsing for command inputs and outputs, an optional PID controller for joint state handling, text output on an LCD
      display using the SerLCD module, servo control, 
     
      - More info on Qwiic here: https://www.sparkfun.com/qwiic
      - Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
      - see Serial Input Basics by Robin2: https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
      - AVR-Based Serial Enabled LCDs Hookup Guide
        https://learn.sparkfun.com/tutorials/avr-based-serial-enabled-lcds-hookup-guide
      - Reference for awesome multi-stepper serial command parsing from JustinJKwok's Github
        https://github.com/JustinJKwok/Arduino-multi-stepper/blob/master/multi_stepper.ino

*/
#include <Servo.h>
//#include <PID_v1.h>
#include <Wire.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
#include <AccelStepper.h>
//#include <ezButton.h>
//#include <TMC2208Stepper.h>
//#include <TMC2208Stepper_REGDEFS.h>
#include "Configuration.h"
#include "pins.h"

// Stepper Motors (Up to 10 steppers can be handled as a group by AccelStepper)
const int numSteppers = 5;
AccelStepper joint0Stepper = AccelStepper(motorInterfaceType, JOINT0_STEP_PIN, JOINT0_DIR_PIN);  // Rotating base joint
AccelStepper joint1Stepper = AccelStepper(motorInterfaceType, JOINT1_STEP_PIN, JOINT1_DIR_PIN);  // Joint 1 (shoulder rotation)
AccelStepper joint2Stepper = AccelStepper(motorInterfaceType, JOINT2_STEP_PIN, JOINT2_DIR_PIN);  // Joint 2 ( elbow rotation)
AccelStepper joint3Stepper = AccelStepper(motorInterfaceType, JOINT3_STEP_PIN, JOINT3_DIR_PIN);  // Joint 3 (rotating wrist joint)
AccelStepper joint4Stepper = AccelStepper(motorInterfaceType, JOINT4_STEP_PIN, JOINT4_DIR_PIN);  // Joint 4 (moves the gripper)
AccelStepper* steppers[numSteppers] = {&joint0Stepper, &joint1Stepper, &joint2Stepper, &joint3Stepper, &joint4Stepper};

// PID controllers
//double Setpoint0, Setpoint1, Setpoint2, Setpoint3, Setpoint4;
//float setpointPos[] = {&Setpoint0, &Setpoint1, &Setpoint2, &Setpoint3, &Setpoint4};
//double Input0, Input1, Input2, Input3, Input4;
//float inputPos[] = {&Input0, &Input1, &Input2, &Input3, &Input4};
//double Output0, Output1, Output2, Output3, Output4;
//float outputPos[] = {&Output0, &Output1, &Output2, &Output3, &Output4};
//PID joint0PID(&Input0, &Output0, &Setpoint0, Kp, Ki, Kd, DIRECT);
//PID joint1PID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
//PID joint2PID(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);
//PID joint3PID(&Input3, &Output3, &Setpoint3, Kp, Ki, Kd, DIRECT);
//PID joint4PID(&Input4, &Output4, &Setpoint4, Kp, Ki, Kd, DIRECT);
//PID* stepperPID[numSteppers] = {&joint0PID, &joint1PID, &joint2PID, &joint3PID, &joint4PID};

// Servos
Servo gripper;

//ezButton joint0Switch(JOINT0_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint1Switch(JOINT1_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint2Switch(JOINT2_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint3Switch(JOINT3_MIN_PIN);  // create ezButton object that attach to pin 7;
//ezButton joint4Switch(JOINT4_MIN_PIN);  // create ezButton object that attach to pin 7;

SerLCD lcd; // Initialize the library with default I2C address 0x72

//Custom Characters
byte selectChar[] = {
  B00000,
  B00000,
  B00110,
  B01111,
  B01111,
  B00110,
  B00000,
  B00000
};

// Stepper motor and servo specific variables including limit switches
int homeSwitchPin[] = {JOINT0_MIN_PIN, JOINT1_MIN_PIN, JOINT2_MIN_PIN, JOINT3_MIN_PIN, JOINT4_MIN_PIN};
int jointEnablePin[] = {JOINT0_ENABLE_PIN, JOINT1_ENABLE_PIN, JOINT2_ENABLE_PIN, JOINT3_ENABLE_PIN, JOINT4_ENABLE_PIN};
float minPosition[] = {JOINT0_MIN_POS, JOINT1_MIN_POS, JOINT2_MIN_POS, JOINT3_MIN_POS, JOINT4_MIN_POS};
float maxPosition[] = {JOINT0_MAX_POS, JOINT1_MAX_POS, JOINT2_MAX_POS, JOINT3_MAX_POS, JOINT4_MAX_POS};
float maxMotorSpeed[] = {JOINT0_MAX_SPEED, JOINT1_MAX_SPEED, JOINT2_MAX_SPEED, JOINT3_MAX_SPEED, JOINT4_MAX_SPEED};
float maxMotorAcceleration[] = {JOINT0_MAX_ACCELERATION, JOINT1_MAX_ACCELERATION, JOINT2_MAX_ACCELERATION, JOINT3_MAX_ACCELERATION, JOINT4_MAX_ACCELERATION};
float homingBuffer[] = {JOINT0_HOMING_BUFFER, JOINT1_HOMING_BUFFER, JOINT2_HOMING_BUFFER, JOINT3_HOMING_BUFFER, JOINT4_HOMING_BUFFER}; 
float stepsPerRev[] = {JOINT0_REV_STEPS, JOINT1_REV_STEPS, JOINT2_REV_STEPS, JOINT3_REV_STEPS, JOINT4_REV_STEPS}; 
bool isDisabled[] = {JOINT0_DISABLED, JOINT1_DISABLED, JOINT2_DISABLED, JOINT3_DISABLED, JOINT4_DISABLED};
float stepperPos[numSteppers] = {0.0,0.0,0.0,0.0,0.0};

// Global state-tracking variables
//int n_repeat, t_start, x;
bool hasHomed[] = {false, false, false, false, false};
bool isHoming[] = {false, false, false, false, false};
bool hasReported[] = {true, true, true, true, true};
int gripper_state;

// Serial communication handling for commands 
bool VERBOSE = false;
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
  // ==== Serial Setup ====
  Serial.begin(BAUDRATE);
  Serial.println("Ready");
  
  // ===== Initialize stepper motors  and servos =====
  setupPins();
  //setupSwitches(1);
  setupJoints();
  setupServos();
  
  // === Select one or the other for zero positioning ===
  //homeJoints();
  //zeroJoints();
  setCurrentPosToMid();
  //setCurrentPosToZero();
  //send_home_state();
  delay(100);
  whereMotor();
 
  // ===== LCD Setup =====
  setupLCD();
    
  //t_start = millis();  

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
    else if (strcmp(incomingParams[0], "setmotorspeed") == 0)
    {
        setMotorSpeed();
    }
    else if (strcmp(incomingParams[0], "setmotoracceleration") == 0)
    {
        setMotorAcceleration();
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
            steppers[i]->move(homingBuffer[i] * stepsPerRev[i]);
            hasReported[i] = false;   
        }
        if (steppers[i]->distanceToGo() != 0)
        {
            // can this cause intervals to skip? because not EXACTLY a multiple of 100 on evaluation
            // might depend on how slow/fast the main loop is, if too fast then it will repeat, if to slow it may skip
            // not too critical since only used for printing 
            if (millis() % 200 == 0 && VERBOSE)
            {
                Serial.print("Moving, distance to go for stepper ");
                Serial.print(i+1);
                Serial.print(" is: ");
                Serial.print(steppers[i]->distanceToGo()*360/stepsPerRev[i]);
                Serial.print(" at ");
                Serial.println(convertStepperPosToDeg(i, steppers[i]->currentPosition()));
            }
            //if (USE_PID) stepperPID[i]->Compute();
            steppers[i]->run();
            stepperPos[i] = steppers[i]->currentPosition()*360 / stepsPerRev[i];
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
    if (isDisabled[motorNum-1])
    {
      Serial.print("Motor ");
      Serial.print(motorNum-1);
      Serial.println(" disabled");
      return;
    }
    int new_pos = convertDegToStepperPos(motorNum-1, targetPos);
    if (VERBOSE)
    {
        Serial.print("New joint value position received: ");
        Serial.println(new_pos);
    }
    //if (USE_PID)
    //{
    //    setpointPos[motorNum-1] = targetPos;
    //    updateMotorPosPID(motorNum-1, targetPos);
    //}
    steppers[motorNum-1]->enableOutputs();
    steppers[motorNum-1]->moveTo(new_pos);
    hasReported[motorNum-1] = false;
     
}

int convertDegToStepperPos(int motorNum, int targetPos)
{
  float midpos = float( (maxPosition[motorNum] + minPosition[motorNum])/2 );
  int new_pos = int( (midpos + targetPos*stepsPerRev[motorNum]/360) );
  return new_pos;
}

int convertStepperPosToDeg(int motorNum, int targetPos)
{
  float midpos = float( (maxPosition[motorNum] + minPosition[motorNum])/2 );
  //Serial.print("mid position: ");
  //Serial.println(midpos);
  int new_pos = int( (targetPos - midpos)*360/stepsPerRev[motorNum] );
  return new_pos;
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
    if (numTokens == 0 || numTokens == 1)
    {
        for (int i = 0; i < numSteppers; ++i)
        {
            Serial.print("Stepper ");
            Serial.print(i+1);
            Serial.print(" is at = ");
            Serial.print(convertStepperPosToDeg(i, steppers[i]->currentPosition()));
            Serial.print(" deg [");
            Serial.print(steppers[i]->currentPosition());
            Serial.println(" steps]");
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
        Serial.print(convertStepperPosToDeg(motorNum-1, steppers[motorNum-1]->currentPosition()));
        Serial.print(" deg [");
        Serial.print(steppers[motorNum-1]->currentPosition());
        Serial.println(" steps]");
    }
    else
    {
        Serial.println("Invalid number of parameters");
    }
}

void setMotorSpeed()
{
    if (numTokens == 3)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        double targetSpeed;
        if (!isValidMotorSpeed(incomingParams[2], motorNum, targetSpeed))
        {
            return;
        }
        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->disableOutputs();
        steppers[motorNum-1]->setSpeed(targetSpeed);
        steppers[motorNum-1]->setMaxSpeed(targetSpeed);
        steppers[motorNum-1]->enableOutputs();
        if (VERBOSE)
        {
            Serial.print("Stepper ");
            Serial.print(motorNum);
            Serial.print(" speed has been set to ");
            Serial.println(targetSpeed);
        }
    }
    else
    {
        Serial.println("Invalid number of parameters");
    }
}

void setMotorAcceleration()
{
    if (numTokens == 3)
    {
        long motorNum;
        if (!isValidMotorNum(incomingParams[1], motorNum))
        {
            return;
        }

        double targetAcceleration;
        if (!isValidMotorAcceleration(incomingParams[2], motorNum, targetAcceleration))
        {
            return;
        }
        steppers[motorNum-1]->stop();
        steppers[motorNum-1]->disableOutputs();
        steppers[motorNum-1]->setAcceleration(targetAcceleration);
        steppers[motorNum-1]->enableOutputs();
        if (VERBOSE)
        {
            Serial.print("Stepper ");
            Serial.print(motorNum);
            Serial.print(" acceleration has been set to ");
            Serial.println(targetAcceleration);
        }
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

bool isValidMotorAcceleration(char* str, long motorNum, double& val)
{
    char* endptr;
    val = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0')
    {
        Serial.print("Parameter ");
        Serial.print(incomingParams[2]);
        Serial.println(" is not a number");
        return false;
    }
    else
    {
        if (val < 0 || val > maxMotorAcceleration[motorNum-1])
        {
            Serial.print("Acceleration ");
            Serial.print(val);
            Serial.println(" is out of range");
            return false;
        }
        else
        {
          return true;
        }
    }
}

bool isValidMotorSpeed(char* str, long motorNum, double& val)
{
    char* endptr;
    val = strtod(str, &endptr);
    if (endptr == str || *endptr != '\0')
    {
        Serial.print("Parameter ");
        Serial.print(incomingParams[2]);
        Serial.println(" is not a number");
        return false;
    }
    else
    {
        if (val < 0 || val > maxMotorSpeed[motorNum-1])
        {
            Serial.print("Speed ");
            Serial.print(val);
            Serial.println(" is out of range");
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
        int min_limit = convertStepperPosToDeg(motorNum-1, minPosition[motorNum-1]);
        int max_limit = convertStepperPosToDeg(motorNum-1, maxPosition[motorNum-1]);
        if (pos < min_limit || pos > max_limit)
        {
            Serial.print("Position ");
            Serial.print(pos);
            Serial.print(" is out of range. Limit for joint is [");
            Serial.print(min_limit);
            Serial.print(" ");
            Serial.print(max_limit);
            Serial.println("]");
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

const char* logicToText(const bool b) {
    return b ? "True" : "False";
}

void printMotorInfo(int i)
{
    Serial.print("\n");
    Serial.print(F("===== Stepper "));
    Serial.print(i+1);
    Serial.println(F(" Info ====="));

    Serial.print(F("Has been homed?:                  "));
    Serial.println(logicToText(hasHomed[i]));
    
    Serial.print(F("Current position (deg):           "));
    Serial.println(convertStepperPosToDeg(i, steppers[i]->currentPosition()));

    Serial.print(F("Lower position limit (deg):       "));
    Serial.println(convertStepperPosToDeg(i, minPosition[i]));

    Serial.print(F("Upper position limit (deg):       "));
    Serial.println(convertStepperPosToDeg(i, maxPosition[i]));

    Serial.print(F("Disable when not moving?:         "));
    Serial.println(logicToText(isDisabled[i]));

    //Serial.print(F("Is forward CCW?:                "));
    //Serial.println(isForwardCCW[i]);

    Serial.print(F("Max speed (steps/s):              "));
    //Serial.print(maxMotorSpeed[i] * distPerRev[i]);
    //Serial.print(" [");
    Serial.println(maxMotorSpeed[i]);
    //Serial.println("]");

    Serial.print(F("Acceleration (steps/s^2):         "));
    //Serial.print(maxMotorAcceleration[i] * distPerRev[i]);
    //Serial.print(" [");
    Serial.println(maxMotorAcceleration[i]);
    //Serial.println("]");

    //Serial.print(F("Steps per distance (steps/mm):  "));
    //Serial.println(stepsPerDist[i]);

    Serial.print(F("Steps per revolution:             "));
    Serial.println(stepsPerRev[i]);

    //Serial.print(F("Distance per revolution (mm/rev): "));
    //Serial.println(distPerRev[i]);
}

void printHelp()
{
    Serial.println(F("=================================== List of commands ==================================="));
    Serial.println(F("MOTOR = motor number from 1 to total number of motors"));
    Serial.println(F("VALUE = distance of rotation in deg, gripper state (0 or 1), speed, acceleration"));
    Serial.println(F(" "));
    Serial.println(F("debugon                          // Enables the verbose output to get system information on commands"));
    Serial.println(F("debugoff                         // Disables the verbose output to get system information on commands"));
    Serial.println(F("disablefan                       // Disables internal cooling fan"));
    Serial.println(F("disablemotor MOTOR               // Disables motor A's stepper driver. Stepper may be rotated by external force when not moving. Leave A blank to disable all motors."));
    Serial.println(F("enablefan                        // Enables internal cooling fan"));
    Serial.println(F("enablemotor MOTOR                // Enables motor's stepper driver. Stepper holds position when not moving. Leave A blank to enable all motors."));
    Serial.println(F("motorinfo MOTOR                  // Prints information about stepper motor A. Leave A blank for info on all motors."));
    Serial.println(F("movemotor MOTOR VALUE            // Moves motor A to absolute position B (deg)."));
    Serial.println(F("gripper VALUE                    // Gripper command to send the state opened (1) or closed (0)"));
    Serial.println(F("gripperinfo                      // Prints information about the gripper's status"));
    Serial.println(F("robotinfo                        // Prints information about the robot system, including motors, grippers, and sensors."));
    Serial.println(F("setmotoracceleration MOTOR VALUE // Sets a new acceleration value for a specified motor "));
    Serial.println(F("setmotorspeed MOTOR VALUE        // Sets a new speed value for a specified motor "));
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

void setCurrentPosToZero()
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

void setCurrentPosToMid()
{
    for (int i = 0; i < numSteppers; ++i)
    {
        int midpos = int((maxPosition[i] - minPosition[i])/2);
        steppers[i]->setCurrentPosition(midpos); // Set the current position to 0
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
        //stepsPerDist[i] = stepsPerRev[i] / distPerRev[i];
        steppers[i]->setEnablePin(jointEnablePin[i]);
        pinMode(homeSwitchPin[i], INPUT); // limit switches
    }
    pinMode(FAN_0_PIN, OUTPUT);    // Internal fan control
    //pinMode(JOINT1_ENABLE_PIN, OUTPUT);
    pinMode(CONNECT_LED_PIN_OUT, OUTPUT); // Serial connection status
    
}

void setupLCD()
{
    //unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    //int delta_time = current_time - last_time; //delta time interval
    //if (delta_time >= T)
    Wire.begin();
    lcd.begin(Wire); //Set up the LCD for I2C communication
    lcd.setBacklight(5, 205, 205); //Set backlight to bright white
    lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
    lcd.clear(); //Clear the display - this moves the cursor to home position as well
    lcd.print("Desktop Arm");
    lcd.saveSplash();//Save this current text as the splash screen at next power on
    lcd.enableSplash(); //This will cause the splash to be displayed at power on
    //lcd.disableSplash(); //This will supress any splash from being displayed at power on
}

void updateLCD()
{ 
  // Update counter on screen (testing)
  lcd.setCursor(0, 1);
  // Print the number of seconds since reset:
  lcd.print(millis() / 1000);
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
    //setupPIDControl();
}

void setupServos() 
{
  // Just one servo controlling gripper
  gripper.attach(SERVO0_PIN);
}

/*
void setupPIDControl()
{  
    if (ENABLED(USE_PID))
    {
        for (int i=0; i<numSteppers; i++)
        {
            stepperPID[i]->SetMode(AUTOMATIC);
        }
        if (VERBOSE)
        {
            Serial.println("PID control for stepper motors has been enabled");
        }
    }
    else
    {
      if (VERBOSE)
        {
            Serial.println("PID control for stepper motors has been disabled");
        }
    }
}

void updateMotorPosPID(int motorNum)
{
    long motorNum;
    if (!isValidMotorNum(incomingParams[1], motorNum))
    {
        return;
    }
    intputPos[motorNum-1] = steppers[motorNum-1]->currentPosition();
}

*/
