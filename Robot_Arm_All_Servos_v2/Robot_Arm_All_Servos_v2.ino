/*
 * 
Robot Arm Sweep and Retract_1_25_20

Moves all servos within a system based on positional values stored in a multidimmensional array. The position array is 6 rows long, 
and the length is determined by duration of the entire servo motion set. In this case, 6 servos will create a row each for
positional array correspondign to that servo. In the actionArray variable, information pertaining to certain "actions" are 
collected with the time to start movement, name of servo, start/end positions and start/end times. The duration of the servo 
action, along with the frequency of data being written to the servos, creates a "linspace" of numbers interpolatd from starting to
ending positions within the array, according to when the action time happens within the overal duration of the actions

Modified 26th January 2020
by Jonathan Shulgach

 */

#include <Servo.h>

Servo Torso;
Servo Shoulder;
Servo Elbow;
Servo WristRot;
Servo WristBend;
Servo Claw;

int frequency = 10;
long previousTime = 0;
int max_val = 0;
int max_idx = 0;
unsigned long compare = 0;
int val = 0;

String uniqueServos[] = {"Torso","Shoulder","Elbow","Claw", "WristRot","WristBend"};

struct ServoAction
{
  unsigned long actionTime; // Number of milliseconds after sequence starts to do this action
  Servo ServoToAct;         // Which servo is it?
  String servoName;           // Name of servo (need this for now)
  int startPos;             // Number of degrees to send the designated servo to
  int endPos;               // Number of degrees to send the designated servo to
  int startTime;            // Start time
  int endTime;              // End time
};

ServoAction actionArray[] = {
  {    0,       Torso,     "Torso",     0,    90,     0,    1,}, // When sequence starts, send Torso to 90 degrees
  {    0,    Shoulder,  "Shoulder",     0,    45,     0,    1,},
  {    0,       Elbow,     "Elbow",    90,   180,     0,    1,},
  {    0,        Claw,      "Claw",     0,    90,     0,    1,}, 
  { 9000,    WristRot,  "WristRot",     0,    90,     0,    1,},
  {10000,   WristBend, "WristBend",     90,   90,     0,    1,},
  {10000,       Torso,     "Torso",     90,  180,     0,    1,},
  {11000,       Torso,     "Torso",   180,    90,     0,    1,},
  /* etc. etc. */
};

int N_Actions = sizeof actionArray / sizeof actionArray[0];
int N_Servos = sizeof uniqueServos / sizeof uniqueServos[0];

void initialize() {
  // Initialize whole array with largest size needed
  for (int k=0; k<N_Actions; k += 1){
    if (actionArray[k].actionTime > max_val){
      max_val = actionArray[k].actionTime;
      max_idx = k;
    }
  }
  int max_delta_t = abs(actionArray[max_idx].endTime-actionArray[max_idx].startTime);
  static unsigned long total_duration = actionArray[max_idx].actionTime+round(max_delta_t*1000);
  unsigned long N_elem = round(total_duration*frequency/1000);
  
  Serial.print( F("Frequency: ") ); Serial.println(frequency);
  Serial.print( F("max delta: ") ); Serial.println(max_delta_t);
  Serial.print( F("total duration: ") ); Serial.println(total_duration);
  Serial.print( F("Array size: ") ); Serial.print(N_Servos); Serial.print(" by "); Serial.println(N_elem); Serial.println(" ");
  
  unsigned int test_pos[N_Servos][N_elem]={};
  //unsigned int test_pos[N_elem]={};

  Serial.println( F("Servos: ") );
  for (int n=0; n<N_Servos; n += 1){ 
    Serial.println(uniqueServos[n]);
    
    for (int k=0; k<N_Actions; k += 1){
      if (actionArray[k].servoName==uniqueServos[n]){
        int delta_t = abs(actionArray[k].endTime-actionArray[k].startTime);
        int N_incr = round(delta_t*frequency);
        int elem = actionArray[k].actionTime*frequency/1000;
      
        for (int i=0; i < N_incr; i+=1){
          test_pos[n][elem + i] = actionArray[k].startPos + (i)*(actionArray[k].endPos-actionArray[k].startPos)/(N_incr-1);
        }
        for (int i=(elem + N_incr); i<N_elem; i+=1){
          test_pos[n][i] = actionArray[k].endPos;
        }
      }
    }                                                                       
  }

  Serial.println(" ");
  Serial.println( F("Position array: ") );
  for (int n=0; n<N_Servos; n += 1){ 
    for (int i=0; i<N_elem; i += 1){
      Serial.print(test_pos[n][i]); Serial.print(" ");
    }
    Serial.println(" ");
  }
}

//void execute_Action(int pos_Array, int N_Actions){
//  // Run through all position values for each servo
//  for (int i=0; i<total_duration; i+=1){ // for each position
//    while (( millis() - previousTime) < 1000/frequency){
//      // wait until time interval has passed
//    }
//    for (int k = 0; k < N_Actions; k+=1){ // cycle through each servo
//      actionArray[k].ServoToAct.write(pos_Array[k][i]);
//    }
//    previousTime = millis();  
//  }    
//}


void setup() {
  Torso.attach(2);
  Shoulder.attach(4);
  Elbow.attach(3);
  WristRot.attach(5);
  WristRot.write(90);
  WristBend.attach(6);
  Claw.attach(7);
  Serial.begin(115200);

  //int *p_array;
  initialize();
  //execute_action(p_array, N_Actions);
  
}

void loop() {
  
}
