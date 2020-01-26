/*
Robot Arm Sweep and Retract_1_25_20

Moves all servos with specific times and rotations

Modified 25th January 2020
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
int total_duration = 0;

struct ServoAction
{
  unsigned long actionTime; // Number of milliseconds after sequence starts to do this action
  Servo ServoToAct;         // Which servo is it?
  int startPos;         // Number of degrees to send the designated servo to
  int endPos;           // Number of degrees to send the designated servo to
  int startTime;            // Start time
  int endTime;              // End time
};

ServoAction actionArray[] = {
  {    0,       Torso,     0,    90,     0,    1,}, // When sequence starts, send Torso to 90 degrees
  { 1000,    Shoulder,     0,    45,     0,    1,},
  { 1000,        Claw,   180,   180,     0,    1,},
  { 2000,       Elbow,     0,     0,     0,    1,}, 
  { 9000,    WristRot,     0,    90,     0,    1,},
  {10000,   WristBend,     0,     0,     0,    1,},
  {10000,       Torso,     0,     0,     0,    1,},
  {11000,       Torso,   180,    90,     0,    1,},
  /* etc. etc. */
};

const byte N_Actions = sizeof actionArray / sizeof actionArray[0];

int initialize() {
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
  unsigned long stepsize = (1000/frequency);
  Serial.print("step size: "); Serial.println(stepsize);
  Serial.print("max delta: "); Serial.println(max_delta_t);
  Serial.print("total duration: "); Serial.println(total_duration);
  unsigned int pos_Array[N_Actions][N_elem]={}; //Array creation

  Serial.print("Array size: "); Serial.print(N_Actions); Serial.print(" by "); Serial.println(N_elem);
  // Fill in array with padded values to start 
  for (int k=0; k<N_Actions; k += 1){
    unsigned long action_time = actionArray[k].actionTime;
    Serial.print(action_time); 
    if (action_time > 0){
//      for (long i=0; i<actionArray[k].actionTime; i += stepsize){
//        Serial.print(actionArray[k].startTime); Serial.print(" ");
//        pos_Array[k][i] = actionArray[k].startTime;
//      }
//      Serial.println(": not zero ");
//    } else{
    Serial.println(": zero");
    }
//    // Create the positional values from linspace
//    int delta_t = abs(actionArray[k].endTime-actionArray[k].startTime);
//    int N_incr = round(delta_t*frequency);
//    for (int j=1; j <= N_incr; j+=1){
//      Serial.print(actionArray[k].startPos + (j-1)*(actionArray[k].endPos-actionArray[k].startPos)/(N_incr-1));
//      Serial.println(" ");
//      pos_Array[k][actionArray[k].startPos + j] = actionArray[k].startPos + (j-1)*(actionArray[k].endPos-actionArray[k].startPos)/(N_incr-1);
//    }
//    Serial.println(" ");
//    
//    // Finish padding at end
//    if (actionArray[k].endTime < max_val){
//      for (int i=max_val; i> actionArray[k].endTime; i-=stepsize){
//        Serial.print(actionArray[k].endTime); Serial.println(" ");
//        pos_Array[k][i] = actionArray[k].endTime;
//      }
//      Serial.println(" ");
//    }
//    for (int m=0; m<total_duration; m+=1){
//      Serial.print("Servo: ");
//      Serial.println(k);
//      Serial.print("Position array: ");
//      Serial.print(pos_Array[k][m]); Serial.print(" ");
//      Serial.println(" ");
//    }
  }
  
  //return pos_Array;
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
  Serial.begin(9600);


  int *p_array;
  p_array = initialize();
  //execute_action(p_array, N_Actions);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
