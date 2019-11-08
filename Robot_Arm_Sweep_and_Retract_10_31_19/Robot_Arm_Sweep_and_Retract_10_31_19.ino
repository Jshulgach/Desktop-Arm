#include <Servo.h>

Servo Torso;
Servo Shoulder;
Servo Elbow;
Servo WristRot;
Servo WristBend;
Servo Claw;

int pos = 0;
int oldPos = 0;
int runXtimes = 0;
float delta_t = 0;
float delta_x = 0;
long interval = 0;
long previousMillis = 0;


struct ServoAction
{
  unsigned long ActionTime;  // Number of milliseconds after sequence starts to do this action
  Servo ServoToAct;         // Which servo is it?
  bool ActionDone;           // Is action done yet
  int startDegrees;               // Number of degrees to send the designated servo to
  int endDegrees;               // Number of degrees to send the designated servo to
  };

ServoAction retract_Actions[] = {
  {   0,      Torso, false,     0,    90}, // When sequence starts, send Torso to 90 degrees
  {   0,   Shoulder, false,     0,    45},
  {   0,       Claw, false,   180,   180},
  {   0,      Elbow, false,     0,     0}, 
  {   0,   WristRot, false,     0,    90},
  {   0,  WristBend, false,     0,     0},
  {1000,       Claw, false,     0,     0},
  {1200,       Claw, false,    0,    180},
  /* etc. etc. */
};

const byte NoActions = sizeof retract_Actions / sizeof retract_Actions[0];
unsigned long StartTime;

void setup() {
  // put your setup code here, to run once:
  Torso.attach(2);
  Shoulder.attach(4);
  Elbow.attach(3);
  WristRot.attach(5);
  WristRot.write(90);
  WristBend.attach(6);
  Claw.attach(7);
  Serial.begin(9600);
  StartTime = millis();
}

boolean AllActionsDone() {
for (int i = 0; i < NoActions; i++)
  if (!retract_Actions[i].ActionDone)
    return false;
return true;   
}

void ResetActions() {
for (int i = 0; i < NoActions; i++)
  retract_Actions[i].ActionDone=false;
  StartTime = millis();
}

void sweep_servo(Servo &myservo, int pause) {
  for (pos = 0; pos <= 180; pos += 1) {
    // in steps of 1 degree
    myservo.write(pos);
    delay(pause);                       
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(pause);                         
  }
}


void rotate(Servo &myservo, int startpos, int endpos, float rpm) {
  delta_x = abs(startpos-endpos);
  interval = int((rpm*1000)/delta_x); //divide time over angle to get delay per angle step (converted to millis)
  if (startpos < endpos) {    
    for (pos = startpos; pos <= endpos; pos += 1) { 
      while(pos != oldPos) {
        unsigned long currentMillis = millis(); 
        if (currentMillis - previousMillis > interval){
          previousMillis = currentMillis;
          myservo.write(pos);
          oldPos = pos;
        }
      }                 
    }
  } else {
    for (pos = startpos; pos >= endpos; pos -= 1) {
      while(pos != oldPos) {
        unsigned long currentMillis = millis(); 
        if (currentMillis - previousMillis > interval){
          previousMillis = currentMillis;
          myservo.write(pos);
          oldPos = pos;
        }
      }  
    }
  }
}

void retract() {
   for (int i = 0; i < NoActions; i++)
      if (!retract_Actions[i].ActionDone)
        if (millis() - StartTime > retract_Actions[i].ActionTime) {
          retract_Actions[i].ActionDone = true;
          rotate(retract_Actions[i].ServoToAct, retract_Actions[i].startDegrees, retract_Actions[i].endDegrees, 3);
        }
        
    if(AllActionsDone()) {
      ResetActions(); 
      }
}

void retract_instant() {
  ResetActions(); 
  for (int i = 0; i < NoActions; i++) {
    Serial.print(i);
    Serial.print(" | Waiting..");
    while (!retract_Actions[i].ActionDone) {
      if (millis() - StartTime > retract_Actions[i].ActionTime) {
        Serial.println(" | Perform Action!");
        retract_Actions[i].ActionDone = true;
        retract_Actions[i].ServoToAct.write(retract_Actions[i].endDegrees);
      }
      
    }
       
    if(AllActionsDone()) {
      Serial.println("All Done!");
      ResetActions(); 
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  
    
  //Serial.println(runXtimes);
  //delay(5000);
  //sweep_servo(Torso, 5);
  //rotate(Torso, 0, 45, 10);
  //rotate(Torso, 45, 90, 5);

  
  if (runXtimes<1) { // run just once
   delay(1000);
   rotate(Torso, 0, 180, 2);
   delay(500);
   rotate(Torso, 180, 0, 2);
   delay(500);
   rotate(Claw, 0, 120, 1);
   delay(500);
   rotate(Claw, 120, 0, 1);
   delay(1000);
   //retract();
   retract_instant();

   runXtimes++;
  }
}
