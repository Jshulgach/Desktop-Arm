#include <Servo.h>

Servo Torso;

int startTime = 0;
int time_s = 1;
int startPos = 0;
int endPos = 180;


void move_servo(Servo &myservo, int move_Array[], int N){
  int previousTime = 0;
  for (int j = 0; j < N; j += 1){
    while (( millis() - previousTime) < 1){
    }
      myservo.write(move_Array[j]);
      previousTime = millis();
  }
}


void setup() {
  Torso.attach(2);
  Serial.begin(9600);
  
  int delta_pos = abs(startPos-endPos);
  int incr = round((time_s*1000)/delta_pos);
  int j=0;
  int pos_Array[incr*delta_pos]={};
  int N = (sizeof(pos_Array) / sizeof(pos_Array[0]));
  
  Serial.print("detla pos: ");
  Serial.println(delta_pos);
  Serial.print("increment step: ");
  Serial.println(incr);
  Serial.print("array size: ");
  Serial.println(N);
  for (int pos = startPos; pos < endPos; pos += 1){
    for (int i = 0; i < incr; i += 1){
      pos_Array[j*incr + i] = pos;
    }
    j += 1;
  }

  move_servo(Torso, pos_Array, N);
  
}

void loop() {

}
