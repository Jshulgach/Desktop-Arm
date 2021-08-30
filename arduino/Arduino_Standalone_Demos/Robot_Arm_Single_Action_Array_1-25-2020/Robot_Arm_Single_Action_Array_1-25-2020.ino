/*
Robot Arm Single Action

Moves a single servo (Torso) from 0 to 180 degrees in 2 seconds. Positions written to servo are at rate of 100Hz

Modified 25th January 2020
by Jonathan Shulgach

*/

#include <Servo.h>
Servo Torso;

// Initialize constants
int startTime = 0;
int endTime = 1;        // seconds
int startPos = 0;      // Starting servo position (degrees)
int endPos = 90;      // Ending servo position (degrees)
int frequency = 10;   // Determines the number of position values written to servo per second (Hz)

void move_servo(Servo &myservo, int move_Array[], int N, int frequency){
  int previousTime = 0;
  for (int j = 0; j < N; j += 1){
    while (( millis() - previousTime) < 1000/frequency){
      myservo.write(move_Array[j]);
      previousTime = millis();
    }
  }
}

void setup() {
  // Initialize servo and baud rate
  Torso.attach(2);
  Serial.begin(9600);
  
  // Collect information to create the position array
  int delta_pos = abs(endPos-startPos);
  int delta_t = abs(endTime-startTime);
  int N_incr = round(delta_t*frequency); // Finds microseconds between prior and latter position increments to write to servo
  
  // Build and fill position array for writing to servo
  int pos_Array[delta_t*frequency]={};
  for (int j=1; j <= N_incr; j+=1){
    pos_Array[j] = startPos + (j-1)*(endPos-startPos)/(N_incr-1);
    Serial.print(pos_Array[j]); Serial.print(" ");
  }
  move_servo(Torso, pos_Array, N_incr, frequency);
}

void loop() {

}
