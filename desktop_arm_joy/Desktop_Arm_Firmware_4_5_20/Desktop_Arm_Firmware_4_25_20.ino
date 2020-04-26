/*
 * This sketch controls the desktop arm using rosserial to communicate
 * between roscore and the arduino
 * 
 * Last Updated:
 * 4/5/2020
 * Jonathan Shulgach
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>

// RGB LED pins
int redPin = 10;
int greenPin = 11;
int bluePin = 12;
#define COMMON_ANODE //uncomment this line if using a Common Anode LED
int brightness = 5 // from 0(off) to 10(bright)

// Timer parameters
long previousMillis = 0;        // will store last time LED was updated
long interval = 1000;           // interval at which to blink (milliseconds)

int raspi_connected_state = false;


ros::NodeHandle  nh;
std_msgs::String str_msg;

Servo Torso;
Servo Shoulder;
Servo Elbow;
Servo WristRot;
Servo WristBend;
Servo Claw;

void torso_cb( const std_msgs::UInt16& cmd_msg){
  Torso.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void shoulder_cb( const std_msgs::UInt16& cmd_msg){
  Shoulder.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void elbow_cb( const std_msgs::UInt16& cmd_msg){
  Elbow.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void wristrot_cb( const std_msgs::UInt16& cmd_msg){
  WristRot.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void wristbend_cb( const std_msgs::UInt16& cmd_msg){
  WristBend.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void claw_cb( const std_msgs::UInt16& cmd_msg){
  Claw.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void raspi_connect_cb( const std_msgs::String& str_msg){
  if (str_msg=="disconnected"){
    raspi_connected_state = false;
  else if (str_msg=="connected"){
    raspi_connected_state = true;
  }
}

ros::Subscriber<std_msgs::UInt16> torso_sub("torso_servo", torso_cb);
ros::Subscriber<std_msgs::UInt16> shoulder_sub("shoulder_servo", shoulder_cb);
ros::Subscriber<std_msgs::UInt16> elbow_sub("elbow_servo", elbow_cb);
ros::Subscriber<std_msgs::UInt16> wristrot_sub("wristrot_servo", wristrot_cb);
ros::Subscriber<std_msgs::UInt16> wristbend_sub("wristbend_servo", wristbend_cb);
ros::Subscriber<std_msgs::UInt16> claw_sub("claw_servo", claw_cb);
ros::Subscriber<std_msgs::UInt16> raspi_connect_sub("raspi_connect", raspi_connect_cb);
ros::Publisher arduino_connect_pub("arduino_connect", &str_msg);

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = round(255 - red/(10-brightness));
    green = round(255 - green/(10-brightness));
    blue = round(255 - blue/(10-brightness));
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

void setup(){
  //initialize pins
  pinMode(13, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 

  //Friendly "ON" color
  setColor(65,100,255); //blue

  //setup ROS handles
  nh.initNode();
  nh.subscribe(torso_sub);
  nh.subscribe(shoulder_sub);
  nh.subscribe(elbow_sub);
  nh.subscribe(wristrot_sub);
  nh.subscribe(wristbend_sub);
  nh.subscribe(claw_sub);
  nh.subscribe(raspi_connect);
  nh.advertise(arduino_connect);

  //setup robot servo joints
  Torso.attach(2);
  Shoulder.attach(3);
  Elbow.attach(4);
  WristRot.attach(5);
  WristBend.attach(6);
  Claw.attach(7);

  //Send message to notify Arduino is on
  char arduino_msg[11] = "connected";
  str_msg.data = arduino_state;
  arduino_connect_pub.publish( &str_msg );
  delay(5000);
}

void loop(){

// Waiting for connection to Raspberry Pi
  while (raspi_connected_state==false)

    unsigned long currentMillis = millis();

    if(currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
      if (ledState == LOW)
        ledState = HIGH;
        setColor(255,0,0);  // red
      else
        ledState = LOW;
        setColor(0,0,0);  // red
      }
    }
  }

  // Once connected, 
  setColor(0,255,0);  // lime green
  nh.spinOnce();
  delay(1);

}
