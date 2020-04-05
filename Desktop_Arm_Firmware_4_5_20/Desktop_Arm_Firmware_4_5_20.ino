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

ros::NodeHandle  nh;

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

ros::Subscriber<std_msgs::UInt16> torso_sub("torso_servo", torso_cb);
ros::Subscriber<std_msgs::UInt16> shoulder_sub("shoulder_servo", shoulder_cb);
ros::Subscriber<std_msgs::UInt16> elbow_sub("elbow_servo", elbow_cb);
ros::Subscriber<std_msgs::UInt16> wristrot_sub("wristrot_servo", wristrot_cb);
ros::Subscriber<std_msgs::UInt16> wristbend_sub("wristbend_servo", wristbend_cb);
ros::Subscriber<std_msgs::UInt16> claw_sub("claw_servo", claw_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(torso_sub);
  nh.subscribe(shoulder_sub);
  nh.subscribe(elbow_sub);
  nh.subscribe(wristrot_sub);
  nh.subscribe(wristbend_sub);
  nh.subscribe(claw_sub);


  Torso.attach(2);
  Shoulder.attach(3);
  Elbow.attach(4);
  WristRot.attach(5);
  WristBend.attach(6);
  Claw.attach(7);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
