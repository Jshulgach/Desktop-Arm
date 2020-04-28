/*
 * This sketch controls the desktop arm using rosserial to communicate
 * between roscore and the arduino
 * 
 * Last Updated:
 * 4/25/2020
 * Jonathan Shulgach
 */

/*
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

int melody[] = {
  
  // Mii Channel theme 
  // Score available at https://musescore.com/user/16403456/scores/4984153
  // Uploaded by Catalina Andrade 
  
  NOTE_FS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //1
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, REST,4, REST,8, NOTE_CS4,8,
  NOTE_D4,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,-4, NOTE_DS5,8, NOTE_D5,8, REST,8, REST,4,
  
  NOTE_GS4,8, REST,8, NOTE_CS5,8, NOTE_FS4,8, REST,8,NOTE_CS5,8, REST,8, NOTE_GS4,8, //5
  REST,8, NOTE_CS5,8, NOTE_G4,8, NOTE_FS4,8, REST,8, NOTE_E4,8, REST,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,8, REST,4, NOTE_E4,8, NOTE_E4,8,
  NOTE_E4,8, REST,8, REST,4, NOTE_DS4,8, NOTE_D4,8, 

  NOTE_CS4,8, REST,8, NOTE_A4,8, NOTE_CS5,8, REST,8,NOTE_A4,8, REST,8, NOTE_FS4,8, //9
  NOTE_D4,8, NOTE_D4,8, NOTE_D4,8, REST,8, NOTE_E5,8, NOTE_E5,8, NOTE_E5,8, REST,8,
  REST,8, NOTE_FS4,8, NOTE_A4,8, NOTE_CS5,8, REST,8, NOTE_A4,8, REST,8, NOTE_F4,8,
  NOTE_E5,2, NOTE_D5,8, REST,8, REST,4,

  NOTE_B4,8, NOTE_G4,8, NOTE_D4,8, NOTE_CS4,4, NOTE_B4,8, NOTE_G4,8, NOTE_CS4,8, //13
  NOTE_A4,8, NOTE_FS4,8, NOTE_C4,8, NOTE_B3,4, NOTE_F4,8, NOTE_D4,8, NOTE_B3,8,
  NOTE_E4,8, NOTE_E4,8, NOTE_E4,8, REST,4, REST,4, NOTE_AS4,4,
  NOTE_CS5,8, NOTE_D5,8, NOTE_FS5,8, NOTE_A5,8, REST,8, REST,4, 

  REST,2, NOTE_A3,4, NOTE_AS3,4, //17 
  NOTE_A3,-4, NOTE_A3,8, NOTE_A3,2,
  REST,4, NOTE_A3,8, NOTE_AS3,8, NOTE_A3,8, NOTE_F4,4, NOTE_C4,8,
  NOTE_A3,-4, NOTE_A3,8, NOTE_A3,2,

  REST,2, NOTE_B3,4, NOTE_C4,4, //21
  NOTE_CS4,-4, NOTE_C4,8, NOTE_CS4,2,
  REST,4, NOTE_CS4,8, NOTE_C4,8, NOTE_CS4,8, NOTE_GS4,4, NOTE_DS4,8,
  NOTE_CS4,-4, NOTE_DS4,8, NOTE_B3,1,
  
  NOTE_E4,4, NOTE_E4,4, NOTE_E4,4, REST,8,//25

};
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
int redPin = 9;
int greenPin = 10;
int bluePin = 11;
#define COMMON_ANODE //uncomment this line if using a Common Anode LED
int brightness = 50; // from 0(off) to 255(bright)

// Timer parameters
long LEDpreviousMillis = 0;        // will store last time LED was updated
long LED_interval = 500;           // interval at which to blink (milliseconds)
int ledState = LOW;

/*
//buzzer settings
int tempo = 114;// change this to make the song slower or faster
int buzzer = 5;// change this to whichever pin you want to use
// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
// there are two values per note (pitch and duration), so for each note there are four bytes
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;// this calculates the duration of a whole note in ms
int divider = 0, noteDuration = 0;
*/

unsigned long CONNECT_arduino_ping = 0;
unsigned long CONNECT_raspi_ping = 0;

int CONNECT_timeout = 8000; // milliseconds
int raspi_connected_state = false;
int arduino_connected_state = true;

ros::NodeHandle  nh;
std_msgs::UInt16  arduino_msg;

// Servo objects
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


void raspi_connect_cb(  const std_msgs::UInt16& cmd_msg){

  CONNECT_raspi_ping = millis();
  raspi_connected_state = true;
  
  int raspi_state = cmd_msg.data;
  if (raspi_state == 2){
    // Play mii_song
    //play_mii_song();  
  }

}

/*
void play_mii_song(){
  
   for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, melody[thisNote], noteDuration*0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
    
    // stop the waveform generation before the next note.
    noTone(buzzer);
  }
}
*/

ros::Subscriber<std_msgs::UInt16> torso_sub("torso_servo", torso_cb);
ros::Subscriber<std_msgs::UInt16> shoulder_sub("shoulder_servo", shoulder_cb);
ros::Subscriber<std_msgs::UInt16> elbow_sub("elbow_servo", elbow_cb);
ros::Subscriber<std_msgs::UInt16> wristrot_sub("wristrot_servo", wristrot_cb);
ros::Subscriber<std_msgs::UInt16> wristbend_sub("wristbend_servo", wristbend_cb);
ros::Subscriber<std_msgs::UInt16> claw_sub("claw_servo", claw_cb);
ros::Subscriber<std_msgs::UInt16> raspi_connect_sub("raspi_connect", raspi_connect_cb);
ros::Publisher arduino_connect_pub("arduino_connect", &arduino_msg);

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = round((255 - red));
    green = round((255 - green));
    blue = round((255 - blue));
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

void setup(){
  
  Serial.begin(57600);
  //Serial.println("setup");
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT); 

  //setup ROS handles
  nh.initNode();
  nh.subscribe(torso_sub);
  nh.subscribe(shoulder_sub);
  nh.subscribe(elbow_sub);
  nh.subscribe(wristrot_sub);
  nh.subscribe(wristbend_sub);
  nh.subscribe(claw_sub);
  nh.subscribe(raspi_connect_sub);
  nh.advertise(arduino_connect_pub);

  //setup robot servo joints
  Torso.attach(2);
  Shoulder.attach(3);
  Elbow.attach(4);
  WristRot.attach(7);
  WristBend.attach(8);
  Claw.attach(12);  
  
  //Friendly "ON" color
  setColor(0,0,255); //blue
  
  //Send message to notify Arduino is on
  //char arduino_msg[11] = "connected";
  //str_msg.data = arduino_msg;
  CONNECT_arduino_ping = millis();
  CONNECT_raspi_ping = millis();
  arduino_msg.data = 1;
  arduino_connect_pub.publish( &arduino_msg );
  delay(3000);
}

void loop(){  
  
  CONNECT_arduino_ping = millis();

  if (raspi_connected_state==true){  
    // Once connected, 
    setColor(80,0,80);  // purple
    
    // Check for connection loss
    int diff = abs(CONNECT_arduino_ping - CONNECT_raspi_ping);    
    if (diff > CONNECT_timeout){
      raspi_connected_state=false;
    }
    
    // (Later...) Connection to UI
    //setColor(0,255,0);  //lime green
    
    // Do other stuff...
    //....
    //....
    
    
  } else if (raspi_connected_state==false){ 
    
    // Waiting for connection to Raspberry Pi
    while (raspi_connected_state==false){

      unsigned long LEDcurrentMillis = millis();
      if (LEDcurrentMillis - LEDpreviousMillis > LED_interval) {
        LEDpreviousMillis = LEDcurrentMillis;
        if (ledState == LOW){
          ledState = HIGH;
          setColor(255,0,0);  // red
        } else {
          ledState = LOW;
          setColor(0,0,0);  // off
        }
      }
      nh.spinOnce();
      delay(1);
    }    
  }
  nh.spinOnce();
  delay(1);
}

