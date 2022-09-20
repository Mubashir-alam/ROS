
//ros
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
//#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
ros::NodeHandle nh;

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

//Encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

//The number of pulses produced by the encoder within a revolution.
const int PPR = 64;
//The value is '1' if the encoder is not attached to any motor.
const int gearRatio = 19;
//When using 2X encoding the value is '2'. In this code 4X encoding is used.
const int decodeNumber = 4;
//record the cuurent number of pulses received
volatile long int currentPosition = 0;

unsigned int timer_previous = 0;
double wheel_turns = 0.0;
double current_wheel_distance = 0.0;
double previous_wheel_distance = 0.0;
double time_taken = 0;
double wheel_speed_mpms = 0.0;
double wheel_speed_mpm = 0.0;
double pi = 3.14;
double r = 0.04775; // Radius of the wheel in meters
char result[128];   //current wheel disrtance
char result2[128]; //curent wheel velocity
char result_statment[128]="distance";
char result_statment2[128]="velocity";



//ros
void messageCb( const std_msgs::String& toggle_msg)
{
  
nh.loginfo(toggle_msg.data);
//dtostrf(current_wheel_distance,6,5, result);
//nh.loginfo(result);



if(toggle_msg.data[0] == 'M' && current_wheel_distance>=2.50)
{  
  nh.loginfo("Recived M if-statment ");  
  digitalWrite(13, HIGH);   
  delay(2000);                      
  digitalWrite(13, LOW);    
  delay(2000);  
  dtostrf(current_wheel_distance,6,5, result);
  nh.loginfo(result);
 // current_wheel_distance=0;   // to rest the postion 

  
}

if (toggle_msg.data[0] == 'R' && current_wheel_distance==2.50)
{  
   nh.loginfo("Recived R if-statment ");  
   digitalWrite(12, HIGH);   
   delay(2000);                      
   digitalWrite(12, LOW);    
   delay(2000);
//  nh.loginfo(result);
//  current_wheel_distance=0;   // to rest the postion 

}

if (toggle_msg.data[0] == 's' && current_wheel_distance==2.50)
{  

 nh.loginfo("Recived s if-statment ");  
   digitalWrite(11, HIGH);   
   delay(2000);                      
   digitalWrite(11, LOW);    
   delay(2000);
    dtostrf(current_wheel_distance,6,5, result);
//  nh.loginfo(result);
//  current_wheel_distance=0;   // to rest the postion 


}

}




ros::Subscriber<std_msgs::String> sub("talker_vision", &messageCb );




void setup() {

// ROS 
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
//  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rightPub);

//  encoder 
  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), doEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), doEncoderB, CHANGE);
//  Serial.begin (9600);


}


void loop() {
// ROS 
  nh.spinOnce();
  delay(1);

  if (current_wheel_distance - previous_wheel_distance > time_taken) 
 {
     
    previous_wheel_distance = current_wheel_distance;
     
    rightPub.publish( &right_wheel_tick_count );
    // leftPub.publish( &left_wheel_tick_count );
//    nh.spinOnce();
  }
  
// nh.loginfo(current_wheel_distance);
//  dtostrf(current_wheel_distance,6,5, result);
//  while(result)
//  {
// dtostrf(current_wheel_distance,6,5, result);
//  nh.loginfo(result);
//  }
//  encoder
  current_wheel_distance = (currentPosition * 2 * pi * r) / (PPR * gearRatio * decodeNumber) ; // Gives the distance which the wheel traveled. 'r' is the radius of the wheel in meters.
  time_taken = micros() - timer_previous; // Time taken since the previous iteration of the loop.
  wheel_speed_mpms = (current_wheel_distance-previous_wheel_distance) / time_taken; // This gives the speed in meters per microsecond.
  wheel_speed_mpm = wheel_speed_mpms * 1000000 * 60; // This gives the speed in meters per minute.

//  Serial.print(current_wheel_distance, 4);
//  Serial.print(" ");
//  Serial.print(wheel_speed_mpm, 4);
//  Serial.println();

  timer_previous = micros();
  previous_wheel_distance = current_wheel_distance;

//  ROS nh.loginfo for printing in terminal
   dtostrf(current_wheel_distance,6,5, result);
//   nh.logdebug("Debug Statement");
//   dtostrf(wheel_speed_mpm,6,5, result2);


// nh.loginfo("Program info" nh.loginfo(result));
// nh.loginfo(char::String["distance"] + result);
//  nh.loginfo(result2);
//String a = String(result);
//  Serial.print("this is a"+a);
//  Serial.println();

Serial.print(current_wheel_distance);
Serial.println();

//this if statment is check current_wheel_distance is giving us value or not
//if (current_wheel_distance>=2.50)
//{
//  Serial.print("2.50 rechead");
//  Serial.println();
//  
//  }

//Serial.print(result);
//Serial.println();
//nh.loginfo(result);
//nh.loginfo(result2);

//  nh.loginfo("angulardistance",result);


  nh.spinOnce();


  
}

void doEncoderA()
{
  if (digitalRead(encoderPinA) != digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}
void doEncoderB()
{
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB))
  {
    currentPosition++;
  }
  else
  {
    currentPosition--;
  }
}
