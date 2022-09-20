/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

std_msgs::String msg;


void messageCb_M( const std_msgs::String & toggle_msg)
{

if (msg.data == "M")
{
  Serial.println("arduino M")
 digitalWrite(8, HIGH-digitalRead(8));   // blink the led
 delay(2000);
 digitalWrite(8,LOW-digitalRead(8));

}

if (msg.data == "R")
{
Serial.println("arduino R")
digitalWrite(9, HIGH-digitalRead(9));   // blink the led
delay(2000);
digitalWrite(9,LOW-digitalRead(9));
}

if (msg.data == "s")
{ Serial.println("arduino s")
  digitalWrite(10, HIGH-digitalRead(10));   // blink the led
  delay(2000);
  digitalWrite(10,LOW-digitalRead(10)); 
}

}

// void messageCb( const std_msgs::Empty& toggle_msg)
// {
//   digitalWrite(13, HIGH-digitalRead(13));   // blink the led
// }

// ros::Subscriber<std_msgs::Empty> sub("toggle_led", messageCb );

//callback for M
// void messageCb_M( const std_msgs::String & talker_vision)
// {

//   digitalWrite(8, HIGH-digitalRead(8));   // blink the led
// }

// //callback for R
// void messageCb_R( const std_msgs::String & talker_vision)
// {
//   digitalWrite(9, HIGH-digitalRead(9));   // blink the led
// }

// //callback for s
// void messageCb_s( const std_msgs::String & talker_vision)
// {
//   digitalWrite(10, HIGH-digitalRead(10));   // blink the led
// }



ros::Subscriber<std_msgs::String>sub_M("talker_vision", &messageCb_M );
// ros::Subscriber<std_msgs::String>sub_R("talker_vision", &messageCb_R );
// ros::Subscriber<std_msgs::String>sub_s("talker_vision", &messageCb_s );



// // for M
// if (std_msgs/String =="M") 
// {
// ros::Subscriber<std_msgs::String> sub("talker_vision", messageCb );
// }

// // for R 
// if (std_msgs/String == "R") 
// {
// ros::Subscriber<std_msgs::String> sub("talker_vision", messageCb );
// }

// // for S
// if (std_msgs/String =="s") 
// {
// ros::Subscriber<std_msgs::String> sub("talker_vision", messageCb );
// }





//publisher 
//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//
//char hello[13] = "hello world!";

void setup()
{
//  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  nh.initNode();
  //nh.advertise(chatter);
//  nh.subscribe(sub_M,sub_R,sub_s);
  
  nh.subscribe(sub_M);
  Serial.begin(9600)
  // nh.subscribe(sub_R);
  // nh.subscribe(sub_s);


}


void loop()
{

// if (msg.data == "M")
// {
//   // ros::Subscriber<std_msgs::String >sub_M("talker_vision", &messageCb_M );
//   // nh.subscribe(sub_M);

//   }

// if (msg.data == "R")
// {
//   // ros::Subscriber<std_msgs::String >sub_R("talker_vision", &messageCb_R );
//   // nh.subscribe(sub_R);


//   }
// if (msg.data == "s")
// {
//   // ros::Subscriber<std_msgs::String >sub_s("talker_vision", &messageCb_s );
//   // nh.subscribe(sub_s);
//   }
  
 // str_msg.data = hello;
  //chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
