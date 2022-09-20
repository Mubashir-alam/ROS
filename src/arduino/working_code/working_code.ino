
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>


ros::NodeHandle  nh;


void messageCb( const std_msgs::String& toggle_msg)
{
  

nh.loginfo(toggle_msg.data);



// switch(toggle_msg.data[0])
// {
//   case 'M':
//   nh.loginfo("Recived M-condtion ");  
//   digitalWrite(13, HIGH);   
//   delay(2000);                      
//   digitalWrite(13, LOW);    
//   delay(2000);
//   break;
  
//   case 'R':
//   nh.loginfo("Recived M-condtion ");  
//   digitalWrite(12, HIGH);   
//   delay(2000);                      
//   digitalWrite(12, LOW);    
//   delay(2000);
//   break;
  
//   case 's':
//   nh.loginfo("Recived M-condtion ");  
//   digitalWrite(12, HIGH);   
//   delay(2000);                      
//   digitalWrite(12, LOW);    
//   delay(2000);
//   break;
  
//   }
  
//switch(toggle_msg.data[0])
//{
//  case 'R':
//  break;
//  }


if (toggle_msg.data[0] == 'M')
{  
 nh.loginfo("Recived M if-statment ");  
 digitalWrite(13, HIGH);   
 delay(2000);                      
 digitalWrite(13, LOW);    
 delay(2000);
}

else if(toggle_msg.data[0] == 'R')
{  
 nh.loginfo("Recived R if-statment ");  
 digitalWrite(12, HIGH);   
 delay(2000);                      
 digitalWrite(12, LOW);    
 delay(2000);
}

else if (toggle_msg.data[0] == 's')
{  
 nh.loginfo("Recived s if-statment ");  
 digitalWrite(11, HIGH);   
 delay(2000);                      
 digitalWrite(11, LOW);    
 delay(2000);
}


}


ros::Subscriber<std_msgs::String> sub("talker_vision", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
//  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.getHardware()->setBaud(9600);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
