
//verison3

#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <Dabble.h>//
#include <RoboClaw.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <MechaQMC5883.h>
MechaQMC5883 qmc;

// Roboclaw configuration
#define address_lft 0x80
#define address_rght 0x81

long int ref_left_pps;
long int ref_rght_pps;

long int enc_lft, enc_rght, rght_pps, lft_pps;
long int LS = 0, RS = 0, S = 0, cc = 0, x = 0;
char PRESS;

bool AUTO_ = LOW, INTEGRATE = LOW;

//Velocity PID coefficients.
#define Kp 0.25
#define Ki 0.8
#define Kd 0.35
#define qpps 45000

#define PPS_MAX 2000
#define ANGULAR_MAX 225.0
#define kh 1.2
float ANGULAR = 0;
int ANGLE = 90, last_ANGLE = 90, e = 0; 
byte data_ = 0;
long int PPS = 0;


#define LM 9
#define L 8
#define M 10
#define R 11
#define RM 12
//#define BOOM_COUNT 3300 //5300  //3560 //7120
#define BOOM_COUNT -5
#define SPRAY_TIME 500
//#define SPRAY_TIME 10000


bool LM_STATE[5] = {LOW, LOW, LOW, LOW, LOW};
bool L_STATE[5] = {LOW, LOW, LOW, LOW, LOW};
bool M_STATE[5] = {LOW, LOW, LOW, LOW, LOW};
bool R_STATE[5] = {LOW, LOW, LOW, LOW, LOW};
bool RM_STATE[5] = {LOW, LOW, LOW, LOW, LOW};

long int LM_POS[5] = {0, 0, 0, 0, 0};
long int L_POS[5] = {0, 0, 0, 0, 0};
long int M_POS[5] = {0, 0, 0, 0, 0};
long int R_POS[5] = {0, 0, 0, 0, 0};
long int RM_POS[5] = {0, 0, 0, 0, 0};

long int dLM[5] = {0, 0, 0, 0, 0};
long int dL[5] = {0, 0, 0, 0, 0};
long int dM[5] = {0, 0, 0, 0, 0};
long int dR[5] = {0, 0, 0, 0, 0};
long int dRM[5] = {0, 0, 0, 0, 0};

long int pre_LM[5] = {0, 0, 0, 0, 0};
long int pre_L[5] = {0, 0, 0, 0, 0};
long int pre_M[5] = {0, 0, 0, 0, 0};
long int pre_R[5] = {0, 0, 0, 0, 0};
long int pre_RM[5] = {0, 0, 0, 0, 0};
float a[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // filter
//char toggle_msg;
RoboClaw roboclaw(&Serial2, 10000); // timeout and header file format

//serial::Serial ros_ser;


ros::NodeHandle nh;
//String b;
char buf[50];
//char Byte = buf[50];



void messageCb( const std_msgs::String & toggle_msg)
{
  String a = toggle_msg.data;  //a is to read incoming string data as a char
  nh.loginfo(toggle_msg.data);
a.toCharArray(buf, 50);
nh.loginfo("This before B[0]");
nh.loginfo(buf);
nh.loginfo("This after B[0]");

  read_grid();
  spray_noz();
  // nh.loginfo(enc_lft);
}

ros::Subscriber<std_msgs::String> sub("talker_vision", &messageCb );


//----------------------------------------------------------------
void setup() {
  pinMode(RM, OUTPUT); //right most
  pinMode(R, OUTPUT); //right
  pinMode(M, OUTPUT); //middle
  pinMode(L, OUTPUT); //left
  pinMode(LM, OUTPUT); //left most

   roboclaw.begin(38400);
   Serial.begin(38400);
   Serial1.begin(38400);
   Dabble.begin(9600);

  digitalWrite(RM, LOW);
  digitalWrite(R, LOW);
  digitalWrite(M, LOW);
  digitalWrite(L, LOW);
  digitalWrite(LM, LOW);

  Wire.begin();
  qmc.init();

  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address_lft, Kd, Kp, Ki, qpps);
  roboclaw.SetM1VelocityPID(address_rght, Kd, Kp, Ki, qpps);
  roboclaw.SetEncM1(address_rght,0);
  roboclaw.SetEncM1(address_lft,0);

  nh.initNode();
  nh.subscribe(sub);
}
//-----------------------------------------------------------------
void states() {
  uint8_t status_pos, status_vel;
  bool valid_pos, valid_vel;
  int32_t pulses = roboclaw.ReadEncM1(address_rght, &status_pos, &valid_pos);
  int32_t pps = roboclaw.ReadSpeedM1(address_rght, &status_vel, &valid_vel);

  if (valid_pos)
    enc_rght = pulses;

  if (valid_vel)
    rght_pps = pps;
//  nh.loginfo("states=valid_vel_right");
  //    rght_lvel= (2*Pi*r*rght_pps)/4000.0; // right wheel velocity in cm/s

  pulses = roboclaw.ReadEncM1(address_lft, &status_pos, &valid_pos);
  pps = roboclaw.ReadSpeedM1(address_lft, &status_vel, &valid_vel);
  if (valid_pos)
    enc_lft = pulses;
//  nh.loginfo("states=valid_pos_left");
  if (valid_vel)
    lft_pps = pps;
//  nh.loginfo("states=valid_vel_left");
  //    lft_lvel= (2*Pi*r*lft_pps)/4000.0; // left wheel velocity in cm/s


}
//----------------------------------------------------------------
void read_grid() {

  if (buf>0)    //compersion number of char
  {

    for (int c = 0; c < 5; c++)
    { 
//      nh.loginfo("inside for loop");
      // five plants register
      if (buf[0] == 'K') {
        if (LM_STATE[c] == LOW) {
          LM_STATE[c] = HIGH;
          LM_POS[c] = (enc_rght + enc_lft) / 2;
          break;
        }
      } else if (buf[0] == 'L') { //L
        if (L_STATE[c] == LOW) {
          L_STATE[c] = HIGH;
          L_POS[c] = (enc_rght + enc_lft) / 2;
          break;
        }
      } else if (buf[0]== 'M') {
         nh.loginfo("True buf[0]= M");
        if (M_STATE[c] == LOW) {
          M_STATE[c] = HIGH;
          M_POS[c] = (enc_rght + enc_lft) / 2;
          nh.loginfo("M_STATE[c] == LOW)");
//        nh.loginfo(M_POS[c]);         //giving empty value 

          break;
        }
      } else if (buf[0] == 'R') {
        if (R_STATE[c] == LOW) {
          R_STATE[c] = HIGH;
          R_POS[c] = (enc_rght + enc_lft) / 2;
          // nh.loginfo("R byte recived");

          break;
        }
      } else if (buf[0] == 'S') {
        if (RM_STATE[c] == LOW) {
          RM_STATE[c] = HIGH;
          RM_POS[c] = (enc_rght + enc_lft) / 2;
          // nh.loginfo("S byte recived");
          break;
        }
      }

    }//for loop
  }
}
//-----------------------------------------------------------
void spray_noz() {
  long int robo_count = (enc_rght + enc_lft) / 2;
  for (int c = 0; c < 5; c++) {

    if (LM_STATE[c]) {
      dLM[c] = robo_count - LM_POS[c];
      if (dLM[c] >= BOOM_COUNT){
        noz_LM(pre_LM[c], c);
      }
      else
        pre_LM[c] = millis();
    }

    if (L_STATE[c]) {
      dL[c] = robo_count - L_POS[c];
      if (dL[c] >= BOOM_COUNT){
        noz_L[c](pre_L[c], c);
      }
      else
        pre_L[c] = millis();
    }

    if (M_STATE[c]) {
       nh.loginfo("M_state[c]");
      dM[c] = robo_count - M_POS[c];
      if (dM[c] >= BOOM_COUNT){
        nh.loginfo("dM[c] >= BOOM_COUNT");
        noz_M(pre_M[c], c);
        }
      else
        pre_M[c] = millis();
    }
    if (R_STATE[c]) {
      dR[c] = robo_count - R_POS[c];
      if (dR[c] >= BOOM_COUNT){
        noz_R(pre_R[c], c);
      }
      else
        pre_R[c] = millis();
    }
    if (RM_STATE[c]) {
      dRM[c] = robo_count - RM_POS[c];
      if (dRM[c] >= BOOM_COUNT){
        noz_RM(pre_RM[c], c);
      }
      else
        pre_RM[c] = millis();
    }

  }// for loop
}
//-------------------------------------------------------------------------
void noz_LM(long int SET_t, int n) {
  digitalWrite(LM, HIGH);
  if ((millis() - SET_t) >= SPRAY_TIME) {
    delay(1000);
    digitalWrite(LM, LOW);
    LM_STATE[n] = LOW;
    LM_POS[n] = 0;
    dLM[n] = 0;
  }
}

void noz_L(long int SET_t, int n) {
  digitalWrite(L, HIGH);
  if ((millis() - SET_t) >= SPRAY_TIME) {
    delay(1000);
    digitalWrite(L, LOW);
    L_STATE[n] = LOW;
    L_POS[n] = 0;
    dL[n] = 0;
  }
}
void noz_M(long int SET_t, int n) {
   nh.loginfo("M,vale is ON");
  digitalWrite(M, HIGH);
  if ((millis() - SET_t) > SPRAY_TIME) {
    delay(1000);
    digitalWrite(M, LOW);
//    nh.loginfo((millis() - SET_t));
    nh.loginfo("M,vale is OFF");
    M_STATE[n] = LOW;
    M_POS[n] = 0;
    dM[n] = 0;
  }
}
void noz_R(long int SET_t, int n) {
  digitalWrite(R, HIGH);
  if ((millis() - SET_t) > SPRAY_TIME) {
    delay(1000);
    digitalWrite(R, LOW);
    R_STATE[n] = LOW;
    R_POS[n] = 0;
    dR[n] = 0;
  }
}
void noz_RM(long int SET_t, int n) {
  digitalWrite(RM, HIGH);
  if ((millis() - SET_t) > SPRAY_TIME) {
    delay(1000);
    digitalWrite(RM, LOW);
    RM_STATE[n] = LOW;
    RM_POS[n] = 0;
    dRM[n] = 0;
  }
}

// -----------------------------------------
void dabble_control(){
 Dabble.processInput();
  long int a = GamePad.getAngle();
//  long int b = GamePad.getRadius();
  long int speed_ = 1000;
  long int delta_speed = 1000;
//  if(b == 0 && S>0){S-=1;}else {S = 18.14*b; S = filter(S);}
//  S = 18.14*b;
//  S = filter(S);

  if (GamePad.isCrossPressed()){
     AUTO_ = LOW;
     roboclaw.SpeedM1(address_rght,0);
     roboclaw.SpeedM1(address_lft,0);
     }else
  if (GamePad.isTrianglePressed()){
     AUTO_ = HIGH;
     }else
  if(GamePad.isSquarePressed())
    broadcast(HIGH);
    else
  if(GamePad.isCirclePressed())
    broadcast(LOW);


if(AUTO_ == HIGH){
  if(a>0 && a<=90)
     cc+=10;
     else
  if(a>90 && a<=180)
     cc-=10;
  if(cc> delta_speed)
     cc = delta_speed;
  if(cc< -delta_speed)
     cc = -delta_speed;
  if(a==0)
     cc=0;

  e = 90-ANGLE;
  if(ANGLE<150 && ANGLE>30){
    last_ANGLE = ANGLE;
     ANGULAR = kh*e;// adding integral controller
     PPS = ANGULAR*(PPS_MAX/ANGULAR_MAX);

     ref_left_pps = speed_+cc+PPS;
     ref_rght_pps = speed_-cc-PPS;
     
     roboclaw.SpeedM1(address_rght,ref_rght_pps);
     roboclaw.SpeedM1(address_lft,ref_left_pps);
//roboclaw.SpeedAccelM1(address_lft,1000,2000);
    }else 
     transition();
  }else
   if(AUTO_ == LOW)
  {
  if (GamePad.isUpPressed())
  {PRESS = 'U';
  if(x < 100)
  x = x + 3;
  roboclaw.ForwardM1(address_rght,x);
  roboclaw.ForwardM1(address_lft,x);
   }else
  if (GamePad.isDownPressed())
  {PRESS = 'D';
  if(x < 100)
  x = x + 3;
  roboclaw.BackwardM1(address_rght,x);
  roboclaw.BackwardM1(address_lft,x);
  }else
  if (GamePad.isLeftPressed())
  {PRESS = 'L';
  if(x < 100)
  x = x + 3;
  roboclaw.ForwardM1(address_rght,x);
  roboclaw.BackwardM1(address_lft,x);
  }else
  if (GamePad.isRightPressed())
  {PRESS = 'R';
  if(x < 100)
  x = x + 3;
  roboclaw.BackwardM1(address_rght,x);
  roboclaw.ForwardM1(address_lft,x);
  }else
  {  
if(x > 0)
x = x - 5;
else
x=0;

if(PRESS == 'U')
{
  roboclaw.ForwardM1(address_rght,x);
  roboclaw.ForwardM1(address_lft,x);
  }else
  if(PRESS == 'D'){
  roboclaw.BackwardM1(address_rght,x);
  roboclaw.BackwardM1(address_lft,x);
    }else
  if(PRESS == 'L'){
  roboclaw.ForwardM1(address_rght,x);
  roboclaw.BackwardM1(address_lft,x);
  }else
  if(PRESS == 'R'){
  roboclaw.BackwardM1(address_rght,x);
  roboclaw.ForwardM1(address_lft,x);
   }
  }//else
 }
}// dabble

//-----------------------------------------------------------
void display_info(){
  Serial.print(millis());
  Serial.print("   ");
  Serial.print(enc_rght);
  Serial.print("   ");
  Serial.print(enc_lft);
  Serial.print("   ");
  Serial.print(rght_pps);
  Serial.print("   ");
  Serial.print(lft_pps);
  Serial.print("   ");
  Serial.println(ANGLE);
  }
//-----------------------------------------------------------

void transition(){
  roboclaw.SpeedM1(address_rght,0);
  roboclaw.SpeedM1(address_lft,0);
  delay(1000);

  
//int cur = millis();
//int pre = cur;
while(ANGLE>150 || ANGLE<30){
  read_grid();
if(ANGLE>last_ANGLE-10){
  roboclaw.SpeedM1(address_rght,-1000);
  roboclaw.SpeedM1(address_lft,1000);
}else
if(ANGLE<last_ANGLE+10){
  roboclaw.SpeedM1(address_rght,1000);
  roboclaw.SpeedM1(address_lft,-1000);
  }
 // Serial.println(ANGLE);
 }
  roboclaw.SpeedM1(address_rght,0);
  roboclaw.SpeedM1(address_lft,0);
  delay(1000);
}
//-----------------------------------------------------------
void broadcast(bool SPRAY_STATE){
  if(SPRAY_STATE == HIGH){
  digitalWrite(RM, HIGH);
  digitalWrite(R, HIGH);
  digitalWrite(M, HIGH);
  digitalWrite(L, HIGH);
  digitalWrite(LM, HIGH);
  }else
  if(SPRAY_STATE == LOW){
  digitalWrite(RM, LOW);
  digitalWrite(R, LOW);
  digitalWrite(M, LOW);
  digitalWrite(L, LOW);
  digitalWrite(LM, LOW);
  }  
}

//---------------------------------------------------------



void loop() {
  // put your main code here, to run repeatedly:
  //  read_grid();
  dabble_control();
  states();
  nh.spinOnce();
   delay(1);
}
