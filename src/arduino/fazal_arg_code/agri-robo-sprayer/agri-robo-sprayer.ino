// Robot Navigation and Localization using encoders and digital compass data
// Authur: Fazal Nasir Khan
//         Department of Mechatronics Engg. ARAL Lab, UET Peshawar
// Date:   April 25, 2022
// File Address: NCRA Research/Demos
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>//
#include <RoboClaw.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <MechaQMC5883.h>
MechaQMC5883 qmc;
// Roboclaw configuration
#define address_lft 0x80
#define address_rght 0x81

long int enc_lft, enc_rght, rght_pps, lft_pps;
long int LS = 0, RS = 0, S = 0, cc = 0, x = 0;
char PRESS;

bool AUTO_ = LOW, INTEGRATE = LOW;

//Velocity PID coefficients.
#define Kp 0.25
#define Ki 0.8
#define Kd 0.35
#define qpps 45000

#define LM 9
#define L 8
#define M 10
#define R 11
#define RM 12
#define BOOM_COUNT 5300
#define SPRAY_TIME 500

bool LM_STATE[5]={LOW,LOW,LOW,LOW,LOW};
bool L_STATE[5] ={LOW,LOW,LOW,LOW,LOW};
bool M_STATE[5] ={LOW,LOW,LOW,LOW,LOW};
bool R_STATE[5] ={LOW,LOW,LOW,LOW,LOW};
bool RM_STATE[5]={LOW,LOW,LOW,LOW,LOW};

long int LM_POS[5]={0,0,0,0,0};
long int L_POS[5] ={0,0,0,0,0};
long int M_POS[5] ={0,0,0,0,0};
long int R_POS[5] ={0,0,0,0,0};
long int RM_POS[5]={0,0,0,0,0};

long int dLM[5]={0,0,0,0,0};
long int dL[5] ={0,0,0,0,0};
long int dM[5] ={0,0,0,0,0};
long int dR[5] ={0,0,0,0,0};
long int dRM[5]={0,0,0,0,0};

long int pre_LM[5]={0,0,0,0,0};
long int pre_L[5] ={0,0,0,0,0};
long int pre_M[5] ={0,0,0,0,0};
long int pre_R[5] ={0,0,0,0,0};
long int pre_RM[5]={0,0,0,0,0};
float a[8]={0,0,0,0,0,0,0,0};// filter
RoboClaw roboclaw(&Serial2,10000);// timeout and header file format

void setup() {
  pinMode(RM,OUTPUT);//right most
  pinMode(R,OUTPUT);//right
  pinMode(M,OUTPUT);//middle
  pinMode(L,OUTPUT);//left
  pinMode(LM,OUTPUT);//left most
  
  roboclaw.begin(38400);
  Serial.begin(9600);
  Dabble.begin(9600);

  digitalWrite(RM, LOW);
  digitalWrite(R, LOW);
  digitalWrite(M, LOW);
  digitalWrite(L, LOW);
  digitalWrite(LM, LOW);

  Wire.begin();
  qmc.init();
  
  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address_lft,Kd,Kp,Ki,qpps);
  roboclaw.SetM1VelocityPID(address_rght,Kd,Kp,Ki,qpps);
}
//---------------------------------------------------------------
float filter(float c_p){
  a[7]=a[6];
  a[6]=a[5];
  a[5]=a[4];
  a[4]=a[3];
  a[3]=a[2];
  a[2]=a[1];
  a[1]=a[0];
  a[0]=c_p;
  return ((a[0]+a[1]+a[2]+a[3]+a[4]+a[5]+a[6]+a[7])/8);
}
//----------------------------------------------------------------------
void states(){
  uint8_t status_pos,status_vel;
  bool valid_pos,valid_vel;
  int32_t pulses = roboclaw.ReadEncM1(address_rght, &status_pos, &valid_pos);
  int32_t pps = roboclaw.ReadSpeedM1(address_rght, &status_vel, &valid_vel);
  
  if(valid_pos)
    enc_rght = pulses;
  if(valid_vel)
    rght_pps = pps;
//    rght_lvel= (2*Pi*r*rght_pps)/4000.0; // right wheel velocity in cm/s
    
    pulses= roboclaw.ReadEncM1(address_lft, &status_pos, &valid_pos);
    pps = roboclaw.ReadSpeedM1(address_lft, &status_vel, &valid_vel);
  if(valid_pos)
    enc_lft = pulses;
  if(valid_vel)
    lft_pps = pps;
//    lft_lvel= (2*Pi*r*lft_pps)/4000.0; // left wheel velocity in cm/s
 }
//---------------------------------------------------------------------
void read_grid(){
  if (Serial.available() > 0) {
     char Byte = Serial.read();
  for(int c=0; c<5; c++){// five plants register

  if (Byte == 'K'){
  if(LM_STATE[c]==LOW){
    LM_STATE[c]=HIGH;
    LM_POS[c]=(enc_rght + enc_lft)/2;
    break;
  }        
 }else   
     if (Byte == 'L'){//L
  if(L_STATE[c]==LOW){
    L_STATE[c]=HIGH;
    L_POS[c]=(enc_rght + enc_lft)/2;
    break;
  }        
 }else
     if (Byte == 'M'){
  if(M_STATE[c]==LOW){
    M_STATE[c]=HIGH;
    M_POS[c]=(enc_rght + enc_lft)/2;
    break;
  }
 }else
     if (Byte == 'R'){
  if(R_STATE[c]==LOW){
    R_STATE[c]=HIGH;
    R_POS[c]=(enc_rght + enc_lft)/2;
    break;
   }  
  }else
     if (Byte == 'S'){
  if(RM_STATE[c]==LOW){
    RM_STATE[c]=HIGH;
    RM_POS[c]=(enc_rght + enc_lft)/2;
    break;
    }  
   }
 
  }//for loop
 }
}
//-----------------------------------------------------------
void spray_noz(){
  long int robo_count = (enc_rght + enc_lft)/2;
for(int c=0; c<5; c++){

  if(LM_STATE[c]){
    dLM[c] = robo_count - LM_POS[c];
    if(dLM[c] >= BOOM_COUNT)
      noz_LM(pre_LM[c], c);
      else
      pre_LM[c] = millis();
     }
 
  if(L_STATE[c]){
    dL[c] = robo_count - L_POS[c];
    if(dL[c] >= BOOM_COUNT)
      noz_L[c](pre_L[c], c);
      else
      pre_L[c] = millis();
     }
     
  if(M_STATE[c]){
    dM[c] = robo_count - M_POS[c];
    if(dM[c] >= BOOM_COUNT)
      noz_M(pre_M[c], c);
      else
      pre_M[c] = millis();
    }
  if(R_STATE[c]){
    dR[c] = robo_count - R_POS[c];
    if(dR[c] >= BOOM_COUNT)
      noz_R(pre_R[c], c);
      else
      pre_R[c] = millis();
    }
  if(RM_STATE[c]){
    dRM[c] = robo_count - RM_POS[c];
    if(dRM[c] >= BOOM_COUNT)
      noz_RM(pre_RM[c], c);
      else
      pre_RM[c] = millis();
    }  
  
 }// for loop
}
//-----------------------------------------------------------
void noz_LM(long int SET_t, int n){
  digitalWrite(LM, HIGH);
if((millis()-SET_t) >= SPRAY_TIME){
  digitalWrite(LM, LOW);
  LM_STATE[n] = LOW;
  LM_POS[n] = 0;
  dLM[n] = 0;
  }
 }

void noz_L(long int SET_t, int n){
  digitalWrite(L, HIGH);
if((millis()-SET_t) >= SPRAY_TIME){
  digitalWrite(L, LOW);
  L_STATE[n] = LOW;
  L_POS[n] = 0;
  dL[n] = 0;
  }
 }
 void noz_M(long int SET_t, int n){
  digitalWrite(M, HIGH);
if((millis()-SET_t) > SPRAY_TIME){
  digitalWrite(M, LOW);
  M_STATE[n] = LOW;
  M_POS[n] = 0;
  dM[n] = 0;
  }
 }
 void noz_R(long int SET_t, int n){
  digitalWrite(R, HIGH);
if((millis()-SET_t) > SPRAY_TIME){
  digitalWrite(R, LOW);
  R_STATE[n] = LOW;
  R_POS[n] = 0;
  dR[n] = 0;
  }
 }
  void noz_RM(long int SET_t, int n){
  digitalWrite(RM, HIGH);
if((millis()-SET_t) > SPRAY_TIME){
  digitalWrite(RM, LOW);
  RM_STATE[n] = LOW;
  RM_POS[n] = 0;
  dRM[n] = 0;
  }
 }
//--------------------------------------------------------
void dabble_control(){
 Dabble.processInput();
  long int a = GamePad.getAngle();
  long int b = GamePad.getRadius();
  long int speed_ = 1000;
  long int delta_speed = 1000;
  if(b == 0 && S>0){S-=1;}else {S = 18.14*b; S = filter(S);}
//  S = 18.14*b;
//  S = filter(S);

  if (GamePad.isCrossPressed()){
     AUTO_ = LOW; //broadcast(SPRAY_ALL);
     transition();
     delay(500);
     }else
  if (GamePad.isTrianglePressed()){
     AUTO_ = HIGH; //broadcast(SPRAY_ALL);
     transition();
     delay(500);
     }else
  if(GamePad.isSquarePressed())
    broadcast(HIGH); //SPRAY_ALL = HIGH;
    else
  if(GamePad.isCirclePressed())
    broadcast(LOW); //SPRAY_ALL = LOW;
    else
  if(GamePad.isStartPressed())
    { transition();
      INTEGRATE = HIGH;
      delay(500);}
    else
  if (GamePad.isSelectPressed())
    { transition();
      INTEGRATE = LOW;
      delay(500);}


if(AUTO_ == HIGH){
  if(a>0 && a<=90 ){//&& (cc<=delta_speed && cc>=0)
        cc+=20;}else
  if(a>90 && a<=180){//&&(cc<=0 && cc>=-delta_speed)
    cc-=20;}
    if(cc> delta_speed)
      cc = delta_speed;
    if(cc< -delta_speed)
      cc = -delta_speed;
    if (a==0)
      cc=0;
roboclaw.SpeedM1(address_rght,speed_-cc);
roboclaw.SpeedM1(address_lft,speed_+cc);
 }else
if(INTEGRATE == HIGH)
{  
  if(a>0 && a<=90){
      LS = S;
      RS = (a*S/90);    
roboclaw.ForwardM1(address_rght,RS);//roboclaw.SpeedM1(address_rght,RS);
roboclaw.ForwardM1(address_lft,LS);//roboclaw.SpeedM1(address_lft,LS);
    }else
  if(a>90 && a<=180){
      LS = (180-a)*S/90;
      RS = S;    
roboclaw.ForwardM1(address_rght,RS);//roboclaw.SpeedM1(address_rght,RS);
roboclaw.ForwardM1(address_lft,LS);//roboclaw.SpeedM1(address_lft,LS);
    }else
 if(a>180 && a<=270){
      LS = 0;
      RS = S;  
roboclaw.ForwardM1(address_rght,RS);//roboclaw.SpeedM1(address_rght,RS);
roboclaw.BackwardM1(address_lft,LS);//roboclaw.SpeedM1(address_lft,LS);
    }else
  if(a>270 && a<=359){
      LS = S;
      RS = 0;//0;// 
roboclaw.BackwardM1(address_rght,RS);//roboclaw.SpeedM1(address_rght,RS);
roboclaw.ForwardM1(address_lft,LS);//roboclaw.SpeedM1(address_lft,LS);
    }else
{
transition();  
  }
   
  }else
if(INTEGRATE == LOW)
{
  if (GamePad.isUpPressed())
  {
  PRESS = 'U';
  if(x < 100)
  x = x + 2;
  roboclaw.ForwardM1(address_rght,x);
  roboclaw.ForwardM1(address_lft,x);
   }else
  if (GamePad.isDownPressed())
  {PRESS = 'D';
  if(x < 100)
  x = x + 2;
  roboclaw.BackwardM1(address_rght,x);
  roboclaw.BackwardM1(address_lft,x);
  }else
  if (GamePad.isLeftPressed())
  {PRESS = 'L';
  if(x < 100)
  x = x + 1;
  roboclaw.ForwardM1(address_rght,x);
  roboclaw.BackwardM1(address_lft,x);
  }else
  if (GamePad.isRightPressed())
  {PRESS = 'R';
  if(x < 100)
  x = x + 1;
  roboclaw.BackwardM1(address_rght,x);
  roboclaw.ForwardM1(address_lft,x);
  }else
  {  
if(x > 0)
x = x - 2;
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
  } 
 }
}

//-----------------------------------------------------------
void display_info(){
//  Serial.print(rght_pps);
//  Serial.print("   ");
//  Serial.println(lft_pps);
//  Serial.print("   ");
//  Serial.print(orint);
//  Serial.print("   ");
//  Serial.println(orint_c);
  }
//-----------------------------------------------------------

void transition(){
  roboclaw.ForwardM1(address_rght,0);
  roboclaw.ForwardM1(address_lft,0);
  roboclaw.BackwardM1(address_rght,0);
  roboclaw.BackwardM1(address_lft,0);
  delay(100);
  roboclaw.SpeedM1(address_rght,0);
  roboclaw.SpeedM1(address_lft,0);
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

void loop(){
  dabble_control();
  states();
  read_grid();
  spray_noz();
//  display_info();
}
