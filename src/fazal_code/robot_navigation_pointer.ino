// Robot Navigation and Localization using encoders and digital compass data
// Authur: Fazal Nasir Khan
//         Department of Mechatronics Engg. ARAL Lab, UET Peshawar
// Date:   August 30, 2021
#include <HMC5883L.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>
#include <MechaQMC5883.h>
MechaQMC5883 qmc;
// Roboclaw configuration
#define address_lft 0x80
#define address_rght 0x81
SoftwareSerial serial(10,11); // S_RX 10, S_TX 11
RoboClaw roboclaw(&serial,10000);
long int enc_lft, enc_rght, renc_pre=0, lenc_pre=0, speed_lft, speed_rght;

// PID parameters
#define kp  6
#define ki  0.5
#define kd  1
float left_[3] ={0,0,0}; // signal, pre_er, integ
float rght_[3] ={0,0,0};
float turn_[3] ={0,0,0};
float er, d_er, dc, dc_sat;
int pwm_r, pwm_l, drive;


float a[8]={0,0,0,0,0,0,0,0};// filter

// Robot stats variables
#define kv 0.6
#define kh 1
#define L 1
#define r 0.66

#define Pi 3.141

float orint, orint_c, lvel_ref, dis_d, avel_ref;
float x, y, vr, vl, x_pre, y_pre, orint_pre, orint_d, x_d, y_d; 

// path
#define path_c 4
float x_cord[path_c]={5, 5, 0, 0};
float y_cord[path_c]={0, 5, 5, 0};
int c = 0;
#define DIS_THRESHOLD 0.1     // meters
#define NORTH 0
#define SOUTH 180
#define WEST 90
#define EAST 270

void setup() {
  Wire.begin();
  Serial.begin(38400);
  roboclaw.begin(38400);
  qmc.init();
  //turn(NORTH);
  home_pos();
}
//--------------------------------------------------------------------
void home_pos(){ // call this function once at the beginning of the sketch
// set global position & orientation to zeros
// set encoders to zeros
//SetEncM1(address_lft,0)
while(1){
roboclaw.SetEncM1(address_rght,0);
roboclaw.SetEncM1(address_lft,0);
delay(100);
}
  x = 0;  
  y = 0;
  x_pre = 0;
  y_pre = 0;
  orint_pre = 0;
  orint = 0;
  renc_pre = 0; 
  lenc_pre = 0;
  }
//---------------------------------------------------------------------
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
float transform(float c){
  if(c < 0)
    c += 2*Pi;
if(c > 2*Pi)
    c -= 2*Pi;
    //c = orint_c*180/PI;
  return c;
  }
//-----------------------------------------------------------
void heading(){   // current orientation of robot
  int x,y,z;
  qmc.read(&x, &y, &z);
  orint_c = atan2(y, x);
  orint_c = filter(orint_c);
  orint_c = transform(orint_c);
}
//---------------------------------------------------------------
void states(){  // encoders positions and velocities
  uint8_t status_pos,status_vel;
  bool valid_pos,valid_vel;
  
  int32_t apose = roboclaw.ReadEncM1(address_rght, &status_pos, &valid_pos);
  int32_t avel = roboclaw.ReadSpeedM1(address_rght, &status_vel, &valid_vel);
  if(valid_pos)
    enc_rght = apose;
  if(valid_vel)
    rght_[0] = 2*Pi*r*avel/4000.0; // right wheel velocity in m/s

    apose= roboclaw.ReadEncM1(address_lft, &status_pos, &valid_pos);
    avel = roboclaw.ReadSpeedM1(address_lft, &status_vel, &valid_vel);
  if(valid_pos)
    enc_lft = apose;
  if(valid_vel)
    left_[0] = 2*Pi*r*avel/4000.0;  // left wheel velocity in m/s
  
//------------------------------------------------------------
 // global position and orientation of robot
    float dr = (2.0*Pi*(enc_rght-renc_pre))/4000.0; // differential angular displacment of wheel in radians
    float dl = (2.0*Pi*(enc_lft-lenc_pre))/4000.0;
    float ds = r*(dr+dl)/2;                         // differential displacment of robot in meters
    float dorint = r*(dr-dl)/L;                     // differential orientation of robot in radians
   
    orint = orint_pre + dorint;                     // global orintation in radians
//  Kalman filter for fusing orint and orint_c    
   
    x = x_pre + ds*cos(orint_c);                    // global x cordinate in meters
    y = y_pre + ds*sin(orint_c);                    // global y cordinate in meters
//  Kalman filter for fusing x, y with GPS
    
    renc_pre = enc_rght;
    lenc_pre = enc_lft;
    x_pre = x;
    y_pre = y;
    orint_pre = orint;
//--------------------------------------------------------------
// reference wheels velocities
  dis_d = sqrt(pow((x_d-x),2)+pow((y_d-y),2));
  orint_d = atan2((y_d-y),(x_d-x));
  orint_d = transform(orint_d);
  
  lvel_ref = kv*dis_d;                  // lvel_ref = kv*dis_d*cos(orint_d-orint_c);
  avel_ref = kh*(orint_d-orint_c);      // avel_ref = kv*sin(e)cos(e)+ kh*e;

  vr = lvel_ref + L*avel_ref/2;         // required right wheel linear velocity
  vl = lvel_ref - L*avel_ref/2;         // required left wheel linear velocity

  if(dis_d < DIS_THRESHOLD)
  path();
  }
//---------------------------------------------------------------------
void path(){
  x_d = x_cord[c];
  y_d = y_cord[c];
  float targ_angle = atan2((y_cord[c]-y),(x_cord[c]-x));
  targ_angle = transform(targ_angle);
  turn(targ_angle);
  ++c;
//  if(c == path_c)
//  stop
  }
//----------------------------------------------------------------
void turn(float axis){
while((orint_c > axis + 0.5) && (orint_c < axis - 0.5)){
heading();
turn_[0] = orint_c;

// PID for wheels velocities to attain the ref.(axis) orientation
drive = pid(axis, turn_);


//motor_rght(FORWARD_, pwm_r);
//motor_lft(FORWARD_, pwm_l);


//if(){
//roboclaw.ForwardM1(address_rght,drive);
//roboclaw.BackwardM1(address_lft,drive);
//}else
//if(){
//roboclaw.BackwardM1(address_rght,drive);
//roboclaw.ForwardM1(address_lft,drive);  
//  }
 }
}
//---------------------------------------------------------------
  float pid(float ref, float *data_){

    float &sys = *((float*)data_);
    float &pre_er = *((float*)data_+1);
    float &integ = *((float*)data_+2);
    er = ref - sys;
    d_er = er - pre_er;
    pre_er = er;
    integ += er;
    dc = kp*er+ki*integ+kd*d_er;
    integ -= er;

// clamp saturation block--------------------------------------
    if(dc > 127)
    dc_sat = 127;
    else
    if(dc < 0)
    dc_sat = 0;
    else
    dc_sat = dc;
 //--------------------------------------------------------------     
    bool sat = dc != dc_sat;
    bool sign = (er<0 && dc<0) || (er>0 && dc>0);
    if(sat && sign)
    integ += 0;
    else
    integ +=er;
    dc = kp*er+ki*integ+kd*d_er;
    
// control signal saturation block------------------------------
    if(dc > 127)
    dc = 127;
    else
    if(dc < 0)
    dc = 0;

// if dc is less than 0, then exponential function to zero    
//    d_dc = dc-pre_dc;
//  if(d_dc>100){
//    // increase to dc in t=0 to time where d_dc is no more than 100
//    }
//  if(d_dc<-100){
//      }
    return dc;
  }
//------------------------------------------------------------
void display_info(){
  Serial.print(x);
  Serial.print("   ");
  Serial.print(y);
  Serial.print("   ");
  Serial.print(orint);
  Serial.print("   ");
  Serial.print(orint_c);
  Serial.print("   ");
  Serial.print(pwm_l);
  Serial.print("   ");
  Serial.println(pwm_r); 
  }
//-----------------------------------------------------------
void loop(){
heading();
states();
   
pwm_r = pid(vr, rght_);
pwm_l = pid(vl, left_);

roboclaw.ForwardM1(address_rght,pwm_r);
roboclaw.ForwardM1(address_lft,pwm_l);
//display_info();
}
