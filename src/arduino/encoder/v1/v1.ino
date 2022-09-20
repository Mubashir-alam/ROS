// this script only read the encoder value and see on serial monitor
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

void setup() {
  pinMode (encoderPinA, INPUT_PULLUP);
  pinMode (encoderPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (encoderPinA), doEncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (encoderPinB), doEncoderB, CHANGE);
  Serial.begin (9600);
}
void loop() {
  current_wheel_distance = (currentPosition * 2 * pi * r) / (PPR * gearRatio * decodeNumber) ; // Gives the distance which the wheel traveled. 'r' is the radius of the wheel in meters.
  time_taken = micros() - timer_previous; // Time taken since the previous iteration of the loop.
  wheel_speed_mpms = (current_wheel_distance-previous_wheel_distance) / time_taken; // This gives the speed in meters per microsecond.
  wheel_speed_mpm = wheel_speed_mpms * 1000000 * 60; // This gives the speed in meters per minute.

  Serial.print(current_wheel_distance, 4);
  Serial.print(" ");
  Serial.print(wheel_speed_mpm, 4);
  Serial.println();

  timer_previous = micros();
  previous_wheel_distance = current_wheel_distance;
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
