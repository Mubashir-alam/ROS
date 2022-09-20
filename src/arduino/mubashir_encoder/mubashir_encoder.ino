// Encoder Interrupt declaration
#define readA bitRead(PIND,2)//faster than digitalRead()
#define readB bitRead(PIND,3)//faster than digitalRead()
const byte encoderPinA = 3;//outputA digital pin2
const byte encoderPinB = 2;//outoutB digital pin3
volatile int count = 0;
long protectedCount = 0;

void setup() {
  Serial.begin (9600);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, RISING);
  interrupts();
}
void loop() {

  protectedCount += count;
  Serial.println(protectedCount);
 }
void isrA() {
  if(readB != readA) {
    count ++;
  } else {
    count --;
  }
}
void isrB() {
  if (readA == readB) {
    count ++;
  } else {
    count --;
  }
} 
