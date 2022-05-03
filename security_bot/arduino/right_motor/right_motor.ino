
#include <PinChangeInt.h>
#include <Wire.h>
#include <PID_v1.h>
#define encodPinA1      2                       
#define encodPinB1      8                       
#define M1              10                       
#define M2              9
double kp =1, ki =20 , kd =0;             
double input = 0, output = 0, setpoint = 0;
unsigned long lastTime,now;
volatile long encoderPos = 0,last_pos=0,lastpos=0;
PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);  
void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);                  
  pinMode(encodPinB1, INPUT_PULLUP);                  
  attachInterrupt(0, encoder, FALLING);               
  TCCR1B = TCCR1B & 0b11111000 | 1;                   
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  Wire.begin(9);                
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
}
void loop() {
   now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=500 )
   {
      input = (360.0*1000*(encoderPos-last_pos)) /(1856.0*(now - lastTime));
      lastTime=now;
      last_pos=encoderPos;
   }
  myPID.Compute();                                    
  pwmOut(output);                                     
  delay(10);
}
void pwmOut(int out) {                               
  if (out > 0) {
    analogWrite(M1, 0);                            
    analogWrite(M2, out);
  }
  else {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0);                        
  }
}
void encoder()  {                                       
  if (PINB & 0b00000001)    encoderPos++;             
  else                      encoderPos--;             
}
void requestEvent() {
  int8_t s;
  
  s= (360.0*(encoderPos-lastpos))/1856.0; 
  lastpos=encoderPos;
  Wire.write(s); 
}
void receiveEvent(int howMany)
{
  uint8_t a,b;
  a = Wire.read();
  b = Wire.read();
  setpoint= (double)((b<<8)|a);
}
