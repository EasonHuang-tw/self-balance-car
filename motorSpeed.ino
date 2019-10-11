#include <TimerOne.h>
#include "MPU9250.h"
int counter[2] ={0,0}; //left ,right
int last_counter =0;
int lef_motor[3]={6,7,8};
int P_error=0;
int last_P=0;
int I_error=0;
int D_error=0;
int target = 275;
float kp=0.85;
float ki=0.07;
float kd=0.018;
double Speed=0;
int mpu_counter =0;
MPU9250 mpu;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  pinMode(4,INPUT);
  pinMode(5,INPUT);
  for(int i=0;i<3;i++)
      pinMode(lef_motor[i],OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), plus_left, RISING);
  attachInterrupt(digitalPinToInterrupt(2), plus_right, RISING);
  Serial.begin(115200);
  Timer1.initialize(50000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  delay(200);
  mpu.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  /*digitalWrite(7,HIGH);
  digitalWrite(8,LOW);
  analogWrite(6,255);
  if(counter!=counter_last){
    Serial.println(counter);
    counter_last=counter;
  }
  */
  if(Serial.available()){
    target=Serial.parseInt();
    Serial.println(target);
    }
  static uint32_t prev_ms = millis();
  if ((millis() - prev_ms) > 16)
  {
      mpu.update();
      //mpu.print();
      
      prev_ms = millis();
      mpu_counter++;
  }
  if(mpu_counter>20){
        Serial.print("pitch (y-right (east))    : ");
        Serial.println(mpu.getPitch());
        Serial.print("left:");
        Serial.print(counter[0]);
        Serial.print("right:");
        Serial.println(counter[1]);
        mpu_counter=0;

   }
  P_error=P_E(counter[0],target,550); //pose now , target ,whole role
  int error = kp*(double)P_error+ki*(double)I_error+kd*D_error;
  digitalWrite(7,error<0?0:1);
  digitalWrite(8,error<0?1:0);\
  if(abs(error)>255)error=255;
  analogWrite(6,abs(error));
  /*Serial.print("counter:");
  Serial.print(counter);*/
  //Serial.print(" I:");
  //Serial.println(error);
}
void plus_left() {
  if(digitalRead(4)==HIGH)
    counter[0]++;
  else
    counter[0]--;
}
void plus_right() {
  if(digitalRead(5)==HIGH)
    counter[1]++;
  else
    counter[1]--;
}
void timerIsr()
{
    Speed = (double)(counter[0]-last_counter)/550.0*360.0/0.05;
    last_counter = counter[0];
    D_error=P_error-last_P;
    last_P=P_error;
    I_error = I_error+P_error;
}
int P_E(int pose , int target , int scaler){
   if(pose>=0)                                  //determine the pose in range 0 to scaler
     pose = pose%scaler;
    else 
     pose = pose%scaler+scaler;
     
    int error = target-pose;                    
    if(error<=scaler/2&&error>-scaler/2)        //make sure the error can turn right the most effeciently
      return(error);
    else if(error>scaler/2&&error<scaler)
      return(error-scaler);
    else if(error>-scaler)
      return(error+scaler);
}
