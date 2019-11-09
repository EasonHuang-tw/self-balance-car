#include <TimerOne.h>
#include "MPU9250.h"

#define LEFT_INTERRUPT_0 3
#define LEFT_INTERRUPT_1 4
#define RIGHT_INTERRUPT_0 2
#define RIGHT_INTERRUPT_1 5 
//*********  motor control *************
int counter[2] ={0,0}; //left ,right step
int left_motor[3]={8,7,6};    //pin8 for pwm
int right_motor[3]={9,10,11}; //pin11 for pwm

//*********  error with target *********
// "0" for left   ,  "1" for right

int error[2]= {0};
float P_error[2]={0};
float I_error[2]={0};
float D_error[2]={0};
float target[2] = {0};
//float kp=0.85;
static float kp=9;
static float ki=0.075;
static float kd=10.325;

//********* time and speed
double Speed[2]={0};
float last_P[2]={0};
int last_counter[2] = {0};
int mpu_counter =0;


//********* IMU
float pitch;
MPU9250 mpu;


void setup() {
  // for IMU
  
  Wire.begin();
  delay(200);
  
  Serial.begin(9600);
  Serial.println("haha");
  mpu.setup();
  Serial.println("haha2");
  // for motor
  pinMode(LEFT_INTERRUPT_1,INPUT);
  pinMode(RIGHT_INTERRUPT_1,INPUT);
  
   
  for(int i=0;i<3;i++){
      pinMode(left_motor[i],OUTPUT);
      pinMode(right_motor[i],OUTPUT);
  }
  /*
  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_0), plus_left, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_0), plus_right, RISING);
 */
  //for Serial communication
  

  //for velocity count
  Timer1.initialize(50000); // set a timer of length 50000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here

}

void loop() {
 
  //for imu update
  static uint32_t prev_ms = millis();
  if ((millis() - prev_ms) > 16){
      mpu.update();
      //mpu.print();
      pitch=mpu.getPitch()-1;
      target[0]=pitch;
      target[1]=pitch;
      prev_ms = millis();
      mpu_counter++;
  }

  //motor out put
  for(int i =0;i<2;i++){
    P_error[i]=P_E(target[i]); //pose now , target ,whole role
    error[i] = kp*P_error[i]+ki*I_error[i]+kd*D_error[i];//+ki*I_error+kd*D_error;
    if(error[i]>150)
      error[i]=150+(error[i]-150.)*0.75;
    else if(error[i]<-125)
      error[i]=-150+(error[i]+150.)*0.75;
  }
  /*
  if(error[0]>0)error[0]+=60;
    else if(error[0]<0)error[0]-=60;
    if(error[1]>0)error[1]+=60;
    else if(error[1]<0)error[1]-=60;
   */
  //left motor
  digitalWrite(left_motor[0],error[0]<0?0:1);
  digitalWrite(left_motor[1],error[0]<0?1:0);
  if(abs(error[0])>255)error[0]=255;
  analogWrite(left_motor[2],abs(error[0]));
  //analogWrite(left_motor[2],0);
  //right motor
  digitalWrite(right_motor[0],error[1]<0?0:1);
  digitalWrite(right_motor[1],error[1]<0?1:0);
  if(abs(error[1])>255)error[1]=255;
  analogWrite(right_motor[2],abs(error[1]));
  //analogWrite(right_motor[2],0);
  //out put by Serial
   if(mpu_counter>20){
        Serial.print("pitch (y-right (east))    : ");
        Serial.println(mpu.getPitch()-1);
        Serial.println(error[1]);
        Serial.println(error[0]);
        //Serial.print("right counter:");
        //Serial.println(counter[0]);
        //Serial.print("left counter:");
        //Serial.println(counter[1]);
        /*Serial.print("left:");
        Serial.print(counter[0]);
        Serial.print("right:");
        Serial.println(counter[1]);*/
        mpu_counter=0;
   }
}



//      motor  encoder
void timerIsr()
{
    for(int i =0;i<2;i++){
      Speed[i] = (double)(counter[i]-last_counter[i])/550.0*360.0/0.05;
      last_counter[i] = counter[i];
      D_error[i]=P_error[i]-last_P[i];
      last_P[i]=P_error[i];
     // if(I_error[i]<50)
      I_error[i] = I_error[i]+P_error[i];
    }
}

// get p error
int P_E(float target ){
      return(target);
}
