#include <TimerOne.h>
#include "MPU9250.h"

#define LEFT_INTERRUPT_0 3
#define LEFT_INTERRUPT_1 4
#define RIGHT_INTERRUPT_0 2
#define RIGHT_INTERRUPT_1 5 
//*********  motor control *************
int counter[2] ={0,0}; //left ,right step
int left_motor[3]={6,7,8};    //pin8 for pwm
int right_motor[3]={9,10,11}; //pin11 for pwm

//*********  error with target *********
// "0" for left   ,  "1" for right

int error[2]= {0};
float P_error[2]={0};
float I_error[2]={0};
float D_error[2]={0};
int target[2] = {0};
//float kp=0.85;
static float kp=8;
static float ki=0.07;
static float kd=0.018;

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
  mpu.setup();

  // for motor
  pinMode(LEFT_INTERRUPT_1,INPUT);
  pinMode(RIGHT_INTERRUPT_1,INPUT);
  for(int i=0;i<3;i++){
      pinMode(left_motor[i],OUTPUT);
      pinMode(right_motor[i],OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_0), plus_left, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_0), plus_right, RISING);

  //for Serial communication
  Serial.begin(115200);

  //for velocity count
  Timer1.initialize(50000); // set a timer of length 50000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt( timerIsr ); // attach the service routine here

}

void loop() {

  // hand write your target
  if(Serial.available()){
    target[0]=Serial.parseInt();
    Serial.println(target[0]);
    }
 
  //for imu update
  static uint32_t prev_ms = millis();
  if ((millis() - prev_ms) > 16)
  {
      mpu.update();
      //mpu.print();
      pitch=mpu.getPitch();
      target[0]=counter[0]+pitch*550.0/360.0;
      target[1]=counter[1]+pitch*550.0/360.0;
      prev_ms = millis();
      mpu_counter++;
  }

  //motor out put
  for(int i =0;i<2;i++){
    P_error[i]=P_E(counter[i],target[i],550); //pose now , target ,whole role
    error[i] = kp*P_error[i];//+ki*I_error+kd*D_error;
  }
  //left motor
  digitalWrite(left_motor[0],error<0?0:1);
  digitalWrite(left_motor[1],error<0?1:0);
  if(abs(error[0])>255)error[0]=255;
  analogWrite(left_motor[2],abs(error[0]));

  //right motor
  digitalWrite(right_motor[0],error<0?0:1);
  digitalWrite(right_motor[1],error<0?1:0);
  if(abs(error[1])>255)error[1]=255;
  analogWrite(right_motor[2],abs(error[1]));
  
  //out put by Serial
   if(mpu_counter>20){
        Serial.print("pitch (y-right (east))    : ");
        Serial.println(mpu.getPitch());
        /*Serial.print("target:");
        Serial.println(target[0]);
        Serial.print("left counter:");
        Serial.println(counter[0]);*/
        Serial.print("left error:");
        Serial.println(error[0]);
        Serial.print("right error");
        Serial.println(error[1]);
        /*Serial.print("left:");
        Serial.print(counter[0]);
        Serial.print("right:");
        Serial.println(counter[1]);*/
        mpu_counter=0;
   }
}



//      motor  encoder
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


//      get motor speed
void timerIsr()
{
    for(int i =0;i<2;i++){
      Speed[i] = (double)(counter[i]-last_counter[i])/550.0*360.0/0.05;
      last_counter[i] = counter[i];
      D_error[i]=P_error[i]-last_P[i];
      last_P[i]=P_error[i];
      I_error[i] = I_error[i]+P_error[i];
    }
}

// get p error
int P_E(int pose , int target , int scaler){
   if(pose>=0)                                  //determine the pose in range 0 to scaler
     pose = pose%scaler;
    else 
     pose = pose%scaler+scaler;
   if(target>=0)
    target = target%scaler;
    else 
    target = target%scaler+scaler;
     
    int error = target-pose;                    
    if(error<=scaler/2&&error>-scaler/2)        //make sure the error can turn right the most effeciently
      return(error);
    else if(error>scaler/2&&error<scaler)
      return(error-scaler);
    else if(error>-scaler)
      return(error+scaler);
}
