#include<Wire.h>

float RatePitch,RateRoll,RateYaw;
float RateRollCal,RatePitchCal,RateYawCal;
int cal_num;


uint32_t LoopTimer;

float DesiredRateRoll,DesiredRatePitch,DesiredRateYaw;
float ErrorRateRoll,ErrorRatePitch,ErrorRateYaw;
float InputRoll,InputPitch,InputYaw,InputThrottle;
float PrevErrorRateRoll,PrevErrorRatePitch,PrevErrorRateYaw;
float PrevItermRateRoll,PrevItermRatePitch,PrevItermRateYaw;
float PIDReturn[]={0,0,0};
float PRateRoll=0.6;float PRatePitch=PRateRoll;float PRateYaw=2;
float IRateRoll=3.5;float IRatePitch=IRateRoll;float IRateYaw=12;
float DRateRoll=0.03;float DRatePitch=DRateRoll;float DRateYaw=0;

void gyro_signals(void){
  Wire.beginTransmission(0x68);  // address of gyro or mpu6050
  Wire.write(0x1A); //low pass filter
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);// 1st address of register of gyro
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t Gyro_X=Wire.read()<<8 |Wire.read();
  int16_t Gyro_Y=Wire.read()<<8 |Wire.read();
  int16_t Gyro_Z=Wire.read()<<8 |Wire.read();

  RateRoll=(float)Gyro_X/65.5;
  RatePitch=(float)Gyro_Y/65.5;
  RateYaw=(float)Gyro_Z/65.5;

}

void pid_equation(float Error, float P,float I,float D,float PrevError, float PrevIterm){
  float Pterm=P*Error;
  float Iterm=PrevIterm+ I*(Error+PrevIterm)*0.004/2;
  if(Iterm>400){
    Iterm=400;
  }
  else if(Iterm<-400){
    Iterm=-400;
  }
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput=Pterm+Iterm+Dterm;
  if(PIDOutput>400){
    PIDOutput=400;
  }
  else if(PIDOutput<-400){
    PIDOutput=-400;
  }

  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void){         // resetting the values for when the mototrs turn off
  PrevErrorRateRoll=0;PrevErrorRatePitch=0;PrevErrorRateYaw=0;
  PrevItermRateRoll=0;PrevItermRatePitch=0;PrevItermRateYaw=0;
}



void setup() {
  // put your setup code here, to run once:

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for(cal_num=0;cal_num<2000;cal_num++){
    gyro_signals();
    RateRollCal+=RateRoll;
    RatePitchCal+=RatePitch;
    RateYawCal+=RateYaw;
    delay(1);

  }
  RateRollCal/=2000;
  RatePitchCal/=2000;
  RateYawCal/=2000;
  LoopTimer=micros();



}

void loop() {
  // put your main code here, to run repeatedly:

  gyro_signals();
  RateRoll-=RateRollCal;
  RatePitch-=RatePitchCal;
  RateYaw-=RateYawCal;
  DesiredRateRoll=0;
  DesiredRatePitch=0;
  DesiredRateYaw=0;//receiveer commands



  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  pid_equation(ErrorRateRoll,PRateRoll,IRateRoll,DRateRoll,PrevErrorRateRoll,PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1];
  PrevItermRateRoll=PIDReturn[2];

  pid_equation(ErrorRatePitch,PRatePitch,IRatePitch,DRatePitch,PrevErrorRatePitch,PrevItermRatePitch);
  InputPitch=PIDReturn[0];
  PrevErrorRatePitch=PIDReturn[1];
  PrevItermRatePitch=PIDReturn[2];

  pid_equation(ErrorRateYaw,PRateYaw,IRateYaw,DRateYaw,PrevErrorRateYaw,PrevItermRateYaw);
  InputYaw=PIDReturn[0];
  PrevErrorRateYaw=PIDReturn[1];
  PrevItermRateYaw=PIDReturn[2];




  delay(50);

}
