#include<Wire.h>


float RateRoll,RatePitch,RateYaw;
float RateRollCal,RatePitchCal,RateYawCal;
int cal_num;


void gyro_signals(void){
  Wire.beginTransmission(0x68);  // address of gyro or mpu6050
  Wire.write(0x1A); //lpf
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

}

void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals();
  RateRoll-=RateRollCal;
  RatePitch-=RatePitchCal;
  RateYaw-=RateYawCal;
  Serial.print("Rate Roll:   ");
  Serial.print(RateRoll);
  Serial.print("   Rate Pitch:  ");
  Serial.print(RatePitch);
  Serial.print("   Rate Yaw:  ");
  Serial.println(RateYaw);
  delay(50);

}
