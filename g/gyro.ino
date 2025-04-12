void gyro_signals(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); //low pass filter
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
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
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;

  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  RateRoll = RateRoll - GyroErrorX;
  RatePitch = RatePitch - GyroErrorY;
  RateYaw = RateYaw - GyroErrorZ;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
/*
  RateRoll = (1.0 - B_gyro)*GyroX_prev + B_gyro*RateRoll;
  RatePitch = (1.0 - B_gyro)*GyroY_prev + B_gyro*RatePitch;
  RateYaw = (1.0 - B_gyro)*GyroZ_prev + B_gyro*RateYaw;
  GyroX_prev = RateRoll;
  GyroY_prev = RatePitch; 
  GyroZ_prev = RateYaw;
*/
}
