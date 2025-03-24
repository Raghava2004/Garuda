 // functions by drehm and useful for me
// 
#include<PWMServo.h>
#include<Wire.h>

const int ch1Pin=8;
const int ch2Pin=9;
const int ch3Pin=10;
const int ch4Pin=11;
const int ch5Pin=12;

float PIDReturn[]={0, 0, 0};

const int servo1Pin=0;
const int servo2Pin=1;
const int servo3Pin=2;
const int servo4Pin=3;

float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;
float B_madgwick = 0.04;  //Madgwick filter parameter

//float yaw_PID=0,pitch_PID=0,roll_PID=0;

float i_limit = 25.0; 
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0; 

unsigned long prev_time,current_time,print_counter;

float RatePitch,RateRoll,RateYaw;
float RateRollCal,RatePitchCal,RateYawCal;

float AccX,AccY,AccZ;
float AngleRoll,AnglePitch;

float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;

float PrevErrorRateRoll,PrevErrorRatePitch,PrevErrorRateYaw;
float PrevItermRateRoll,PrevItermRatePitch,PrevItermRateYaw;

float s1_command_scaled,s2_command_scaled,s3_command_scaled,s4_command_scaled;
int s1_command_PWM,s2_command_PWM,s3_command_PWM,s4_command_PWM;

float errorRoll,errorPitch,errorYaw,dt;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

bool armedFly=false;

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;


volatile uint32_t ch1_start = 0, ch2_start = 0, ch3_start = 0, ch4_start = 0,ch5_start = 0;
volatile uint16_t ch1_value = 0, ch2_value = 0, ch3_value = 0, ch4_value = 0,ch5_value = 0;
unsigned long channel_1_raw,channel_2_raw,channel_3_raw,channel_4_raw;




float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;
float B_gyro=0.1;
float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

unsigned long channel_1_fs=1500;
unsigned long channel_2_fs=1500;
unsigned long channel_3_fs=1000;
unsigned long channel_4_fs=1500;
unsigned long channel_5_fs=2000;

float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

float GyroX_prev, GyroY_prev, GyroZ_prev;


void setup() {
  // put your setup code here, to run once:
  

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(2000);
  for(int cal_num=0;cal_num<2000;cal_num++){
    gyro_signals();
    RateRollCal+=RateRoll;
    RatePitchCal+=RatePitch;
    RateYawCal+=RateYaw;
    delay(1);
  }
  RateRollCal/=2000;
  RatePitchCal/=2000;
  RateYawCal/=2000;

  delay(500);
  pinMode(13,OUTPUT);
  delay(5);
  servo1.attach(servo1Pin,900,2100);
  servo2.attach(servo2Pin,900,2100);
  servo3.attach(servo3Pin,900,2100);
  servo4.attach(servo4Pin,900,2100);
  delay(5);

  radioSetup();

  ch1_value=channel_1_fs;
  ch2_value=channel_2_fs;
  ch3_value=channel_3_fs;
  ch4_value=channel_4_fs;
  ch5_value=channel_5_fs;
  delay(5);

  //delay(100);

  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);

  delay(5);
  Serial.print("Done ");

  //calibrateESC();

  setupBlink(3,160,70);
  Serial.println("Hello!  Main loop Entered");
}

void loop() {
  // put your main code here, to run repeatedly:h
  
  prev_time = current_time;      
  current_time = micros();  
  dt = (current_time - prev_time)/1000000.0;
  //printRollPitchYaw();
  //printPIDoutput();
  printAccel();
  //printServoCommands();
  //printRadioCommands();// this works but yaw is showing some problem
 // printDesiredState();// this also works
  armedStatus();
  gyro_signals();
  RateRoll-=RateRollCal;
  RatePitch-=RatePitchCal;
  RateYaw-=RateYawCal;
  Madgwick6DOF(RateRoll, -RatePitch, -RateYaw, -AccX, AccY, AccZ, dt);

  //Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);

  getDesState();
  //controlPID();
  controlANGLE();

  //controlRATE();

  controlMixer();
  scaledCommands();

  throttleCut();

  servo1.write(s1_command_PWM);
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);

  getCommands();
  failSafe();

  loopRate(2000);
  //Serial.println("1 iteration done");

}

void getDesState(){
  thro_des=(ch3_value-1000.0)/1000.0;
  roll_des=(ch1_value-1500.0)/500.0;
  pitch_des=(ch2_value-1500.0)/500.0;
  yaw_des=(ch4_value-1500.0)/500.0;
  roll_passthru=roll_des/2.0;
  pitch_passthru=pitch_des/2.0;
  yaw_passthru=yaw_des/2.0;

  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);

}



void controlMixer(){
  s1_command_scaled=thro_des- pitch_PID + roll_PID + yaw_PID;
  s2_command_scaled=thro_des- pitch_PID - roll_PID - yaw_PID;
  s3_command_scaled=thro_des+ pitch_PID - roll_PID + yaw_PID;
  s4_command_scaled=thro_des + pitch_PID + roll_PID - yaw_PID;
}

void armedStatus(){
  if((ch5_value<1500) && (ch3_value<1050)){
    armedFly=true;
    //Serial.println("armed");
  }
  //Serial.println("NOt armed");
}

void scaledCommands(){

  s1_command_PWM=s1_command_scaled*180;
  s2_command_PWM=s2_command_scaled*180;
  s3_command_PWM=s3_command_scaled*180;
  s4_command_PWM=s4_command_scaled*180;

  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);

}
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}


void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (ch1_value < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = RateRoll;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (ch1_value < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = RatePitch;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - RateYaw;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (ch1_value < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}



void controlPID(){
  error_roll = roll_des - RateRoll;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (ch1_value < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - RatePitch;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (ch1_value < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - RateYaw;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (ch1_value < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = RatePitch;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = RateYaw;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

/*
void controlPID(){
  ErrorRateRoll=roll_des-RateRoll;
  ErrorRatePitch=pitch_des-RatePitch;
  ErrorRateYaw=yaw_des-RateYaw;
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  roll_PID=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1]; 
  PrevItermRateRoll=PIDReturn[2];
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  pitch_PID=PIDReturn[0]; 
  PrevErrorRatePitch=PIDReturn[1]; 
  PrevItermRatePitch=PIDReturn[2];
  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
  yaw_PID=PIDReturn[0]; 
  PrevErrorRateYaw=PIDReturn[1]; 
  PrevItermRateYaw=PIDReturn[2];

  roll_PID = constrain(roll_PID, -30, 30);   // Roll range [-30, 30]
  pitch_PID = constrain(pitch_PID, -30, 30); // Pitch range [-30, 30]
  yaw_PID = constrain(yaw_PID, -160, 160); 

}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;
  if (Iterm > 400) Iterm=400;
  else if (Iterm <-400) Iterm=-400;
  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;
  if (PIDOutput>400) PIDOutput=400;
  else if (PIDOutput <-400) PIDOutput=-400;
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}*/
void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}


void failSafe(){
  unsigned minVal=800;
  unsigned maxVal=2200;

  int check1=0;
  int check2=0;
  int check3=0;
  int check4=0;
  int check5=0;
  if (ch1_value > maxVal || ch1_value < minVal) check1 = 1;
  if (ch2_value > maxVal || ch2_value < minVal) check2 = 1;
  if (ch3_value > maxVal || ch3_value < minVal) check3 = 1;
  if (ch4_value > maxVal || ch4_value < minVal) check4 = 1;
  if (ch5_value > maxVal || ch5_value < minVal) check5 = 1;

  if ((check1 + check2 + check3 + check4 + check5 ) > 0) {
    ch1_value = channel_1_fs;
    ch2_value = channel_2_fs;
    ch3_value = channel_3_fs;
    ch4_value = channel_4_fs;
    ch5_value = channel_5_fs;
}
}

void loopRate(int freq){
  float invFreq=1.0/freq*1000000.0;
  unsigned long checker=micros();

  while(invFreq>(checker-current_time)){
    checker=micros();
  }
}
void throttleCut(){
  if((ch5_value>1500) && (armedFly==true)){
    s1_command_PWM=0;
    s2_command_PWM=0;
    s3_command_PWM=0;
    s4_command_PWM=0;
    reset_pid();
  }
}

void radioSetup(){
  pinMode(ch1Pin,INPUT);
  pinMode(ch2Pin,INPUT);
  pinMode(ch3Pin,INPUT);
  pinMode(ch4Pin,INPUT);
  pinMode(ch5Pin,INPUT);

  delay(20);

  attachInterrupt(digitalPinToInterrupt(ch1Pin), pwmISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), pwmISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3Pin), pwmISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch4Pin), pwmISR4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch5Pin), pwmISR5, CHANGE);

  delay(20);
}


void getCommands(){

  ch1_value=channel_1_raw;
  ch2_value=channel_2_raw;
  ch3_value=channel_3_raw;
  ch4_value=channel_4_raw;

}



void pwmISR1() {
  if (digitalRead(ch1Pin) == HIGH)
    ch1_start = micros();  // Rising edge: store start time
  else
    channel_1_raw = micros() - ch1_start;  // Falling edge: calculate pulse width
}

void pwmISR2() {
  if (digitalRead(ch2Pin) == HIGH)
    ch2_start = micros();
  else
    channel_2_raw = micros() - ch2_start;
}

void pwmISR3() {
  if (digitalRead(ch3Pin) == HIGH)
    ch3_start = micros();
  else
    channel_3_raw = micros() - ch3_start;
}

void pwmISR4() {
  if (digitalRead(ch4Pin) == HIGH)
    ch4_start = micros();
  else
    channel_4_raw = micros() - ch4_start;
}
void pwmISR5() {
  if (digitalRead(ch5Pin) == HIGH)
    ch5_start = micros();
  else
    ch5_value = micros() - ch5_start;
}


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

void calibrateESC(){
  while(true){
    prev_time=current_time;
    current_time=micros();

    getCommands();
    failSafe();
    getDesState();
    //getIMUData();
    //Madgwick();
    getDesState();
    s1_command_scaled = thro_des;
    s2_command_scaled = thro_des;
    s3_command_scaled = thro_des;
    s4_command_scaled = thro_des;


    scaledCommands();

    servo1.write(s1_command_PWM);
    servo2.write(s2_command_PWM);
    servo3.write(s3_command_PWM);
    servo4.write(s4_command_PWM);


  }
}

void printPIDoutput(){
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(yaw_PID);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(RateRoll);
    Serial.print(F(" pitch:"));
    Serial.print(RatePitch);
    Serial.print(F(" yaw:"));
    Serial.println(RateYaw);
  }
}
void printAccel() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(AccX);
    Serial.print(F(" pitch:"));
    Serial.print(AccY);
    Serial.print(F(" yaw:"));
    Serial.println(AccZ);
  }
}

void printRadioCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(ch1_value);
    Serial.print(F(" CH2:"));
    Serial.print(ch2_value);
    Serial.print(F(" CH3:"));
    Serial.print(ch3_value);
    Serial.print(F(" CH4:"));
    Serial.print(ch4_value);
    Serial.print(F(" CH5:"));
    Serial.println(ch5_value);

  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("s1_command:"));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command:"));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command:"));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command:"));
    Serial.println(s4_command_PWM);

  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}
void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

float invSqrt(float x) {
  return 1.0/sqrtf(x); 
}
