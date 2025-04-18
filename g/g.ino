// functions by drehm and useful for me
// 
#include<PWMServo.h>
#include<Wire.h>

#include <ros.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle nh;

unsigned long roll_des_us;
unsigned long pitch_des_us;
unsigned long yaw_des_us;
unsigned long thro_des_us;

//servo library for motor initalization
PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;

// receiver initalization
const int PPM_Pin=11;
int ppm_counter = 0;
unsigned long time_ms = 0;

/*
const int ch1Pin=8;
const int ch2Pin=9;
const int ch3Pin=10;
const int ch4Pin=11;
const int ch5Pin=12;
*/
float PIDReturn[]={0, 0, 0};

//motor pins initalization
const int servo1Pin=0;
const int servo2Pin=1;
const int servo3Pin=2;
const int servo4Pin=3;

float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;
float B_madgwick = 0.04;  //Madgwick filter parameter

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.01;
float AccErrorY = 0.00;
float AccErrorZ = 0.06;
float GyroErrorX = -8.20;
float GyroErrorY = -1.19;
float GyroErrorZ = -0.39;

float angle_yaw=0;

//float yaw_PID=0,pitch_PID=0,roll_PID=0;

float i_limit = 25.0; 
float maxRoll = 15.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 15.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
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
float Kd_roll_angle = 0.06;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = Kp_roll_angle;   //Pitch P-gain - angle mode
float Ki_pitch_angle = Ki_roll_angle;   //Pitch I-gain - angle mode
float Kd_pitch_angle = Kd_roll_angle;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;

/*

// new values
float Kp_roll_angle = 0.25;  // A bit more aggressive
float Ki_roll_angle = 0.15;  // Lower I term to reduce oscillation
float Kd_roll_angle = 0.08;  // Slightly more D for damping

float Kp_pitch_angle = Kp_roll_angle;
float Ki_pitch_angle = Ki_roll_angle;
float Kd_pitch_angle = Kd_roll_angle;

*/
float PrevErrorRateRoll,PrevErrorRatePitch,PrevErrorRateYaw;
float PrevItermRateRoll,PrevItermRatePitch,PrevItermRateYaw;

float s1_command_scaled,s2_command_scaled,s3_command_scaled,s4_command_scaled;
int s1_command_PWM,s2_command_PWM,s3_command_PWM,s4_command_PWM;

float errorRoll,errorPitch,errorYaw,dt;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;

bool armedFly=false;

volatile uint32_t ch1_start = 0, ch2_start = 0, ch3_start = 0, ch4_start = 0,ch5_start = 0;
volatile uint16_t ch1_value = 0, ch2_value = 0, ch3_value = 0, ch4_value = 0,ch5_value = 0,ch6_value = 0;
unsigned long channel_1_raw,channel_2_raw,channel_3_raw,channel_4_raw,channel_5_raw,channel_6_raw;


//GPS variables
uint8_t read_serial_byte, incomming_message[100], number_used_sats, fix_type;
uint8_t waypoint_set, latitude_north, longiude_east ;
uint16_t message_counter;
int16_t gps_add_counter;
int32_t l_lat_gps, l_lon_gps, lat_gps_previous, lon_gps_previous;
int32_t lat_gps_actual, lon_gps_actual, l_lat_waypoint, l_lon_waypoint;
float gps_pitch_adjust_north, gps_pitch_adjust, gps_roll_adjust_north, gps_roll_adjust;
float lat_gps_loop_add, lon_gps_loop_add, lat_gps_add, lon_gps_add;
uint8_t new_line_found, new_gps_data_available, new_gps_data_counter;
uint8_t gps_rotating_mem_location;
int32_t gps_lat_total_avarage, gps_lon_total_avarage;
int32_t gps_lat_rotating_mem[40], gps_lon_rotating_mem[40];
int32_t gps_lat_error, gps_lon_error;
int32_t gps_lat_error_previous, gps_lon_error_previous;
uint32_t gps_watchdog_timer;


float Kp_roll_rate = 0.1;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.15;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = Kp_roll_rate;   //Pitch P-gain - rate mode
float Ki_pitch_rate = Ki_roll_rate;    //Pitch I-gain - rate mode
float Kd_pitch_rate = Kd_roll_rate; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.2;           //Yaw P-gain
float Ki_yaw = 0.1;          //Yaw I-gain
float Kd_yaw = 0.0003;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

float gps_p_gain = 2.7;                    //Gain setting for the GPS P-controller (default = 2.7).
float gps_d_gain = 6.5; 

//failsafe values for receiver 
unsigned long channel_1_fs=1500;
unsigned long channel_2_fs=1500;
unsigned long channel_3_fs=1000;
unsigned long channel_4_fs=1500;
unsigned long channel_5_fs=2000;

unsigned long autonomous_ch1_value = 1500.0;
unsigned long autonomous_ch2_value = 1500.0;
unsigned long autonomous_ch3_value = 1000.0;
unsigned long autonomous_ch4_value = 1500.0;



float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

float GyroX_prev, GyroY_prev, GyroZ_prev;

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(roll_sub);
  nh.subscribe(pitch_sub);
  nh.subscribe(yaw_sub);
  nh.subscribe(throttle_sub);
  gps_setup();
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(2000);
 //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 
  servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);


  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  ch1_value = channel_1_fs;
  ch2_value = channel_2_fs;
  ch3_value = channel_3_fs;
  ch4_value = channel_4_fs;
  ch5_value = channel_5_fs;

  //Initialize IMU communication
  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
  delay(5000);
  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  
  delay(5);

  //calibrateESC(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}
void loop() {
  // put your main code here, to run repeatedly:h
  
  prev_time = current_time;      
  current_time = micros();  
  dt = (current_time - prev_time)/1000000.0;
  nh.spinOnce();

  //printRollPitchYaw();
  //printPIDoutput();
  //printAccel();
  //printServoCommands();
  //printMad();
  // printRadioCommands();// this works but yaw is showing some problem
  //printDesiredState();// this also works
  armedStatus();
  gyro_signals();/*
  RateRoll-=RateRollCal;
  RatePitch-=RatePitchCal;
  RateYaw-=RateYawCal;*/
  read_gps();
  Madgwick6DOF(RateRoll, -RatePitch, -RateYaw, -AccX, AccY, AccZ, dt);

  getDesState();
  //controlPID();
  handleGpsHold();
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
  if((ch4_value>1500) && (ch3_value<1050)){
    armedFly=true;
  }
  if ((ch4_value < 1200) && (ch3_value < 1050)) {
    armedFly = false;
  }
}

void handleGpsHold() {
  if (ch5_value > 1500 && waypoint_set == 1) { // Assuming GPS lock is ready
    roll_des += gps_roll_adjust;
    pitch_des += gps_pitch_adjust;
  }
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
/*
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
*/
void radioSetup() {
  //PPM Receiver 

    pinMode(PPM_Pin, INPUT_PULLUP);
    delay(20);
    attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

}

void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig==1) { //Only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();

    
    if (dt_ppm > 5000) { //Waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }
  
    if (ppm_counter == 1) { //First pulse
      channel_1_raw = dt_ppm;
    }
  
    if (ppm_counter == 2) { //Second pulse
      channel_2_raw = dt_ppm;
    }
  
    if (ppm_counter == 3) { //Third pulse
      channel_3_raw = dt_ppm;
    }
  
    if (ppm_counter == 4) { //Fourth pulse
      channel_4_raw = dt_ppm;
    }
  
    if (ppm_counter == 5) { //Fifth pulse
      channel_5_raw = dt_ppm;
    }
  
    if (ppm_counter == 6) { //Sixth pulse
      channel_6_raw = dt_ppm;
    }
    
    ppm_counter = ppm_counter + 1;
  }
}

void getCommands(){
  if(channel_5_raw>1600){
  ch1_value=channel_1_raw;
  ch2_value=channel_2_raw;
  ch3_value=channel_3_raw;
  ch4_value=channel_4_raw;
  ch5_value=channel_5_raw;
  ch6_value=channel_6_raw;}
  else{
    ch1_value=roll_des_us;
    ch2_value=pitch_des_us;
    ch3_value=throttle_des_us;
    ch4_value=yaw_des_us;
    ch5_value=channel_5_raw;
    ch6_value=channel_6_raw;

    Serial.print("Autonomous");
  }
}
/*
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
*/
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
    Serial.print(AngleRoll);
    Serial.print(F(" pitch:"));
    Serial.println(AnglePitch);/*
    Serial.print(F(" yaw:"));
    Serial.println(AccZ);*/
  }
}

void printMad() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
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
void rollCallback(const std_msgs::UInt32& msg){
  roll_des_us = msg.data;
}

void pitchCallback(const std_msgs::UInt32& msg){
  pitch_des_us = msg.data;
}

void yawCallback(const std_msgs::UInt32& msg){
  yaw_des_us = msg.data;
}

void throttleCallback(const std_msgs::UInt32& msg){
  thro_des_us = msg.data;
}

ros::Subscriber<std_msgs::UInt32> roll_sub("roll_des_us", &rollCallback);
ros::Subscriber<std_msgs::UInt32> pitch_sub("pitch_des_us", &pitchCallback);
ros::Subscriber<std_msgs::UInt32> yaw_sub("yaw_des_us", &yawCallback);
ros::Subscriber<std_msgs::UInt32> throttle_sub("thro_des_us", &throttleCallback);


float invSqrt(float x) {
  return 1.0/sqrtf(x); 
}
