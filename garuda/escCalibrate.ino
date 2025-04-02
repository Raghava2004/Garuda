
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
    Serial.print("calibration done");


  }
}
