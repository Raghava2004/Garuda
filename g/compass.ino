#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// Replace with your actual compass I2C address
#define COMPASS_ADDRESS 0x1E  

// Compass calibration
int compass_cal_values[6];
float compass_scale_y = 1.0, compass_scale_z = 1.0;
int compass_offset_x = 0, compass_offset_y = 0, compass_offset_z = 0;

// Compass raw and corrected values
int16_t compass_x, compass_y, compass_z;
float compass_x_horizontal, compass_y_horizontal;
float actual_compass_heading;
float declination = 0.0; // Set your local declination in degrees
bool compass_calibration_on = false;
float angle_roll = 0.0, angle_pitch = 0.0;

float course_a, base_course_mirrored, actual_course_mirrored;

void setup_compass() {
  Wire.begin();
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x00); Wire.write(0x78); // Config Register A
  Wire.write(0x20);                   // Config Register B
  Wire.write(0x00);                   // Mode Register
  Wire.endTransmission();

  // Read calibration from EEPROM
  for (int i = 0; i < 6; i++) compass_cal_values[i] = EEPROM.read(0x10 + i);

  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) /
                    (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) /
                    (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

void read_compass() {
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(COMPASS_ADDRESS, 6);

  if (Wire.available() == 6) {
    compass_y = Wire.read() << 8 | Wire.read();
    compass_y *= -1;
    compass_z = Wire.read() << 8 | Wire.read();
    compass_x = Wire.read() << 8 | Wire.read();
    compass_x *= -1;

    if (!compass_calibration_on) {
      compass_y += compass_offset_y;
      compass_y *= compass_scale_y;
      compass_z += compass_offset_z;
      compass_z *= compass_scale_z;
      compass_x += compass_offset_x;
    }

    compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) +
                           (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) -
                           (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);

    compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) +
                           (float)compass_z * sin(angle_roll * 0.0174533);

    if (compass_y_horizontal < 0)
      actual_compass_heading = 180 + (180 + atan2(compass_y_horizontal, compass_x_horizontal) * (180 / M_PI));
    else
      actual_compass_heading = atan2(compass_y_horizontal, compass_x_horizontal) * (180 / M_PI);

    actual_compass_heading += declination;
    if (actual_compass_heading < 0) actual_compass_heading += 360;
    else if (actual_compass_heading >= 360) actual_compass_heading -= 360;
  }
}

float course_deviation(float course_b, float course_c) {
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    base_course_mirrored = (course_c > 180) ? course_c - 180 : course_c + 180;
    actual_course_mirrored = (course_b > 180) ? course_b - 180 : course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}
