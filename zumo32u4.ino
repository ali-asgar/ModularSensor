
#include <Zumo32U4.h>
#include <Wire.h>
#include <stdio.h>
#include <math.h>
extern "C"
{
#include <pid.h>
}

#define SPEED           200 // Maximum motor speed when going straight; variable speed when turning
#define TURN_BASE_SPEED 100 // Base speed when turning (added to variable speed)


#define CALIBRATION_SAMPLES 44  // Number of compass readings to take when calibrating
#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

// Allowed deviation (in degrees) relative to target angle that must be achieved before driving straight
#define DEVIATION_THRESHOLD 5

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;

LSM303 compass;
L3G gyro;

lrad_pid_t pid1, pid2;

int32_t encLeft, encRight, fooLeft, fooRight = 0;
char c;
float speedLeft = 0;
float speedRight = 0;
unsigned long lastTime1, lastTime2 = 0;
double lastRevolutions1, lastRevolutions2 = 0;
char buff[25];

void setup() {

  Serial.begin(9600);

  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = { -32767, -32767, -32767};
  unsigned int index;

  Wire.begin();

  compass.init();
  if (!compass.init())
  {
    ledRed(1);
  }
  compass.enableDefault();

  compass.writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate

  gyro.init();
  if (!gyro.init())
  {
    ledRed(1);
  }
  gyro.enableDefault();

  motors.setLeftSpeed(200);
  motors.setRightSpeed(-200);
  for (index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    compass.read();
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    delay(50);
  }
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  lrad_pid_init(&pid1, 10, 0, 0, 40, 5, NULL, false);
  lrad_pid_init(&pid2, 10, 0, 0, 40, 5, NULL, false);
  lrad_pid_update_set_point(&pid1, 0);
  lrad_pid_update_set_point(&pid2, 0);

  delay(1000);
}

void loop() {
  pid_motor1();
  pid_motor2();

  if (Serial.available() > 0)
  {
    c = Serial.read();

    if (c == 'f')
    {
      lrad_pid_update_set_point(&pid1, 2);
      lrad_pid_update_set_point(&pid2, 2);
    }
    else if (c == 's')
    {
      lrad_pid_update_set_point(&pid1, 0);
      lrad_pid_update_set_point(&pid2, 0);
    }
    else if (c == 'd')
    {
      fooLeft += -encoders.getCountsAndResetLeft();
      fooRight += - encoders.getCountsAndResetRight();

      encLeft = (int16_t)(fooLeft & 0xFFFF);
      encRight = (int16_t)(fooRight & 0xFFFF);

      float heading = averageHeading();
      Serial.println(heading);
      int k = 0;
      sprintf(buff, "%03d%011ld%011ld", (int)heading, fooLeft, fooRight);
      Serial.println(buff);
      Serial.println(" ");
    }
    else if (c == 'l')
    {
      lrad_pid_update_set_point(&pid1, 2);
      lrad_pid_update_set_point(&pid2, 1);
    }
    else if (c == 'k')
    {
      lrad_pid_update_set_point(&pid1, 1);
      lrad_pid_update_set_point(&pid2, 2);
    }
    else if (c == 'b')
    {
      lrad_pid_update_set_point(&pid1, -2);
      lrad_pid_update_set_point(&pid2, -2);
    }
    else if (c == 'r')
    {
      setup();
    }
    else if(c == 'n')
    {
      lrad_pid_update_set_point(&pid1, -0.15);
      lrad_pid_update_set_point(&pid2, 0.15);
    }
    else if(c == 'm')
    {
      lrad_pid_update_set_point(&pid1, 0.15);
      lrad_pid_update_set_point(&pid2, -0.15);
    }
  }
}

void pid_motor1()
{
  unsigned long now = millis();

  fooLeft += -encoders.getCountsAndResetLeft();

  double ticks =  (encLeft = (int16_t)(fooLeft & 0xFFFF));

  double revolutions = ticks / 1800;

  double deltaTime = (now - lastTime1);

  double rps = (revolutions - lastRevolutions1) / (deltaTime / 1000);

  double output = lrad_pid_compute(&pid1, rps, (deltaTime / 1000));

  speedLeft += output;

  if (speedLeft > 350) {
    speedLeft = 350;
  }
  else if (speedLeft  < -350) {
    speedLeft = -350;
  }

  motors.setLeftSpeed(speedLeft);

  lastTime1 = now;
  lastRevolutions1 = revolutions;

  delay(20);

}

void pid_motor2()
{
  unsigned long now = millis();

  fooRight += - encoders.getCountsAndResetRight();

  double ticks = (encRight = (int16_t)(fooRight & 0xFFFF));

  double revolutions = ticks / 1800;

  double deltaTime = (now - lastTime2);

  double rps = (revolutions - lastRevolutions2) / (deltaTime / 1000);

  double output = lrad_pid_compute(&pid2, rps, (deltaTime / 1000));
  speedRight += output;

  if (speedRight > 350) {
    speedRight = 350;
  }
  else if (speedRight  < -350) {
    speedRight = -350;
  }

  motors.setRightSpeed(speedRight);

  lastTime2 = now;
  lastRevolutions2 = revolutions;

  delay(20);
}

template <typename T> float heading(LSM303::vector<T> v)
{
  float x_scaled =  2.0 * (float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0 * (float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled) * 180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

float averageHeading()
{
  LSM303::vector<int32_t> avg = {0, 0, 0};

  for (int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

float relativeHeading(float heading_from, float heading_to)
{
  float relative_heading = heading_to - heading_from;

  if (relative_heading > 180)
  {
    relative_heading -= 360;
  }
  if (relative_heading < -180)
  {
    relative_heading += 360;
  }
  return relative_heading;
}





