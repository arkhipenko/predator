
#include "MPU9250_t3.h"

MPU9250 compass(0x69, 0); 

float xv, yv, zv;

void setup()
{   
  Serial.begin(9600);
//  delay(1000);
//  Serial.println("Mag Calibration");
  int beginStatus = compass.begin(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
  if (beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while (1) {}
  }     
}

void loop()
{
  getHeading();

  Serial.flush(); 
  Serial.print(xv); 
  Serial.print(",");
  Serial.print(yv);
  Serial.print(",");
  Serial.print(zv);
  Serial.println();

  delay(100); 
} 

 
void getHeading()
{ 
  int16_t d[9];
  compass.getMotion9Counts(&d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7], &d[8]);
  xv = (float)d[6];
  yv = (float)d[7];
  zv = (float)d[8];
}
