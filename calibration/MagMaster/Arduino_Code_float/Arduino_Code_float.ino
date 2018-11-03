
#include "MPU9250_t3.h"

MPU9250 compass(0x68, 1); 

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
  float d[9];
  compass.getMotion9(&d[0], &d[1], &d[2], &d[3], &d[4], &d[5], &d[6], &d[7], &d[8]);
  xv = d[6];
  yv = d[7];
  zv = d[8];
}
