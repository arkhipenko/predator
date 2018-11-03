#include "MPU9250_t3.h"

// 0x68, 1
#define M11 4.064 
#define M12 -0.079
#define M13 -0.064
#define M21 0.054
#define M22 4.711
#define M23 -0.28
#define M31 -0.249
#define M32 0.012
#define M33 4.054 

#define Bx -396.031
#define By 569.815
#define Bz -672.453

// 0x69, 0
//#define M11 6.893
//#define M12 -0.355
//#define M13 0.567
//#define M21 -0.057
//#define M22 6.913
//#define M23 -0.283
//#define M31 -0.822
//#define M32 0.11
//#define M33 6.644
//
//#define Bx -112.344
//#define By 1087.946
//#define Bz 719.295

// 0x69, 0
//const float CMagTM[] = { 6.893, -0.355,  0.567, \
//                        -0.057,  6.913, -0.283, \
//                        -0.822,  0.110,  6.644 };
//const float CMagBias[] = { -112.344, 1087.946, 719.295 };

// 0x68, 1
const float CMagTM[] = {  4.064, -0.079, -0.064, \
                           0.054,  4.711, -0.28, \
                          -0.249,  0.012,  4.054 };
const float CMagBias[] = {-396.031, 569.815, -672.453 };

MPU9250 compass(0x68, 1); 

float xv, yv, zv;

//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibrated_values[3];   
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  float calibration_matrix[3][3] = 
  {
    {M11, M12, M13},
    {M21, M22, M23},
    {M31, M32, M33}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  float bias[3] = 
  {
    Bx,
    By,
    Bz
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

//vector_length_stabilasation() - is the function of the magnetometer vector length stabilasation (stabilisation of the sphere radius)
float scaler;
boolean scaler_flag = false;
float normal_vector_length;
void vector_length_stabilasation(){
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //calculate the current scaler
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

void setup()
{   
  Serial.begin(9600);
  delay(1000);
  int beginStatus = compass.begin(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
  if (beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while (1) {}
  }      
  compass.setMagTMandBias( (float*) &CMagTM[0], (float*) &CMagBias[0] ); 
}

void loop()
{
  float values_from_magnetometer[3];
  float vv[3]; 
  
 Serial.flush(); 

 
  getHeading();
  compass.getMag(&vv[0], &vv[1], &vv[2]);

  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);
  
  vector_length_stabilasation();
    
//  Serial.print(calibrated_values[0]); 
//  Serial.print(",");
//  Serial.print(calibrated_values[1]);
//  Serial.print(",");
//  Serial.print(calibrated_values[2]);
//  Serial.print("\t");
//  
//
//  Serial.print(calibrated_values[0]); 
//  Serial.print(",");
//  Serial.print(calibrated_values[1]);
//  Serial.print(",");
//  Serial.print(calibrated_values[2]);
//  Serial.println();

  Serial.print(calibrated_values[0]); 
  Serial.print(" ");
  Serial.print(vv[0]); 
  Serial.print("\t");
  Serial.print(calibrated_values[1]); 
  Serial.print(" ");
  Serial.print(vv[1]); 
  Serial.print("\t");  
  Serial.print(calibrated_values[2]); 
  Serial.print(" ");
  Serial.print(vv[2]); 
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



