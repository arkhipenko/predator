#include "MPU9250_t3.h"

// 0x68, 1
#define M11 2.831
#define M12 -0.067
#define M13 -0.18
#define M21 -0.037
#define M22 3.524
#define M23 0.128
#define M31 -0.044
#define M32 -0.092
#define M33 3.335 

#define Bx -55.675
#define By 91.405
#define Bz -128.844

// 0x69, 0
//#define M11 4.707
//#define M12 0.005
//#define M13 0.474
//#define M21 0.032
//#define M22 5.118
//#define M23 -0.189
//#define M31 -0.022
//#define M32 -0.28
//#define M33 5.123
//
//#define Bx -15.612
//#define By 200.022
//#define Bz 114.4

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
}

void loop()
{
  float values_from_magnetometer[3];
  
  getHeading();
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);
  
  vector_length_stabilasation();

  Serial.flush(); 
  Serial.print(calibrated_values[0]); 
  Serial.print(",");
  Serial.print(calibrated_values[1]);
  Serial.print(",");
  Serial.print(calibrated_values[2]);
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



