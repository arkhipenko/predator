#include "MPU9250_t3.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int16_t buffersize = 500;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int16_t acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int16_t giro_deadzone = 8;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro(0x68, 0);
//MPU9250 accelgyro(0x69, 1);

int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  //  Wire.begin();
  //  // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
  //  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

  // initialize serial communication
  Serial.begin(115200);


  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()) {
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }
  while (Serial.available() && Serial.read()); // empty buffer again


  Serial.println("\nMPU Calibration Sketch");
  delay(2000);
  Serial.println("\nYour MPU should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  // verify connection
  Serial.println(  accelgyro.begin(ACCEL_RANGE_2G, GYRO_RANGE_250DPS) == 0 ? "MPU connection successful" : "MPU connection failed");
  delay(1000);
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  float dest1[3], dest2[3];
  //  if (state == 0) {
  //    Serial.println("\nReading sensors for first time...");
  //    meansensors();
  //    state++;
  //    delay(1000);
  //  }
  //
  //  if (state == 1) {
  //    Serial.println("\nCalculating offsets...");
  //    calibration();
  //    state++;
  //    delay(1000);
  //  }
  //
  //  if (state == 2) {
  //    meansensors();
  Serial.println("\nFINISHED!");
  Serial.print("\nSensor readings with offsets:\t");
  Serial.print(dest1[0],6);
  Serial.print("\t");
  Serial.print(dest1[1],6);
  Serial.print("\t");
  Serial.print(dest1[2],6);
  Serial.print("\t");
  Serial.print(dest2[0],6);
  Serial.print("\t");
  Serial.print(dest2[1],6);
  Serial.print("\t");
  Serial.println(dest2[2],6);
  Serial.println("\nData is printed as: tbd");
  Serial.println("If calibration was succesful write down your offsets so you can set them in your projects.");
  while (1);
  //  }
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
//void meansensors() {
//  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
//
//  while (i < (buffersize + 101)) {
//    // read raw accel/gyro measurements from device
//    accelgyro.getMotion6Counts(&ax, &ay, &az, &gx, &gy, &gz);
//
//    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
//      buff_ax = buff_ax + ax;
//      buff_ay = buff_ay + ay;
//      buff_az = buff_az + az;
//      buff_gx = buff_gx + gx;
//      buff_gy = buff_gy + gy;
//      buff_gz = buff_gz + gz;
//    }
//    if (i == (buffersize + 100)) {
//      mean_ax = buff_ax / buffersize;
//      mean_ay = buff_ay / buffersize;
//      mean_az = buff_az / buffersize;
//      mean_gx = buff_gx / buffersize;
//      mean_gy = buff_gy / buffersize;
//      mean_gz = buff_gz / buffersize;
//    }
//    i++;
//    delay(2); //Needed so we don't get repeated measures
//  }
//}
//
//void calibration() {
//  int16_t offs[6];
//  ax_offset = -mean_ax / 8;
//  ay_offset = -mean_ay / 8;
//  az_offset = (16384 - mean_az) / 8;
//  //
//  //  gx_offset = -mean_gx / 4;
//  //  gy_offset = -mean_gy / 4;
//  //  gz_offset = -mean_gz / 4;
//
//  gx_offset = -mean_gx / 8;
//  gy_offset = -mean_gy / 8;
//  gz_offset = -mean_gz / 8;
//
//  while (1) {
//    int ready = 0;
//    offs[0] = ax_offset;
//    offs[1] = ay_offset;
//    offs[2] = az_offset;
//    offs[3] = gx_offset;
//    offs[4] = gy_offset;
//    offs[5] = gz_offset;
//    accelgyro.setOffsets(offs);
//
//    meansensors();
//    Serial.println("...");
//
//    if (abs(mean_ax) <= acel_deadzone) ready |= 1;
//    else ax_offset = ax_offset - mean_ax / acel_deadzone;
//
//    if (abs(mean_ay) <= acel_deadzone) ready |= 0b10;
//    else ay_offset = ay_offset - mean_ay / acel_deadzone;
//
//    if (abs(16384 - mean_az) <= acel_deadzone) ready |= 0b100;
//    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
//
//    if (abs(mean_gx) <= giro_deadzone) ready  |= 0b1000;
//    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);
//
//    if (abs(mean_gy) <= giro_deadzone) ready |= 0b10000;
//    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);
//
//    if (abs(mean_gz) <= giro_deadzone) ready |= 0b100000;
//    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);
//
//    if (ready == 0b111111) break;
//    Serial.print("Mean ax="); Serial.println(mean_ax);
//    Serial.print("Mean ay="); Serial.println(mean_ay);
//    Serial.print("Mean az="); Serial.println(mean_az);
//    Serial.print("Mean gx="); Serial.println(mean_gx);
//    Serial.print("Mean gy="); Serial.println(mean_gy);
//    Serial.print("Mean gz="); Serial.println(mean_gz);
//
//    accelgyro.getOffsets(offs);
//    Serial.print("Offset ax="); Serial.print(ax_offset); Serial.print("\t"); Serial.println(offs[0]);
//    Serial.print("Offset ay="); Serial.print(ay_offset); Serial.print("\t"); Serial.println(offs[1]);
//    Serial.print("Offset az="); Serial.print(az_offset); Serial.print("\t"); Serial.println(offs[2]);
//    Serial.print("Offset gx="); Serial.print(gx_offset); Serial.print("\t"); Serial.println(offs[3]);
//    Serial.print("Offset gy="); Serial.print(gy_offset); Serial.print("\t"); Serial.println(offs[4]);
//    Serial.print("Offset gz="); Serial.print(gz_offset); Serial.print("\t"); Serial.println(offs[5]);
//  }
//}


void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  //  // shoot for ~fifteen seconds of mag data
  //  if (MPU9250Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
  //  if (MPU9250Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms

  sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
  for (ii = 0; ii < sample_count; ii++) {
    accelgyro.getMotion9Counts(&ax, &ay, &az, &gx, &gy, &gz, &mag_temp[0], &mag_temp[1], &mag_temp[2]);

    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    //    if (MPU9250Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    //    if (MPU9250Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
    delay(10);
  }


  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * MPU9250mRes * MPU9250magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * MPU9250mRes * MPU9250magCalibration[1];
  dest1[2] = (float) mag_bias[2] * MPU9250mRes * MPU9250magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println("Mag Calibration done!");
}
