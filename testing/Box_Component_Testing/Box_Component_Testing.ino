#include <Arduino.h>
#include <DFRobotDFPlayerMini.h>
//#include <Wire.h>
#include "i2c_t3.h"
#include <PWMServoDriver_t3.h>

DFRobotDFPlayerMini player;

const int playing = 2;
const int player_address = 0x10;

#define SP  Serial1


// Gun constants --------------------------------------------------------

const int LASER  = 11; // laser pointer
const int GUN    = 12; // gun sound in the toy
const int PLASMA = 10; // flashing front LEDs

const int DLY = 40; // Servo "settle" delay
const int STP = 20; // Servo "step" interval

const int RED = 7;  // Tricolor LED red color
const int GRN = 6;
const int BLU = 5;

const int TILT = 3;
const int PAN = 15;

// Servo control:

// The typical RC servo expects to see a pulse every 20 ms - 50 Hz frequency
// 1000 mcS - -90
// 1250 mcS - -45
// 1500 mcS - Neutral
// 1750 mcS - +45
// 2000 mcS - +90
//

// 50Hz setup:
// 1 ms = 4096/20 = 204.8 = 205
// 1.5 ms = 307.5 = 308
// 2 ms = 409.6 = 410

const int FREQ = 100;                 // ex: 50 or 100 Hz
const int INTR = (1000 / FREQ);      // ex: 20, 10
const int CYCL = (4096 / INTR);      // ex: 204, 409

const int TILTMIN = 1 * CYCL + 1;        // ex: 204 for 50Hz,
const int TILTMED = 15 * CYCL / 10;      // ex: 306, 613
const int TILTMAX = 20 * CYCL / 10 - 1;  // ex: 408, 818
const int DTILT = TILTMED - TILTMIN;

const int PANMIN = TILTMIN;
const int PANMED = TILTMED;
const int PANMAX = TILTMAX;
const int DPAN = PANMED - PANMIN;

PWMServoDriverT3 pwm = PWMServoDriverT3(0x40, 0);


void setupPlayer() {
  SP.begin(9600);
  delay(100);
  while (!SP) ;;
  player.begin(SP);
  delay(100);
  player.reset();
  delay(100);

  //  player.setTimeOut(500);

  //----Set different EQ----
  player.EQ(DFPLAYER_EQ_NORMAL);
  //  player.EQ(DFPLAYER_EQ_POP);
  //  player.EQ(DFPLAYER_EQ_ROCK);
  //  player.EQ(DFPLAYER_EQ_JAZZ);
  //  player.EQ(DFPLAYER_EQ_CLASSIC);
  //  player.EQ(DFPLAYER_EQ_BASS);
  delay(1000);

  //----Set device we use SD as default----
  player.outputDevice(DFPLAYER_DEVICE_SD);
}

const uint8_t _bus = 0;
const uint8_t _pins = I2C_PINS_18_19;
const uint8_t _pullups = I2C_PULLUP_EXT;
#define   _i2cRate  400000

void setup() {
  delay(1000);
  Serial.begin(115200);

  // put your setup code here, to run once:
  pinMode(playing, INPUT);
  setupPlayer();
  Serial.println("Initiated DPPlayer");

  i2c_t3(_bus).begin(I2C_MASTER, 0); //, _pins, _pullups, _i2cRate);
  Serial.print("Intiated i2c_t3");

  pwm.begin();
  pwm.setPWMFreq(FREQ);
  Serial.print("Intiated ServoPWM board");

  delay(1000);

  components_test();

}

void loop() {
  // put your main code here, to run repeatedly:

}


/*
   This routine tests all costume components
*/
const int vol = 30;

void components_test() {
  bool test_ok = false;
  byte c;

  player.volume(vol);  //Set volume value (0~30).
  player.playFolder(99, 001); //001 - Initiating components testing
  delay(100);
  test_ok = ( digitalRead(playing) == LOW );
  Serial.print("test_ok = "); Serial.println (test_ok);

  if ( test_ok) {
    while ( digitalRead(playing) == LOW ) ;
    delay (500);
  }

  // main sound system
  player.volume(vol);  //Set volume value (0~30).
  player.playFolder(99, 100); // main sound system
  delay(200);
  test_ok = ( digitalRead(playing) == LOW );
  if ( test_ok) {
    while ( digitalRead(playing) == LOW ) ;
  }

  // head mc as a i2c slave
  delay (1000);
  playAndWait(99, 101); // head microcontroller

  //  Wire.beginTransmission(player_address);
  i2c_t3(_bus).beginTransmission(player_address);
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  playAndWait(99, test_ok ? 002 : 004 );


  // head sound system
  delay (1000);
  playAndWait(99, 102);

  i2c_t3(_bus).beginTransmission(player_address);
  c = 200;
  i2c_t3(_bus).write(c); // set volume
  c = 0;
  i2c_t3(_bus).write(c); // folder does not matter
  c = 10;
  i2c_t3(_bus).write(c); // volume
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  delay(200);

  i2c_t3(_bus).beginTransmission(player_address);
  c = 1;
  i2c_t3(_bus).write(c); // play single file command
  i2c_t3(_bus).write(c); // play folder 1
  i2c_t3(_bus).write(c); // play file 1
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  delay(200);

  i2c_t3(_bus).requestFrom(player_address, (uint8_t)1);
  delay(10);
  if ( !i2c_t3(_bus).available() ) {
    test_ok = false;
    playAndWait(99, 4);
  }
  else {
    c = i2c_t3(_bus).readByte();
    //    Serial.print("first read c = "); Serial.println(c);
    playAndWait(99, c == 0 ? 2 : 3);
  }

  test_ok = true;
  while ( test_ok ) {
    i2c_t3(_bus).requestFrom(player_address, (uint8_t)1);
    delay(10);
    if ( !i2c_t3(_bus).available() ) break;
    c = i2c_t3(_bus).readByte();
    //    Serial.print("cont read c = "); Serial.println(c);
    test_ok = ( c == 0 );
  }

  // head gyroscope
  delay (1000);
  playAndWait(99, 103);

  //  Wire.beginTransmission(0x69);
  i2c_t3(_bus).beginTransmission(0x68);
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  playAndWait(99, test_ok ? 002 : 004 );


  // head LED's
  delay (1000);
  playAndWait(99, 104);
  i2c_t3(_bus).beginTransmission(player_address);
  c = 231;
  i2c_t3(_bus).write(c); // flash LEDs
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  playAndWait(99, test_ok ? 002 : 004 );
  delay(500);


  // plasma gun PWM board
  delay (1000);
  playAndWait(99, 200);

  i2c_t3(_bus).beginTransmission(0x40);
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  playAndWait(99, test_ok ? 002 : 004 );

  //201 - plasma gun horizontal servo motor
  delay (1000);
  playAndWait(99, 201);
  pwm.setPWM(PAN, 0, PANMED - 60); delay(500);
  pwm.setPWM(PAN, 0, PANMED + 60); delay(500);
  pwm.setPWM(PAN, 0, PANMED); delay(500);


  //  202 - plasma gun vertical servo motor
  delay (1000);
  playAndWait(99, 202);
  pwm.setPWM(TILT, 0, TILTMED - 60); delay(500);
  pwm.setPWM(TILT, 0, TILTMED + 60); delay(500);
  pwm.setPWM(TILT, 0, TILTMED); delay(500);


  // gun gyroscope
  delay (1000);
  playAndWait(99, 207);

  i2c_t3(_bus).beginTransmission(0x69);
  test_ok = ( i2c_t3(_bus).endTransmission() == 0 );
  playAndWait(99, test_ok ? 002 : 004 );



  // 203 - plasma gun three color LED
  delay (1000);
  playAndWait(99, 203);

  int i;
  for (i = 0; i < 4000; i += 50) {
    pwm.setPWM(RED, 0, i);
    delay(1);
  }
  delay (500);
  for (i = 4000; i > 0; i -= 50) {
    pwm.setPWM(RED, 0, i);
    delay(1);
  }
  pwm.setPWM(RED, 0, 4096);

  for (i = 0; i < 4000; i += 50) {
    pwm.setPWM(GRN, 0, i);
    delay(1);
  }
  delay (500);
  for (i = 4000; i > 0; i -= 50) {
    pwm.setPWM(GRN, 0, i);
    delay(1);
  }
  pwm.setPWM(GRN, 0, 4096);

  for (i = 0; i < 4000; i += 50) {
    pwm.setPWM(BLU, 0, i);
    delay(1);
  }
  delay (500);
  for (i = 4000; i > 0; i -= 50) {
    pwm.setPWM(BLU, 0, i);
    delay(1);
  }
  pwm.setPWM(BLU, 0, 4096);




  // 204 - plasma gun laser pointer
  delay (1000);
  playAndWait(99, 204);
  pwm.setPWM(LASER, 4096, 0);
  delay(2000);
  pwm.setPWM(LASER, 0, 4096);


  // 205 - plasma gun front LED assembly
  delay (1000);
  playAndWait(99, 205);

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j <= 4000; j += 400) {
      pwm.setPWM(PLASMA, 0, j);
      delay(2);
    }

    pwm.setPWM(PLASMA, 4096, 0);
    delay(200);

    for (int j = 4000; j >= 0; j -= 40) {
      pwm.setPWM(PLASMA, 0, j);
      delay(1);
    }
    pwm.setPWM(PLASMA, 0, 4096);
    delay(400);
  }


  

  // the end
  delay(2000);
  playAndWait(99, 6 );
  delay(5000);
}

void playAndWait(int folder, int file) {
  player.volume(vol);  //Set volume value (0~30).
  player.playFolder(folder, file);
  delay(200);
  while ( digitalRead(playing) == LOW ) ;
}
