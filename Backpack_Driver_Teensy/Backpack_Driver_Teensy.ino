#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_PRIORITY
//#define _TASK_TIMECRITICAL
#include "TaskScheduler.h"

#include "PWMServoDriver_t3.h"
#include "AvgFilter.h"
#include "MPU9250_t3.h"
#include "i2c_t3.h"

#include "DFPlayerMini_t3.h"
#include "QueueArray.h"
//#include "VirtualWire.h"

// Do not forget to uncomment #define I2C_AUTORETRY
// around line #102 of the i2c_t3.h file

#include <predator_sounds.h>
#include <predator_commands.h>
#include <predator_head_commands.h>



// Debug and Test options
//#define _DEBUG_
//#define _DISPLAY_
//#define _TEST_

#ifdef _DEBUG_
#define INITIAL_VOLUME 10
#else
#define INITIAL_VOLUME 25
#endif

#ifdef _DEBUG_
#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);
#else
#define _PP(a)
#define _PL(a)
#endif

// Gun constants --------------------------------------------------------

const int LASER  = 11; // laser pointer pwm pin
const int GUN    = 12; // gun sound in the toy pwm pin  NOT USED
const int PLASMA = 10; // flashing front LEDs pwm pin

const int DLY = 40; // Servo "settle" delay
const int STP = 20; // Servo "step" interval

const int RED = 7;  // Tricolor LED red color pin on PWM driver
const int GRN = 6;  // Tricolor LED green color pin on PWM driver
const int BLU = 5;  // Tricolor LED blue color pin on PWM driver

const int TILT = 3; // Pin for Tilt Servo control
const int PAN = 15; // Pin for Pan servo control


// Servo control:

// The typical RC servo expects to see a pulse every 20 ms - 50 Hz frequency
// Pulse width and rotation angles:
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


// ============================================================================
const int PWM_SERVO_ADDR = 0x40;
PWMServoDriverT3 pwm = PWMServoDriverT3(PWM_SERVO_ADDR, 0);

// Acclelrometer and magnetic offsets need to be calibrated for each IMU unit
// individually.

// IMU unit in the Gun:
// 0x69, 0
const int16_t COffsetsGun[] = {5594,  3188,  8798,  31,  -11, 54};
const float CMagTMGun[] =    {  4.707,   0.005,    0.474, \
                                0.032,   5.118,   -0.189, \
                                -0.022,  -0.28,     5.123
                             };
const float CMagBiasGun[] = { -15.612, 200.022,  114.4 };

// IMU unit in the Helmet:
// 0x68, 1
const int16_t COffsetsHead[] = {4865, -5359, 8077,  1, 43,  0};
const float CMagTMHead[]   = {  2.831,  -0.067,   -0.18, \
                                -0.037,   3.524,    0.128, \
                                -0.044,  -0.092,    3.335
                             };
const float CMagBiasHead[] = { -55.675,  91.405, -128.844 };

const float QT[] = { 1.0, 0.0, 0.0, 0.0 };

// an MPU9250 object with its I2C address
// of 0x68 (ADDR to GRND) and on Teensy bus 0
const int IMU_GUN_ADDR = 0x69;
const int IMU_HEAD_ADDR = 0x68;

MPU9250 IMUGun (IMU_GUN_ADDR, 0); //, I2C_PINS_18_19, I2C_PULLUP_EXT);
MPU9250 IMUHead(IMU_HEAD_ADDR, 0); //, I2C_PINS_18_19, I2C_PULLUP_EXT);

enum sound_target : byte { SOUND_BOX, SOUND_HEAD };
enum sound_control : byte {SOUND_ONCE = 0b00000001,
                           SOUND_CONT = 0b00000010,
                           SOUND_CNXT = 0b00000100,
                           SOUND_RAND = 0b00001000
                          };

enum gun_state : byte { GUN_INACTIVE        = 0b00000001,
                        GUN_CALIBRATING     = 0b00000010,
                        GUN_ACTIVE          = 0b00000100,
                        GUN_ACTIVE_TRACKING = 0b00001000,
                        GUN_ACTIVE_HOLDING  = 0b00010000
                      };

enum laser_state : byte { LASER_OFF   = 0b00000001,
                          LASER_ON    = 0b00010000,
                          LASER_HOLD  = 0b00100000,
                          LASER_SOUND = 0b10000000
                        };

enum tracking_method : byte { TRACKING_IMU, TRACKING_MARG };

byte gunState;
byte laserState;
bool bladesOut = false;
byte trackingMode;


#ifdef _DEBUG_
long          tHeadFV[100], tGunFV[100];
avgFilter     tHeadF(100, tHeadFV), tGunF(100, tGunFV);
#endif

unsigned long tHead, tGun;
float         aGun = 0.0, aHead = 0.0;
float         gunRotations = 0.0, headRotations = 0.0;

float         tiltHead, tiltGun, panHead, panGun, prevPanHead, prevPanGun;
float         tiltGunHeld, panGunHeld;
float         panGunCalib, panHeadCalib;
float         tiltGunCalib, tiltHeadCalib;

long          deltaTilt, deltaPan;
int           targetTilt, targetPan;

const int     ANGLE_ADJ = 10;
const int     ANGLE_ADJ_ROUGH   = 2;
const int     ANGLE_ADJ_PRECISE = 4;

const int     FILTER_DEPTH = 10;
long          tiltFV[FILTER_DEPTH], panFV[FILTER_DEPTH];
avgFilter     tiltF(FILTER_DEPTH, tiltFV), panF(FILTER_DEPTH, panFV);
int           soundVolume;

DFPlayerMini  player;
const int     SOUND_CARD_DELAY = 100;
const int     SOUND_PLAYING_PIN = 2;
const int     HEAD_MC_ADDR = 0x10;
const int     GUN_SHOT_INTERVAL = 20; // ms
const int     GUN_SHOT_LENGTH = 3000; // ms
const int     GUN_BURST_LENGTH = 1100; // ms
const int     GLOW_INTERVAL = 50; // ms

// VirtualWire parameters:
//========================
//const int RX_PIN = 35;
#define BTserial  Serial5

const int TX_RATE = 9600;
const int TX_TIMEOUT = 50; // ms

const int HEARTBEAT_PERIOD = 1000; // ms
const int HEARTBEAT_ERR_COUNT = 3; // if hearbeat command is missing twice the hearbeat interval this many times - beep
int heartbeatCount = 0;            // number of missed hearbeat packets
unsigned long lastHeartbeat;       // millis of the last hearbeat received

const int I2C_BUS = 0;

typedef struct  {
  byte  cmd;
  //  int  pload[2];
} TransmitBuffer;

TransmitBuffer r_buf;

#define BUF_LEN 128
byte radio_buf[BUF_LEN];

uint8_t     radio_len;
QueueArray  <TransmitBuffer> q;
uint8_t     rx_errors;


void measureCallbackIMU();
void measureCallbackMARG();
void anglesCallback();
void servoCallback();
void initOnDisable();
void playSound(byte aTarget, byte aFolder, byte aSound, byte aControl = SOUND_ONCE);
void dsrCallback();
void commandCallback();

bool glowGreenOE();
void glowGreenOD();
void glowGreen();

bool glowRedOE();
void glowRedOD();
void glowRed();

bool glowBlueOE();
void glowBlueOD();
void glowBlue();

bool gunShotOE();
void gunShotOD();
void gunShot();

bool gunBurstOE();
void gunBurstOD();
void gunBurst();

void playSoundCallback();
void restartSoundCallback();
void iterateCallback();
void calibrateOD();

void heartBeat();

// ================= TASKS =====================
Scheduler ts, hts;

Task  tMeasure        (2 * TASK_MILLISECOND, TASK_FOREVER, &measureCallbackIMU,   &hts, false);
Task  tAngles         (20 * TASK_MILLISECOND, TASK_FOREVER, &anglesCallback,    &ts, false);
Task  tServo          (STP * TASK_MILLISECOND, TASK_FOREVER, &servoCallback,    &ts, false);

Task  tSound          (SOUND_CARD_DELAY * TASK_MILLISECOND, 2, &playSoundCallback, &ts, false);
Task  tSoundRestarter (TASK_IMMEDIATE, TASK_ONCE, &restartSoundCallback, &ts, false);
Task  tPlayerIterator(5 * TASK_MILLISECOND, TASK_FOREVER, &iterateCallback, &ts, false);

Task  tDSRPoll        (10 * TASK_MILLISECOND, TASK_FOREVER, &dsrCallback, &hts, false);
Task  tProcessCommand (TASK_IMMEDIATE, TASK_FOREVER, &commandCallback, &ts, false);
Task  tHeartbeat      (TASK_SECOND, TASK_FOREVER, &heartBeat, &ts, false);

Task  tGunShot        (GUN_SHOT_INTERVAL * TASK_MILLISECOND, GUN_SHOT_LENGTH / GUN_SHOT_INTERVAL, &gunShot, &ts, false, &gunShotOE, &gunShotOD);
Task  tGunBurst       (GUN_SHOT_INTERVAL * TASK_MILLISECOND, GUN_BURST_LENGTH / GUN_SHOT_INTERVAL, &gunBurst, &ts, false, &gunBurstOE, &gunBurstOD);
Task  tGlowGreen      (GLOW_INTERVAL * TASK_MILLISECOND, TASK_FOREVER, &glowGreen, &ts, false, &glowGreenOE, &glowGreenOD);
Task  tGlowRed        (GLOW_INTERVAL * TASK_MILLISECOND, TASK_FOREVER, &glowRed, &ts, false, &glowRedOE, &glowRedOD);
Task  tGlowBlue       (GLOW_INTERVAL * TASK_MILLISECOND, TASK_FOREVER, &glowBlue, &ts, false, &glowBlueOE, &glowBlueOD);
Task  tCalibrate      (10 * TASK_MILLISECOND, FILTER_DEPTH * 10, NULL, &ts, false, NULL, &calibrateOD);

#ifdef _DISPLAY_
void displayCallback();
Task  tDisplay (500 * TASK_MILLISECOND, TASK_FOREVER, &displayCallback, &ts, false);
#endif





// ============== CODE ============================
// Main setup should just dispatch setup jobs
void setup_serial() {
  // serial to display data
  Serial.begin(115200);
  delay(1000);
  //  while (!Serial) ;  // do not uncomment in case serial is not connected it will hang here
}

void setup_pins() {
  pinMode (13, OUTPUT);
  //  pinMode(3, OUTPUT);
  pinMode(SOUND_PLAYING_PIN, INPUT);
  //  pinMode(RX_PIN, INPUT);

  randomSeed(analogRead(0));
}

bool setup_pwm() {
  bool rc;

  i2c_t3(I2C_BUS).beginTransmission(PWM_SERVO_ADDR);
  rc = ( i2c_t3(I2C_BUS).endTransmission() == 0 );
  if (!rc ) return rc;

  pwm.begin();
  pwm.setPWMFreq(FREQ);

  reset_gun();
  return true;
}

void reset_gun() {
  //  pwm.setPWM(TILT, 0, TILTMIN); delay(50);

  _PL("Move servos in the middle position");
  pwm.setPWM(PAN, 0, PANMED); delay(DLY);
  pwm.setPWM(TILT, 0, TILTMED); delay(DLY);

  _PL("Turn Laser off");
  pwm.setPWM(LASER, 0, 4096); delay(DLY);

  _PL("Turn Gun Sound off");
  pwm.setPWM(GUN, 0, 4096); delay(DLY);

  _PL("Turn Gun Plasma Light off");
  pwm.setPWM(PLASMA, 0, 4096); delay(DLY);

  _PL("Turn Glowing LED Light off");
  pwm.setPWM(RED, 0, 4096); delay(DLY);
  pwm.setPWM(GRN, 0, 4096); delay(DLY);
  pwm.setPWM(BLU, 0, 4096); delay(DLY);

  // Sending servos to the middle position again
  // just in case. Should not be needed
  delay(3 * DLY);
  pwm.setPWM(PAN, 0, PANMED); delay(DLY);
  pwm.setPWM(TILT, 0, TILTMED); delay(DLY);

  for (int i = 0; i < FILTER_DEPTH; i++) {
    tiltF.value(TILTMED);
    panF.value(PANMED);
  }
}

bool setup_head_imu() {

  // start communication with IMU and
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS

  int rc;

  rc = IMUHead.begin(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
  if (rc < 0) {
    _PL("IMUHead initialization unsuccessful");
    _PL("Check IMUHead wiring or try cycling power");
    return false;
  }
  else {
    //  IMUHead.setTransformMatrix( (int16_t**) CTMHead );
    //  IMUHead.setFilt(DLPF_BANDWIDTH_41HZ, 4);
    //  IMUHead.setFilt(DLPF_BANDWIDTH_184HZ, 2);
    IMUHead.enableInt(true);
    if ( trackingMode == TRACKING_IMU) {
      IMUHead.setBeta(0.033); // for IMU
    }
    else {
      IMUHead.setBeta(0.041); // for MARG
    }

    IMUHead.setOffsets( (int16_t*) &COffsetsHead[0]);
    IMUHead.setMagTMandBias( (float*) &CMagTMHead[0], (float*) &CMagBiasHead[0] );
  }
  return true;
}

bool setup_gun_imu() {

  // start communication with IMU and
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS

  int rc;

  rc = IMUGun.begin(ACCEL_RANGE_2G, GYRO_RANGE_250DPS);
  if (rc < 0) {
    _PL("IMUGun initialization unsuccessful");
    _PL("Check IMUGun wiring or try cycling power");
    return false;
  }
  else {
    //  IMUGun.setTransformMatrix( (int16_t**) CTMGun );
    //  IMUGun.setFilt(DLPF_BANDWIDTH_41HZ, 4);
    //  IMUGun.setFilt(DLPF_BANDWIDTH_184HZ, 2);
    IMUGun.enableInt(true);
    if ( trackingMode == TRACKING_IMU) {
      IMUGun.setBeta(0.033); // for IMU
    }
    else {
      IMUGun.setBeta(0.041); // for MARG
    }

    IMUGun.setOffsets( (int16_t*) &COffsetsGun[0]);
    IMUGun.setMagTMandBias( (float*) &CMagTMGun[0], (float*) &CMagBiasGun[0] );
  }

  return true;
}


#define BOX_SOUND_SERIAL  Serial1
void setup_gun_sound() {
  BOX_SOUND_SERIAL.begin(9600);
  delay(100);
  while (!BOX_SOUND_SERIAL) ;;
  player.begin(BOX_SOUND_SERIAL, false, true);

  //  player.setTimeOut(100); //?
  //  player.disableACK();

  //----Set different EQ----
  player.EQ(DFPLAYER_EQ_NORMAL);
  //  player.EQ(DFPLAYER_EQ_POP);
  //  player.EQ(DFPLAYER_EQ_ROCK);
  //  player.EQ(DFPLAYER_EQ_JAZZ);
  //  player.EQ(DFPLAYER_EQ_CLASSIC);
  //  player.EQ(DFPLAYER_EQ_BASS);
  player.completeCommand();
  delay(100);

  //----Set device we use SD as default----
  player.outputDevice(DFPLAYER_DEVICE_SD);
  player.completeCommand();
  delay(100);
}

bool setup_head_sound() {
  int rc;
  i2c_t3(I2C_BUS).begin(I2C_MASTER, 0); //, _pins, _pullups, _i2cRate);
  _PL("DONE: i2c_t3(I2C_BUS).begin(I2C_MASTER, 0)");
  i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
  _PL("DONE: i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR)");
  rc = i2c_t3(I2C_BUS).endTransmission();
  _PL("DONE: rc = i2c_t3(I2C_BUS).endTransmission()");
  return (rc == 0);
}

void setup_radio() {

  BTserial.begin(TX_RATE);
  delay(100);
  while (!BTserial) ;;
  _PL("BT serial setup complete");
}

// ========== SETUP =============
void setup() {
  bool rc;

#if defined(_DEBUG_) || defined(_TEST_)
  setup_serial();
#endif

  _PL("HeadTracking Box Driver");
#if defined(_DEBUG_) || defined(_TEST_)
  Serial.flush();
  delay(2000);
#endif

  _PL("Setting up pins");
  setup_pins();

  _PL("Setting up gun sound system");
  setup_gun_sound();

  soundVolume = INITIAL_VOLUME;
  playSound(SOUND_BOX, S_FLD_SYSTEM, S_MAC_BOOT, SOUND_ONCE); playSoundCallback();
  delay(2500);

  //  playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_HP_BEEP, SOUND_ONCE);    // first beep - box sound system
  //  delay(1000);

  _PL("Setting up head sound system");
  rc = setup_head_sound();

#ifndef _DEBUG_
  playSound(SOUND_HEAD, S_FLD_SYSTEM, S_STARTING_UP, SOUND_ONCE); playSoundCallback();
  delay(1000);
#endif
  playSound(SOUND_BOX, S_FLD_TESTING, rc ? S_TEST_HP_BEEP : S_TEST_ERROR, SOUND_ONCE); playSoundCallback(); // first beep - head microcontroller
  delay(1000);
  while (!rc) ;

  _PL("Setting up gun PWM controller");
  rc = setup_pwm();
  playSound(SOUND_BOX, S_FLD_TESTING, rc ? S_TEST_HP_BEEP : S_TEST_ERROR, SOUND_ONCE); playSoundCallback(); // second beed - PWM board
  delay(100); // 900 ms is spent in the routine itself
  while (!rc) ;

  //  for now always start in IMU mode. later may change to remember the last state
  trackingMode = TRACKING_IMU;
  //  loadConfig();
  if ( trackingMode == TRACKING_IMU ) {
    tMeasure.setCallback(&measureCallbackIMU);
  }
  else {
    tMeasure.setCallback(&measureCallbackMARG);
  }

  _PL("Setting up gun IMU");
  rc = setup_gun_imu();
  playSound(SOUND_BOX, S_FLD_TESTING, rc ? S_TEST_HP_BEEP : S_TEST_ERROR, SOUND_ONCE);  playSoundCallback(); // third beep - gun IMU
  delay(800);
  while (!rc) ;

  _PL("Setting up head IMU");
  rc = setup_head_imu();
  playSound(SOUND_BOX, S_FLD_TESTING, rc ? S_TEST_HP_BEEP : S_TEST_ERROR, SOUND_ONCE); playSoundCallback(); // forht beep - head IMU
  delay(800);
  while (!rc) ;

  setup_radio();

  ts.setHighPriorityScheduler(&hts);

  playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_LP_BEEP, SOUND_ONCE);  playSoundCallback(); // final long beep - success
  delay(1000);
#ifndef _DEBUG_
  playSound(SOUND_HEAD, S_FLD_SYSTEM, S_ALL_SET, SOUND_ONCE); playSoundCallback();
#endif

  gunState = GUN_INACTIVE;
  r_buf.cmd = C_GUN_OFF;
  q.push(r_buf);

  //  ts.startNow();
  ts.disableAll(true);
  tMeasure.enable();
  tAngles.enable();
  tServo.enable();
  tDSRPoll.enable();
  tProcessCommand.enable();
  tHeartbeat.enable();

#ifdef _DISPLAY_
  tDisplay.enable();
#endif
}
// =============== END SETUP =================


void loop() {
  ts.execute();
}

void activateTargeting() {
  tMeasure.enable();
}

void deactivateTargeting() {
  tMeasure.disable();
}

void laser_off(bool aForce = false) {
  if ( aForce || (laserState & LASER_ON) ) {
    laser_sound_off();
    pwm.setPWM(LASER, 0, 4096);
    sendHeadCommand(C_LASERS_OFF);
  }
  laserState = LASER_OFF;
}
void laser_sound_off() {
  if (laserState & LASER_SOUND) {
    detachInterrupt(digitalPinToInterrupt(SOUND_PLAYING_PIN));
    player.pause();
    tPlayerIterator.restart();
    laserState ^= LASER_SOUND;
  }
}

void measureCallbackIMU() {

  unsigned long ct;
  float dt;

  if (IMUHead.isAccelGyroDataReady()) {
    ct = micros();
    dt = (ct - tHead) / 1000000.0f;
    IMUHead.MadgwickQuaternionUpdateIMU( dt );
#ifdef _DEBUG_
    tHeadF.value((long) (ct - tHead));
#endif
    tHead = ct;
  }


  if (IMUGun.isAccelGyroDataReady()) {
    ct = micros();
    dt = (ct - tGun) / 1000000.0f;
    IMUGun.MadgwickQuaternionUpdateIMU( dt );
#ifdef _DEBUG_
    tGunF.value((long) (ct - tGun));
#endif
    tGun = ct;
  }
}

void measureCallbackMARG() {

  unsigned long ct;
  float dt;

  if (IMUHead.isAccelGyroDataReady()) {
    ct = micros();
    dt = (ct - tHead) / 1000000.0f;
    IMUHead.MadgwickQuaternionUpdate( dt );
#ifdef _DEBUG_
    tHeadF.value((long) (ct - tHead));
#endif
    tHead = ct;
  }


  if (IMUGun.isAccelGyroDataReady()) {
    ct = micros();
    dt = (ct - tGun) / 1000000.0f;
    IMUHead.MadgwickQuaternionUpdate( dt );
#ifdef _DEBUG_
    tGunF.value((long) (ct - tGun));
#endif
    tGun = ct;
  }
}

void anglesCallback() {

  //  roll:
  //  up: 270  (2500)
  //  level: 180  (1500)
  //  down: 90 (500)

  if ( gunState == GUN_INACTIVE ) {
    targetTilt = TILTMIN;
    targetPan  = PANMED;
  }
  else if ( gunState == GUN_CALIBRATING ) {
    targetTilt = TILTMED;
    targetPan  = PANMED;
  }
  else {

    tiltHead = (laserState & LASER_HOLD) ? tiltGunHeld : IMUHead.getRoll();
    tiltGun = IMUGun.getRoll();
    deltaTilt = (long) (tiltGun - tiltHead);

    int a_adj = abs(deltaTilt) > ANGLE_ADJ ? ANGLE_ADJ_ROUGH : ANGLE_ADJ_PRECISE;
    targetTilt = tiltF.currentValue() + map (deltaTilt, -45, 45, -DTILT, DTILT) / a_adj;
    targetTilt = constrain(targetTilt, TILTMIN, TILTMAX);

    panHead = (laserState & LASER_HOLD) ? panGunHeld : IMUHead.getYaw();
    if ( panHead - prevPanHead > 180 ) headRotations -= 360.0;
    if ( panHead - prevPanHead < -180 ) headRotations += 360.0;
    prevPanHead = panHead;
    aHead = panHead + headRotations;

    panGun = IMUGun.getYaw();
    if ( panGun - prevPanGun > 180 ) gunRotations -= 360.0;
    if ( panGun - prevPanGun < -180 ) gunRotations += 360.0;
    prevPanGun = panGun;
    aGun = panGun + gunRotations;

    deltaPan = (long) (aHead - aGun);
    if (trackingMode == TRACKING_MARG) deltaPan += (panGunCalib - panHeadCalib);

    a_adj = abs(deltaPan) > ANGLE_ADJ ? ANGLE_ADJ_ROUGH : ANGLE_ADJ_PRECISE;
    targetPan = panF.currentValue() + map (deltaPan, -45, 45, -DPAN, DPAN) / a_adj;
    targetPan = constrain(targetPan, PANMIN, PANMAX);
  }
  tiltF.value(targetTilt);
  panF.value(targetPan);
}

void servoCallback() {
  //  We do not know what the inclination of the base of the gun is.
  //  Therefore the strategy should be to try and move in the direction of the difference
  //  between head and gun inclination to try to eliminate it.
  //  We are going to request a fraction of the angular difference move

  pwm.setPWM( TILT, 0, tiltF.currentValue() );

  pwm.setPWM( PAN, 0, panF.currentValue() );
}

void calibrateOD() {
  aGun = 0.0;  gunRotations = 0.0;
  aHead = 0.0; headRotations = 0.0;
  panHead = panGun = prevPanHead = prevPanGun = 0.0;

  if ( trackingMode == TRACKING_IMU ) {
    IMUGun.resetQuaternion( (float *) QT );
    IMUHead.resetQuaternion( (float *) QT );
  }
  else {
    panHeadCalib = IMUHead.getYaw();
    panGunCalib = IMUGun.getYaw();
  }

  gunState = GUN_ACTIVE;
}

void sendHeadCommand (byte aCommand) {
  i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
  send_byte(aCommand);
  i2c_t3(I2C_BUS).endTransmission();
}

byte _psTarget, _psFolder, _psSound, _psControl, _psFolderBox, _psSoundBox;
void playSound(byte aTarget, byte aFolder, byte aSound, byte aControl) {
  _psTarget = aTarget;
  _psFolder = aFolder;
  _psSound = aSound;
  _psControl = aControl;
  tSound.restart();
}

void restartSound() {
  tSoundRestarter.restart();
}

void restartSoundCallback() {
  player.playFolder(_psFolderBox, _psSoundBox);
  tPlayerIterator.restart();
}

void iterateCallback() {
  if ( player.commandCompleted() ) tPlayerIterator.disable();
  //  _PL("iterateCallback");
}

void playSoundCallback() {
  switch ( tSound.getRunCounter() ) {
    case 0:  // this is for setup() only
      if ( _psTarget == SOUND_BOX ) {
        player.volume(soundVolume);  //Set volume value (0~30).
        player.completeCommand();
        delay(SOUND_CARD_DELAY);
        if ( _psControl & SOUND_ONCE ) {
          player.playFolder(_psFolder, _psSound);
          player.completeCommand();
        }
        else if ( _psControl & SOUND_RAND ) {
          player.playFolder(_psFolder, random(_psSound) + 1);
          player.completeCommand();
        }
      }
      else if ( _psTarget == SOUND_HEAD ) {
        i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
        send_byte(C_SET_VOLUME);
        send_byte(0);
        send_byte(soundVolume);
        i2c_t3(I2C_BUS).endTransmission();
        delay(SOUND_CARD_DELAY);
        if ( _psControl & SOUND_ONCE ) {
          i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
          send_byte(C_PLAY_FILE_ONCE);
          send_byte(_psFolder);
          send_byte(_psSound);
          i2c_t3(I2C_BUS).endTransmission();
        }
        else if (_psControl & SOUND_RAND ) {
          i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
          send_byte(C_PLAY_RANDOM_FILE);
          send_byte(_psFolder);
          send_byte(_psSound);
          i2c_t3(I2C_BUS).endTransmission();
        }
      }
      break;

    case 1: // volume
      if ( _psTarget == SOUND_BOX ) {
        player.volume(soundVolume);  //Set volume value (0~30).
        tPlayerIterator.restart();
      }
      else if ( _psTarget == SOUND_HEAD ) {
        i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
        send_byte(C_SET_VOLUME);
        send_byte(0);
        send_byte(soundVolume);
        i2c_t3(I2C_BUS).endTransmission();
      }
      break;
    case 2: // sound
      if ( _psTarget == SOUND_BOX ) {
        if ( _psControl & SOUND_ONCE ) {
          player.playFolder(_psFolder, _psSound);
          tPlayerIterator.restart();
          if (_psControl & SOUND_CONT ) { // play this or next file continously
            attachInterrupt(digitalPinToInterrupt(SOUND_PLAYING_PIN), restartSound, RISING);
            _psFolderBox = _psFolder;
            _psSoundBox = _psSound;
          }
          if (_psControl & SOUND_CNXT ) { // play this or next file continously
            _psFolderBox = _psFolder;
            _psSoundBox = _psSound + 1;
            attachInterrupt(digitalPinToInterrupt(SOUND_PLAYING_PIN), restartSound, RISING);
          }
        }
        else if ( _psControl & SOUND_RAND ) {
          player.playFolder(_psFolder, random(_psSound) + 1);
          tPlayerIterator.restart();
        }
      }
      else if ( _psTarget == SOUND_HEAD ) {
        if ( _psControl & SOUND_ONCE ) {
          i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
          send_byte(C_PLAY_FILE_ONCE);
          send_byte(_psFolder);
          send_byte(_psSound);
          i2c_t3(I2C_BUS).endTransmission();
        }
        else if (_psControl & SOUND_RAND ) {
          i2c_t3(I2C_BUS).beginTransmission(HEAD_MC_ADDR);
          send_byte(C_PLAY_RANDOM_FILE);
          send_byte(_psFolder);
          send_byte(_psSound);
          i2c_t3(I2C_BUS).endTransmission();
        }
      }
  }
}

void send_byte(int aByte) {
  i2c_t3(I2C_BUS).write(aByte); // set volume
}


void dsrCallback() {
  //  _PL("dsrCallback");
  radio_len = BTserial.available();
  if ( radio_len == 0 ) return;

  _PP("radio_len="); _PL(radio_len);

  for (int i = 0; i < radio_len; i++) {
    r_buf.cmd = BTserial.read();
    if ( r_buf.cmd < 0 ) {
      rx_errors = true;
      break;
    }
    else {
      q.push(r_buf);
    }
  }
  tProcessCommand.enableIfNot();
}

void heartBeat() {
  if ( (millis() - lastHeartbeat) > (HEARTBEAT_PERIOD + HEARTBEAT_PERIOD) ) {
    if ( ++heartbeatCount > HEARTBEAT_ERR_COUNT ) {
      heartbeatCount = 0;
      playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_HP_BEEP, SOUND_ONCE);
    }
  }
  else {
    heartbeatCount = 0;
  }
}

void commandCallback() {
  if ( q.isEmpty() ) {
    tProcessCommand.disable();
    return;
  }
  TransmitBuffer bf = q.pop();
  switch (bf.cmd) {
    case C_NONE:            // a "nop" command - used for heartbeat
      _PL("C_NONE");
      lastHeartbeat = millis();
      break;

    case C_BLADES_OUT:
      if (!bladesOut && !(gunState & GUN_ACTIVE) ) {
        playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_BLADES_OUT, SOUND_ONCE);
        bladesOut = true;
      }
      break;

    case C_BLADES_IN:
      if (bladesOut) {
        playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_BLADES_IN, SOUND_ONCE);
        bladesOut = false;
      }
      break;

    case C_GUN_OFF:         // jN click: turn gun off, rotate into center yaw, tilt fwd down, stop tracking
      _PL("C_GUN_OFF");
      //      player.pause();
      //      player.completeCommand();
      bladesOut = false;
      tGunShot.disable();
      tGunBurst.disable();
      tGlowRed.disable();
      tGlowBlue.disable();
      deactivateTargeting();
      laser_off(true);
      if (gunState != GUN_INACTIVE) {
        playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_GUN_PWRDOWN, SOUND_ONCE);
      }
      tGlowGreen.restart();
      gunState = GUN_INACTIVE;
      break;

    case C_GUN_RECAL:       // jS click: turn gun on (if was off), recenter and sync with head IMU (head should be positioned fwd centered).
      _PL("C_GUN_RECAL");

      if ( !(gunState & GUN_ACTIVE) ) {
        tGlowGreen.disable();
        playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_GUN_ACTIVE_END, SOUND_RAND);
        laser_off(true);
        activateTargeting();
        tGlowBlue.restart();
      }
      gunState = GUN_CALIBRATING;
      tCalibrate.restart();
      break;

    case C_TRACK_IMU:
      if ( !(gunState & GUN_ACTIVE) && trackingMode == TRACKING_MARG ) {
        trackingMode = TRACKING_IMU;
        IMUHead.setBeta(0.033); // for IMU
        IMUGun.setBeta(0.033); // for IMU
        tMeasure.setCallback(&measureCallbackIMU);
        playSound(SOUND_BOX, S_FLD_TESTING, S_TRACK_IMU, SOUND_ONCE);
      }
      break;

    case C_TRACK_MARG:
      //      if ( !(gunState & GUN_ACTIVE) && trackingMode == TRACKING_IMU ) {
      //        trackingMode = TRACKING_MARG;
      //        IMUHead.setBeta(0.041);
      //        IMUGun.setBeta(0.041);
      //        tMeasure.setCallback(&measureCallbackMARG);
      //        playSound(SOUND_BOX, S_FLD_TESTING, S_TRACK_MARG, SOUND_ONCE);
      //      }
      playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_SHRT_DBEEP, SOUND_ONCE);
      break;

    case C_TRACK_CALIB:
    case C_SYS_TEST:
      //  Not yet implemented
      playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_SHRT_DBEEP, SOUND_ONCE);
      break;

    case C_GUN_RECAL_HLD:  // jS long press: recenter the gun and hold it in that position.
      _PL("C_GUN_RECAL_HLD");
      if ( !(gunState & GUN_ACTIVE) ) {
        tGlowGreen.disable();
        tGlowBlue.restart();
        playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_GUN_ACTIVE_END, SOUND_RAND);
      }
      gunState = GUN_CALIBRATING;
      break;

    case C_GUN_SHOT:        // zB click: one shot from the gun
      _PL("C_GUN_SHOT");
      if (gunState & GUN_ACTIVE) {
        bladesOut = false;
        tGlowRed.disable();
        tGlowBlue.disable();
        tGlowGreen.disable();
        tGunBurst.disable();
        laser_sound_off();
        tGunShot.restart();

        // experimental: hold the aim where it was during the shot
        laserState |= LASER_HOLD;
        tiltGunHeld = tiltGun;
        panGunHeld = panGun;
        // ============
      }
      break;

    case C_GUN_BURST:       // zB long press: a burst of gunfire until the button is released
      _PL("C_GUN_BURST");
      if (gunState & GUN_ACTIVE) {
        bladesOut = false;
        tGlowRed.disable();
        tGlowBlue.disable();
        tGlowGreen.disable();
        laser_sound_off();
        tGunBurst.restart();
      }
      break;

    case C_GUN_STOP:        // zB lp release: stop the gunfire burst
      _PL("C_GUN_STOP");
      tGunBurst.disable();
      tGlowRed.disable();
      tGlowGreen.disable();
      tGlowBlue.restart();
      break;

    case C_GUN_LASER:       // zC click: toggle laser pointer of the gun. one click on, one click off
      _PL("C_GUN_LASER");
      bladesOut = false;
      if (gunState & GUN_ACTIVE)  {
        if (laserState == LASER_OFF) {
          laserState |= LASER_ON;
          tGlowBlue.disable();
          tGlowGreen.disable();
          //          player.pause();
          laserState |= LASER_SOUND;
          playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_AIM_CLICK + random(0, 2) * 2, SOUND_ONCE | SOUND_CNXT);
          pwm.setPWM(LASER, 4096, 0);
          tGlowRed.restart();
        }
        else {
          laser_off();
          tGlowRed.disable();
          tGlowGreen.disable();
          tGlowBlue.restart();
        }
      }
      else {
        if (laserState == LASER_OFF) {
          laserState |= LASER_ON;
          laserState |= LASER_SOUND;
          playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_AIM_SOFT, SOUND_ONCE | SOUND_CNXT);
          //          playSound(SOUND_HEAD, S_FLD_EQUIPMENT, S_AIM_SOFT, SOUND_ONCE);
          sendHeadCommand(C_LASERS_ON);
        }
        else {
          laser_off();
        }
      }
      break;

    case C_GUN_TRACK:       // zC long press: turn laser pointer on and track that point with the gun.
      _PL("C_GUN_TRACK");
      bladesOut = false;
      if (gunState & GUN_ACTIVE)  {
        if ( !(laserState & LASER_HOLD) ) {
          laserState |= LASER_ON;

          laserState |= LASER_HOLD;
          tiltGunHeld = tiltGun;
          panGunHeld = panGun;
          tGlowBlue.disable();
          tGlowGreen.disable();
          //          player.pause();
          laserState |= LASER_SOUND;
          playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_AIM_LONG, SOUND_ONCE | SOUND_CONT);
          pwm.setPWM(LASER, 4096, 0);
          tGlowRed.restart();
        }
        else {
          laser_off();
          tGlowRed.disable();
          tGlowGreen.disable();
          tGlowBlue.restart();
        }
      }
      break;

    case C_VOL_UP:          // jE click/upsidedown: volume up
      _PL("C_VOL_UP");
      soundVolume = constrain(soundVolume + 5, 1, 30);
      if (soundVolume == 30) {
        playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_MAX, SOUND_ONCE);
      }
      else {
        playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_SHRT_BEEP, SOUND_ONCE);
      }
      break;

    case C_VOL_DOWN:        // jW click/upsidedown: volume down
      _PL("C_VOL_DOWN");
      soundVolume = constrain(soundVolume - 5, 1, 30);
      if (soundVolume == 1) {
        playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_MIN, SOUND_ONCE);
      }
      else {
        playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_SHRT_BEEP, SOUND_ONCE);
      }
      break;

    case C_SND_CLICK:       // jE click: start playing random clicking sound
      _PL("C_SND_CLICK");
      playSound(SOUND_HEAD, S_FLD_CLICKING, S_CLICKING_END, SOUND_RAND);
      break;

    case C_SND_ANYTIME:     // jE dbclick: play "anytime"
      _PL("C_SND_ANYTIME");
      playSound(SOUND_HEAD, S_FLD_SPEAKS, S_ANYTIME, SOUND_ONCE);
      break;

    case C_SND_OVERHERE:     // jE dbclick: play "anytime"
      _PL("C_SND_OVERHERE");
      playSound(SOUND_HEAD, S_FLD_SPEAKS, S_OVERHERE, SOUND_ONCE);
      break;

    case C_SND_ROAR:        // jW click: start playing random roar
      _PL("C_SND_ROAR");
      playSound(SOUND_HEAD, S_FLD_ROARS, S_ROARS_END, SOUND_RAND);
      break;

    case C_SND_GROAL:        // jW click: start playing random roar
      _PL("C_SND_GROAL");
      playSound(SOUND_HEAD, S_FLD_ROARS, S_ROARS_SNARL, SOUND_ONCE);
      break;

    case C_SND_LAUGH:        // jW click: start playing random roar
      _PL("C_SND_LAUGH");
      playSound(SOUND_HEAD, S_FLD_LAUGHS, S_LAUGHS, SOUND_ONCE);
      break;


    case C_GUMMY:        // zC long press/upsidedown: play "i am a gummy bear"
      _PL("C_GUMMY");
      playSound(SOUND_HEAD, S_FLD_TESTING, S_TEST_GUMMYBEAR, SOUND_ONCE);
      break;

    case C_THEME:        // zC click/upsidedown" play the "predator theme" music
      _PL("C_THEME");
      playSound(SOUND_BOX, S_FLD_TESTING, S_TEST_THEME, SOUND_ONCE);
      break;

    case C_LED_FLASH:        // zC double click: flash yellow LEDs on the head
      _PL("C_LED_FLASH");
      sendHeadCommand(C_FLASH_LEDS);
      break;
  }
}


// GLOW_INTERVAL
const long GLOW_MIN = 100;
const long GLOW_MAX = 40900;
const long GLOW_LOW = 1000;
const int  GLOW_OFF = 4096;

// from 10 to 100 in 3 seconds @ 50 ms interval
// callback is called 1000/50 = 20 times per second
int _green_led_increment;
int _green_led_counter;
bool glowGreenOE() {
  pwm.setPWM(GRN, 0, GLOW_LOW / 10);
  _green_led_increment = (GLOW_LOW - GLOW_MIN) / (3000 / GLOW_INTERVAL);
  _green_led_counter = GLOW_MIN;
  return true;
}

void glowGreenOD() {
  pwm.setPWM(GRN, 0, GLOW_OFF);
}


void glowGreen() {
  _green_led_counter = constrain(_green_led_counter + _green_led_increment, GLOW_MIN, GLOW_LOW);
  if ( _green_led_counter == GLOW_MIN || _green_led_counter == GLOW_LOW ) _green_led_increment = -_green_led_increment;
  pwm.setPWM(GRN, 0, _green_led_counter / 10);
}

int _red_led_increment;
int _red_led_counter;
bool glowRedOE() {
  pwm.setPWM(RED, 0, GLOW_LOW / 10);
  _red_led_increment = (GLOW_MAX / 2 - GLOW_MIN) / (3000 / GLOW_INTERVAL);
  _red_led_counter = GLOW_MIN;
  return true;
}
void glowRedOD() {
  pwm.setPWM(RED, 0, GLOW_OFF);
}
void glowRed() {
  _red_led_counter = constrain(_red_led_counter + _red_led_increment, GLOW_MIN, GLOW_MAX / 2);
  if ( _red_led_counter == GLOW_MIN || _red_led_counter == GLOW_MAX / 2 ) _red_led_increment = -_red_led_increment;
  pwm.setPWM(RED, 0, _red_led_counter / 10);
}


// from 10 to 200 in 5 seconds @ 50 ms interval
// callback is called 1000/50 = 20 times per second
int _blue_led_increment;
int _blue_led_counter;
bool glowBlueOE() {
  pwm.setPWM(BLU, 0, 10);
  pwm.setPWM(GRN, 0, 10);
  _blue_led_increment = (GLOW_LOW * 2 - GLOW_MIN) / (5000 / GLOW_INTERVAL);
  _blue_led_counter = GLOW_MIN;
  return true;
}
void glowBlueOD() {
  pwm.setPWM(BLU, 0, GLOW_OFF);
  pwm.setPWM(GRN, 0, GLOW_OFF);
}
void glowBlue() {
  _blue_led_counter = constrain(_blue_led_counter + _blue_led_increment, GLOW_MIN, GLOW_LOW * 2);
  if ( _blue_led_counter == GLOW_MIN || _blue_led_counter == GLOW_LOW * 2 ) _blue_led_increment = -_blue_led_increment;
  pwm.setPWM(BLU, 0, _blue_led_counter / 10);
  pwm.setPWM(GRN, 0, _blue_led_counter / 10);
}




long _gun_led_increment, _gun_blu_increment;
long _gun_led_counter, _gun_blu_counter;

// GUN_SHOT_LENGTH / GUN_SHOT_INTERVAL
const int GUN_SHOT_PHASE1_LENGTH = 1500; // slow build up of "energy" in the blue LED to MAX within 1.5 seconds
const int GUN_SHOT_PHASE1_ITER = GUN_SHOT_PHASE1_LENGTH / GUN_SHOT_INTERVAL;
const int GUN_SHOT_PHASE2_LENGTH = 200; // flash front LEDS and dim the energy blue led
const int GUN_SHOT_PHASE2_ITER = GUN_SHOT_PHASE1_ITER + GUN_SHOT_PHASE2_LENGTH / GUN_SHOT_INTERVAL;
const int GUN_SHOT_PHASE3_LENGTH = 200; // keep front LEDS on
const int GUN_SHOT_PHASE3_ITER = GUN_SHOT_PHASE2_ITER + GUN_SHOT_PHASE3_LENGTH / GUN_SHOT_INTERVAL;
const int GUN_SHOT_PHASE4_LENGTH = 1100; // gradually dim front LEDS


bool gunShotOE() {
  _gun_blu_increment = (GLOW_MAX - GLOW_MIN) / (GUN_SHOT_PHASE1_LENGTH / GUN_SHOT_INTERVAL);
  _gun_blu_counter = GLOW_MIN;
  playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_GUN_SHOT, SOUND_ONCE);
  pwm.setPWM(PLASMA, 0, GLOW_OFF);
  pwm.setPWM(BLU, 0, GLOW_OFF);
  pwm.setPWM(GRN, 0, GLOW_OFF);
  return true;
}
void gunShotOD() {
  pwm.setPWM(PLASMA, 0, GLOW_OFF);
  pwm.setPWM(BLU, 0, GLOW_OFF);
  pwm.setPWM(GRN, 0, GLOW_OFF);
  laser_off(true);
  tGlowBlue.restart();
}

void gunShot() {
  if (tGunShot.getRunCounter() < GUN_SHOT_PHASE1_ITER) {  // 1 second
    _gun_blu_counter = constrain(_gun_blu_counter + _gun_blu_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(BLU, 0, _gun_blu_counter / 10);
    pwm.setPWM(laserState & LASER_ON ? RED : GRN, 0, _gun_blu_counter / 10);

  }
  else if (tGunShot.getRunCounter() == GUN_SHOT_PHASE1_ITER) {
    _gun_blu_increment =  -((GLOW_MAX - GLOW_MIN) / (GUN_SHOT_PHASE2_LENGTH / GUN_SHOT_INTERVAL));
    _gun_blu_counter = GLOW_MAX;
    _gun_led_increment = -_gun_blu_increment;
    _gun_led_counter = GLOW_MIN;
    pwm.setPWM(PLASMA, 0, _gun_led_counter);
    if (laserState == LASER_ON) {
      //      laserState = LASER_OFF;
      pwm.setPWM(LASER, 0, 4096);
    }
  }
  else if (tGunShot.getRunCounter() < GUN_SHOT_PHASE2_ITER) {
    _gun_blu_counter = constrain(_gun_blu_counter + _gun_blu_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(BLU, 0, _gun_blu_counter / 10);
    pwm.setPWM(laserState & LASER_ON ? RED : GRN, 0, _gun_blu_counter / 10);

    _gun_led_counter = constrain(_gun_led_counter + _gun_led_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(PLASMA, 0, _gun_led_counter / 10);
  }
  else if (tGunShot.getRunCounter() == GUN_SHOT_PHASE2_ITER) {
    pwm.setPWM(BLU, 0, GLOW_OFF);
    pwm.setPWM(laserState & LASER_ON ? RED : GRN, 0, GLOW_OFF);
    byte _held = (laserState & LASER_HOLD);
    laser_off(true);
    if (_held) laserState = LASER_HOLD;
  }
  else if (tGunShot.getRunCounter() == GUN_SHOT_PHASE3_ITER) {
    _gun_led_increment = -(GLOW_MAX - GLOW_MIN) / (GUN_SHOT_PHASE4_LENGTH / GUN_SHOT_INTERVAL);
  }
  else if (tGunShot.getRunCounter() > GUN_SHOT_PHASE3_ITER) {
    _gun_led_counter = constrain(_gun_led_counter + _gun_led_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(PLASMA, 0, _gun_led_counter / 10);
  }
}



// GUN_BURST_LENGTH / GUN_SHOT_INTERVAL
const int GUN_BURST_PHASE1_LENGTH = 550; // slow build up of "energy" in the blue LED to MAX within 1.5 seconds
const int GUN_BURST_PHASE1_ITER = GUN_BURST_PHASE1_LENGTH / GUN_SHOT_INTERVAL;
const int GUN_BURST_PHASE2_LENGTH = 100; // flash front LEDS and dim the energy blue led
const int GUN_BURST_PHASE2_ITER = GUN_BURST_PHASE1_ITER + GUN_BURST_PHASE2_LENGTH / GUN_SHOT_INTERVAL;
const int GUN_BURST_PHASE3_LENGTH = 150; // keep front LEDS on
const int GUN_BURST_PHASE3_ITER = GUN_BURST_PHASE2_ITER + GUN_BURST_PHASE3_LENGTH / GUN_SHOT_INTERVAL;
const int GUN_BURST_PHASE4_LENGTH = 300;


bool gunBurstOE() {
  _gun_blu_increment = (GLOW_MAX - GLOW_MIN) / (GUN_BURST_PHASE1_LENGTH / GUN_SHOT_INTERVAL);
  _gun_blu_counter = GLOW_MIN;
  byte _held = (laserState & LASER_HOLD);
  laser_off(true);
  if (_held) laserState = LASER_HOLD;
  playSound(SOUND_BOX, S_FLD_EQUIPMENT, S_GUN_BURST, SOUND_ONCE);
  //  playSound(SOUND_HEAD, S_FLD_EQUIPMENT, S_GUN_BURST, SOUND_ONCE);
  pwm.setPWM(PLASMA, 0, GLOW_OFF);
  pwm.setPWM(BLU, 0, GLOW_OFF);
  pwm.setPWM(GRN, 0, GLOW_OFF);
  return true;
}
void gunBurstOD() {
  pwm.setPWM(PLASMA, 0, GLOW_OFF);
  pwm.setPWM(BLU, 0, GLOW_OFF);
  pwm.setPWM(GRN, 0, GLOW_OFF);
  laser_off(true);
  tGlowBlue.restart();
}

void gunBurst() {
  if (tGunBurst.isLastIteration()) {
    tGunBurst.restart();
    return;
  }
  if (tGunBurst.getRunCounter() < GUN_BURST_PHASE1_ITER) {  // 1 second
    _gun_blu_counter = constrain(_gun_blu_counter + _gun_blu_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(BLU, 0, _gun_blu_counter / 10);
    pwm.setPWM(GRN, 0, _gun_blu_counter / 10);

  }
  else if (tGunBurst.getRunCounter() == GUN_BURST_PHASE1_ITER) {
    _gun_blu_increment =  -((GLOW_MAX - GLOW_MIN) / (GUN_BURST_PHASE2_LENGTH / GUN_SHOT_INTERVAL));
    _gun_blu_counter = GLOW_MAX;
    _gun_led_increment = -_gun_blu_increment;
    _gun_led_counter = GLOW_MIN;
    pwm.setPWM(PLASMA, 0, _gun_led_counter);
  }
  else if (tGunBurst.getRunCounter() < GUN_BURST_PHASE2_ITER) {
    _gun_blu_counter = constrain(_gun_blu_counter + _gun_blu_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(BLU, 0, _gun_blu_counter / 10);
    pwm.setPWM(GRN, 0, _gun_blu_counter / 10);

    _gun_led_counter = constrain(_gun_led_counter + _gun_led_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(PLASMA, 0, _gun_led_counter / 10);
  }
  else if (tGunBurst.getRunCounter() == GUN_BURST_PHASE2_ITER) {
    pwm.setPWM(BLU, 0, GLOW_OFF);
    pwm.setPWM(GRN, 0, GLOW_OFF);
  }
  else if (tGunBurst.getRunCounter() == GUN_BURST_PHASE3_ITER) {
    _gun_led_increment = -(GLOW_MAX - GLOW_MIN) / (GUN_BURST_PHASE4_LENGTH / GUN_SHOT_INTERVAL);
  }
  else if (tGunBurst.getRunCounter() > GUN_BURST_PHASE3_ITER) {
    _gun_led_counter = constrain(_gun_led_counter + _gun_led_increment, GLOW_MIN, GLOW_MAX);
    pwm.setPWM(PLASMA, 0, _gun_led_counter / 10);
  }
}

#ifdef _DISPLAY_

void displayCallback() {

  char line[256];
  //
  //  float rollGun  = IMUGun.getRoll();
  //  float rollHead = IMUHead.getRoll();
  //
  //  //  if (rollHead < 0.)
  //  //  rollHead += 360.;
  //  //  if (rollGun < 0.)
  //  //  rollGun += 360.;
  //
  //  float pitchGun = IMUGun.getPitch();
  //  float pitchHead = IMUHead.getPitch();
  //  float yawGun = IMUGun.getYaw();
  //  float yawHead = IMUHead.getYaw();

  float *q1 = (float*) IMUHead.getQuaternion();
  float *q2 = (float*) IMUGun.getQuaternion();
  //
  snprintf(line, 255, "Q head: %6.2f %6.2f %6.2f %6.2f\tQ gun: %6.2f %6.2f %6.2f %6.2f", q1[0], q1[1], q1[2], q1[3], q2[0], q2[1], q2[2], q2[3]);
  Serial.println(line);

  //  snprintf(line, 255, "Head: %06ld r: %8.2f\tp: %8.2f\ty: %8.2f", millis(), tiltHead, IMUHead.getPitch(), panHead);
  //  Serial.println(line);
  //
  //  snprintf(line, 255, "Gun:  %06ld r: %8.2f\tp: %8.2f\ty: %8.2f\tt: %04ld\tp: %04ld", millis(), tiltGun, IMUGun.getPitch(), panGun, tiltF.currentValue(), panF.currentValue());
  //  Serial.println(line);
  //
  //  //  snprintf(line, 255, "Control: deltaTilt=%04ld, targetTilt=%04d", deltaTilt, targetTilt);
  //  //  Serial.println(line);
  //  snprintf(line, 255, "Angles: aHead=%8.2f\taGun=%8.2f", aHead, aGun);
  //  Serial.println(line);

  snprintf(line, 255, "Avg measure intervals, uS. Head: %06ld, Gun: %06ld", tHeadF.currentValue(), tGunF.currentValue());
  Serial.println(line);


}

#endif

