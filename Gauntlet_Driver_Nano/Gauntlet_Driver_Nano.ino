// Left gauntlet "HAND" controller v1.0.0
//
// Changelog:
//  2018-08-24:
//    v1.0.0 - started

// DEFINES
//
//#define _DEBUG
//#define _TEST

#define WDT_TIMEOUT      WDTO_2S // watchdog resets the device after its locked for over 2 seconds

// INCLUDES

//#include <I2Cdev.h>
#include <Wire.h>
#include <avr/wdt.h>

#ifdef _DEBUG
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#endif

#include <Nunchuk.h>
#include <OneButton.h>
#include <QueueArray.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_TIMEOUT
#include <TaskScheduler.h>

const int TX_RATE = 9600;
const int TX_TIMEOUT = 50; // ms

const int CHUK_READ_INTERVAL = 50; // ms
const int RESET_THRESHOLD = 5; // seconds of chuk or nRF24 errors, then reset

typedef struct  {
  byte  cmd;
  //  int  pload[2];
} TransmitBuffer;

TransmitBuffer buf, t_buf;

QueueArray <TransmitBuffer> q;

#include <predator_commands.h>

volatile bool tx_errors = false;

//ArduinoNunchuk chuk;
bool  goodChukData = false;
bool  deviceUpsidedown = false;


// -- Tasks

void nunchukUpdate();
void transmitCallback();
bool heartbeatEnable();
void heartbeatDisable();
void heartbeatCallback();
void rtsCallback();
void rtsOnDisable();

//StatusRequest
Scheduler ts;

Task tChukUpdate (CHUK_READ_INTERVAL * TASK_MILLISECOND, TASK_FOREVER, &nunchukUpdate, &ts, true);
Task tTransmit    (8 * TASK_MILLISECOND, TASK_FOREVER, &transmitCallback, &ts);

Task tHeartbeat   (1 * TASK_SECOND, TASK_FOREVER, &heartbeatCallback, &ts, false, &heartbeatEnable, &heartbeatDisable);
Task tRTSPoll (4 * TASK_MILLISECOND, TASK_FOREVER, &rtsCallback, &ts, false, NULL, &rtsOnDisable);

#ifdef _DEBUG
void displayCallback();
Task tDisplay (100 * TASK_MILLISECOND, TASK_FOREVER, &displayCallback, &ts, true);
SoftwareSerial dbSerial(9,10); // rx, tx
#endif

// CODE

const int CLICKTICKS = 400;
const int DEBOUNCETICKS = 10;
const int PRESSTICKS = 800;

OneButton cB, zB, jN, jS, jE, jW, aZ;

void transmitCallback() {
#ifdef _DEBUG
  //  dbSerial.print(F("tr. i=")); Serial.println( tTransmit.getRunCounter() );
#endif
  if (tRTSPoll.isEnabled()) {
    return;
  }
  if ( q.isEmpty() ) {
    tTransmit.disable();
    return;
  }

#ifdef _DEBUG
  t_buf = q.peek();
  if ( t_buf.cmd ) {
    dbSerial.print(F("cmd=")); dbSerial.println( t_buf.cmd );
  }
#endif
  t_buf = q.pop();
  //  vw_send( (uint8_t *) &t_buf, sizeof(t_buf) );
  Serial.write((uint8_t *) &t_buf, sizeof(t_buf) );
  tRTSPoll.restart();
  //#ifdef _DEBUG
  digitalWrite(13, HIGH);
  //#endif
}

void rtsCallback() {
  //  if ( !vw_tx_active() ) tRTSPoll.disable();
  if ( Serial.availableForWrite() > 0) tRTSPoll.disable();
}

void rtsOnDisable() {
  tx_errors = tRTSPoll.timedOut();
  digitalWrite(13, LOW);
}


void initPins() {

  //  pinMode(CSN_PIN, OUTPUT); // SPI needs it?
  //  pinMode(2, INPUT); // nRF24 interrupt pin
  //  pinMode(TX_PIN, OUTPUT);
  //  pinMode(RX_PIN, INPUT);
  //  pinMode(EN_PIN, OUTPUT);
  //#ifdef _DEBUG
  pinMode(13, OUTPUT);
  //#endif
}

void initBuf() {
  buf.cmd = C_NONE;
  //  buf.pload[0] = 0;
  //  buf.pload[1] = 0;
}

void initRadio() {
  // Initialise the IO and ISR
  //  vw_set_tx_pin(TX_PIN);
  // vw_set_rx_pin(RX_PIN);
  // vw_set_ptt_pin(EN_PIN);
  // vw_set_ptt_inverted(true); // Required for DR3100
  //  vw_setup(TX_RATE);   // Bits per sec
  Serial.begin(TX_RATE);
  delay(50);

  tRTSPoll.setTimeout(TX_TIMEOUT);
  tRTSPoll.disable();
}

void initNunchuk() {
#ifdef _DEBUG
  dbSerial.print(F("Initializing chuk..."));
#endif

  nunchuk_init();
  delay(10);
  // read a few times to get rid of initial garbage
  nunchuk_read(); delay(CHUK_READ_INTERVAL);
  nunchuk_read(); delay(CHUK_READ_INTERVAL);
  nunchuk_read(); delay(CHUK_READ_INTERVAL);
  nunchuk_read(); delay(CHUK_READ_INTERVAL);
  goodChukData = nunchuk_read();

#ifdef _DEBUG
  dbSerial.println(F("done."));
#endif
}

void jNClick();
//void jNDoubleClick();
void jNLongPressStart();
void jNLongPressStop();
void jSClick();
void jSLongPressStart();
void jSLongPressStop();
void jEClick();
void jEDoubleClick();
void jELongPressStart();
void jWClick();
void jWDoubleClick();
void jWLongPressStart();
//void jWLongPressStop();
void zBClick();
void zBLongPressStart();
void zBLongPressStop();
void cBClick();
void cBDoubleClick();
void cBLongPressStart();
//void cBLongPressStop();
void aZLongPressStart();
void aZLongPressStop();

void initButtons() {
  jN.reset();
  jN.setClickTicks(CLICKTICKS);
  jN.setDebounceTicks(DEBOUNCETICKS);
  jN.setPressTicks(PRESSTICKS);
  jN.attachClick(jNClick);
  //  jN.attachLongPressStart(jNClick);
  jN.attachLongPressStart(jNLongPressStart);
  jN.attachLongPressStop(jNLongPressStop);
  //  jN.attachDoubleClick(jNDoubleClick);

  jS.reset();
  jS.setClickTicks(CLICKTICKS);
  jS.setDebounceTicks(DEBOUNCETICKS);
  jS.setPressTicks(PRESSTICKS);
  jS.attachClick(jSClick);
  jS.attachLongPressStart(jSLongPressStart);
  jS.attachLongPressStop(jSLongPressStop);

  jE.reset();
  jE.setClickTicks(CLICKTICKS);
  jE.setDebounceTicks(DEBOUNCETICKS);
  jE.setPressTicks(PRESSTICKS);
  jE.attachClick(jEClick);
  jE.attachLongPressStart(jELongPressStart);
  //  jE.attachLongPressStop(jELongPressStop);
  jE.attachDoubleClick(jEDoubleClick);

  jW.reset();
  jW.setClickTicks(CLICKTICKS);
  jW.setDebounceTicks(DEBOUNCETICKS);
  jW.setPressTicks(PRESSTICKS);
  jW.attachClick(jWClick);
  jW.attachLongPressStart(jWLongPressStart);
  //  jW.attachLongPressStop(jWLongPressStop);
  jW.attachDoubleClick(jWDoubleClick);

  zB.reset();
  zB.setClickTicks(CLICKTICKS);
  zB.setDebounceTicks(DEBOUNCETICKS);
  zB.setPressTicks(PRESSTICKS);
  zB.attachClick(zBClick);
  zB.attachLongPressStart(zBLongPressStart);
  zB.attachLongPressStop(zBLongPressStop);

  cB.reset();
  cB.setClickTicks(CLICKTICKS);
  cB.setDebounceTicks(DEBOUNCETICKS);
  cB.setPressTicks(PRESSTICKS);
  cB.attachClick(cBClick);
  cB.attachDoubleClick(cBDoubleClick);
  cB.attachLongPressStart(cBLongPressStart);
  //  cB.attachLongPressStop(cBLongPressStop);

  aZ.reset();
  aZ.setClickTicks(CLICKTICKS);
  aZ.setDebounceTicks(DEBOUNCETICKS);
  aZ.setPressTicks(PRESSTICKS);
  aZ.attachLongPressStart(aZLongPressStart);
  aZ.attachLongPressStop(aZLongPressStop);
}

// --- SETUP ----------------------------------------------------
void setup() {
  wdt_disable();


#ifdef _DEBUG
  dbSerial.begin(9600);
  delay(1000);
  dbSerial.println(F("Predtor Arm Controller Circuit test"));
  q.setPrinter (dbSerial);
#endif


  // put your setup code here, to run once:
  initPins();
  initBuf();

#ifndef _DEBUG
   power_adc_disable();
  power_spi_disable();
#endif

  // Configure radio
  tHeartbeat.enable();
  Wire.begin();       //Initiate the Wire library and join the I2C bus as a master
  wdt_reset();
#ifdef _DEBUG
  dbSerial.println(F("Wire initialized"));
#endif

  initNunchuk();
  wdt_reset();
#ifdef _DEBUG
  dbSerial.println(F("Wii controller initialized"));
#endif

  initRadio();
  wdt_reset();
#ifdef _DEBUG
  dbSerial.println(F("Radio initialized"));
#endif


  initButtons();
  //  ts.startNow();
  //  tRTSPoll.disable();
}

void loop() {
  ts.execute();
}

int chukErrorCounter = 0;
void nunchukUpdate() {
  goodChukData = nunchuk_read();
  if ( goodChukData ) {
    tickButtons();
    if ( --chukErrorCounter < 0 ) chukErrorCounter = 0;
  }
  else {
    ++chukErrorCounter;
  }
}


void tickButtons() {
  aZ.tick( nunchuk_accelZ() < 0 );  // check if device is upside-down
  jS.tick( nunchuk_joystickY() < -120 ); // joystick Y in full backwards position
  jN.tick( nunchuk_joystickY() > 120 ); // joystick Y in full forward position
  zB.tick( nunchuk_buttonZ() == 1 );
  cB.tick( nunchuk_buttonC() == 1 );
  jE.tick( nunchuk_joystickX() > 120 ); // joystick Y in full backwards position
  jW.tick( nunchuk_joystickX() < -120 ); // joystick Y in full forward position
  if ( !q.isEmpty() ) {
    tTransmit.enableIfNot();
  }
}


/*
   X: Center = 0, left= -127, right=128
   Y: center = 0, fwd=127, back= -128
   Normal Horiz: joystick is str8 up
     aX ~ 0, aY ~ 0, aZ ~ 256
   Controller facing up:
     ax ~ 0, aY ~ -255, aZ ~ 0
   Controller tolted 90 deg right:
     aX ~ 255, aY ~ 0, aZ ~0
   Controller tilted 90 deg left:
     aX ~ -255, aY ~ 0, aZ ~ 0
   Controller upside down but straight
     aX ~ 0, aY ~ 0, aZ ~ -300
   Z Button: pressed 1, 0 otherwise
   C Button: pressed 1, 0 otherwise
*/

//  C_GUN_OFF,        // jN click: turn gun off, rotate into center yaw, tilt fwd down, stop tracking
void jNClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jNClick USD=C_TRACK_IMU"));
#endif
    buf.cmd = C_TRACK_IMU;
    //    q.enqueue(buf);
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jNClick=C_GUN_OFF"));
#endif
    buf.cmd = C_GUN_OFF;
  }
  q.enqueue(buf);
}


//  C_BLADES,      // jN doublecick: sound of blades drawn out and back in
//void jNDoubleClick() {
//  if ( deviceUpsidedown ) {
//#ifdef _DEBUG
//    dbSerial.println(F("jNClick USD=C_NONE"));
//#endif
//    buf.cmd = C_NONE;
//    //    q.enqueue(buf);
//  }
//  else {
//#ifdef _DEBUG
//    dbSerial.println(F("jNClick=C_BLADES"));
//#endif
//    buf.cmd = C_BLADES;
//  }
//  q.enqueue(buf);
//}


void jNLongPressStart() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jNClick USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
    //    q.enqueue(buf);
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jNClick=C_BLADES_OUT"));
#endif
    buf.cmd = C_BLADES_OUT;
  }
  q.enqueue(buf);
}

void jNLongPressStop() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jNClick USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
    //    q.enqueue(buf);
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jNClick=C_BLADES_IN"));
#endif
    buf.cmd = C_BLADES_IN;
  }
  q.enqueue(buf);
}




//  C_GUN_RECAL,      // jS click: turn gun on (if was off), recenter and sync with head IMU (head should be positioned fwd centered).
void jSClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jSClick USD=C_TRACK_MARG"));
#endif
    buf.cmd = C_TRACK_MARG;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jSClick=C_GUN_RECAL"));
#endif
    buf.cmd = C_GUN_RECAL;
  }
  q.enqueue(buf);
}

//  C_GUN_RECAL_HLD,  // jS long press: recenter the gun and hold it in that position.
//  On release, send C_GUN_RECAL again to start tracking
void jSLongPressStart() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jSLongPressStart USD=C_TRACK_CALIB"));
#endif
    buf.cmd = C_TRACK_CALIB;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jSLongPressStart=C_GUN_RECAL_HLD"));
#endif
    buf.cmd = C_GUN_RECAL_HLD;
  }
  q.enqueue(buf);
}

void jSLongPressStop() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jSLongPressStop USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jSLongPressStop=C_GUN_RECAL"));
#endif
    buf.cmd = C_GUN_RECAL;
  }
  q.enqueue(buf);
}

//  C_SND_CLICK,      // jE click: start playing random clicking sound
void jEClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jEClick USD=C_VOL_UP"));
#endif
    buf.cmd = C_VOL_UP;
    // q.enqueue(buf);
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jEClick=C_SND_CLICK"));
#endif
    buf.cmd = C_SND_CLICK;
  }
  q.enqueue(buf);
}


//  C_SND_ANYTIME     // jE dbclick: play "anytime"
void jEDoubleClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jEDoubleClick USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerialial.println(F("jEDoubleClick=C_SND_ANYTIME"));
#endif
    buf.cmd = C_SND_ANYTIME;
  }
  q.enqueue(buf);
}


void jELongPressStart() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jELongPressStart USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jELongPressStart=C_SND_OVERHERE"));
#endif
    buf.cmd = C_SND_OVERHERE;
  }
  q.enqueue(buf);
}


//   C_SND_ROAR,       // jW click: start playing random roar
void jWClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jWClick USD=C_VOL_DOWN"));
#endif
    buf.cmd = C_VOL_DOWN;
    q.enqueue(buf);
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jWClick=C_SND_ROAR"));
#endif
    buf.cmd = C_SND_ROAR;
    q.enqueue(buf);
  }
}

//  C_SND_LAUGH,      // jW dbclick: play laugh
void jWDoubleClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("JWDoubleClick USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("JWDoubleClick=C_SND_LAUGH"));
#endif
    buf.cmd = C_SND_LAUGH;
  }
  q.enqueue(buf);
}


void jWLongPressStart() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("jWLongPressStart USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("jWLongPressStart=C_SND_GROAL"));
#endif
    buf.cmd = C_SND_GROAL;
  }
  q.enqueue(buf);
}


//  C_GUN_SHOT,       // zB click: one shot from the gun
void zBClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("zBClick USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("zBClick=C_GUN_SHOT"));
#endif
    buf.cmd = C_GUN_SHOT;
  }
  q.enqueue(buf);
}

//  C_GUN_BURST,      // zB long press: a burst of gunfire until the button is released
//  On release: stop the burst (next command)
void zBLongPressStart() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("zBLongPressStart USD=C_SYS_TEST"));
#endif
    buf.cmd = C_SYS_TEST;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("zBLongPressStart=C_GUN_BURST"));
#endif
    buf.cmd = C_GUN_BURST;
  }
  q.enqueue(buf);
}
void zBLongPressStop() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("zBLongPressStop USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("zBLongPressStop=C_GUN_STOP"));
#endif
    buf.cmd = C_GUN_STOP;
  }
  q.enqueue(buf);
}

//  C_GUN_LASER,      // cB click: toggle laser pointer of the gun. one click on, one click off
void cBClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("cBClick USD=C_THEME"));
#endif
    buf.cmd = C_THEME;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("cBClick=C_GUN_LASER"));
#endif
    buf.cmd = C_GUN_LASER;
  }
  q.enqueue(buf);
}


void cBDoubleClick() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("cBDoubleClick USD=C_NONE"));
#endif
    buf.cmd = C_NONE;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("cBDoubleClick=C_LED_FLASH"));
#endif
    buf.cmd = C_LED_FLASH;
  }
  q.enqueue(buf);
}

//  C_GUN_TRACK,      // zC long press: turn laser poainter on and track that point with the gun.
//  On release do nothing.
void cBLongPressStart() {
  if ( deviceUpsidedown ) {
#ifdef _DEBUG
    dbSerial.println(F("cBLongPressStart USD=C_GUMMY"));
#endif
    buf.cmd = C_GUMMY;
  }
  else {
#ifdef _DEBUG
    dbSerial.println(F("cBLongPressStart=C_GUN_TRACK"));
#endif
    buf.cmd = C_GUN_TRACK;
  }
  q.enqueue(buf);
}
//void cBLongPressStop();


//  Detect Upside-down device situation
void aZLongPressStart() {
#ifdef _DEBUG
  dbSerial.println(F("aZLongPressStart=deviceUpsidedown"));
#endif
  deviceUpsidedown = true;
}

void aZLongPressStop() {
#ifdef _DEBUG
  dbSerial.println(F("aZLongPressStop=deviceUp"));
#endif
  deviceUpsidedown = false;
}

int heartbeatCounter = 0;
void heartbeatCallback() {
#ifdef _DEBUG
  //  Serial.print(millis());
  //  Serial.println(F(": heartbeatCallback."));
  if ( chukErrorCounter || tx_errors ) {
    dbSerial.print(F("Chuck ERR=")); dbSerial.print(chukErrorCounter); dbSerial.print(F("\tTX_ERR=")); dbSerial.println(tx_errors);
  }
#endif
  if ( chukErrorCounter || tx_errors ) {
    heartbeatCounter++;
  }
  else {
    heartbeatCounter = 0;
  }

  if ( heartbeatCounter < RESET_THRESHOLD ) {
    wdt_reset();
  }

  // Send a heartbeat command to the backpack
  buf.cmd = C_NONE;
  q.enqueue(buf);
  tTransmit.restart();
}

bool heartbeatEnable() {
#ifdef _DEBUG
  dbSerial.print(millis());
  dbSerial.println(F(": HBEnable."));
#endif
  //disable interrupts
  cli();
  //reset watchdog
  wdt_reset();
  //set up WDT interrupt
  WDTCSR = (1 << WDCE) | (1 << WDE);
  //Start watchdog timer with delay prescaller
  WDTCSR = (1 << WDIE) | (1 << WDE) | (WDT_TIMEOUT & 0x27);
  //Enable global interrupts
  sei();

  return true;
}

void heartbeatDisable() {
#ifdef _DEBUG
  //  Serial.print(millis());
  //  Serial.println(F(": HBDisable."));
#endif
  wdt_disable();
}



#ifdef _DEBUG
void displayCallback() {
  if ( goodChukData ) {
    //    Serial.print("X="); Serial.print( nunchuk_joystickX() ); Serial.print("\t");
    //    Serial.print("Y="); Serial.print( nunchuk_joystickY() ); Serial.print("\t");
    //    Serial.print("aX="); Serial.print( nunchuk_accelX() ); Serial.print("\t");
    //    Serial.print("aY="); Serial.print( nunchuk_accelY() ); Serial.print("\t");
    //    Serial.print("aZ="); Serial.print( nunchuk_accelZ() ); Serial.print("\t");
    //    Serial.print("zB="); Serial.print( nunchuk_buttonZ() ); Serial.print("\t");
    //    Serial.print("cB="); Serial.print( nunchuk_buttonC() ); Serial.println();
  }
  else {
    //    Serial.println(F("Error reading data from the controller"));
  }
}
#endif
