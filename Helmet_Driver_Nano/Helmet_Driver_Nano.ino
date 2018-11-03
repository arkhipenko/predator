#include <Arduino.h>
#include "DFPlayerMini_t3.h"

#include <Wire.h>
#include <QueueArray.h>

// #define _TASK_TIMECRITICAL      // Enable monitoring scheduling overruns
#define _TASK_SLEEP_ON_IDLE_RUN // Enable 1 ms SLEEP_IDLE powerdowns between tasks if no callback methods were invoked during the pass
// #define _TASK_STATUS_REQUEST    // Compile with support for StatusRequest functionality - triggering tasks on status change events in addition to time only
// #define _TASK_WDT_IDS           // Compile with support for wdt control points and task ids
// #define _TASK_LTS_POINTER       // Compile with support for local task storage pointer
// #define _TASK_PRIORITY          // Support for layered scheduling priority
// #define _TASK_MICRO_RES         // Support for microsecond resolution
// #define _TASK_STD_FUNCTION      // Support for std::function (ESP8266 ONLY)
// #define _TASK_DEBUG             // Make all methods and variables public for debug purposes
// #define _TASK_INLINE         // Make all methods "inline" - needed to support some multi-tab, multi-file implementations
// #define _TASK_TIMEOUT

#include <TaskScheduler.h>

void restartPlay();
void handleCommand();
void iterateCallback();

Scheduler ts;
Task  tLedsFlash(0, 0, NULL, &ts);
Task  tRestartPlay(TASK_IMMEDIATE, TASK_ONCE, &restartPlay, &ts, false);
Task  tHandleCommand(TASK_IMMEDIATE, TASK_FOREVER, &handleCommand, &ts, false);
Task  tPlayerIterator(5 * TASK_MILLISECOND, TASK_FOREVER, &iterateCallback, &ts, false);

const int LEDFLASH_DURATION1 = 200;
const int LEDFLASH_DURATION2 = 800;
const int LEDFLASH_DURATION_ON = 100;

//#define _TEST
#ifdef _TEST
void testCallback();
Task tTest(3 * TASK_SECOND, TASK_FOREVER, &testCallback, &ts);
#endif

const byte OWN_ADDR = 0x10;

const int BUSY = 2;
const int LED1 = 9;
const int LED2 = 10;

const int LASER1 = 5;
const int LASER2 = 3;
const int LASER3 = 4;


const int volume = 1;

DFPlayerMini player;

//volatile byte command = 0, mp3_folder, mp3_file;
//volatile bool file_folder_set;
int folderNumber = 0, fileNumber = 0;

typedef struct  {
  byte  cmd;
  byte  folder;
  byte  file;
} TransmitBuffer;
//TransmitBuffer buf;
QueueArray <TransmitBuffer> q;

/* -----------------
    I2C protocol is as follows:
    Master always sending 1 byte command
    Command - Action
    0 - return player status (LOW: playing/HIGH: idle)
    1 - play file once
    2 - play file continously
    3 - play file once, then play next one continously
    4 - play a random file from the folder

    200 - set volume
    next byte 1 to 30 is the actual volume

    231 - flash yellow LEDs

    232 - turn lasers on
    233 - turn lasers off

    234-254 - not used so far

    255 - pause (stop palying)

*/

#include <predator_head_commands.h>

#ifdef _TEST
#include "predator_sounds.h"

//void testCallback() {
//  TransmitBuffer b;
//  switch (tTest.getRunCounter()) {
//    case 1: b.cmd = C_SET_VOLUME; b.file = 15; break;
//    case 2: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_INIT; break;
//    case 3: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_HEAD_SND; break;
//    case 4: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_VOLUME; break;
//    case 5: b.cmd = C_SET_VOLUME; b.file = 1; break;
//    case 6: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_MIN; break;
//    case 7: b.cmd = C_SET_VOLUME; b.file = 15; break;
//    case 8: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_MED; break;
//    case 9: b.cmd = C_SET_VOLUME; b.file = 30; break;
//    case 10: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_MAX; break;
//    case 11: b.cmd = C_SET_VOLUME; b.file = 15; break;
//    case 12: b.cmd = C_PLAY_FILE_ONCE; b.folder = S_FLD_TESTING; b.file = S_TEST_HEAD_LED; break;
//    case 13: b.cmd = C_LASERS_ON; break;
//    case 14: b.cmd = C_LASERS_OFF; break;
//    case 15: b.cmd = C_FLASH_LEDS; break;
//    case 16: tTest.disable();
//  }
//  q.push(b);
//  tHandleCommand.restart();
//}

void testCallback() {
  TransmitBuffer b;
  switch (tTest.getRunCounter()) {
    case 1: b.cmd = C_LASERS_ON; tTest.delay(30 * TASK_SECOND); break;
    case 2: b.cmd = C_LASERS_OFF; break;
    case 3: b.cmd = C_FLASH_LEDS; break;
    case 4: tTest.disable();
  }
  q.push(b);
  tHandleCommand.restart();
}

#endif

void setupPins() {
  pinMode(BUSY, INPUT);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LASER1, OUTPUT);
  digitalWrite(LASER1, LOW);
  pinMode(LASER2, OUTPUT);
  digitalWrite(LASER2, LOW);
  pinMode(LASER3, OUTPUT);
  digitalWrite(LASER3, LOW);
}

void setupPlayer() {
  Serial.begin(9600);
  delay(100);
  while (!Serial) ;;

  player.begin(Serial, false, true);
  //  delay(100);
  //  player.reset();
  //  delay(100);

  player.setTimeOut(500);
  player.volume(volume);  //Set volume value (0~30).
  player.completeCommand();

  //----Set different EQ----
  player.EQ(DFPLAYER_EQ_NORMAL);
  player.completeCommand();
  //  player.EQ(DFPLAYER_EQ_POP);
  //  player.EQ(DFPLAYER_EQ_ROCK);
  //  player.EQ(DFPLAYER_EQ_JAZZ);
  //  player.EQ(DFPLAYER_EQ_CLASSIC);
  //  player.EQ(DFPLAYER_EQ_BASS);
  delay(100);

  //----Set device we use SD as default----
  player.outputDevice(DFPLAYER_DEVICE_SD);
  player.completeCommand();

  //  player.playMp3Folder(4);
  //  delay(100);
}

void setup() {

  setupPins();
  setupPlayer();

  // put your setup code here, to run once:
  // config TinyWire library for I2C slave functionality
  Wire.begin( OWN_ADDR );
  Wire.onReceive( &onI2CReceive );
  Wire.onRequest( &onI2CRequest );

#ifdef _TEST
  delay(1000);
  tTest.enable();
#endif
}

void leds_up() {
  int cycle = map(tLedsFlash.getRunCounter(), 1, LEDFLASH_DURATION1, 0, 255);
  analogWrite(LED1, cycle);
  analogWrite(LED2, cycle);

  if (tLedsFlash.isLastIteration()) {
    tLedsFlash.set(1, LEDFLASH_DURATION2, &leds_off);
    tLedsFlash.restartDelayed(LEDFLASH_DURATION_ON);
  }
}

void leds_off() {
  int cycle = map(LEDFLASH_DURATION2 - tLedsFlash.getRunCounter(), LEDFLASH_DURATION2 - 1, 0, 255, 0);
  analogWrite(LED1, cycle);
  analogWrite(LED2, cycle);
  if (tLedsFlash.isLastIteration()) {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }
}

void flash_leds() {
  tLedsFlash.set(1, LEDFLASH_DURATION1, &leds_up);
  tLedsFlash.restart();
}

void lasers_on() {
  digitalWrite(LASER1, HIGH);
  digitalWrite(LASER2, HIGH);
  digitalWrite(LASER3, HIGH);
}

void lasers_off() {
  digitalWrite(LASER1, LOW);
  digitalWrite(LASER2, LOW);
  digitalWrite(LASER3, LOW);
}


void loop() {
  ts.execute();
}

bool seeded = false;
void handleCommand() {

  TransmitBuffer b;

  if ( q.isEmpty() ) {
    tHandleCommand.disable();
    return;
  }
  b = q.pop();
  if ( b.cmd == C_PLAY_FILE_ONCE ) {  // play once
    player.playFolder(b.folder, b.file);
    tPlayerIterator.restart();
  }
  else if ( b.cmd == C_PLAY_FILE_CONT) {  // play continously
    startContinousPlay(b.folder, b.file);
  }
  else if ( b.cmd == C_PLAY_FILE_NEXT_CONT) {  // play continously
    startContinousPlay(b.folder, b.file);
    fileNumber++;  // next time and over - play next file
  }
  else if ( b.cmd == C_PLAY_RANDOM_FILE) {  // play random file. files shoul dbe named 001, 002, ...  file parameter is the inclusive maximum in a folder
    if ( !seeded ) {
      randomSeed(millis());
      seeded = true;
    }
    if ( b.file > 0) {
      player.playFolder(b.folder, random(1, b.file + 1) );
      tPlayerIterator.restart();
    }
  }
  else if ( b.cmd == C_SET_VOLUME ) {
    if ( b.file >= 1 && b.file <= 30 ) {
      player.volume( b.file );
      tPlayerIterator.restart();
    }
  }
  else if ( b.cmd == C_PAUSE ) {
    stopContinousPlay();
  }
  else if ( b.cmd == C_FLASH_LEDS ) {
    flash_leds();
  }
  else if ( b.cmd == C_LASERS_ON ) {
    lasers_on();
  }
  else if ( b.cmd == C_LASERS_OFF ) {
    lasers_off();
  }
}


void onI2CReceive(int howMany) {
  TransmitBuffer b;

  if (howMany > 0) {
    b.cmd = Wire.read();
    b.folder = 0;
    b.file = 0;
    if (howMany == 3) {
      b.folder = Wire.read();
      b.file = Wire.read();
    }
    q.push(b);
    tHandleCommand.restart();
  }

  // clear the buffer if there is anything else
  while (Wire.available()) Wire.read();
}

void onI2CRequest() {
  Wire.write( (byte) (digitalRead(BUSY)) );
}

void startContinousPlay(int aFolder, int aFile) {
  if ( fileNumber > 0 ) stopContinousPlay();
  fileNumber = aFile;
  folderNumber = aFolder;
  player.playFolder(folderNumber, fileNumber);
  tPlayerIterator.restart();
  attachInterrupt(digitalPinToInterrupt(BUSY), restartPlayISR, RISING);
}

void stopContinousPlay() {
  detachInterrupt(digitalPinToInterrupt(BUSY));
  player.pause();
  tPlayerIterator.restart();
  fileNumber = 0;
  folderNumber = 0;
}

void restartPlay() {
  player.playFolder(folderNumber, fileNumber);
  tPlayerIterator.restart();
}

void restartPlayISR() {
  tRestartPlay.restart();
}

void iterateCallback() {
  if ( player.commandCompleted() ) tPlayerIterator.disable();
}
