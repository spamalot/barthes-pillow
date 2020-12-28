// (c) 2019 Matthew Lakier (matthew.lakier@uwaterloo.ca)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <avr/sleep.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <EEPROM.h>
#include "pillowtypes.h"
#include "logging.h"


/* *************************************************************** */
/* Constants
/* *************************************************************** */

/**
 * Arduino Pro Mini has 2 external interrupts: 0 and 1, associated
 * with digital pins 2 and 3.
 *
 * Because we only have two interrupts, we multiplex the
 * VIBRATION_PIN so that it also triggers when the *switch* goes LOW
 * (the phone is removed).
 */
#define VIBRATION_PIN 2
#define VIBRATION_INT 0
#define SWITCH_PIN 3
#define SWITCH_INT 1

/** Pin controlling power for MP3 module. */
#define RELAY_PIN 6
/** TX pin for MP3 module. */
#define MP3_TX_PIN 8
/**
 * RX pin for MP3 module -- note we don't actually read from the
 * MP3 module.
 */
#define MP3_RX_PIN 7

/**
 * LED pins.
 */
#define INNER_LED_PIN 10
#define OUTER1_LED_PIN 9
#define OUTER2_LED_PIN 11

/**
 * Animation constants, all in MILLISECONDS.
 */
#define HEARTBEAT_PERIOD 1800ul
#define BUMP_BLINK_TIME 500ul
#define INSERT_BLINK_TIME 500ul
#define FADE_TIME 500ul
/** Frame rate for animation. */
#define TIME_STEP 20ul
/** How much time to allow the phone to be removed for before
 * triggering a removed event.
 */
#define NOPHONE_MARGIN_LIMIT 1000ul

//
// Time-frame constants, all in SECONDS.
//
/** How long the heartbeat lights and sounds go on for. */
#define HEARTBEAT_TIME 30ul
/**
 * How much delay between inserting the phone and having the
 * heartbeat routine begin. Note that initializing the MP3 module
 * takes 2-3 additional seconds.
 */
#define PRE_HEARTBEAT_DELAY_TIME 3ul
/** How often to write a elapsed time event to the log. */
#define ELAPSED_TIME_LOG_TIME 1800ul
/**
 * How much time to wait before starting logging (to allow for the
 * erase command to be initiated).
 */
#define DONT_LOG_UNTIL_TIME 10ul



/**
 * MP3 module volume (int). Valid range 0 - 30.
 */
#define MP3_VOLUME 22

/**
 * LED brightnesses (int). Valid range 0 - 255.
 */
#define LED_BRIGHT 255
#define LED_MEDIUM 128
#define LED_DIM 64
#define LED_VERY_DIM 32

/**
 * Logging-related constants.
 */
#define LOG_ADDR 16


/* *************************************************************** */
/* General
/* *************************************************************** */

SoftwareSerial mp3ModuleSoftwareSerial(MP3_RX_PIN, MP3_TX_PIN); // RX, TX
DFRobotDFPlayerMini mp3Module;

/** Animation time counter */
unsigned long ctr = 0;

/** Time elapsed counter. Adds 1 for every ~8 seconds passed. */
unsigned long elapsedTimeCtr = 0;

/** Time elapsed since detecting the phone as being removed. Counts
 * up until NOPHONE_MARGIN_LIMIT, at which point we trigger a
 * removed event.
 */
unsigned long nophoneMarginCtr = 0;

State currentState = DEFAULT_STATE;

/** Logging State */
struct PillowLog globalLog;

/**
 * Whether the Arduino is was in sleep mode before an interrupt.
 * Needed by the watchdog timer to know if to log an event.
 */
bool is_sleep = false;
/**
 * Whether the Arduino was awakened due to the watchdog timer.
 * Needed to distinguish an external interrupt from the watchdog
 * interrupt.
 */
volatile bool is_wdt = false;


/**
 * ISR for watchdog timer.
 */
ISR(WDT_vect)
{
  if (!is_sleep) {
    return;
  }
  is_wdt = true;
}

/**
 * Write debug text to Arduino software console.
 */
inline void debugLog(char * text) {
  Serial.print(F("LOG: "));
  Serial.println(text);
  Serial.flush();
}

inline void debugLog(const __FlashStringHelper * text) {
  // Overload that enables printing strings from flash memory.
  Serial.print(F("LOG: "));
  Serial.println(text);
  Serial.flush();
}

/**
 * Write an event of the given type to the current log trial.
 */
void updateLog(char eventType) {
  if (globalLog.overflowed) {
    debugLog(F("Log overflowed: no more events written"));
    return;
  }
  globalLog.eventType[globalLog.idx] = eventType;
  globalLog.idx++;
  if (globalLog.idx >= LOGGING_EVENT_COUNT) {
    globalLog.overflowed = true;
  }
  EEPROM.put(LOG_ADDR, globalLog);
}

/**
 * Enable Serial for the MP3 module.
 */
inline void startSerial() {
  digitalWrite(RELAY_PIN, LOW);
  delay(100);
  mp3ModuleSoftwareSerial.begin(9600);
}

/**
 * Disable serial for the MP3 module.
 */
inline void stopSerial() {
  mp3ModuleSoftwareSerial.flush();
  mp3ModuleSoftwareSerial.end();
  delay(100);
  digitalWrite(MP3_TX_PIN, LOW);
  digitalWrite(RELAY_PIN, HIGH);
}

void setup() {
  // Relay pin is ACTIVE LOW
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(OUTER1_LED_PIN, OUTPUT);
  pinMode(INNER_LED_PIN, OUTPUT);
  pinMode(OUTER2_LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);
  // Don't float TX pin
  digitalWrite(MP3_TX_PIN, LOW);

  //
  // Set up watchdog timer.
  //

  // Clear watchdog flag to prevent reset.
  MCUSR &= ~(1<<WDRF);
  // Enable watchdog and enable clearing of watchdog.
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set watchdog prescaler
  WDTCSR = 1<<WDP0 | 1<<WDP3; // 1 0 0 1 => 8 seconds (max)
  // Enable watchdog interrupt instead of reset
  WDTCSR |= _BV(WDIE);

  Serial.begin(115200);
  delay(1000);
  debugLog(F("Serial output is working"));
  Serial.flush();

  //
  // Log handling.
  //

  EEPROM.get(LOG_ADDR, globalLog);

  // Update log counter.
  if (globalLog.magic != LOGGING_HEADER_MAGIC) {
    Serial.println(F("Incorrect magic; doing first time init"));
    globalLog.magic = LOGGING_HEADER_MAGIC;
    globalLog.count = 0;
    globalLog.idx = 0;
    globalLog.overflowed = false;
  } else {
    globalLog.count++;
  }

  EEPROM.put(LOG_ADDR, globalLog);

  // Read data stored in log.
  Serial.println(F("META"));
  Serial.print(F("Power On Count "));
  Serial.println(globalLog.count);
  Serial.print(F("Overflowed? "));
  Serial.println(globalLog.overflowed);
  Serial.print(F("["));
  for (int i = 0; i < globalLog.idx; i++) {
    Serial.print((char) globalLog.eventType[i]);
  }
  Serial.println(F("]"));
  Serial.flush();

  delay(DONT_LOG_UNTIL_TIME*1000ul);

  // Enable clearing of log.
  if (Serial.read() == '!') {
    debugLog(F("Erase command initiated, clearing trial data..."));
    globalLog.idx = 0;
    globalLog.overflowed = false;
    EEPROM.put(LOG_ADDR, globalLog);
    debugLog(F("Please restart."));
    while(1);
  }

  updateLog(TYPE_POWER_ON);
}

/**
 * Necessary to have external interrupts trigger. Do nothing.
 */
void onWake() {}

/**
 * Enable the specified interrupt (either VIBRATION_INT or
 * SWITCH_INT) and put the Arduino to sleep.
 */
void cfgIntAndSleep(int theInt) {
  is_wdt = false;
  is_sleep = true;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // External interrupts can only trigger on LOW.
  attachInterrupt(theInt, onWake, LOW);
  sleep_enable();
  sleep_bod_disable();
  // Wakes up on LOW external trigger or watchdog.
  sleep_mode();
  // Resumes here:
  sleep_disable();
  detachInterrupt(theInt);
  is_sleep = false;
}

/**
 * Return true once the phone has been removed for longer
 * than NOPHONE_MARGIN_LIMIT.
 */
PhoneMarginState isPhoneRemoved() {
  if (digitalRead(SWITCH_PIN) == HIGH) {
    nophoneMarginCtr++;
    if (nophoneMarginCtr >= NOPHONE_MARGIN_LIMIT/TIME_STEP) {
      nophoneMarginCtr = 0;
      return PHONE_MARGIN_REMOVED;
    }
    return PHONE_MARGIN_OUT;
  }
  nophoneMarginCtr = 0;
  return PHONE_MARGIN_IN;
}

/**
 * Same as isPhoneRemoved(), but the function will block until the
 * phone is either removed for longer than NOPHONE_MARGIN_LIMIT or
 * put back in. This is necessary to prevent the Arduino from going
 * back to sleep and never interrupting again.
 */
PhoneMarginState isPhoneRemovedSync() {
  bool wasRemoved = false;
  while (true) {
    PhoneMarginState marginState = isPhoneRemoved();
    if (marginState == PHONE_MARGIN_OUT) {
      wasRemoved = true;
    }
    if (marginState == PHONE_MARGIN_IN) {
      if (wasRemoved) {
        return PHONE_MARGIN_OUT;
      }
      return PHONE_MARGIN_IN;
    }
    if (marginState == PHONE_MARGIN_REMOVED) {
      return PHONE_MARGIN_REMOVED;
    }
    delay(TIME_STEP);
  }
}

/**
 * Return whether the watchdog timer triggered. If it did, log an
 * event every ELAPSED_TIME_LOG_TIME seconds.
 */
bool handleWdt() {
  if (!is_wdt) {
    return false;
  }
  elapsedTimeCtr++;
  if (elapsedTimeCtr % (ELAPSED_TIME_LOG_TIME / 8) == 0) {
    debugLog(F("Time elapsed"));
    updateLog(TYPE_TIME_ELAPSED);
  }
  return true;
}


/* *************************************************************** */
/* Animation Routines
/* *************************************************************** */

/**
 * Heartbeat animation routine.
 */
void asyncStepOutputs() {
  float xw = ctr*TIME_STEP*(2.0*PI)/HEARTBEAT_PERIOD;

  // Envelope
  float envelopeBase = ctr*TIME_STEP/(HEARTBEAT_TIME*1000.0);

  // Heartbeat pattern
  float heartbeat =     LED_BRIGHT
                          * max(0, sin(xw + PI/2.0*0.95)
                                 * sin(2*xw)
                                )*1.2
                          * (1 - envelopeBase);
  float heartbeatecho = LED_MEDIUM
                          * max(0, sin(xw + PI/2.0*0.95 - 2.0*PI*0.05)
                                 * sin(2*xw - 2*2.0*PI*0.05)
                                )*1.2
                          * (1 - envelopeBase);
  analogWrite(INNER_LED_PIN, heartbeat);
  analogWrite(OUTER1_LED_PIN, heartbeatecho);
  analogWrite(OUTER2_LED_PIN, heartbeatecho);

  if (ctr*TIME_STEP % HEARTBEAT_PERIOD == 0) {
    mp3Module.volume((int) (MP3_VOLUME * (1 - envelopeBase*envelopeBase) ));
    mp3Module.play(1);
  }

  ctr++;
  delay(TIME_STEP);
}

/**
 * Executes when vibration switch triggered in sleep mode.
 */
void syncBlinkOutputs() {
  for (unsigned long i = 0;
        i*TIME_STEP < BUMP_BLINK_TIME + BUMP_BLINK_TIME/4.0; i++) {
    float xw = i*TIME_STEP*(2.0*PI)/BUMP_BLINK_TIME;

    if (i*TIME_STEP > BUMP_BLINK_TIME) {
      analogWrite(INNER_LED_PIN, 0);
    } else {
      analogWrite(INNER_LED_PIN, LED_DIM/2 + (LED_DIM/2-1)*sin(xw - PI/2.0));
    }
    if (i*TIME_STEP < BUMP_BLINK_TIME/4.0) {
      analogWrite(OUTER1_LED_PIN, 0);
      analogWrite(OUTER2_LED_PIN, 0);
    } else {
      analogWrite(OUTER1_LED_PIN,
        LED_VERY_DIM/2 + (LED_VERY_DIM/2-1)*sin(xw - PI));
      analogWrite(OUTER2_LED_PIN,
        LED_VERY_DIM/2 + (LED_VERY_DIM/2-1)*sin(xw - PI));
    }

    delay(TIME_STEP);
  }
}

/**
 * Executes when phone is inserted.
 */
void asyncBlinkOutputs() {
  float xw = ctr*TIME_STEP*(2.0*PI)/INSERT_BLINK_TIME;

  if (ctr*TIME_STEP < INSERT_BLINK_TIME) {
    analogWrite(OUTER1_LED_PIN,
      LED_BRIGHT/2 + (LED_BRIGHT/2-1)*sin(xw - PI/2.0));
    analogWrite(OUTER2_LED_PIN,
      LED_BRIGHT/2 + (LED_BRIGHT/2-1)*sin(xw - PI/2.0));
  } else {
    analogWrite(OUTER1_LED_PIN, 0);
    analogWrite(OUTER2_LED_PIN, 0);
  }

  ctr++;
  delay(TIME_STEP);
}

/**
 * Executes when phone is removed.
 */
void syncFadeOutputs() {
  for (unsigned long i = 0; i*TIME_STEP < FADE_TIME; i++) {
    float xw = i*TIME_STEP*(2.0*PI)/FADE_TIME;
    analogWrite(INNER_LED_PIN,
      LED_BRIGHT/2 + (LED_BRIGHT/2-1)*sin(xw - PI/2.0));
    delay(TIME_STEP);
  }
}

/**
 * Makes sure all LEDs go off between states and resets animation
 * frame counter.
 */
void shutoffAction() {
  analogWrite(OUTER1_LED_PIN, 0);
  analogWrite(INNER_LED_PIN, 0);
  analogWrite(OUTER2_LED_PIN, 0);
  ctr = 0;
}


/* *************************************************************** */
/* State Machine
/* *************************************************************** */

State doDefaultState() {
  debugLog(F("default -> nophone"));
  return NOPHONE_STATE;
}

State doNophoneState() {
  if (digitalRead(SWITCH_PIN) == LOW) {
    updateLog(TYPE_INSERT);
    debugLog(F("nophone -> phoneinserted"));
    return PHONE_INSERTED_STATE;
  }

  debugLog(F("nophone -> nophone_sleep"));
  return NOPHONE_SLEEP_STATE;
}

State doNophoneSleepState() {
  debugLog(F("sleeping! (wake up with sw->LOW)"));
  shutoffAction();
  cfgIntAndSleep(SWITCH_INT);

  if (handleWdt()) {
    debugLog(F("nophone_sleep -> nophone_sleep"));
    return NOPHONE_SLEEP_STATE;
  }

  debugLog(F("nophone_sleep -> nophone"));
  return NOPHONE_STATE;
}

State doPhoneInsertedState() {
  if (isPhoneRemoved() == PHONE_MARGIN_REMOVED) {
    shutoffAction();
    updateLog(TYPE_REMOVE_ON);
    debugLog(F("phoneinserted -> nophone"));
    return PHONE_REMOVED_STATE;
  }

  if (ctr*TIME_STEP >= PRE_HEARTBEAT_DELAY_TIME*1000ul) {
    shutoffAction();
    debugLog(F("phoneinserted -> phonein"));
    return PHONEIN_SOUNDON_STATE;
  }

  asyncBlinkOutputs();
  return PHONE_INSERTED_STATE;
}

State doPhoneinSoundonState() {
  startSerial();
  delay(1000);
  debugLog(F("Initializing MP3 module"));
  if (!mp3Module.begin(mp3ModuleSoftwareSerial, true, false)) {
    // If the code gets here, there's probably a hardware issue with
    // the MP3 module.
    debugLog(F("Unable to initialize MP3 module"));
    while(1);
  } else {
    debugLog(F("MP3 module initialized"));
    mp3Module.reset();
    delay(100);
    mp3Module.volume(MP3_VOLUME);
  }

  debugLog(F("phonein_soundon -> phonein"));
  return PHONEIN_STATE;
}

State doPhoneinState() {
  if (isPhoneRemoved() == PHONE_MARGIN_REMOVED) {
    shutoffAction();
    debugLog(F("phonein -> phone_removed_soundoff"));
    return PHONE_REMOVED_SOUNDOFF_STATE;
  }

  if (ctr*TIME_STEP >= HEARTBEAT_TIME*1000ul) {
    shutoffAction();
    debugLog(F("phonein -> phonein_sleep_soundoff"));
    return PHONEIN_SLEEP_SOUNDOFF_STATE;
  }

  asyncStepOutputs();
  return PHONEIN_STATE;
}

State doPhoneRemovedSoundoffState() {
  stopSerial();
  updateLog(TYPE_REMOVE_ON);
  debugLog(F("phone_removed_soundoff -> phone_removed"));
  return PHONE_REMOVED_STATE;
}

State doPhoneinSleepSoundoffState() {
  stopSerial();
  debugLog(F("phonein_sleep_soundoff -> phonein_sleep"));
  return PHONEIN_SLEEP_STATE;
}

State doPhoneinSleepState() {
  debugLog(F("sleeping! (wake up with vib->HIGH or sw->HIGH)"));
  cfgIntAndSleep(VIBRATION_INT);

  PhoneMarginState marginState = isPhoneRemovedSync();
  if (marginState == PHONE_MARGIN_REMOVED) {
    updateLog(TYPE_REMOVE_SLEEP);
    debugLog(F("phonein_sleep -> phone_removed"));
    return PHONE_REMOVED_STATE;
  }
  if (marginState == PHONE_MARGIN_OUT) {
    // Pretend as if nothing happened
    debugLog(F("phonein_sleep -> phonein_sleep"));
    return PHONEIN_SLEEP_STATE;
  }

  if (handleWdt()) {
    debugLog(F("phonein_sleep -> phonein_sleep"));
    return PHONEIN_SLEEP_STATE;
  }

  debugLog(F("phonein_sleep -> vibrated"));
  return VIBRATED_STATE;
}

State doVibratedState() {
  syncBlinkOutputs();
  shutoffAction();

  updateLog(TYPE_BUMP);
  debugLog(F("vibrated -> phonein_sleep"));
  return PHONEIN_SLEEP_STATE;
}

State doPhoneRemovedState() {
    syncFadeOutputs();
    shutoffAction();

    debugLog(F("phone_removed -> nophone"));
    return NOPHONE_STATE;
}

void loop() {
  switch (currentState) {
    case DEFAULT_STATE:
                    currentState = doDefaultState(); break;
    case NOPHONE_STATE:
                    currentState = doNophoneState(); break;
    case NOPHONE_SLEEP_STATE:
                    currentState = doNophoneSleepState(); break;
    case PHONE_INSERTED_STATE:
                    currentState = doPhoneInsertedState(); break;
    case PHONE_REMOVED_STATE:
                    currentState = doPhoneRemovedState(); break;
    case PHONEIN_SOUNDON_STATE:
                    currentState = doPhoneinSoundonState(); break;
    case PHONEIN_STATE:
                    currentState = doPhoneinState(); break;
    case PHONE_REMOVED_SOUNDOFF_STATE:
                    currentState = doPhoneRemovedSoundoffState(); break;
    case PHONEIN_SLEEP_SOUNDOFF_STATE:
                    currentState = doPhoneinSleepSoundoffState(); break;
    case PHONEIN_SLEEP_STATE:
                    currentState = doPhoneinSleepState(); break;
    case VIBRATED_STATE:
                    currentState = doVibratedState(); break;
    case INVALID_STATE:
    default: debugLog(F("invalid state")); break;
  }
}
