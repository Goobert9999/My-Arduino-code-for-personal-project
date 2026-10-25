/*
  ============================================================================
  SMART AUTOMATIC PLANT WATERING SYSTEM
  ============================================================================
  Hardware (matches the wiring diagram provided):
    - Arduino Uno R3
    - Capacitive soil moisture sensor -> AO to A0, VCC/GND to breadboard rails
    - 1-channel 5V relay module      -> IN to D7, VCC/GND to breadboard rails
    - Breadboard +/- rails fed from Arduino 5V / GND
    - 4.8V battery pack -> Relay COM, Relay NO -> Pump(+), Pump(-) -> Battery(-)
      (This is a fully separate circuit from the Arduino logic side. The
       relay's contacts isolate the pump/battery loop from the Arduino, which
       is exactly right and protects the Uno from the pump's current draw.)

  WHY THIS DESIGN IS "SMART":
    1. Median-filtered sensor readings   -> ignores electrical noise/spikes
    2. Calibration constants             -> works with YOUR sensor, not a guess
    3. Hysteresis-free threshold + soak  -> avoids re-triggering on damp topsoil
    4. Minimum cooldown between waterings-> prevents overwatering / root rot
    5. Hard-capped watering duration     -> pump can never run away
    6. Post-watering verification        -> confirms the water actually helped
    7. Reservoir-empty / dry-run detection -> stops trying if watering isn't
       raising moisture (protects a submersible pump from running dry)
    8. Sensor fault detection            -> won't "water blind" if the sensor
       is disconnected or shorted, and self-heals if it reconnects
    9. Daily watering cap                -> extra safety ceiling
   10. Fault state survives resets (EEPROM) -> a brownout/reset won't erase
       a "reservoir empty" warning and silently resume pumping
   11. Non-blocking state machine (millis()) -> stays responsive, easy to extend
   12. Serial console for calibration, status, manual test, and reset

  ----------------------------------------------------------------------------
  FIRST-TIME SETUP - READ THIS:
  ----------------------------------------------------------------------------
  1. Set RELAY_ACTIVE_LOW below. Most cheap single-channel relay boards are
     ACTIVE LOW (a LOW signal energizes the relay). If your pump turns ON
     when you expect OFF (or vice versa), flip this constant.
  2. Calibrate the sensor:
       - Upload this sketch, open Serial Monitor at 9600 baud.
       - Type 'c' and press Enter with the sensor completely dry (in air).
         Note the raw value printed -> put it in calAirValue.
       - Type 'c' again with the sensor tip fully submerged in a cup of
         water. Note the raw value -> put it in calWaterValue.
       - Re-upload with the updated numbers.
  3. Use 't' to run a 2-second pump test pulse to confirm wiring/tube priming
     before you trust the automatic logic.
  4. Tune MOISTURE_LOW_THRESHOLD, WATER_DURATION_MS, SOAK_TIME_MS and
     COOLDOWN_MS for your pot size, soil, and pump flow rate.
  ============================================================================
*/

#include <EEPROM.h>

// ============================================================================
// PIN ASSIGNMENTS
// ============================================================================
const uint8_t PIN_SOIL       = A0;
const uint8_t PIN_RELAY      = 7;
const uint8_t PIN_STATUS_LED = LED_BUILTIN; // pin 13, onboard LED, no extra wiring needed

// ============================================================================
// RELAY LOGIC
// ============================================================================
// true  = board energizes the relay on a LOW signal (most common on cheap boards)
// false = board energizes the relay on a HIGH signal
const bool RELAY_ACTIVE_LOW = true;

// ============================================================================
// SENSOR CALIBRATION (raw 0-1023 analogRead values)
// Capacitive sensors read HIGHER when dry, LOWER when wet.
// ============================================================================
int calAirValue   = 520;   // <-- replace with YOUR sensor's dry (in-air) reading
int calWaterValue = 260;   // <-- replace with YOUR sensor's wet (submerged) reading

// ============================================================================
// MOISTURE THRESHOLDS (in %, after calibration)
// ============================================================================
const int MOISTURE_LOW_THRESHOLD = 30;  // start watering when moisture drops below this
const int MOISTURE_SAFETY_MAX    = 85;  // abort watering early if this is ever exceeded

// ============================================================================
// TIMING (all in milliseconds) -- tune these for your setup
// ============================================================================
const unsigned long WATER_DURATION_MS         = 5000UL;              // pump run time per cycle
const unsigned long HARD_MAX_WATER_MS         = 15000UL;             // absolute ceiling, never exceeded even if logic misbehaves
const unsigned long SOAK_TIME_MS              = 10UL * 60UL * 1000UL;  // wait 10 min after watering before re-checking soil
const unsigned long COOLDOWN_MS               = 4UL * 60UL * 60UL * 1000UL; // minimum 4 hrs between watering attempts
const unsigned long SENSOR_POLL_INTERVAL_MS   = 2000UL;              // how often to check soil while idle
const unsigned long WATERING_SAFETY_CHECK_MS  = 1000UL;              // how often to check the safety-max moisture cutoff while pumping
const unsigned long DAY_MS                    = 24UL * 60UL * 60UL * 1000UL;
const unsigned long FAULT_REMINDER_MS         = 30UL * 1000UL;       // how often to reprint the fault warning
const unsigned long SENSOR_FAULT_REMINDER_MS  = 15UL * 1000UL;

// ============================================================================
// SAFETY LIMITS
// ============================================================================
const uint8_t MAX_WATERINGS_PER_DAY    = 4;   // hard cap even if soil stays dry
const uint8_t MAX_CONSECUTIVE_FAILURES = 3;   // moisture not rising this many times in a row -> assume empty reservoir / bad tube
const uint8_t MOISTURE_IMPROVE_MARGIN  = 3;   // % increase required to count a watering as "successful"
const uint8_t SENSOR_FAULT_COUNT_TRIP  = 4;   // consecutive bad raw readings before declaring a sensor fault
const uint8_t SENSOR_FAULT_COUNT_CLEAR = 4;   // consecutive good raw readings before clearing a sensor fault

// ============================================================================
// EEPROM LAYOUT (persists the fault state across resets/power loss)
// ============================================================================
const int EE_ADDR_MAGIC     = 0;
const int EE_ADDR_FAULT     = 1;
const int EE_ADDR_FAILCOUNT = 2;
const byte EE_MAGIC_VALUE   = 0xA5;

// ============================================================================
// STATE MACHINE
// ============================================================================
enum SystemState {
  STATE_IDLE,
  STATE_WATERING,
  STATE_SOAKING,
  STATE_FAULT,          // reservoir/pump problem suspected - watering halted
  STATE_SENSOR_ERROR    // sensor readings look invalid - watering halted
};

SystemState state = STATE_IDLE;

// ---- runtime variables ----
bool relayIsOn = false;

unsigned long lastWateringAttemptTime = 0;   // used for the cooldown timer
unsigned long wateringStartTime       = 0;
unsigned long soakStartTime           = 0;
unsigned long lastSensorPoll          = 0;
unsigned long lastWateringSafetyCheck = 0;
unsigned long lastFaultReminder       = 0;
unsigned long dayWindowStart          = 0;

int  preWateringMoisture = 0;
bool manualTestInProgress = false;   // true when 't' pump-test pulse is running

uint8_t wateringsToday       = 0;
uint8_t consecutiveFailures  = 0;
uint8_t badSensorStreak      = 0;
uint8_t goodSensorStreak     = 0;
bool    inFault              = false;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Set the relay pin to its OFF state BEFORE anything else touches it, to
  // minimize any chance of a power-up glitch turning the pump on briefly.
  pinMode(PIN_RELAY, OUTPUT);
  setRelay(false);

  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.begin(9600);
  delay(200);

  loadPersistentState();

  dayWindowStart = millis();
  lastWateringAttemptTime = 0; // 0 means "no cooldown restriction yet" (handled in logic)

  Serial.println(F("============================================"));
  Serial.println(F(" Smart Auto-Watering System - booting up"));
  Serial.println(F("============================================"));
  printHelp();

  if (inFault) {
    state = STATE_FAULT;
    Serial.println(F("[BOOT] Resuming in FAULT state from before reset."));
    Serial.println(F("       Check the reservoir/tubing, then send 'r' to clear."));
  } else {
    Serial.println(F("[BOOT] Starting in IDLE state."));
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  handleSerialCommands();
  rollDailyCounterIfNeeded();
  updateStatusLED();

  switch (state) {
    case STATE_IDLE:        runIdle();        break;
    case STATE_WATERING:    runWatering();    break;
    case STATE_SOAKING:     runSoaking();     break;
    case STATE_FAULT:       runFault();       break;
    case STATE_SENSOR_ERROR: runSensorError(); break;
  }
}

// ============================================================================
// STATE HANDLERS
// ============================================================================
void runIdle() {
  if (millis() - lastSensorPoll < SENSOR_POLL_INTERVAL_MS) return;
  lastSensorPoll = millis();

  int raw = readSoilRawFiltered();

  if (isRawSuspicious(raw)) {
    badSensorStreak++;
    goodSensorStreak = 0;
    if (badSensorStreak >= SENSOR_FAULT_COUNT_TRIP) {
      state = STATE_SENSOR_ERROR;
      lastFaultReminder = 0; // force an immediate message
      Serial.println(F("[WARN] Soil sensor readings look invalid (disconnected or shorted)."));
      Serial.println(F("       Pausing automatic watering until this resolves."));
    }
    return;
  } else {
    badSensorStreak = 0;
  }

  int percent = soilRawToPercent(raw);
  Serial.print(F("[IDLE] Soil moisture: "));
  Serial.print(percent);
  Serial.println(F("%"));

  if (percent >= MOISTURE_LOW_THRESHOLD) {
    return; // soil is fine, nothing to do
  }

  // Soil is dry - check whether we're allowed to water right now.
  bool cooldownOk = (lastWateringAttemptTime == 0) ||
                     (millis() - lastWateringAttemptTime >= COOLDOWN_MS);
  bool dailyLimitOk = (wateringsToday < MAX_WATERINGS_PER_DAY);

  if (!dailyLimitOk) {
    Serial.println(F("[IDLE] Soil is dry, but the daily watering limit has been reached. Waiting for reset window."));
    return;
  }
  if (!cooldownOk) {
    unsigned long remaining = COOLDOWN_MS - (millis() - lastWateringAttemptTime);
    Serial.print(F("[IDLE] Soil is dry, but still in cooldown. Time left: "));
    printDuration(remaining);
    return;
  }

  startWateringCycle(percent);
}

void runWatering() {
  unsigned long elapsed = millis() - wateringStartTime;

  // Periodic safety check: abort early if moisture spikes unexpectedly high
  // (guards against a misread or a flooded pot).
  if (millis() - lastWateringSafetyCheck >= WATERING_SAFETY_CHECK_MS) {
    lastWateringSafetyCheck = millis();
    int raw = readSoilRawFiltered();
    if (!isRawSuspicious(raw)) {
      int percent = soilRawToPercent(raw);
      if (percent >= MOISTURE_SAFETY_MAX) {
        Serial.println(F("[SAFETY] Moisture spiked above safety threshold mid-cycle - stopping pump early."));
        finishWateringCycle();
        return;
      }
    }
  }

  unsigned long cap = manualTestInProgress ? 2000UL : WATER_DURATION_MS;
  if (cap > HARD_MAX_WATER_MS) cap = HARD_MAX_WATER_MS; // absolute ceiling, always enforced

  if (elapsed >= cap) {
    finishWateringCycle();
  }
}

void runSoaking() {
  if (millis() - soakStartTime < SOAK_TIME_MS) return;

  int raw = readSoilRawFiltered();
  if (isRawSuspicious(raw)) {
    // Can't verify the result - treat conservatively as a sensor problem,
    // not a watering failure, so we don't punish the reservoir for a bad sensor.
    Serial.println(F("[SOAK] Sensor reading invalid after soak - cannot verify watering result."));
    state = STATE_SENSOR_ERROR;
    lastFaultReminder = 0;
    return;
  }

  int postPercent = soilRawToPercent(raw);
  Serial.print(F("[SOAK] Moisture after soak: "));
  Serial.print(postPercent);
  Serial.print(F("% (was "));
  Serial.print(preWateringMoisture);
  Serial.println(F("% before watering)."));

  lastWateringAttemptTime = millis(); // cooldown applies whether it worked or not
  wateringsToday++;

  if (postPercent >= preWateringMoisture + MOISTURE_IMPROVE_MARGIN) {
    Serial.println(F("[SOAK] Watering successful - moisture improved."));
    consecutiveFailures = 0;
    savePersistentState();
    state = STATE_IDLE;
  } else {
    consecutiveFailures++;
    Serial.print(F("[SOAK] Watering did not raise moisture as expected. Failure "));
    Serial.print(consecutiveFailures);
    Serial.print(F("/"));
    Serial.println(MAX_CONSECUTIVE_FAILURES);
    savePersistentState();

    if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      inFault = true;
      savePersistentState();
      state = STATE_FAULT;
      lastFaultReminder = 0;
      Serial.println(F("[FAULT] Repeated failed waterings. Reservoir may be empty, tube may be"));
      Serial.println(F("        blocked/kinked, or pump may have failed. Watering is now paused."));
      Serial.println(F("        Fix the issue, then send 'r' over Serial to resume."));
    } else {
      state = STATE_IDLE;
    }
  }
}

void runFault() {
  ensureRelayOff();
  if (millis() - lastFaultReminder >= FAULT_REMINDER_MS) {
    lastFaultReminder = millis();
    Serial.println(F("[FAULT] System paused - check reservoir/tubing/pump, then send 'r' to reset."));
  }
}

void runSensorError() {
  ensureRelayOff();

  if (millis() - lastFaultReminder >= SENSOR_FAULT_REMINDER_MS) {
    lastFaultReminder = millis();
    Serial.println(F("[SENSOR ERROR] Soil sensor reading is out of range - check the connection."));
    Serial.println(F("               Automatic watering is paused. Will resume automatically once"));
    Serial.println(F("               readings look valid again, or send 'r' to force-clear."));
  }

  // Auto-recovery: keep polling; if several consecutive readings look sane again,
  // trust the sensor again without requiring a manual reset.
  if (millis() - lastSensorPoll < SENSOR_POLL_INTERVAL_MS) return;
  lastSensorPoll = millis();

  int raw = readSoilRawFiltered();
  if (isRawSuspicious(raw)) {
    goodSensorStreak = 0;
  } else {
    goodSensorStreak++;
    if (goodSensorStreak >= SENSOR_FAULT_COUNT_CLEAR) {
      Serial.println(F("[SENSOR] Readings look valid again. Resuming normal operation."));
      badSensorStreak = 0;
      goodSensorStreak = 0;
      state = STATE_IDLE;
    }
  }
}

// ============================================================================
// WATERING CYCLE HELPERS
// ============================================================================
void startWateringCycle(int currentPercent) {
  preWateringMoisture = currentPercent;
  wateringStartTime = millis();
  lastWateringSafetyCheck = millis();
  manualTestInProgress = false;
  setRelay(true);
  state = STATE_WATERING;
  Serial.print(F("[WATER] Starting pump. Soil was at "));
  Serial.print(currentPercent);
  Serial.println(F("%."));
}

void finishWateringCycle() {
  setRelay(false);
  Serial.println(F("[WATER] Pump stopped."));

  if (manualTestInProgress) {
    // Manual test pulses don't count toward daily limits, cooldowns, or
    // fault detection - they're just for confirming the plumbing works.
    manualTestInProgress = false;
    state = STATE_IDLE;
    return;
  }

  soakStartTime = millis();
  state = STATE_SOAKING;
  Serial.print(F("[WATER] Soaking for "));
  printDuration(SOAK_TIME_MS);
}

void ensureRelayOff() {
  if (relayIsOn) setRelay(false);
}

// ============================================================================
// SENSOR HELPERS
// ============================================================================
int readSoilRawFiltered() {
  const uint8_t N = 15;
  int samples[N];
  for (uint8_t i = 0; i < N; i++) {
    samples[i] = analogRead(PIN_SOIL);
    delay(8); // brief settle time between samples
  }
  // insertion sort (N is tiny, this is plenty fast)
  for (uint8_t i = 1; i < N; i++) {
    int key = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > key) {
      samples[j + 1] = samples[j];
      j--;
    }
    samples[j + 1] = key;
  }
  return samples[N / 2]; // median value, robust against single-sample noise spikes
}

bool isRawSuspicious(int raw) {
  // Values pinned near the ADC rails almost always mean a disconnected,
  // shorted, or unpowered sensor rather than a real reading.
  return (raw <= 2 || raw >= 1021);
}

int soilRawToPercent(int raw) {
  int lo = min(calAirValue, calWaterValue);
  int hi = max(calAirValue, calWaterValue);
  raw = constrain(raw, lo, hi);
  int percent = map(raw, calAirValue, calWaterValue, 0, 100);
  return constrain(percent, 0, 100);
}

// ============================================================================
// RELAY / LED HELPERS
// ============================================================================
void setRelay(bool energize) {
  int pinState;
  if (RELAY_ACTIVE_LOW) {
    pinState = energize ? LOW : HIGH;
  } else {
    pinState = energize ? HIGH : LOW;
  }
  digitalWrite(PIN_RELAY, pinState);
  relayIsOn = energize;
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledOn = false;

  switch (state) {
    case STATE_WATERING:
      digitalWrite(PIN_STATUS_LED, HIGH); // solid ON while actively pumping
      break;
    case STATE_FAULT:
    case STATE_SENSOR_ERROR: {
      unsigned long interval = (state == STATE_FAULT) ? 200UL : 600UL; // fast=fault, slower=sensor issue
      if (millis() - lastBlink >= interval) {
        lastBlink = millis();
        ledOn = !ledOn;
        digitalWrite(PIN_STATUS_LED, ledOn ? HIGH : LOW);
      }
      break;
    }
    default:
      digitalWrite(PIN_STATUS_LED, LOW); // off while idle/soaking
      break;
  }
}

// ============================================================================
// DAILY COUNTER
// ============================================================================
void rollDailyCounterIfNeeded() {
  if (millis() - dayWindowStart >= DAY_MS) {
    dayWindowStart = millis();
    wateringsToday = 0;
    Serial.println(F("[INFO] Daily watering counter reset."));
  }
}

// ============================================================================
// EEPROM PERSISTENCE (survives reset / power loss)
// ============================================================================
void loadPersistentState() {
  byte magic = EEPROM.read(EE_ADDR_MAGIC);
  if (magic == EE_MAGIC_VALUE) {
    inFault = (EEPROM.read(EE_ADDR_FAULT) == 1);
    consecutiveFailures = EEPROM.read(EE_ADDR_FAILCOUNT);
  } else {
    EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC_VALUE);
    EEPROM.write(EE_ADDR_FAULT, 0);
    EEPROM.write(EE_ADDR_FAILCOUNT, 0);
    inFault = false;
    consecutiveFailures = 0;
  }
}

void savePersistentState() {
  EEPROM.update(EE_ADDR_FAULT, inFault ? 1 : 0);
  EEPROM.update(EE_ADDR_FAILCOUNT, consecutiveFailures);
}

// ============================================================================
// SERIAL CONSOLE
// ============================================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 'r': case 'R':
      Serial.println(F("[CMD] Clearing fault state."));
      inFault = false;
      consecutiveFailures = 0;
      badSensorStreak = 0;
      goodSensorStreak = 0;
      savePersistentState();
      ensureRelayOff();
      state = STATE_IDLE;
      break;

    case 'c': case 'C': {
      int raw = readSoilRawFiltered();
      Serial.print(F("[CAL] Raw ADC: "));
      Serial.print(raw);
      if (!isRawSuspicious(raw)) {
        Serial.print(F("  ->  "));
        Serial.print(soilRawToPercent(raw));
        Serial.print(F("% (using current calibration)"));
      } else {
        Serial.print(F("  (looks invalid - check wiring)"));
      }
      Serial.println();
      break;
    }

    case 't': case 'T':
      if (state == STATE_IDLE) {
        Serial.println(F("[CMD] Running 2-second pump test pulse..."));
        manualTestInProgress = true;
        wateringStartTime = millis();
        lastWateringSafetyCheck = millis();
        setRelay(true);
        state = STATE_WATERING;
      } else {
        Serial.println(F("[CMD] Ignored - system is not idle right now."));
      }
      break;

    case 'f': case 'F':
      if (state == STATE_IDLE) {
        Serial.println(F("[CMD] Forcing a watering cycle now (bypassing threshold check)."));
        int raw = readSoilRawFiltered();
        int percent = isRawSuspicious(raw) ? 0 : soilRawToPercent(raw);
        startWateringCycle(percent);
      } else {
        Serial.println(F("[CMD] Ignored - system is not idle right now."));
      }
      break;

    case 's': case 'S':
      printStatus();
      break;

    case '?':
    case 'h': case 'H':
      printHelp();
      break;

    default:
      // ignore newlines, carriage returns, and anything else unrecognized
      break;
  }
}

void printHelp() {
  Serial.println(F("Serial commands:"));
  Serial.println(F("  c = show current soil reading (for calibration)"));
  Serial.println(F("  t = 2-second pump test pulse (only while idle)"));
  Serial.println(F("  f = force a full watering cycle now (only while idle)"));
  Serial.println(F("  r = clear fault state and resume"));
  Serial.println(F("  s = print full status"));
  Serial.println(F("  h = show this help"));
}

void printStatus() {
  Serial.println(F("---------------- STATUS ----------------"));
  Serial.print(F("State: "));
  switch (state) {
    case STATE_IDLE:         Serial.println(F("IDLE")); break;
    case STATE_WATERING:     Serial.println(F("WATERING")); break;
    case STATE_SOAKING:      Serial.println(F("SOAKING")); break;
    case STATE_FAULT:        Serial.println(F("FAULT")); break;
    case STATE_SENSOR_ERROR: Serial.println(F("SENSOR_ERROR")); break;
  }
  Serial.print(F("Waterings today: "));
  Serial.print(wateringsToday);
  Serial.print(F(" / "));
  Serial.println(MAX_WATERINGS_PER_DAY);
  Serial.print(F("Consecutive failures: "));
  Serial.print(consecutiveFailures);
  Serial.print(F(" / "));
  Serial.println(MAX_CONSECUTIVE_FAILURES);
  if (lastWateringAttemptTime == 0) {
    Serial.println(F("No watering attempted yet since boot."));
  } else {
    Serial.print(F("Time since last watering attempt: "));
    printDuration(millis() - lastWateringAttemptTime);
  }
  int raw = readSoilRawFiltered();
  Serial.print(F("Current soil raw/percent: "));
  Serial.print(raw);
  if (!isRawSuspicious(raw)) {
    Serial.print(F(" / "));
    Serial.print(soilRawToPercent(raw));
    Serial.println(F("%"));
  } else {
    Serial.println(F(" (invalid)"));
  }
  Serial.println(F("-----------------------------------------"));
}

void printDuration(unsigned long ms) {
  unsigned long totalSec = ms / 1000UL;
  unsigned long h = totalSec / 3600UL;
  unsigned long m = (totalSec % 3600UL) / 60UL;
  unsigned long s = totalSec % 60UL;
  Serial.print(h);
  Serial.print(F("h "));
  Serial.print(m);
  Serial.print(F("m "));
  Serial.print(s);
  Serial.println(F("s"));
}
