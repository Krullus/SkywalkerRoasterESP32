#include <Arduino.h>
#include <limits.h>
#include <Wire.h>
#include <SparkFun_Qwiic_Humidity_AHT20.h>
#include <TMC2209.h>

#include "RoasterReceiver.h"
#include "RoasterTransmitter.h"

// --- Pins ---
constexpr int LED_PIN      = 10;
constexpr int OE_PIN       = 2;   // TXS0102 OE
constexpr int RMT_TX_PIN   = 0;   // to roaster
constexpr int RMT_RX_PIN   = 1;   // from roaster
constexpr int UART_RX_PIN  = 20;  // TMC2209 UART
constexpr int UART_TX_PIN  = 21;  // TMC2209 UART

// --- Message lengths ---
// ROASTER_LEN comes from RoasterReceiver.h
constexpr size_t CONTROLLER_LEN  = 6; // bytes (to roaster)

// --- Indices in controller buffer ---
constexpr int VENT_IDX   = 0;
constexpr int FILTER_IDX = 1;
constexpr int COOL_IDX   = 2;
constexpr int DRUM_IDX   = 3;
constexpr int HEAT_IDX   = 4;
constexpr int CHECK_IDX  = 5;

// --- Globals ---
RoasterTransmitter roasterTx(RMT_TX_PIN);
RoasterReceiver   roasterRX(RMT_RX_PIN);

uint8_t receiveBuffer[ROASTER_LEN]   = {0};
uint8_t sendBuffer[CONTROLLER_LEN]   = {0};

// Temperature state
double temp = 0.0;       // display units
double tempC = 0.0;      // internal Celsius
char   CorF = 'C';

// AHT20 ambient
AHT20 aht20;
float ambientTemp = 0.0;
float ambientHumidity = 0.0;

// Failsafe
const int maxTemp = 300;
unsigned long lastEventTime = 0;               // micros when last control/READ/actuation happened
const unsigned long lastEventTimeout = 10000000; // 10 s

// Stepper / TMC2209
HardwareSerial& serial_stream = Serial1;
const long SERIAL_BAUD_RATE = 115200;
const int  STEPS_PER_REV = 200;
const int  MICROSTEPS = 4;
constexpr float gearRatio = 4.29f; // 60/14

int targetMotorRPM = 0;
int currentMotorRPM = 0;
unsigned long lastMotorUpdate = 0;
const unsigned long motorUpdateInterval = 20; // ms

TMC2209 stepper_driver;
bool invert_direction = false;

// Current and stall settings
const uint8_t RUN_CURRENT_PERCENT  = 28; // <=40% recommended
const uint8_t HOLD_CURRENT_PERCENT = 10;
const uint8_t STALL_GUARD_THRESHOLD = 8;
const TMC2209::CurrentIncrement  CURRENT_INCREMENT   = TMC2209::CURRENT_INCREMENT_8;
const TMC2209::MeasurementCount MEASUREMENT_COUNT   = TMC2209::MEASUREMENT_COUNT_2;
const uint32_t COOL_STEP_DURATION_THRESHOLD         = 2000;
const uint8_t  COOL_STEP_LOWER_THRESHOLD            = 2;
const uint8_t  COOL_STEP_UPPER_THRESHOLD            = 8;

// Stall handling
const int MIN_RPM_FOR_STALL = 20;
unsigned long ignoreStallUntil = 0;
const unsigned long STALL_CHECK_INTERVAL_MS = 50;
const unsigned long STALL_COOLDOWN_MS = 300;
unsigned long lastStallCheckMs = 0;
unsigned long stallEventTimeMs = 0;
int  stallRetryCount = 0;
int  savedTargetRpm = 0;
uint16_t sgBaseline = 0;
bool sgBaselineValid = false;

enum StallState { STALL_OK, STALL_COOLDOWN, STALL_RECOVER };
StallState stallState = STALL_OK;

// --- Helpers ---
void setControlChecksum() {
  uint8_t sum = 0;
  for (int i = 0; i < (int)CONTROLLER_LEN - 1; ++i) sum += sendBuffer[i];
  sendBuffer[CHECK_IDX] = sum;
}

inline void setValue(uint8_t* bytePtr, uint8_t v) {
  *bytePtr = v;
  setControlChecksum();
}

void shutdown() {
  memset(sendBuffer, 0, CONTROLLER_LEN);
  setControlChecksum();
}

void eStop() {
  setValue(&sendBuffer[HEAT_IDX], 0);
  setValue(&sendBuffer[VENT_IDX], 100);
}

// --- Temperature calculation & filtering ---
const int MEDIAN_WINDOW = 5;
float tempBuffer[MEDIAN_WINDOW] = {0};
int   tempIndex = 0;
bool  bufferFull = false;
float filteredTemp = 0.0;
const float alpha = 0.8f; // EMA weight

float applyMedianWeightedFilter(float rawTemp) {
  tempBuffer[tempIndex] = rawTemp;
  tempIndex = (tempIndex + 1) % MEDIAN_WINDOW;
  if (tempIndex == 0) bufferFull = true;

  const int n = bufferFull ? MEDIAN_WINDOW : tempIndex;
  float sorted[MEDIAN_WINDOW];
  memcpy(sorted, tempBuffer, sizeof(float) * n);
  // simple insertion sort to avoid <algorithm>
  for (int i = 1; i < n; ++i) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; --j; }
    sorted[j + 1] = key;
  }
  float median = (n % 2) ? sorted[n / 2] : (sorted[n/2 - 1] + sorted[n/2]) * 0.5f;
  filteredTemp = (1.0f - alpha) * filteredTemp + alpha * median;
  return filteredTemp;
}

uint16_t intx, inty;
double calculateTemp() {
  intx = ((receiveBuffer[0] << 8) + receiveBuffer[1]);
  inty = ((receiveBuffer[2] << 8) + receiveBuffer[3]);
  const double x = 0.001 * intx;
  const double y = 0.001 * inty;
  double v;
  if (intx > 836 && inty > 221)
    v = -224.2 * y * y * y + 385.9 * y * y - 327.1 * y + 171;
  else
    v = -278.33 * x * x * x + 491.944 * x * x - 451.444 * x + 310.668;
  tempC = applyMedianWeightedFilter(v);
  double w = tempC;
  if (CorF == 'F') w = 1.8 * w + 32; // Fahrenheit
  return w;
}

// --- Artisan serial protocol ---
void handleHEAT(uint8_t value)  { if (value <= 100) setValue(&sendBuffer[HEAT_IDX],   value); lastEventTime = micros(); }
void handleVENT(uint8_t value)  { if (value <= 100) setValue(&sendBuffer[VENT_IDX],   value); lastEventTime = micros(); }
void handleCOOL(uint8_t value)  { if (value <= 100) setValue(&sendBuffer[COOL_IDX],   value); lastEventTime = micros(); }
void handleFILTER(uint8_t value){ if (value <= 100) setValue(&sendBuffer[FILTER_IDX], value); lastEventTime = micros(); }

void handleDRUM(uint8_t value) {
  if (value != 0) {
    setValue(&sendBuffer[DRUM_IDX], 100);
    stepper_driver.enable();
    targetMotorRPM = (int)(value * gearRatio); // drum RPM -> motor RPM
  } else {
    setValue(&sendBuffer[DRUM_IDX], 0);
    targetMotorRPM = 0;
    stepper_driver.disable();
  }
  lastEventTime = micros();
}

void handleCHAN() { Serial.println("# Active channels set to 1234"); }

void handleREAD() {
  Serial.print(ambientTemp); Serial.print(',');
  Serial.print(temp);        Serial.print(',');
  Serial.print(temp);        Serial.print(',');
  Serial.print(ambientTemp); Serial.print(',');
  Serial.print(ambientHumidity); Serial.print(',');
  Serial.print(sendBuffer[HEAT_IDX]); Serial.print(',');
  Serial.print(sendBuffer[VENT_IDX]); Serial.print(',');
  Serial.println('0');
  lastEventTime = micros();
}

void getArtisanMessage() {
  static char line[96];
  static uint8_t idx = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[idx] = '\0';
      idx = 0;
      char* p = line;
      while (*p == ' ' || *p == '\t') ++p;
      if (*p == '\0') continue;
      uint8_t value = 0;
      char* sep = strchr(p, ';');
      if (sep) { *sep = '\0'; value = (uint8_t)atoi(sep + 1); }
      if      (strcmp(p, "READ")   == 0) { handleREAD(); }
      else if (strcmp(p, "OT1")    == 0) { handleHEAT(value); }
      else if (strcmp(p, "OT2")    == 0) { handleVENT(value); }
      else if (strcmp(p, "OFF")    == 0) { shutdown(); }
      else if (strcmp(p, "ESTOP")  == 0) { eStop(); }
      else if (strcmp(p, "DRUM")   == 0) { handleDRUM(value); }
      else if (strcmp(p, "FILTER") == 0) { handleFILTER(value); }
      else if (strcmp(p, "COOL")   == 0) { handleCOOL(value); }
      else if (strcmp(p, "CHAN")   == 0) { handleCHAN(); }
      else if (strcmp(p, "UNITS")  == 0) { if (sep && *(sep+1)) CorF = *(sep+1); }
      continue;
    }
    if (idx < sizeof(line) - 1) line[idx++] = c; else idx = 0;
  }
}

// --- Failsafe ---
bool itsbeentoolong() {
  const unsigned long now = micros();
  const unsigned long duration = now - lastEventTime; // micros() wraps, subtraction is safe unsigned
  return duration > lastEventTimeout;
}

bool isTemperatureOverLimit() { return tempC > maxTemp; }

void failSafeChecks() {
  if (itsbeentoolong()) {
    if (tempC >= 70) {
      memset(sendBuffer, 0, CONTROLLER_LEN);
      handleVENT(90);
      handleCOOL(100);
      lastEventTime = micros();
    } else {
      shutdown();
    }
  }
  if (isTemperatureOverLimit()) eStop();
}

// --- Motor ---
int rpmToStepsPerPeriod(int rpm) {
  const float microsteps_per_sec = (rpm * STEPS_PER_REV * MICROSTEPS) / 60.0f;
  return (int)(microsteps_per_sec / 0.715f);
}

void updateMotor() {
  const unsigned long now = millis();
  if (now - lastMotorUpdate < motorUpdateInterval) return;
  lastMotorUpdate = now;
  if (currentMotorRPM == targetMotorRPM) return;
  const int diff = targetMotorRPM - currentMotorRPM;
  int accel = (abs(diff) > 20) ? 5 : 1;
  currentMotorRPM += (diff > 0) ? accel : -accel;
  if ((diff > 0 && currentMotorRPM > targetMotorRPM) ||
      (diff < 0 && currentMotorRPM < targetMotorRPM)) currentMotorRPM = targetMotorRPM;
  stepper_driver.moveAtVelocity(rpmToStepsPerPeriod(currentMotorRPM));
}

// Stall handling (unchanged logic, tidied)
const uint16_t SG_LOW_ABS  = 6;
const uint8_t  CS_HIGH_ABS = 26;
const int RPM_STABLE_BAND = 5;
const uint32_t BASELINE_WARMUP_MS = 800;
static uint32_t stableSince = 0;

void handleStallNonBlocking() {
  const unsigned long now = millis();
  switch (stallState) {
    case STALL_OK: {
      if (currentMotorRPM < MIN_RPM_FOR_STALL) break;
      if (now < ignoreStallUntil) break;
      if (now - lastStallCheckMs < STALL_CHECK_INTERVAL_MS) break;
      lastStallCheckMs = now;
      static uint8_t stallHits = 0;
      const uint16_t sg = stepper_driver.getStallGuardResult();
      const auto st = stepper_driver.getStatus();
      const uint8_t cs = st.current_scaling;
      const bool stable = (abs(targetMotorRPM - currentMotorRPM) <= RPM_STABLE_BAND);
      if (!stable) { stableSince = 0; stallHits = 0; break; }
      else if (stableSince == 0) stableSince = now;
      if (now - stableSince < BASELINE_WARMUP_MS) { stallHits = 0; break; }
      const bool looksBad = (sg <= SG_LOW_ABS) && (cs >= CS_HIGH_ABS);
      if (looksBad) ++stallHits; else stallHits = 0;
      if (stallHits >= 4) {
        stallHits = 0;
        savedTargetRpm = targetMotorRPM;
        stallEventTimeMs = now;
        targetMotorRPM = 0; // hard stop
        stallState = STALL_COOLDOWN;
      }
      break; }
    case STALL_COOLDOWN: {
      if (now - stallEventTimeMs >= STALL_COOLDOWN_MS) {
        if (stallRetryCount > 0 && (stallRetryCount % 5) == 0) {
          savedTargetRpm = max(20, savedTargetRpm - 5);
        }
        stallState = STALL_RECOVER;
      }
      break; }
    case STALL_RECOVER: {
      targetMotorRPM = savedTargetRpm;
      ignoreStallUntil = millis() + 800;
      stallRetryCount = 0;
      stallState = STALL_OK;
      break; }
  }
}

void setup_motor() {
  stepper_driver.setup(Serial1, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, UART_RX_PIN, UART_TX_PIN);
  stepper_driver.disableAutomaticCurrentScaling();
  stepper_driver.disableAutomaticGradientAdaptation();
  stepper_driver.setMicrostepsPerStep(MICROSTEPS);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.setHoldCurrent(HOLD_CURRENT_PERCENT);
  stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper_driver.enableCoolStep(COOL_STEP_LOWER_THRESHOLD, COOL_STEP_UPPER_THRESHOLD);
  stepper_driver.setCoolStepCurrentIncrement(CURRENT_INCREMENT);
  stepper_driver.setCoolStepMeasurementCount(MEASUREMENT_COUNT);
  stepper_driver.setCoolStepDurationThreshold(COOL_STEP_DURATION_THRESHOLD);
  stepper_driver.disableStealthChop();
}

// Ambient
void readAmbient() {
  static unsigned long lastStart = 0;
  static bool pending = false;
  static unsigned long startTime = 0;

  const unsigned long now = millis();

  // Start a new measurement once per second
  if (!pending && (now - lastStart >= 1000)) {
    aht20.triggerMeasurement();
    startTime = now;
    pending = true;
    lastStart = now;
    return;
  }

  // After typical AHT20 conversion time, check occasionally
  if (pending && (now - startTime >= 90)) {        // ~80–100ms is typical
    static unsigned long lastPoll = 0;
    if (now - lastPoll < 20) return;               // poll every 20ms
    lastPoll = now;

    if (aht20.available()) {
      ambientTemp = aht20.getTemperature();
      ambientHumidity = aht20.getHumidity();
      pending = false;
    }
  }
}

// --- Setup & loop ---
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  roasterTx.begin();
  roasterRX.begin();

  pinMode(OE_PIN, OUTPUT);  digitalWrite(OE_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);

  Wire.begin();
  if (!aht20.begin()) { Serial.println("AHT20 not found."); }

  setup_motor();
  shutdown();
}

void loop() {
  getArtisanMessage();

  // Receive from roaster
  roasterRX.update();
  if (roasterRX.available()) {
    roasterRX.getMessage(receiveBuffer);
    temp = calculateTemp();
  }

  // Transmit to roaster (non-blocking)
  roasterTx.service(sendBuffer, CONTROLLER_LEN);

  updateMotor();
  handleStallNonBlocking();
  readAmbient();
  failSafeChecks();
  yield();
}
