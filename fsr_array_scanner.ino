/*
  Tactile FSR Array - Full matrix scan (separate select pins)

  You now have:
    - Supply mux has its own S0-S3 pins (SUP_S0..SUP_S3)
    - Sense mux has its own S0-S3 pins (SEN_S0..SEN_S3)

  Scan logic:
    For each supply line i:
      select supply mux channel i
      enable supply mux
      drive SUPPLY_COMMON_PIN HIGH
      for each sense line j:
        select sense mux channel j
        enable sense mux
        read ADC (A15)
      drive SUPPLY_COMMON_PIN LOW
      disable supply mux

  Output:
    CFG,<num_supply>,<num_sense>
    F,<t_us>,<v00>,<v01>,...,<v(i)(j)> (row-major: supply-major then sense)
*/

#include <Arduino.h>

// ----------------------------
// USER CONFIG
// ----------------------------
static const uint8_t NUM_SUPPLY = 8;     // how many supply traces you have (<=16)
static const uint8_t NUM_SENSE  = 8;     // how many sense traces you have (<=16)

static const uint32_t BAUD = 921600;

// ---- Supply mux select pins (NEW: separate set)
static const uint8_t SUP_S0 = 2;
static const uint8_t SUP_S1 = 3;
static const uint8_t SUP_S2 = 4;
static const uint8_t SUP_S3 = 5;

// ---- Sense mux select pins (NEW: separate set)
static const uint8_t SEN_S0 = 6;
static const uint8_t SEN_S1 = 7;
static const uint8_t SEN_S2 = 8;
static const uint8_t SEN_S3 = 9;

// ---- Enable pins (CD74HC4067 enable is ACTIVE-LOW)
static const uint8_t EN_SUPPLY = 10;    // supply mux E pin
static const uint8_t EN_SENSE  = 11;    // sense mux E pin

// ---- Supply mux common pin (Arduino digital output)
static const uint8_t SUPPLY_COMMON_PIN = 12;

// ---- Sense mux common pin to Arduino ADC
static const uint8_t SENSE_ADC_PIN = A15;

// Timing / sampling
static const uint16_t SUPPLY_SETTLE_US = 50;  // settle after selecting supply + driving HIGH
static const uint16_t SENSE_SETTLE_US  = 20;  // settle after selecting sense
static const uint8_t  ADC_SAMPLES      = 2;   // average reads (1-4 typical)
static const uint16_t FRAME_DELAY_US   = 0;   // throttle if needed (0 = fastest)

// ----------------------------
// Buffer
// ----------------------------
static uint16_t frame[NUM_SUPPLY][NUM_SENSE];

// ----------------------------
// Helpers
// ----------------------------
static inline void muxEnable(uint8_t enPin)  { digitalWrite(enPin, LOW);  }
static inline void muxDisable(uint8_t enPin) { digitalWrite(enPin, HIGH); }

static inline void selectSupply(uint8_t ch) {
  digitalWrite(SUP_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(SUP_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(SUP_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(SUP_S3, (ch & 0x08) ? HIGH : LOW);
}

static inline void selectSense(uint8_t ch) {
  digitalWrite(SEN_S0, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(SEN_S1, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(SEN_S2, (ch & 0x04) ? HIGH : LOW);
  digitalWrite(SEN_S3, (ch & 0x08) ? HIGH : LOW);
}

static inline uint16_t readAdcAvg(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) acc += analogRead(pin);
  return (uint16_t)(acc / ADC_SAMPLES);
}

static void printConfig() {
  Serial.print("CFG,");
  Serial.print(NUM_SUPPLY);
  Serial.print(",");
  Serial.println(NUM_SENSE);
}

static void printFrame(uint32_t t_us) {
  Serial.print("F,");
  Serial.print(t_us);

  // row-major: supply-major then sense
  for (uint8_t si = 0; si < NUM_SUPPLY; si++) {
    for (uint8_t sj = 0; sj < NUM_SENSE; sj++) {
      Serial.print(",");
      Serial.print(frame[si][sj]);
    }
  }
  Serial.println();
}

void setup() {
  pinMode(SUP_S0, OUTPUT); pinMode(SUP_S1, OUTPUT); pinMode(SUP_S2, OUTPUT); pinMode(SUP_S3, OUTPUT);
  pinMode(SEN_S0, OUTPUT); pinMode(SEN_S1, OUTPUT); pinMode(SEN_S2, OUTPUT); pinMode(SEN_S3, OUTPUT);

  pinMode(EN_SUPPLY, OUTPUT);
  pinMode(EN_SENSE, OUTPUT);

  pinMode(SUPPLY_COMMON_PIN, OUTPUT);
  digitalWrite(SUPPLY_COMMON_PIN, LOW);

  // start with both muxes disabled
  muxDisable(EN_SUPPLY);
  muxDisable(EN_SENSE);

  Serial.begin(BAUD);
  while (!Serial) {}

  printConfig();
}

void loop() {
  uint32_t t0 = micros();

  for (uint8_t si = 0; si < NUM_SUPPLY; si++) {
    // Select and energize ONE supply line
    muxDisable(EN_SUPPLY);
    selectSupply(si);
    muxEnable(EN_SUPPLY);

    digitalWrite(SUPPLY_COMMON_PIN, HIGH);
    delayMicroseconds(SUPPLY_SETTLE_US);

    // Sweep all sense lines while that supply is active
    for (uint8_t sj = 0; sj < NUM_SENSE; sj++) {
      muxDisable(EN_SENSE);
      selectSense(sj);
      muxEnable(EN_SENSE);

      delayMicroseconds(SENSE_SETTLE_US);

      frame[si][sj] = readAdcAvg(SENSE_ADC_PIN);
    }

    // Turn supply off before moving to next supply line
    digitalWrite(SUPPLY_COMMON_PIN, LOW);
    muxDisable(EN_SUPPLY);
  }

  printFrame(t0);

  if (FRAME_DELAY_US) delayMicroseconds(FRAME_DELAY_US);
}
