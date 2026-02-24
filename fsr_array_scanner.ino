/*
  ============================================================
  Tactile FSR Array Scan
  ============================================================

 SCAN SEQUENCE (per frame)
  -------------------------
  For each supply row i (0..9):
    1. Select channel i on supply mux, enable it
    2. Drive SUPPLY_SIG_PIN HIGH  →  energizes row i
    3. Wait SUPPLY_SETTLE_US for row to stabilize
    4. For each sense column j (0..9):
         a. Select channel j on sense mux, enable it
         b. Wait SENSE_SETTLE_US for op-amp output to settle
         c. Read ADC (averaged ADC_SAMPLES times)
         d. Store in frame[i][j]
    5. Drive SUPPLY_SIG_PIN LOW, disable supply mux

  SERIAL OUTPUT FORMAT
  --------------------
  On startup:
    CFG,<num_rows>,<num_cols>,<adc_ref_mv>
    e.g.  CFG,10,10,5000

  Each frame:
    F,<timestamp_us>,<v[0][0]>,<v[0][1]>,...,<v[9][9]>
    - timestamp_us : micros() at start of frame
    - values       : raw 10-bit ADC readings (0–1023), row-major order
    - 100 values per line, comma-separated

  ============================================================
*/

#include <Arduino.h>

// ============================================================
// USER CONFIG
// ============================================================

static const uint8_t NUM_ROWS = 10;   // supply lines  (<= 16)
static const uint8_t NUM_COLS = 10;   // sense lines   (<= 16)

static const uint32_t BAUD = 921600;

// ---- Supply mux select pins
static const uint8_t SUP_S0 = 6;
static const uint8_t SUP_S1 = 7;
static const uint8_t SUP_S2 = 8;
static const uint8_t SUP_S3 = 9;

// ---- Sense mux select pins
static const uint8_t SEN_S0 = 2;
static const uint8_t SEN_S1 = 3;
static const uint8_t SEN_S2 = 4;
static const uint8_t SEN_S3 = 5;

// ---- Mux enable pins (CD74HC4067: active LOW)
static const uint8_t EN_SUPPLY = 11;
static const uint8_t EN_SENSE  = 12;

// ---- Supply mux common: Arduino drives this to energize a row
static const uint8_t SUPPLY_SIG_PIN = 10;

// ---- Sense mux common: op-amp buffered signal → ADC
static const uint8_t SENSE_SIG_PIN = A0;

// ---- Timing (microseconds)
// Increase SUPPLY_SETTLE_US if readings look noisy or cross-coupled.
// Increase SENSE_SETTLE_US  if the op-amp output hasn't settled by read time.
static const uint16_t SUPPLY_SETTLE_US = 100;  // after energizing a row
static const uint16_t SENSE_SETTLE_US  =  50;  // after selecting a sense column

// ---- ADC averaging: more samples = less noise, but slower frame rate
//      Recommended range: 2–8
static const uint8_t ADC_SAMPLES = 4;

// ---- Optional inter-frame throttle (0 = run as fast as possible)
static const uint16_t FRAME_DELAY_US = 0;

// ---- ADC reference voltage in mV (5000 for default 5V ref, 3300 for 3.3V)
//      Used only in the CFG header line for the Python visualizer.
static const uint16_t ADC_REF_MV = 5000;

// ============================================================
// Internal state
// ============================================================

static uint16_t frame[NUM_ROWS][NUM_COLS]; // this initializes 2D array called frame

// ============================================================
// Mux helpers
// ============================================================

void muxEnable (uint8_t enPin) {
  digitalWrite(enPin, LOW); 
}
void muxDisable(uint8_t enPin) {
  digitalWrite(enPin, HIGH);
}

// Write a 4-bit channel address to the supply mux select lines
void selectSupplyChannel(uint8_t ch) {
  if (ch & 0x01) { digitalWrite(SUP_S0, HIGH); } else { digitalWrite(SUP_S0, LOW); }

  if (ch & 0x02) { digitalWrite(SUP_S1, HIGH); } else { digitalWrite(SUP_S1, LOW); }

  if (ch & 0x04) { digitalWrite(SUP_S2, HIGH); } else { digitalWrite(SUP_S2, LOW); }

  if (ch & 0x08) { digitalWrite(SUP_S3, HIGH); } else { digitalWrite(SUP_S3, LOW); }
}

// Write a 4-bit channel address to the sense mux select lines
void selectSenseChannel(uint8_t ch) {
  if (ch & 0x01) { digitalWrite(SEN_S0, HIGH); } else { digitalWrite(SEN_S0, LOW); }

  if (ch & 0x02) { digitalWrite(SEN_S1, HIGH); } else { digitalWrite(SEN_S1, LOW); }

  if (ch & 0x04) { digitalWrite(SEN_S2, HIGH); } else { digitalWrite(SEN_S2, LOW); }

  if (ch & 0x08) { digitalWrite(SEN_S3, HIGH); } else { digitalWrite(SEN_S3, LOW); }
}

// Average multiple ADC reads to reduce quantisation / noise
uint16_t readAdcAvg(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    acc += analogRead(pin);
  }
  return (uint16_t)(acc / ADC_SAMPLES);
}

// ============================================================
// Serial output
// ============================================================

// Sent once on startup so the Python visualizer knows the grid dimensions.
void printConfig() {
  Serial.print(F("CFG,"));
  Serial.print(NUM_ROWS);
  Serial.print(F(","));
  Serial.print(NUM_COLS);
  Serial.print(F(","));
  Serial.println(ADC_REF_MV);
}

// Emit one complete frame as a CSV line.
// Format: F,<timestamp_us>,<v00>,<v01>,...,<v(rows-1)(cols-1)>
void printFrame(uint32_t t_us) {
  Serial.print(F("F,"));
  Serial.print(t_us);

  for (uint8_t row = 0; row < NUM_ROWS; row++) {
    for (uint8_t col = 0; col < NUM_COLS; col++) {
      Serial.print(',');
      Serial.print(frame[row][col]);
    }
  }
  Serial.println();
}

// ============================================================
// Setup
// ============================================================

void setup() {
  // Supply mux select lines
  pinMode(SUP_S0, OUTPUT);
  pinMode(SUP_S1, OUTPUT);
  pinMode(SUP_S2, OUTPUT);
  pinMode(SUP_S3, OUTPUT);

  // Sense mux select lines
  pinMode(SEN_S0, OUTPUT);
  pinMode(SEN_S1, OUTPUT);
  pinMode(SEN_S2, OUTPUT);
  pinMode(SEN_S3, OUTPUT);

  // Enable lines — start disabled (HIGH = disabled for active-LOW enable)
  pinMode(EN_SUPPLY, OUTPUT);
  pinMode(EN_SENSE,  OUTPUT);
  muxDisable(EN_SUPPLY);
  muxDisable(EN_SENSE);

  // Supply common — start LOW (de-energized)
  pinMode(SUPPLY_SIG_PIN, OUTPUT);
  digitalWrite(SUPPLY_SIG_PIN, LOW);

  // ADC pin — set to input (default, but explicit is good practice)
  pinMode(SENSE_SIG_PIN, INPUT);

  // Use the default 5V AVcc reference.
  // If you wire AREF to your 2.5V virtual ground instead, change to:
  //   analogReference(EXTERNAL);
  analogReference(DEFAULT); // this DEFAULT value is defined from the arduino library

  Serial.begin(BAUD);
  while (!Serial) {}  // wait for USB serial on Leonardo/Mega with native USB

  printConfig();
}

// ============================================================
// Main loop — continuous frame capture
// ============================================================

void loop() {
  uint32_t frameStart = micros();

  for (uint8_t row = 0; row < NUM_ROWS; row++) {
    // ── 1. Select and energize supply row ──────────────────────
    muxDisable(EN_SUPPLY);       // disable before changing address (glitch-free
    selectSupplyChannel(row);
    muxEnable(EN_SUPPLY);
    digitalWrite(SUPPLY_SIG_PIN, HIGH);
    delayMicroseconds(SUPPLY_SETTLE_US);

    // ── 2. Sweep all sense columns ──────────────────────────────
    for (uint8_t col = 0; col < NUM_COLS; col++) {
      muxDisable(EN_SENSE);      // disable before changing address
      selectSenseChannel(col);
      muxEnable(EN_SENSE);
      delayMicroseconds(SENSE_SETTLE_US);

      // frame[row][col] = readAdcAvg(SENSE_SIG_PIN); // code to reduce noise
      frame[row][col] = analogRead(SENSE_SIG_PIN);
    }

    // ── 3. De-energize row before moving on ────────────────────
    //    This prevents current leaking through neighbouring FSRs
    //    (the "ghosting" / phantom-pressure problem in passive matrices).
    digitalWrite(SUPPLY_SIG_PIN, LOW);
    muxDisable(EN_SUPPLY);
    muxDisable(EN_SENSE);
  }

  // ── 4. Transmit frame ──────────────────────────────────────
  printFrame(frameStart);

  if (FRAME_DELAY_US > 0) {
    delayMicroseconds(FRAME_DELAY_US);
  }
}
