/*
  High-performance STM32F401 Arduino sketch combining:
  1) PRBS-15 TRIG output at 2000 Hz pin rate, 100 Hz chip rate (continuous)
  2) Dual BNO085/BNO080 UART-RVC readout (two HW UARTs)
  3) Configurable button-matrix scanning (non-blocking, debounced)

  PRBS: 100 Hz chip rate (fixed). TRIG pin updates at 2000 Hz (20 ticks per chip).
  Continuous output: no marker gap phase and no periodic LFSR reset.
  Serial output: prbs_tick (continuous chip index), prbs_level, in_mark (always 0).

  Target: main-loop sampling/logging at 500 Hz (2 ms)
  - Print rate: 500 Hz
  - Matrix scan rate: 500 Hz
  - PRBS TRIG: TIM3 at 2000 Hz, PA8 -> Porti7 TRIG (LEMO centre, 330R), GND -> LEMO shield

  Board: STM32F401 (Arduino STM32 core)
  Library: "Adafruit BNO08x RVC" (Adafruit_BNO08x_RVC)
*/

#include <Arduino.h>
#include "Adafruit_BNO08x_RVC.h"

#ifndef LED_BUILTIN
#define LED_BUILTIN PC13
#endif

// ========================= Feature toggles =========================
#define ENABLE_PRBS_TRIG   1
#define ENABLE_DUAL_IMU    1
#define ENABLE_MATRIX      1
// ===================================================================


// --- PRBS TRIG CONFIG (chip rate fixed at 100 Hz; matches prbs_sync_testing.py) ---
#if ENABLE_PRBS_TRIG
static const uint8_t PRBS_PIN = PA8;

// 100 Hz chip rate, 2000 Hz TRIG pin output (20 ticks per chip)
static const uint32_t CHIP_RATE_HZ = 100;
static const uint32_t TRIG_OUTPUT_HZ = 2000;
static const uint32_t TICKS_PER_CHIP = TRIG_OUTPUT_HZ / CHIP_RATE_HZ;  // 20
static const uint16_t PRBS15_SEED = 0x7ACE;

// LFSR state (PRBS-15): x^15 + x^14 + 1
volatile uint16_t lfsr = PRBS15_SEED;

// Continuous chip index (read by main loop as prbs_tick)
volatile uint32_t prbs_chip_index = 0;
// Current PRBS bit on pin
volatile uint8_t  prbs_level = 0;
// Marker support removed; keep field for log compatibility
volatile uint8_t  in_mark = 0;

static volatile uint32_t chip_counter = 0;   // continuous chip counter
static volatile uint8_t  sub_tick = 0;       // 0..19 within current chip

HardwareTimer *prbsTimer = nullptr;

// PRBS-15: taps at bits 14 and 13 (x^15 + x^14 + 1)
static inline uint8_t lfsr_next_bit_prbs15(volatile uint16_t &s) {
  uint8_t b14 = (s >> 14) & 1;
  uint8_t b13 = (s >> 13) & 1;
  uint8_t newbit = b14 ^ b13;
  s = (uint16_t)((s << 1) | newbit);
  return newbit;
}

// Timer ISR: 2000 Hz — each chip held for 20 ticks (continuous PRBS)
void onPrbsTick() {
  if (sub_tick == 0) {
    uint8_t bit = lfsr_next_bit_prbs15(lfsr);
    prbs_level = bit ? 1 : 0;
    prbs_chip_index = chip_counter;
    digitalWrite(PRBS_PIN, prbs_level ? HIGH : LOW);
  }
  sub_tick++;
  if (sub_tick >= TICKS_PER_CHIP) {
    sub_tick = 0;
    chip_counter++;
  }
}

static void prbsInit() {
  pinMode(PRBS_PIN, OUTPUT);
  digitalWrite(PRBS_PIN, LOW);
  prbs_level = 0;
  in_mark = 0;
  chip_counter = 0;
  sub_tick = 0;
  lfsr = PRBS15_SEED;
  prbs_chip_index = 0;

  // Use TIM3 to avoid conflict with TIM2 (often used by Arduino core for micros/delay)
  prbsTimer = new HardwareTimer(TIM3);
  prbsTimer->setOverflow(TRIG_OUTPUT_HZ, HERTZ_FORMAT);
  prbsTimer->attachInterrupt(onPrbsTick);
  prbsTimer->resume();
}
#endif
// ===================================================================


// ========================= DUAL IMU CONFIG =========================
#if ENABLE_DUAL_IMU
#if defined(ARDUINO_ARCH_STM32)
HardwareSerial IMU1Serial(PA10, PA9);  // RX,TX
HardwareSerial IMU2Serial(PA3,  PA2);  // RX,TX
#else
#define IMU1Serial Serial1
#define IMU2Serial Serial2
#endif

Adafruit_BNO08x_RVC rvc1;
Adafruit_BNO08x_RVC rvc2;

static bool imu1_ok = false;
static bool imu2_ok = false;

static bool initRVC(Adafruit_BNO08x_RVC &rvc, HardwareSerial &port, const char *name) {
  port.begin(115200);
  delay(50);

  const uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    if (rvc.begin(&port)) {
      Serial.print("OK: "); Serial.print(name); Serial.println(" ready");
      return true;
    }
    delay(50);
  }

  Serial.print("ERR: "); Serial.print(name); Serial.println(" rvc.begin failed");
  return false;
}

static void imuInit() {
  imu1_ok = initRVC(rvc1, IMU1Serial, "BNO#1 (USART1)");
  imu2_ok = initRVC(rvc2, IMU2Serial, "BNO#2 (USART2)");
}
#endif
// ===================================================================


// ========================= MATRIX CONFIG ===========================
#if ENABLE_MATRIX
const uint8_t ROWS = 3;
const uint8_t COLS = 3;

const uint8_t rowPins[ROWS] = { PA0, PA1, PA4 };
const uint8_t colPins[COLS] = { PB0, PB1, PB10 };

// Scan + debounce settings
// Target matrix scan rate = 500 Hz -> 2000 us
static const uint32_t MATRIX_SCAN_US   = 2000;
static const uint8_t  DEBOUNCE_SAMPLES = 4;

static uint16_t rawState = 0;
static uint16_t stableState = 0;
static uint16_t lastStableState = 0;

static uint8_t debounceCount[ROWS * COLS];

static inline void allRowsHigh() {
  for (uint8_t r = 0; r < ROWS; r++) digitalWrite(rowPins[r], HIGH);
}

static void matrixInit() {
  for (uint8_t r = 0; r < ROWS; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH);
  }
  for (uint8_t c = 0; c < COLS; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }
  for (uint8_t i = 0; i < ROWS * COLS; i++) debounceCount[i] = 0;
}

static uint16_t matrixScanOnce() {
  uint16_t s = 0;
  for (uint8_t r = 0; r < ROWS; r++) {
    allRowsHigh();
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(80);

    for (uint8_t c = 0; c < COLS; c++) {
      const uint8_t pressed = (digitalRead(colPins[c]) == LOW) ? 1 : 0;
      if (pressed) s |= (1u << (r * COLS + c));
    }
  }
  allRowsHigh();
  return s;
}

static void matrixUpdateDebounced(uint16_t newRaw) {
  rawState = newRaw;

  for (uint8_t i = 0; i < ROWS * COLS; i++) {
    const uint16_t mask = (1u << i);
    const uint8_t rawBit = (rawState & mask) ? 1 : 0;
    const uint8_t stableBit = (stableState & mask) ? 1 : 0;

    if (rawBit == stableBit) {
      debounceCount[i] = 0;
    } else {
      if (debounceCount[i] < 255) debounceCount[i]++;
      if (debounceCount[i] >= DEBOUNCE_SAMPLES) {
        if (rawBit) stableState |= mask;
        else        stableState &= (uint16_t)~mask;
        debounceCount[i] = 0;
      }
    }
  }
}

static inline uint16_t matrixGetPressedMask() { return stableState; }
static inline uint16_t matrixGetRisingEdges() { return (uint16_t)(stableState & ~lastStableState); }
static inline uint16_t matrixGetFallingEdges(){ return (uint16_t)(~stableState & lastStableState); }
#endif
// ===================================================================


// ========================= OUTPUT / SCHEDULING ======================
static uint32_t t0_ms = 0;

static const uint32_t PRINT_HZ = 500;
static const uint32_t PRINT_PERIOD_US = 1000000UL / PRINT_HZ;

static uint32_t nextPrint_us = 0;

static void printHeader() {
  Serial.println("t_ms,imu1_ok,imu2_ok,"
                 "yaw1,pitch1,roll1,ax1,ay1,az1,"
                 "yaw2,pitch2,roll2,ax2,ay2,az2,"
                 "keys_mask,keys_rise,keys_fall,"
                 "prbs_tick,prbs_level,in_mark");
}
// ===================================================================


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delay(50);

#if ENABLE_MATRIX
  matrixInit();
#endif

#if ENABLE_PRBS_TRIG
  prbsInit();
#endif

#if ENABLE_DUAL_IMU
  Serial.println();
  Serial.println("STM32F401: dual BNO08x UART-RVC + matrix + PRBS TRIG");
  imuInit();
#else
  Serial.println();
  Serial.println("STM32F401: matrix + PRBS TRIG (IMU disabled)");
#endif

  t0_ms = millis();
  nextPrint_us = micros() + PRINT_PERIOD_US;

  printHeader();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  const uint32_t now_us = micros();

  // ---------- Non-blocking matrix scan (500 Hz) ----------
#if ENABLE_MATRIX
  static uint32_t nextMatrix_us = 0;
  if ((int32_t)(now_us - nextMatrix_us) >= 0) {
    nextMatrix_us += MATRIX_SCAN_US; // keep stable cadence
    const uint16_t s = matrixScanOnce();
    matrixUpdateDebounced(s);
  }
#endif

  // ---------- Read newest IMU packets (drain queues) ----------
#if ENABLE_DUAL_IMU
  static BNO08x_RVC_Data last1, last2;
  static bool have1 = false, have2 = false;
  BNO08x_RVC_Data tmp;

  if (imu1_ok) {
    while (rvc1.read(&tmp)) { last1 = tmp; have1 = true; }
  }
  if (imu2_ok) {
    while (rvc2.read(&tmp)) { last2 = tmp; have2 = true; }
  }
#endif

  // ---------- Periodic print (500 Hz) ----------
  if ((int32_t)(now_us - nextPrint_us) >= 0) {
    nextPrint_us += PRINT_PERIOD_US;

    const uint32_t now_ms = millis();
    const uint32_t t_ms = now_ms - t0_ms;

#if ENABLE_MATRIX
    const uint16_t keys_mask = matrixGetPressedMask();
    const uint16_t keys_rise = matrixGetRisingEdges();
    const uint16_t keys_fall = matrixGetFallingEdges();
    lastStableState = stableState;
#else
    const uint16_t keys_mask = 0, keys_rise = 0, keys_fall = 0;
#endif

#if ENABLE_PRBS_TRIG
    const uint32_t prbs_tick = prbs_chip_index;
    const uint8_t  prbs_lvl  = prbs_level;
    const uint8_t  in_mark_val = in_mark;
#else
    const uint32_t prbs_tick = 0;
    const uint8_t  prbs_lvl  = 0;
    const uint8_t  in_mark_val = 0;
#endif

    Serial.print(t_ms);
    Serial.print(",");

#if ENABLE_DUAL_IMU
    Serial.print(imu1_ok ? 1 : 0); Serial.print(",");
    Serial.print(imu2_ok ? 1 : 0);

    if (have1) {
      Serial.print(","); Serial.print(last1.yaw);
      Serial.print(","); Serial.print(last1.pitch);
      Serial.print(","); Serial.print(last1.roll);
      Serial.print(","); Serial.print(last1.x_accel);
      Serial.print(","); Serial.print(last1.y_accel);
      Serial.print(","); Serial.print(last1.z_accel);
    } else {
      Serial.print(",,,,,,");
    }

    if (have2) {
      Serial.print(","); Serial.print(last2.yaw);
      Serial.print(","); Serial.print(last2.pitch);
      Serial.print(","); Serial.print(last2.roll);
      Serial.print(","); Serial.print(last2.x_accel);
      Serial.print(","); Serial.print(last2.y_accel);
      Serial.print(","); Serial.print(last2.z_accel);
    } else {
      Serial.print(",,,,,,");
    }
#else
    Serial.print("0,0");
    Serial.print(",,,,,,");
    Serial.print(",,,,,,");
#endif

    Serial.print(",");
    Serial.print(keys_mask);
    Serial.print(",");
    Serial.print(keys_rise);
    Serial.print(",");
    Serial.print(keys_fall);
    Serial.print(",");
    Serial.print(prbs_tick);
    Serial.print(",");
    Serial.print(prbs_lvl);
    Serial.print(",");
    Serial.print(in_mark_val);

    Serial.println();
  }

  // ---------- Slow LED heartbeat ----------
  static uint32_t lastBlink_ms = 0;
  const uint32_t now_ms2 = millis();
  if (now_ms2 - lastBlink_ms >= 500) {
    lastBlink_ms = now_ms2;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}






