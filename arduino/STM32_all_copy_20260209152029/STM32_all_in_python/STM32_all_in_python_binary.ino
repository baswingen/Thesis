/*
  High-performance STM32F401 Arduino sketch (BINARY VERSION) combining:
  1) PRBS-15 TRIG output at 2000 Hz pin rate, 10 Hz chip rate (continuous)
  2) Dual BNO085/BNO080 UART-RVC readout (two HW UARTs)
  3) Configurable button-matrix scanning (non-blocking, debounced)

  Binary Protocol: 921600 baud, 500 Hz cadence.
  Packet structure (67 bytes):
    [0]   Sync1: 0xAA
    [1]   Sync2: 0x55
    [2-5] t_ms: uint32_t
    [6]   imu_ok: uint8_t (bit0=imu1, bit1=imu2)
    [7-30] IMU1: 6x float (yaw, pitch, roll, ax, ay, az)
    [31-54] IMU2: 6x float (yaw, pitch, roll, ax, ay, az)
    [55-56] keys_mask: uint16_t
    [57-58] keys_rise: uint16_t
    [59-60] keys_fall: uint16_t
    [61-64] prbs_tick: uint32_t
    [65]    prbs_bits: uint8_t (bit0=lvl, bit1=in_mark)
    [66]    checksum: uint8_t (XOR of bytes 2-65)

  Target: main-loop sampling/logging at 500 Hz (2 ms)
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

// --- PRBS TRIG CONFIG ---
#if ENABLE_PRBS_TRIG
static const uint8_t PRBS_PIN = PA8;
static const uint32_t CHIP_RATE_HZ = 10;
static const uint32_t TRIG_OUTPUT_HZ = 2000;
static const uint32_t TICKS_PER_CHIP = TRIG_OUTPUT_HZ / CHIP_RATE_HZ;
static const uint16_t PRBS15_SEED = 0x7ACE;

volatile uint16_t lfsr = PRBS15_SEED;
volatile uint32_t prbs_chip_index = 0;
volatile uint8_t  prbs_level = 0;
volatile uint8_t  in_mark = 0;

static volatile uint32_t chip_counter = 0;
static volatile uint8_t  sub_tick = 0;

HardwareTimer *prbsTimer = nullptr;

static inline uint8_t lfsr_next_bit_prbs15(volatile uint16_t &s) {
  uint8_t b14 = (s >> 14) & 1;
  uint8_t b13 = (s >> 13) & 1;
  uint8_t newbit = b14 ^ b13;
  s = (uint16_t)((s << 1) | newbit);
  return newbit;
}

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

  prbsTimer = new HardwareTimer(TIM3);
  prbsTimer->setOverflow(TRIG_OUTPUT_HZ, HERTZ_FORMAT);
  prbsTimer->attachInterrupt(onPrbsTick);
  prbsTimer->resume();
}
#endif

// ========================= DUAL IMU CONFIG =========================
#if ENABLE_DUAL_IMU
HardwareSerial IMU1Serial(PA10, PA9);
HardwareSerial IMU2Serial(PA3,  PA2);

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
      return true;
    }
    delay(50);
  }
  return false;
}

static void imuInit() {
  imu1_ok = initRVC(rvc1, IMU1Serial, "BNO#1");
  imu2_ok = initRVC(rvc2, IMU2Serial, "BNO#2");
}
#endif

// ========================= MATRIX CONFIG ===========================
#if ENABLE_MATRIX
const uint8_t ROWS = 3;
const uint8_t COLS = 3;
const uint8_t rowPins[ROWS] = { PA0, PA1, PA4 };
const uint8_t colPins[COLS] = { PB0, PB1, PB10 };
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
      if (digitalRead(colPins[c]) == LOW) s |= (1u << (r * COLS + c));
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
#endif

// ========================= OUTPUT / SCHEDULING ======================
static uint32_t t0_ms = 0;
static const uint32_t PRINT_HZ = 500;
static const uint32_t PRINT_PERIOD_US = 1000000UL / PRINT_HZ;
static uint32_t nextPrint_us = 0;

typedef struct __attribute__((packed)) {
  uint8_t  sync1;     // 0xAA
  uint8_t  sync2;     // 0x55
  uint32_t t_ms;
  uint8_t  imu_ok;    // bit0=imu1, bit1=imu2
  float    imu1[6];   // y,p,r,ax,ay,az
  float    imu2[6];
  uint16_t keys_mask;
  uint16_t keys_rise;
  uint16_t keys_fall;
  uint32_t prbs_tick;
  uint8_t  prbs_bits; // bit0=lvl, bit1=in_mark
  uint8_t  checksum;
} Packet;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(921600);
  delay(50);
#if ENABLE_MATRIX
  matrixInit();
#endif
#if ENABLE_PRBS_TRIG
  prbsInit();
#endif
#if ENABLE_DUAL_IMU
  imuInit();
#endif
  t0_ms = millis();
  nextPrint_us = micros() + PRINT_PERIOD_US;
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  const uint32_t now_us = micros();

#if ENABLE_MATRIX
  static uint32_t nextMatrix_us = 0;
  if ((int32_t)(now_us - nextMatrix_us) >= 0) {
    nextMatrix_us += MATRIX_SCAN_US;
    matrixUpdateDebounced(matrixScanOnce());
  }
#endif

#if ENABLE_DUAL_IMU
  static BNO08x_RVC_Data last1 = {0}, last2 = {0};
  static bool have1 = false, have2 = false;
  BNO08x_RVC_Data tmp;
  if (imu1_ok) { while (rvc1.read(&tmp)) { last1 = tmp; have1 = true; } }
  if (imu2_ok) { while (rvc2.read(&tmp)) { last2 = tmp; have2 = true; } }
#endif

  if ((int32_t)(now_us - nextPrint_us) >= 0) {
    nextPrint_us += PRINT_PERIOD_US;

    Packet pkg;
    pkg.sync1 = 0xAA;
    pkg.sync2 = 0x55;
    pkg.t_ms = millis() - t0_ms;
    pkg.imu_ok = (imu1_ok ? 1 : 0) | (imu2_ok ? 2 : 0);

#if ENABLE_DUAL_IMU
    pkg.imu1[0] = last1.yaw; pkg.imu1[1] = last1.pitch; pkg.imu1[2] = last1.roll;
    pkg.imu1[3] = last1.x_accel; pkg.imu1[4] = last1.y_accel; pkg.imu1[5] = last1.z_accel;
    pkg.imu2[0] = last2.yaw; pkg.imu2[1] = last2.pitch; pkg.imu2[2] = last2.roll;
    pkg.imu2[3] = last2.x_accel; pkg.imu2[4] = last2.y_accel; pkg.imu2[5] = last2.z_accel;
#else
    memset(pkg.imu1, 0, sizeof(pkg.imu1));
    memset(pkg.imu2, 0, sizeof(pkg.imu2));
#endif

#if ENABLE_MATRIX
    pkg.keys_mask = stableState;
    pkg.keys_rise = (uint16_t)(stableState & ~lastStableState);
    pkg.keys_fall = (uint16_t)(~stableState & lastStableState);
    lastStableState = stableState;
#else
    pkg.keys_mask = 0; pkg.keys_rise = 0; pkg.keys_fall = 0;
#endif

#if ENABLE_PRBS_TRIG
    pkg.prbs_tick = prbs_chip_index;
    pkg.prbs_bits = (prbs_level ? 1 : 0) | (in_mark ? 2 : 0);
#else
    pkg.prbs_tick = 0; pkg.prbs_bits = 0;
#endif

    // Compute checksum (XOR of all bytes except sync and checksum itself)
    uint8_t* p = (uint8_t*)&pkg;
    uint8_t cs = 0;
    for (size_t i = 2; i < sizeof(Packet) - 1; i++) {
      cs ^= p[i];
    }
    pkg.checksum = cs;

    Serial.write((uint8_t*)&pkg, sizeof(Packet));
  }

  static uint32_t lastBlink_ms = 0;
  if (millis() - lastBlink_ms >= 500) {
    lastBlink_ms = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}
