#include <Wire.h>

// =====================================================
// USER SETTINGS
// =====================================================
static const uint8_t ADDR1 = 0x68;   // IMU 1
static const uint8_t ADDR2 = 0x69;   // IMU 2

static const uint32_t I2C_CLOCK_HZ = 100000;  // Start conservative
static const uint32_t SERIAL_BAUD  = 230400;  // 115200 also OK
static const uint32_t SAMPLE_HZ    = 100;     // Output rate

// =====================================================
// BMI160 REGISTERS (I2C)
// =====================================================
static const uint8_t REG_CHIP_ID   = 0x00; // expect 0xD1
static const uint8_t REG_PMU_STAT  = 0x03;
static const uint8_t REG_GYR_DATA  = 0x0C; // 6 bytes
static const uint8_t REG_ACC_DATA  = 0x12; // 6 bytes
static const uint8_t REG_CMD       = 0x7E;

static const uint8_t CMD_SOFTRESET  = 0xB6;
static const uint8_t CMD_ACC_NORMAL = 0x11;
static const uint8_t CMD_GYR_NORMAL = 0x15;

// =====================================================
// DATA STRUCTURE (must be defined early!)
// =====================================================
struct ImuRaw {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  uint8_t ok;
};

// =====================================================
// I2C HELPERS
// =====================================================
bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)n, (int)true);
  if (got != n) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool i2cReadByte(uint8_t addr, uint8_t reg, uint8_t &val) {
  return i2cReadBytes(addr, reg, &val, 1);
}

// =====================================================
// BMI160 FUNCTIONS
// =====================================================
bool bmi160Init(uint8_t addr) {
  if (!i2cWriteByte(addr, REG_CMD, CMD_SOFTRESET)) return false;
  delay(50);

  uint8_t chip = 0;
  if (!i2cReadByte(addr, REG_CHIP_ID, chip)) return false;
  if (chip != 0xD1) return false;

  if (!i2cWriteByte(addr, REG_CMD, CMD_ACC_NORMAL)) return false;
  delay(10);
  if (!i2cWriteByte(addr, REG_CMD, CMD_GYR_NORMAL)) return false;
  delay(50);

  uint8_t pmu = 0;
  if (!i2cReadByte(addr, REG_PMU_STAT, pmu)) return false;

  return true;
}

bool bmi160ReadRaw(uint8_t addr, ImuRaw &out) {
  uint8_t a[6], g[6];

  if (!i2cReadBytes(addr, REG_ACC_DATA, a, 6)) return false;
  if (!i2cReadBytes(addr, REG_GYR_DATA, g, 6)) return false;

  out.ax = (int16_t)((a[1] << 8) | a[0]);
  out.ay = (int16_t)((a[3] << 8) | a[2]);
  out.az = (int16_t)((a[5] << 8) | a[4]);

  out.gx = (int16_t)((g[1] << 8) | g[0]);
  out.gy = (int16_t)((g[3] << 8) | g[2]);
  out.gz = (int16_t)((g[5] << 8) | g[4]);

  return true;
}

// =====================================================
// CHECKSUM (XOR over payload)
// =====================================================
uint8_t xorChecksum(const char* s) {
  uint8_t c = 0;
  while (*s) c ^= (uint8_t)(*s++);
  return c;
}

// =====================================================
// GLOBAL STATE
// =====================================================
uint32_t seq = 0;
uint8_t fail1 = 0, fail2 = 0;

// =====================================================
// I2C RESET
// =====================================================
void resetI2CBus() {
  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(2000, true);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(2000, true);

  bool ok1 = bmi160Init(ADDR1);
  bool ok2 = bmi160Init(ADDR2);

  Serial.println("# BMI160 dual I2C streamer");
  Serial.print("# IMU1 init: "); Serial.println(ok1 ? "OK" : "FAIL");
  Serial.print("# IMU2 init: "); Serial.println(ok2 ? "OK" : "FAIL");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  static uint32_t nextTick = 0;
  const uint32_t period_us = 1000000UL / SAMPLE_HZ;

  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick = now + period_us;

  ImuRaw i1{}, i2{};
  bool ok1 = bmi160ReadRaw(ADDR1, i1);
  bool ok2 = bmi160ReadRaw(ADDR2, i2);

  if (!ok1) fail1++; else fail1 = 0;
  if (!ok2) fail2++; else fail2 = 0;

  if (fail1 >= 5) { bmi160Init(ADDR1); fail1 = 0; }
  if (fail2 >= 5) { bmi160Init(ADDR2); fail2 = 0; }

  if (!ok1 && !ok2) {
    static uint8_t bothFail = 0;
    bothFail++;
    if (bothFail >= 5) {
      resetI2CBus();
      bmi160Init(ADDR1);
      bmi160Init(ADDR2);
      bothFail = 0;
    }
  }

  char payload[220];
  snprintf(payload, sizeof(payload),
    "%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    (unsigned long)seq,
    (unsigned long)now,
    i1.ax, i1.ay, i1.az, i1.gx, i1.gy, i1.gz, ok1 ? 1 : 0,
    i2.ax, i2.ay, i2.az, i2.gx, i2.gy, i2.gz, ok2 ? 1 : 0
  );

  uint8_t chk = xorChecksum(payload);

  Serial.print('$');
  Serial.print(payload);
  Serial.print('*');
  if (chk < 16) Serial.print('0');
  Serial.println(chk, HEX);

  seq++;
}

