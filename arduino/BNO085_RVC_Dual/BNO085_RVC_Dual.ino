/*
  BNO085 Dual IMU - UART RVC Mode using Adafruit Library
  =======================================================
  
  Reads two BNO085 sensors in UART-RVC (Robot Vector Control) mode
  using the Adafruit_BNO08x_RVC library and outputs Euler angles 
  (Yaw, Pitch, Roll) as CSV to Serial.
  
  Hardware Setup for Arduino R4 WiFi/Minima:
  ------------------------------------------
  
  Power & Ground (both sensors):
    • Arduino 3.3V  →  BNO085 VIN
    • Arduino GND   →  BNO085 GND
    
  BNO085 Sensor #1 (Hardware Serial):
    • Arduino RX1 (Pin 0)   →  BNO085 TX
    • Arduino TX1 (Pin 1)   →  BNO085 RX
    • BNO085 PS0            →  3.3V (enables UART-RVC mode)
    • BNO085 PS1            →  GND
  
  BNO085 Sensor #2 (SoftwareSerial):
    • Arduino Pin 2 (RX)    →  BNO085 TX
    • Arduino Pin 3 (TX)    →  BNO085 RX
    • BNO085 PS0            →  3.3V (enables UART-RVC mode)
    • BNO085 PS1            →  GND
  
  Library Required:
  ----------------
  Install via Arduino Library Manager:
    - Adafruit BNO08x RVC library
  
  CSV Output Format:
  -----------------
  t_ms,s1_y,s1_p,s1_r,s2_y,s2_p,s2_r
  
  where:
    t_ms  = timestamp in milliseconds
    s1_y  = sensor 1 yaw (degrees)
    s1_p  = sensor 1 pitch (degrees)
    s1_r  = sensor 1 roll (degrees)
    s2_y  = sensor 2 yaw (degrees)
    s2_p  = sensor 2 pitch (degrees)
    s2_r  = sensor 2 roll (degrees)
*/

#include <SoftwareSerial.h>
#include "Adafruit_BNO08x_RVC.h"

// Initialize RVC objects
Adafruit_BNO08x_RVC rvc1;
SoftwareSerial softSerial(2, 3);  // RX=2, TX=3
Adafruit_BNO08x_RVC rvc2;

// Persistent data structures
BNO08x_RVC_Data data1, data2;

// Timing for output
unsigned long last_print_ms = 0;
const unsigned long PRINT_INTERVAL_MS = 20;  // 50 Hz output rate

void setup() {
  // Initialize USB serial for output to PC
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize serial connections to BNO085 sensors
  Serial1.begin(115200);      // Hardware serial for sensor 1
  softSerial.begin(115200);   // Software serial for sensor 2

  // Initialize RVC objects with their serial ports
  rvc1.begin(&Serial1);
  rvc2.begin(&softSerial);

  // Print CSV header for Python script
  Serial.println("t_ms,s1_y,s1_p,s1_r,s2_y,s2_p,s2_r");
}

void loop() {
  // Constantly poll sensors to keep buffers filled
  rvc1.read(&data1);
  rvc2.read(&data2);

  // Output at fixed intervals
  unsigned long now_ms = millis();
  if (now_ms - last_print_ms >= PRINT_INTERVAL_MS) {
    last_print_ms = now_ms;
    
    // Output CSV line
    Serial.print(now_ms); Serial.print(",");
    Serial.print(data1.yaw); Serial.print(",");
    Serial.print(data1.pitch); Serial.print(",");
    Serial.print(data1.roll); Serial.print(",");
    Serial.print(data2.yaw); Serial.print(",");
    Serial.print(data2.pitch); Serial.print(",");
    Serial.println(data2.roll);
  }
}
