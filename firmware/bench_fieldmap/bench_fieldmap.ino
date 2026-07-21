/*
 * ExoKey bench field-map firmware  --  COUPON.md tests T1 (plunge) / T3 (lateral).
 *
 * Streams the raw 3-axis field (Bx, By, Bz) from one TLV493D over USB serial as CSV, so a
 * caliper-stepped magnet can be logged straight to a file and dropped into the T1/T3 tables.
 *
 *   Board:    Seeed XIAO nRF52840  (FQBN Seeeduino:nrf52:xiaonRF52840). 3.3 V, native-USB serial.
 *   Sensor:   TLV493D-A1B6, I2C.  Wire: VCC->3V3, GND->GND, SDA->D4, SCL->D5.  (Pull-ups are on
 *             the breakout; a bare chip needs 4.7-10 kOhm SDA/SCL pull-ups + a 100 nF cap.)
 *   Library:  Arduino Library Manager -> "XENSIV 3D Magnetic Sensor TLx493D" (Infineon).
 *   Output:   CSV "t_us,Bx_mT,By_mT,Bz_mT".  Lines starting with '#' are info -- skip them.
 *             LSB conversion: counts = mT / 0.098  (the model's HALL_LSB).
 *
 * NOT hardware-tested -- compiled against the library, but verify on the bench. Single sensor; for
 * the 5-sensor device wrap each read in a TCA9548A channel select (see docs/electronics.md). The
 * XENSIV library also drives the device's TLI493D-W2BW -- swap the class to TLx493D_W2BW there.
 */
#include <Wire.h>
#include "TLx493D_inc.hpp"
using namespace ifx::tlx493d;

#define AVERAGE   8         // samples averaged per output line -- cuts the ~0.2 mT noise by ~sqrt(N)
#define I2C_HZ    400000    // fast-mode I2C

TLx493D_A1B6 dut(Wire, TLx493D_IIC_ADDR_A0_e);

void initSensor() {
  dut.begin(false, false, false, true);       // no power-pin (always-on single sensor), run init
  Wire.setClock(I2C_HZ);
  dut.setSensitivity(TLx493D_FULL_RANGE_e);   // +-130 mT so the 61-80 mT plunge/hard-stop never clips
}

// Unstick a TLV493D-A1B6 holding SDA low (its known bus-lock): clock out 9 SCL pulses + a STOP so the
// slave releases, then reset+re-init. Layer 1 of the recovery in docs/electronics.md.
void i2cRecover() {
  Wire.end();
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT);                        // manual STOP: SDA low->high while SCL high
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  Wire.begin();
  Wire.setClock(I2C_HZ);
  dut.reset(true, false);                      // A1B6 reset
  initSensor();
}

void setup() {
  Serial.begin(115200);     // native USB CDC -- baud is nominal, throughput is USB-limited
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) { }  // wait up to 3 s for a monitor, then run headless
  initSensor();
  Serial.print("# ExoKey bench field-map -- TLV493D, 0.098 mT/LSB, average=");
  Serial.println(AVERAGE);
  Serial.println("# hold the magnet still at each caliper step; let it settle, then move on.");
  Serial.println("t_us,Bx_mT,By_mT,Bz_mT");
}

void loop() {
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < AVERAGE; i++) {
    double x, y, z;
    if (!dut.getMagneticField(&x, &y, &z)) {   // false -> read failed
      Serial.println("# i2c error -- recovering");
      i2cRecover();
      return;                                  // drop this line; loop() retries
    }
    sx += x;  sy += y;  sz += z;               // mT
    n++;
    delay(2);
  }
  if (n == 0) return;
  Serial.print(micros());   Serial.print(',');
  Serial.print(sx / n, 3);  Serial.print(',');
  Serial.print(sy / n, 3);  Serial.print(',');
  Serial.println(sz / n, 3);
}
