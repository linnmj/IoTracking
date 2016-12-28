// Accel Test

#include <Wire.h>
#include "Arduino.h"
/*#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep>*/

#define CONSOLE_SERIAL SerialUSB

#define ACCEL_ADR 0b0011110
#define TEST_REG 0x0F

#define TEST_INT1
#define TEST_INT2

volatile bool int1_flag = false;
volatile bool int2_flag = false;
int i = 0;

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void OFF() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}

void setup() 
{
  delay(5000);
  Wire.begin();

  CONSOLE_SERIAL.println("Testing Accelerometer");
  
  CONSOLE_SERIAL.println("Reading from register: 0x" + String(TEST_REG, HEX));
  CONSOLE_SERIAL.println("Response: 0x" + String(readReg(0x0F), HEX));
  
  pinMode(ACCEL_INT1, INPUT_PULLUP);
  pinMode(ACCEL_INT2, INPUT_PULLUP);
  attachInterrupt(ACCEL_INT1, ISR1, FALLING);
  attachInterrupt(ACCEL_INT2, ISR2, FALLING);
  SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;

  // Reboot 
  // CTRL0
  writeReg(0x1F, 0b10000000);

  // Set to 50z all axes active
  // CTRL1
  writeReg(0x20, 0b01010111);

  // Interrupt source 1
  // Set to be Y sensitive
  // IG_SRC1
  writeReg(0x30, 0b10001000); // Axes mask
  writeReg(0x32, 0b00111111); // Threshold
  writeReg(0x33, 0b00000000); // Duration

  // Interrupt source 2
  // Set to be X sensitive
  // IG_SRC2
  writeReg(0x34, 0b10000010); // Axes mask
  writeReg(0x36, 0b00111111); // Threshold
  writeReg(0x37, 0b00000000); // Duration

  // CTRL3 (INT1)
  // INT1 sources from IG_SRC1
#ifdef TEST_INT1
  writeReg(0x22, 0b00100000);
#else
  writeReg(0x22, 0b00000000);
#endif

  // CTRL4 (INT2)
  // INT2 sources from IG_SRC2
  // Note the bit positions are different
  // in CTRL3 and CTRL4 Datasheet p.38
#ifdef TEST_INT2
  writeReg(0x23, 0b00100000);
#else
  writeReg(0x23, 0b00000000);
#endif

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
}

void loop()
{
  if (int1_flag) {
    int1_flag = false;
    CONSOLE_SERIAL.println("INT1 Interrupt");
    RED();
  }
  if (int2_flag) {
    int2_flag = false;
    CONSOLE_SERIAL.println("INT2 Interrupt");
    GREEN();
  }

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  i++;
  if (i == 10){
    CONSOLE_SERIAL.println("HUSH LITTLE U-CONTROLLER");
    __WFI();
    i = 0;
  }
  
  BLUE();
  delay(500);
  OFF();
  delay(500);
  
  /*CONSOLE_SERIAL.print("Reading from register: 0x" + String(0x31, HEX));
  CONSOLE_SERIAL.println(", response: 0x" + String(readReg(0x31), BIN));
  
  CONSOLE_SERIAL.print("Reading from register: 0x" + String(0x35, HEX));
  CONSOLE_SERIAL.println(", response: 0x" + String(readReg(0x35), BIN));

  int16_t x_val = (readReg(0x29) << 8) | readReg(0x28);
  int16_t y_val = (readReg(0x2B) << 8) | readReg(0x2A);
  int16_t z_val = (readReg(0x2D) << 8) | readReg(0x2C);

  CONSOLE_SERIAL.println(String("Accelerometer Readings: ") + x_val + ", " + y_val + ", " + z_val);*/
 
  /*for (int i=0; i<1000; i++) {
    delayMicroseconds(1000);
  }*/
}

uint8_t readReg(uint8_t reg)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(ACCEL_ADR, 0x01);
  
  uint8_t val = Wire.read();
  Wire.endTransmission();

  return val;
}

uint8_t writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}

void ISR1()
{
  int1_flag = true;
  //RED();
}

void ISR2()
{
  int2_flag = true;
  //GREEN();
}

// SLEEP MODE PAGE 147 !!!!
