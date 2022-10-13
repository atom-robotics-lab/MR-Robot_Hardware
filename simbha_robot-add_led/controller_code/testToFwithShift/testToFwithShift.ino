#include "Adafruit_VL53L0X.h"

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x31
#define LOX2_ADDRESS 0x30

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

int latchPin = 15;  // RCLK - 4
int clockPin = 18; // SRCLK - 12
int dataPin = 23;  // SER - 14

byte leds = 0;    // Variable to hold the pattern of which LEDs are currently turned on or off

void updateShiftRegister(int x);
const byte address_set_pattern[8] =
{
  0,
  1,
  3,
  7,
  15,
  31,
  63,
  127,
  //  255
};

void setID() {
  // all reset
  //  bitSet(leds, i);    // Set the bit that controls that LED in the variable 'leds'
  updateShiftRegister(address_set_pattern[0]);
  pinData();
  delay(100);
  //  Serial.println(i);
  // all unreset
  updateShiftRegister(address_set_pattern[3]);
  pinData();
  delay(100);

  // activating LOX1 and resetting LOX2
  updateShiftRegister(address_set_pattern[1]);
  pinData();
  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1);
  }
  delay(100);
  // activating LOX2
  updateShiftRegister(address_set_pattern[2]);
  pinData();
  delay(100);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1);
  }
}

void read_dual_sensors() {

  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if (measure1.RangeStatus != 4) {    // if not out of range
    Serial.print(measure1.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if (measure2.RangeStatus != 4) {
    Serial.print(measure2.RangeMilliMeter);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}
void pinInit()
{
  pinMode(13, INPUT);
  pinMode(27, INPUT);
  pinMode(26, INPUT);
  pinMode(25, INPUT);
  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(5, INPUT);
  pinMode(19, INPUT);
}
void pinData()
{
//  Serial.print("A");
//  Serial.print(digitalRead(13));
//  Serial.print('\t');
//  Serial.print("B");
//  Serial.print(digitalRead(27));
//  Serial.print('\t');
//  Serial.print("C");
//  Serial.print(digitalRead(26));
//  Serial.print('\t');
//  Serial.print("D");
//  Serial.print(digitalRead(25));
//  Serial.print('\t');
//  Serial.print("E");
//  Serial.print(digitalRead(33));
//  Serial.print('\t');
//  Serial.print("F");
//  Serial.print(digitalRead(32));
//  Serial.print('\t');
  Serial.print("G");
  Serial.print(digitalRead(5));
  Serial.print('\t');
  Serial.print("H");
  Serial.print(digitalRead(19));
  Serial.println();
}
void updateShiftRegister(byte x)
{
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, x);
  digitalWrite(latchPin, HIGH);
}
void setup()
{
  // Set all the pins of 74HC595 as OUTPUT
  delay(1500);
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinInit();

  Serial.begin(115200);
  Serial.println("Starting...");
  setID();
  Serial.println("ID set");
}
void loop()
{
  leds = 0;
  read_dual_sensors();
  delay(500);
}
