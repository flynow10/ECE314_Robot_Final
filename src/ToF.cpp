#include "ToF.hpp"

ToF::ToF(uint8_t pin) {
  Sensor = VL53L0X();
  xShutPin = pin;
  pinMode(xShutPin, OUTPUT);
  digitalWrite(xShutPin, LOW);
}

bool ToF::init(uint8_t address) {
  delay(20);
  digitalWrite(xShutPin, HIGH);
  delay(20);
  Sensor.setAddress(address);
  Sensor.setTimeout(500);
  Sensor.setMeasurementTimingBudget(200000);
  return Sensor.init();
}

uint16_t ToF::getRangeMilimeters() {
  return Sensor.readRangeSingleMillimeters();
}
