#include <Arduino.h>
// #include <MIDI.h>
#include <Wire.h>

#include "gpio.h"

bool gpio_get_pin_value(u8 pin) {
  // Serial.printf("Pin called: %d\n", pin);
  return true;  //DF
};

void gpio_set_gpio_pin(u8 pin) {
  // Serial.printf("Trigger on: %d\n", pin);
  // MIDI.sendNoteOn(60, 100, pin);
  if (pin > 0 && pin < 5) {
    char cmd[4] = { 0x00, (char)(pin - 1), 1, 0 };
    Wire.beginTransmission(TELEXO);
    Wire.write(cmd, 4);
    Wire.endTransmission();
  // // } else if (pin > 4 && pin < 9) {
  // //   char cmd[4] = { 0x11, (char)(pin - 5), 0xff, 0 };
  // //   Wire.beginTransmission(0x60);
  // //   Wire.write(cmd, 4);
  // //   Wire.endTransmission();
  }
  else {
    Serial.println("set: Should not happen");
  }
}

void gpio_clr_gpio_pin(u8 pin) {
  // Serial.printf("Trigger off: %d\n", pin);
  // MIDI.sendNoteOff(60, 0, pin);
  if (pin > 0 && pin < 5) {
    char cmd[4] = { 0x00, (char)(pin - 1), 0, 0 };
    Wire.beginTransmission(TELEXO);
    Wire.write(cmd, 4);
    Wire.endTransmission();
  }
  else {
    Serial.println("clr: Should not happen");
  }
  // // } else if (pin > 4 && pin < 9) {
  // //   char cmd[4] = { 0x11, (char)(pin - 5), 0, 0 };
  // //   Wire.beginTransmission(0x60);
  // //   Wire.write(cmd, 4);
  // //   Wire.endTransmission();
}
