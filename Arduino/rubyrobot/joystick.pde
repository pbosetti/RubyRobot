#include <EEPROM.h>

//--------- EEPROM PARAMETERS ----------
#define EEPROM_START 70

//-------- JOYSTICK PARAMETERS ---------
#define PIN_BUTT 2
#define PIN_AXES 0
#define CAL_PIN 13
#define MOVING_PIN 8
#define DEAD 0.06

template <class T> int EEPROM_write(int ee, const T& value)
{
  byte const *p = reinterpret_cast<byte const *>(&value);
  int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T> int EEPROM_read(int ee, T& value)
{
  byte *p = reinterpret_cast<byte *>(&value);
  int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

int remap(int val, int min, int zero, int max, int d_min, int d_max) {
  int result = 0;
  int dead_min = (int)(zero * (1.0 - DEAD));
  int dead_max = (int)(zero * (1.0 + DEAD));
  int middle = (d_max + d_min) / 2;
  if (val < dead_min) {
    result = map(val, min, dead_min, d_min, middle);
  }
  else if (val > dead_max) {
    result = map(val, dead_max, max, middle, d_max);
  }
  else {
    result = middle;
  }  
  return result;
}

int readEEPROM (unsigned int *min, unsigned int *max, unsigned int *zeros) {
  for (int i = 0; i < 4; i++) {
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * i, min[i]);
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
  }
}

int writeEEPROM (unsigned int *min, unsigned int *max, unsigned int *zeros) {
  for (int i = 0; i < 4; i++) {
    EEPROM_write(EEPROM_START + sizeof(unsigned int) * i, min[i]);
    EEPROM_write(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
    EEPROM_write(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
  }
}

int setJoystickPins() {
 for (int i = 0; i < 4; i++) {
    pinMode(i + PIN_BUTT, INPUT);
    pinMode(i + MOVING_PIN, OUTPUT);
  } 
}

int readJoystickButtons(short int buttons) {
  for (int i = PIN_BUTT; i < PIN_BUTT + 4; i++) {
      buttons |= !digitalRead(i);
      buttons <<= 1;
    }
  buttons >>= 1;
}

unsigned int readJoystickAxis(int number) {
  return analogRead(number+PIN_AXES);
}

int axisValue(int i) {
  int v = remap(analogRead(i + PIN_AXES),min[i], zeros[i],max[i], -100, 100);
  digitalWrite(MOVING_PIN + i, v == 0 ? LOW : HIGH);
  return v;
}

int calibrationLED(byte status) {
      digitalWrite(CAL_PIN, status);
}
