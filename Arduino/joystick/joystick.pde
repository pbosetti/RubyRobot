#include <EEPROM.h>

#define PIN_BUTT 2
#define PIN_AXES 0
#define BAUD 57600
#define CAL_PIN 13
#define DEAD 0.06
#define EEPROM_START 70

unsigned int min[] = {
  1024, 1024, 1024, 1024      };
unsigned int max[] = {
  0, 0, 0, 0    };
unsigned int zeros[] = {
  0, 0, 0, 0  };


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

void setup()
{
  for (int i = PIN_BUTT; i < PIN_BUTT + 4; i++)
    pinMode(i, INPUT);
  Serial.begin(BAUD);
  for (int i = 0; i < 4; i++) {
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * i, min[i]);
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
  }
}

void loop() 
{
  static bool calibrated = true;
  short int buttons  = 0;

  for (int i = PIN_BUTT; i < PIN_BUTT + 4; i++) {
    buttons |= !digitalRead(i);
    buttons <<= 1;
  }
  buttons >>= 1;

  if(!calibrated) {
    digitalWrite(CAL_PIN, HIGH);
    unsigned int val = 0;
    for (int i; i < 4; i++) {
      val = analogRead(i+PIN_AXES);
      if(min[i] > val)
        min[i] = val;
      if(max[i] < val)
        max[i] = val;
    }
    if(buttons == 10) {
      calibrated = true;
      for (int i = 0; i < 4; i++) {
        EEPROM_write(EEPROM_START + sizeof(unsigned int) * i, min[i]);
        EEPROM_write(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
        EEPROM_write(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
      }
    }
  }
  else {
    digitalWrite(CAL_PIN, LOW);
  }
  Serial.print(" ");
  Serial.print(buttons);
  Serial.print(" ");
  for (int i = PIN_AXES; i < PIN_AXES + 4; i++) {
    Serial.print(remap(analogRead(i),min[i-PIN_AXES], zeros[i-PIN_AXES],max[i-PIN_AXES], -100, 100));
    Serial.print(" ");
  }
  Serial.println("");
  if(buttons == 14) {
    calibrated = false;
    for (int i = 0; i < 4; i++) {
      zeros[i] = analogRead(i);
      min[i] = 1024;
      max[i] = 0;
    }
  }
  delay(20);  
}
