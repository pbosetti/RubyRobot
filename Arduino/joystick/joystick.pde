#include <SLCD.h>
#include <EEPROM.h>

#define PIN_BUTT 2
#define PIN_AXES 0
#define BAUD 57600
#define CAL_PIN 13
#define MOVING_PIN 8
#define DEAD 0.06
#define EEPROM_START 70

unsigned int min[] = {
  1024, 1024, 1024, 1024      };
unsigned int max[] = {
  0, 0, 0, 0    };
unsigned int zeros[] = {
  0, 0, 0, 0  };

// LCD

int numRows = 2;
int numCols = 16;

SLCD lcd = SLCD(numRows, numCols);

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
  for (int i = 0; i < 4; i++) {
    pinMode(i + PIN_BUTT, INPUT);
    pinMode(i + MOVING_PIN, OUTPUT);
  }
  Serial.begin(BAUD);
  lcd.init();
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
  int v = 0;
  
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
      lcd.print("ended", 1, 5);
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
  for (int i = 0; i < 4; i++) {
    v = remap(analogRead(i + PIN_AXES),min[i], zeros[i],max[i], -100, 100);
    digitalWrite(MOVING_PIN + i, v == 0 ? LOW : HIGH);
    Serial.print(v);
    Serial.print(" ");
  }
  Serial.println("");
  if(buttons == 14) {
    calibrated = false;
    for (int i = 0; i < 4; i++) {
      zeros[i] = analogRead(i + PIN_AXES);
      min[i] = 1024;
      max[i] = 0;
    }
   lcd.clear();
   lcd.print("Calibration", 0, 2);
   lcd.print("mode", 1, 6);
  }
  delay(20);  
}
