#include <SLCD.h>
#include <EEPROM.h>
#include <MegaServo.h>

// -------- GENERAL PARAMETERS ---------
#define BAUD 57600
#define EEPROM_START 70
#define MANUAL_PIN 6

// --------- SERVO PARAMETERS ----------
#define NBR_SERVOS 4
#define FIRST_SERVO_PIN 50

//-------- JOYSTICK PARAMETERS ---------
#define PIN_BUTT 2
#define PIN_AXES 0
#define CAL_PIN 13
#define MOVING_PIN 8
#define DEAD 0.06

//------ POTENTIOMETER PARAMETERS ------
#define POT_PIN 8

//----------- LCD PARAMETERS -----------
static int numRows = 2;
static int numCols = 16;

// --------- GLOBAL VARIABLES ----------

unsigned int min[] = {
  1024, 1024, 1024, 1024      };
unsigned int max[] = {
  0, 0, 0, 0    };
unsigned int zeros[] = {
  0, 0, 0, 0  };

boolean manual;
boolean writeServo = false;
boolean readServo  = false;
int incoming = 0;
int nbyte = 0;
int servoNumber = 0;
byte firstbyte;
byte secondbyte;

SLCD lcd = SLCD(numRows, numCols);
MegaServo Servos[NBR_SERVOS];

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
  for( int i = 0; i < NBR_SERVOS; i++)
    Servos[i].attach( FIRST_SERVO_PIN + i);
  Serial.begin(BAUD);
  lcd.init();
  for (int i = 0; i < 4; i++) {
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * i, min[i]);
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
    EEPROM_read(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
  }
  pinMode(MANUAL_PIN, INPUT);
}

void loop() 
{ 
  if (manual != digitalRead(MANUAL_PIN) && manual == true) {
    lcd.clear();
    lcd.print("Manual",0,4);
    lcd.print("mode",0,6);  
  }
  if (manual != digitalRead(MANUAL_PIN) && manual == false) {
    lcd.clear();
    lcd.print("Automatic",0,2);
    lcd.print("mode",0,6);  
  }  
  manual = digitalRead(MANUAL_PIN);
  if (manual) { // Manual mode
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
      if (buttons == 1) {
        lcd.clear();
        lcd.print("Data capture",0,2);
        lcd.print("begin",1,5);
      }
      if (buttons == 8) {
        lcd.print("ended",1,5);
        delay(1000);
        lcd.clear();
      } 
      if(buttons == 10) {
        calibrated = true;
        lcd.print("ended", 1, 5);
        for (int i = 0; i < 4; i++) {
          EEPROM_write(EEPROM_START + sizeof(unsigned int) * i, min[i]);
          EEPROM_write(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
          EEPROM_write(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
        }
        delay(1000);
        lcd.clear();
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
  else { // Automatic mode
    if (Serial.available() > 0) {
      incoming = Serial.read();
      nbyte++;

      switch (nbyte) {
      case 1:  
        if (incoming%119 == 0) {
          writeServo  = true;
          servoNumber = incoming / 119;
        }
        else {
          readServo   = true;
          servoNumber = incoming / 114;
        }; 
        break;                      
      case 2: 
        firstbyte  = incoming; 
        break;
      case 3: 
        secondbyte = incoming;
        if (writeServo) { 
          int value = map(firstbyte*127+secondbyte,0,18000,600,2400);
          Servos[servoNumber].writeMicroseconds(value);
          writeServo = false; 
          nbyte = 0;
          value = 0;
        }
        break;
      default: 
        Serial.println("Too many char!");
      }
    }
  }
}
