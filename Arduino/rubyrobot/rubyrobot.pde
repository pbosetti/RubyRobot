#include <SLCD.h>
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
=======
#include <EEPROM.h>
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
#include <MegaServo.h>

// -------- GENERAL PARAMETERS ---------
#define PI 3.141592653589793;
#define BAUD 57600
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
=======
#define EEPROM_START 70
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde

// --------- SERVO PARAMETERS ----------
#define NBR_SERVOS 4
#define FIRST_SERVO_PIN 50

<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
=======
//-------- JOYSTICK PARAMETERS ---------
#define PIN_BUTT 2
#define PIN_AXES 0
#define CAL_PIN 13
#define MOVING_PIN 8
#define DEAD 0.06

>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
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

boolean manual = true;
byte incoming;
float offset[4] = {0.5*PI, 0.5*PI, 0.5*PI, 0.5*PI};
float increase[4] = {0.0, 0.0, 0.0, 0.0};
float coords[4] = {100.0, 100.0, -10.0, -0.5*PI};
//float minlimits[4] = {-90.0/180*PI,0.0,-90.0/180*PI,-90.0/180*PI};
float minlimits[4] = {-180.0/180*PI,-90.0/180*PI,-180.0/180*PI,-180.0/180*PI};
//float maxlimits[4] = {90.0/180*PI,85.0/180*PI,90.0/180*PI,90.0/180*PI};
float maxlimits[4] = {180.0/180*PI,90.0/180*PI,180.0/180*PI,180.0/180*PI};
float joints[4] = {0.0, 0.0, 0.0, 0.0};
float l[4]      = {0.0, 100.0, 100.0, 10.0};

SLCD lcd = SLCD(numRows, numCols);
MegaServo Servos[NBR_SERVOS];

<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
void setup()
{
  setJoystickPins();
    for( int i = 0; i < NBR_SERVOS; i++)
    Servos[i].attach( FIRST_SERVO_PIN + i);
  Serial.begin(BAUD);
  lcd.init();
  readEEPROM(min,max,zeros);
=======
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
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
  if (manual) {
    lcd.clear();
    lcd.print("Manual",0,5);
    lcd.print("mode",1,6);
  }
  else {
    lcd.clear();
    lcd.print("Automatic",0,3);
    lcd.print("mode",1,6);
  }
}

void loop() 
{ 
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
  short int buttons  = 0;
=======
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
  if (Serial.available() > 0) {
    incoming = Serial.read();
    switch (incoming) {
      case 'A': manual = false; break;
      case 'M': manual = true;  break;
      default: ;
    }
  if (manual) {
    lcd.clear();
    lcd.print("Manual",0,5);
    lcd.print("mode",1,6);  
  }
  else {
    lcd.clear();
    lcd.print("Automatic",0,3);
    lcd.print("mode",1,6);  
  }  
  }
  if (manual) { // Manual mode
    static bool calibrated = true;
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
    int v = 0;  
    readJoystickButtons(buttons);
    if(!calibrated) {
      calibrationLED(HIGH);
      unsigned int val = 0;
      for (int i; i < 4; i++) {
        val = readJoystickAxis(i);
=======
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
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
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
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
        writeEEPROM (min,max,zeros);
=======
        for (int i = 0; i < 4; i++) {
          EEPROM_write(EEPROM_START + sizeof(unsigned int) * i, min[i]);
          EEPROM_write(EEPROM_START + sizeof(unsigned int) * (4+i), zeros[i]);
          EEPROM_write(EEPROM_START + sizeof(unsigned int) * (8+i), max[i]);
        }
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
        delay(1000);
        lcd.clear();
      }
    }
    else {
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
      calibrationLED(LOW);
=======
      digitalWrite(CAL_PIN, LOW);
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
    }
    Serial.print(" ");
    Serial.print(buttons);
    Serial.print(" ");
    for (int i = 0; i < 4; i++) {
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
      v= axisValue(i);
=======
      v = remap(analogRead(i + PIN_AXES),min[i], zeros[i],max[i], -100, 100);
      digitalWrite(MOVING_PIN + i, v == 0 ? LOW : HIGH);
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
      Serial.print(v);
      Serial.print(" ");
      if (i==3)
        increase[i] = v/5000.0; // phi angle
      else  
        increase[i] = v/100.0;  // xyz coords
      coords[i] += increase[i];      
    }
    Serial.println("");
    if(buttons == 14) {
      calibrated = false;
      for (int i = 0; i < 4; i++) {
<<<<<<< HEAD:Arduino/rubyrobot/rubyrobot.pde
        zeros[i] = readJoystickAxis(i);
=======
        zeros[i] = analogRead(i + PIN_AXES);
>>>>>>> 12d6fd0045be5ce94e14c98b5e778382537e0279:Arduino/rubyrobot/rubyrobot.pde
        min[i] = 1024;
        max[i] = 0;
      }
     lcd.clear();
     lcd.print("Calibration", 0, 2);
     lcd.print("mode", 1, 6);
    }
    delay(20);
    int i;
  int length = 4;
  char j[10];
  int result = ik(coords, joints, l, minlimits, maxlimits, length);
  if (result) {
    for(i = 0; i < length; i++) {
        Servos[i].write((joints[i]+offset[i])*180/PI);
    }
  }
  else {
    lcd.clear();
    lcd.print("Out of range!",0,1);
    for (i = 0; i < length; i++)
      coords[i] -= increase[i];
    delay(100);
  }
  delay(10);
  }
  else { // Automatic mode
  }
  
}

