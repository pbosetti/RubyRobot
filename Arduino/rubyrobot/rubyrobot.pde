#include <SLCD.h>
#include <MegaServo.h>

// -------- GENERAL PARAMETERS ---------
#define PI 3.141592653589793;
#define BAUD 57600

// --------- SERVO PARAMETERS ----------
#define NBR_SERVOS 4
#define FIRST_SERVO_PIN 50

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
boolean servo  = false;
int nbyte = 0;
int length = 4;

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

void setup()
{
  setJoystickPins();
    for( int i = 0; i < NBR_SERVOS; i++)
    Servos[i].attach( FIRST_SERVO_PIN + i);
  Serial.begin(BAUD);
  lcd.init();
  readEEPROM(min,max,zeros);
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
  short int buttons  = 0;
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
    int v = 0;  
    buttons = readJoystickButtons(buttons);
    if(!calibrated) {
      calibrationLED(HIGH);
      unsigned int val = 0;
      for (int i; i < 4; i++) {
        val = readJoystickAxis(i);
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
        writeEEPROM (min,max,zeros);
        delay(1000);
        lcd.clear();
      }
    }
    else {
      calibrationLED(LOW);
    }
    Serial.print(" ");
    Serial.print(buttons);
    Serial.print(" ");
    for (int i = 0; i < 4; i++) {
      v= axisValue(i);
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
        zeros[i] = readJoystickAxis(i);
        min[i] = 1024;
        max[i] = 0;
      }
     lcd.clear();
     lcd.print("Calibration", 0, 2);
     lcd.print("mode", 1, 6);
    }
    delay(20);
    int i;
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
    if (Serial.available() > 0) {
      incoming = Serial.read();
      if (incoming == 'S')
        servo = true;
      if (servo) {
        nbyte++;
        switch (nbyte%2) {
          case 1: joints[nbyte/2] = incoming*256; break;
          case 2: joints[nbyte/2] += incoming; break;
        }
        if (nbyte == 8) {
          servo = false;
          nbyte = 0;
          for(int i = 0; i < length; i++) {
              Servos[i].write((joints[i]+offset[i])*180/PI);
          }          
        }
      }
    }
  }
  
}

