#include <SLCD.h>
#include <MegaServo.h>

// -------- GENERAL PARAMETERS ---------
#define PI 3.141592653589793;
#define BAUD 57600

// --------- SERVO PARAMETERS ----------
#define NBR_SERVOS 4
#define BLS452_PIN 50
#define S9157_PIN  51
#define BLS551_PIN 52
#define S3156_PIN  53

#define BLS452_MIN 782
#define BLS452_MAX 2188
#define BLS452_DEGREES 141.0

#define S9157_MIN 782
#define S9157_MAX 2186
#define S9157_DEGREES 143.0

#define BLS551_MIN 782
#define BLS551_MAX 2182
#define BLS551_DEGREES 141.0

#define S3156_MIN 782
#define S3156_MAX 2185
#define S3156_DEGREES 141.0

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
boolean ruby  = false;
int nbyte = 0;
int length = 4;

byte incoming;
float offset[4] = {-0.389*PI, 0.055*PI, -0.75*PI, -0.66*PI};
float increase[4] = {0.0, 0.0, 0.0, 0.0};
float coords[4] = {250.0, 250.0, -50.0, -0.5*PI}; //[mm]
//float minlimits[4] = {-90.0/180*PI,0.0,-90.0/180*PI,-90.0/180*PI};
float minlimits[4] = {0.0,0.0,6.0,0.0};
//float maxlimits[4] = {90.0/180*PI,85.0/180*PI,90.0/180*PI,90.0/180*PI};
float maxlimits[4] = {BLS452_DEGREES,S9157_DEGREES,
                      BLS551_DEGREES,S3156_DEGREES};
float joints[4] = {0.0, 0.0, 0.0, 0.0};
float l[4]      = {0.0, 250.0, 250.0, 50.0}; //[mm]

SLCD lcd = SLCD(numRows, numCols);
MegaServo Servos[NBR_SERVOS];

void setup()
{
  setJoystickPins();
  Servos[0].attach(BLS452_PIN,BLS452_MIN,BLS452_MAX);
  Servos[1].attach(S9157_PIN,S9157_MIN,S9157_MAX);
  Servos[2].attach(BLS551_PIN,BLS551_MIN,BLS551_MAX);
  Servos[3].attach(S3156_PIN,S3156_MIN,S3156_MAX);
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
  if (manual) { // Manual mode
    if (Serial.available() > 0) {
      incoming = Serial.read();
    if (incoming == 'A') {
        manual = false;
        ruby   = false;
        lcd.clear();
        lcd.print("Automatic",0,3);
        lcd.print("mode",1,6);
        Serial.println("--- Automatic Mode ---");
    }
    else if (incoming == 'R')
      ruby = true;
    }
    else {
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
        lcd.print("Manual",0,5);
        lcd.print("mode",1,6);
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
        increase[i] = v/1000.0; // phi angle
      else if (i==1)
        increase[i] = -v/50.0;  // x coords  
      else  
        increase[i] = v/50.0;  // xyz coords
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
    //delay(20);
    int i;
  char j[10];
  int result = ik(coords, joints, l, minlimits, maxlimits, offset, length);
  if (result && !ruby) {
    Servos[0].writeMicroseconds(map(joints[0],0,BLS452_DEGREES,BLS452_MIN,BLS452_MAX));
    Servos[1].writeMicroseconds(map(joints[1],0,S9157_DEGREES,S9157_MIN,S9157_MAX));
    Servos[2].writeMicroseconds(map(joints[2],0,BLS551_DEGREES,BLS551_MIN,BLS551_MAX));
    Servos[3].writeMicroseconds(map(joints[3],0,S3156_DEGREES,S3156_MIN,S3156_MAX));    
    //for (int i = 0; i < 4; i++) {
    //  Serial.print(joints[i]);
    //  Serial.println("");      
    //}
    //Serial.println(" ");
  }
  else if (!ruby){
    lcd.clear();
    lcd.print("Out of range!",0,1);
//    Serial.println("OoR");
    for (i = 0; i < length; i++)
      coords[i] -= increase[i];
    delay(100);
  }
  }
  }
  else { // Automatic mode
    if (Serial.available() > 0) {
      incoming = Serial.read();
      Serial.print("nbyte = ");
      Serial.print(nbyte);
      Serial.print(" - ");
      Serial.println(incoming);
      if (incoming == 'M' && nbyte == 0) {
        manual = true;
        lcd.clear();
        lcd.print("Manual",0,5);
        lcd.print("mode",1,6);
        Serial.println("--- Manual Mode ---");        
      }
      else {  
        boolean servo;
        //Serial.print(float(incoming));
        //Serial.print(" ");
        nbyte++;
        switch (nbyte%2) {
          case 1: if (incoming >= 100.0)
                    joints[int(nbyte-1)/2] = (incoming-100.0)*-256.0;
                  else
                    joints[int(nbyte-1)/2] = incoming*256.0; break;  
          case 0: if (joints[int(nbyte-1)/2] < 0)
                    joints[int(nbyte-1)/2] -= incoming;
                  else
                    joints[int(nbyte-1)/2] += incoming; break;  
        }
        if (nbyte == 8) {
          //Serial.print("=> ");
          //for (int i = 0; i < 4; i++) {
          //  Serial.print(joints[i]);
          //  Serial.print(" ");      
          //}
          //Serial.println(" ");
          nbyte = 0;
          float joint;
          joint = BLS452_DEGREES - joints[0]/100.0 + offset[0]*180/PI;
          //Serial.print(joint);
          //Serial.print(" ");
          Servos[0].writeMicroseconds(map(joint,0,BLS452_DEGREES,BLS452_MIN,BLS452_MAX));
          joint = S9157_DEGREES  - joints[1]/100.0 + offset[1]*180/PI;
          //Serial.print(joint);
          //Serial.print(" ");
    	  Servos[1].writeMicroseconds(map(joint,0,S9157_DEGREES,S9157_MIN,S9157_MAX));
          joint = BLS551_DEGREES  - joints[2]/100.0 + offset[2]*180/PI;
          //Serial.print(joint);
          //Serial.print(" ");
    	  Servos[2].writeMicroseconds(map(joint,0,BLS551_DEGREES,BLS551_MIN,BLS551_MAX));
          joint = S3156_DEGREES  - joints[3]/100.0 + offset[3]*180/PI;    	  
          //Serial.print(joint);
          //Serial.print(" ");
    	  Servos[3].writeMicroseconds(map(joint,0,S3156_DEGREES,S3156_MIN,S3156_MAX));
          //Serial.println(" ");
        }
      }
    }
  }
  delay(10);
}

