#include <SLCD.h>
#include <MegaServo.h>

#define NBR_SERVOS 2  // the number of servos, up to 48 for Mega, 12 for other boards
#define FIRST_SERVO_PIN 30

#define FIRST_STEPPER_PIN 5
#define FIRST_DIRSTEP_PIN 6
#define NBR_STEPPER 1



int incoming = 0;	// for incoming  data
int nbyte = 0;
int servoNumber = 0;
bool writeServo = false;
bool readServo  = false;
bool writeStepper = false;
byte firstbyte;
byte secondbyte;
int value;
char* str;

// LCD

int numRows = 2;
int numCols = 16;

SLCD lcd = SLCD(numRows, numCols);

// Stepper

int dirpin[NBR_STEPPER];
int steppin[NBR_STEPPER];

MegaServo Servos[NBR_SERVOS];

void setup() {
  for( int i = 0; i < NBR_SERVOS; i++)
    Servos[i].attach( FIRST_SERVO_PIN + i);
  for( int i = 0; i < NBR_STEPPER; i++) {
    dirpin[i]  = FIRST_DIRSTEP_PIN + i;
    steppin[i] = FIRST_STEPPER_PIN + i;
    pinMode(dirpin[i], OUTPUT);
    pinMode(steppin[i], OUTPUT);          
  }  
  .begin(9600);	// opens  port, sets data rate to 9600 bps
  lcd.init();
  lcd.clear();
  lcd.print("J1:", 0, 0);
  lcd.print("J3:", 0, 8);
  lcd.print("J2:", 1, 0);
  lcd.print("J4:", 1, 8);

}

void loop() {

  if (.available() > 0) {
    incoming = .read();
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
      if (writeServo ) { 
        value = map(firstbyte*127+secondbyte,0,18000,600,2400);
        //.print("Value ");
        //.print(value,DEC);
        //.print(" write on servo number ");
        //.println(servoNumber,DEC);
        Servos[servoNumber].writeMicroseconds(value);
        //textWrite(servoNumber,0,"Servo  : ",true);
        //str = (char*)map(value,600,2400,0,180);
        //lcd.print(str,1-servoNumber%2,4*(3*servoNumber/3));
        writeServo = false; 
        nbyte = 0;
        value = 0;
      }
      break;
    default: 
      .println("Too many char!");
    }

    if (readServo) {
      .print("Value ");
      value = Servos[servoNumber].readMicroseconds();
      Serial.println(map(value,1000,2000,0,18000));
      //Serial.print(" read on servo number ");
      //Serial.println(servoNumber,DEC);
      readServo  = false;
      nbyte = 0;
      value = 0;
    } 
  }      
}

