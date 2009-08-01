#include <SLCD.h>
#include <MegaServo.h>

#define NBR_SERVOS 2  // the number of servos, up to 48 for Mega, 12 for other boards
#define FIRST_SERVO_PIN 3

#define FIRST_STEPPER_PIN 5
#define FIRST_DIRSTEP_PIN 10
#define NBR_STEPPER 1

int incoming = 0;	// for incoming serial data
int nbyte = 0;
int servoNumber = 0;
bool writeServo = false;
bool readServo  = false;
bool writeStepper = false;
byte firstbyte;
byte secondbyte;
int value;

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
          Servos[i].attach( FIRST_SERVO_PIN + i,800,2200);
        for( int i = 0; i < NBR_STEPPER; i++) {
          dirpin[i]  = FIRST_DIRSTEP_PIN + i;
          steppin[i] = FIRST_STEPPER_PIN + i;
          pinMode(dirpin[i], OUTPUT);
          pinMode(steppin[i], OUTPUT);          
        }  
        Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
     lcd.init();
}

void loop() {

	if (Serial.available() > 0) {
             incoming = Serial.read();
             nbyte++;
            
             switch (nbyte) {
             case 1:  if (incoming%119 == 0) {
                          writeServo  = true;
                          servoNumber = incoming / 119;
                      }
                      else {
                          readServo   = true;
                          servoNumber = incoming / 114;
                      }; break;                      
             case 2: firstbyte  = incoming; break;
             case 3: secondbyte = incoming;
                     if (writeServo && servoNumber < NBR_SERVOS ) { 
                     value = map(firstbyte*127+secondbyte,0,18000,800,2200);
                     //Serial.print("Value ");
                     //Serial.print(value,DEC);
                     //Serial.print(" write on servo number ");
                     //Serial.println(servoNumber,DEC);
                     Servos[servoNumber].writeMicroseconds(value);
                     writeServo = false; 
                     nbyte = 0;
                     value = 0;}
                     else {
                       // Sistemare la gestione dello stepper
                     }
                     //textWrite(0,3,"Servo moved",true);
                     break;
             default: Serial.println("Too many char!");
             }
                     
         if (readServo) {
           Serial.print("Value ");
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

void textWrite(int row, int col, char* text,bool clear) {
  if (clear)
  	lcd.clear();
  lcd.print(text, row, col);
}

