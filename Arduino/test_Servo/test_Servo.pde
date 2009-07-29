#include <MegaServo.h>
#define NBR_SERVOS 2  // the number of servos, up to 48 for Mega, 12 for other boards
#define FIRST_SERVO_PIN 2

int incoming = 0;	// for incoming serial data
int nbyte = 0;
int servoNumber = 0;
bool writeServo = false;
bool readServo  = false;
byte firstbyte;
byte secondbyte;
int value;

MegaServo Servos[NBR_SERVOS];

void setup() {
	for( int i = 0; i < NBR_SERVOS; i++)
          Servos[i].attach( FIRST_SERVO_PIN + i, 800, 2200);
        Serial.begin(9600);	// opens serial port, sets data rate to 9600 bps
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
                     if (writeServo) { 
                     value = map(firstbyte*127+secondbyte,0,18000,1000,2000);
                     //Serial.print("Value ");
                     //Serial.print(value,DEC);
                     //Serial.print(" write on servo number ");
                     //Serial.println(servoNumber,DEC);
                     Servos[servoNumber].writeMicroseconds(value);
                     writeServo = false; 
                     nbyte = 0;
                     value = 0;}
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
