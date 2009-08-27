//------------------------------------------------------------
// This sketch calculate a ramp acceleration model and
// send the angular position to servo at fixed time
//------------------------------------------------------------
// Initial angle: XXXi [°]
// Final angle : XXXf [°]
// Time: XXXXt [ms]
// Start & stop: s
//------------------------------------------------------------

#include <MegaServo.h>
#include <MsTimer2.h>
#include <SLCD.h>

// LCD

static int numRows = 2;
static int numCols = 16;
SLCD lcd = SLCD(numRows, numCols);
 
// Serial data rate
#define BAUD 9600
 
// Constants
unsigned int const PERIOD = 5; // Period in ms
unsigned int const LOOP_DELAY = 20; // Main loop delay
unsigned int const PIN_SERVO = 30;
unsigned int const PIN_POT = 7;
 
MegaServo Servo;
 
// Globals
bool running = false;
bool finished = false;
unsigned long start;
unsigned int iang = 20;
unsigned int fang = 160;
unsigned int ang  = 0;
unsigned int potvalue;
float time = 0;
float t = 0;
float m = 0;
char str[5];
 
// Functions
 
void pulse() {
  t = (millis()-start);
  if (t <= time) {
    ang = m*t*t+iang;
    Servo.write(ang);  
  }
  else if (Servo.read() < fang) {
    Servo.write(fang);
  }
  else {
    toggle();
    running = false;
  }
}
 
void toggle() {
  if (running)
  {
    MsTimer2::stop();
    running = false;
    finished = true;    
  }
  else
  {
    start = millis();
    MsTimer2::start();
    running = true;
  }
}
 
void status()
{
  Serial.print("Initial angle: ");
  Serial.println(iang);
  Serial.print("Final angle: ");
  Serial.println(fang);
  Serial.print("Time: ");
  Serial.println(time);
}
 
 
// SETUP AND LOOP
void setup() {
  Serial.begin(BAUD);
  lcd.init();
  Servo.attach(PIN_SERVO,800,2200);
  
  MsTimer2::set(PERIOD, pulse);
  lcd.clear();
  lcd.print("- Servo test -",0,1);
  lcd.print("Value :",1,0);
  lcd.print("deg",1,13);
}
 
void loop() {
  static unsigned int v = 0;
  char ch;
  potvalue = map(analogRead(PIN_POT),0,1023,0,3200);
  if (running) {
    Serial.print(t);
    Serial.print(" ");    
    Serial.print(ang);
    Serial.print(" ");
    Serial.println(potvalue);
  } 
  if (potvalue < 10)
    sprintf(str,"00%i.%i",potvalue/10,potvalue%10);
  else if (potvalue <100)
    sprintf(str,"0%i.%i",potvalue/10,potvalue%10);
  else
      sprintf(str,"%i.%i",potvalue/10,potvalue%10);
  lcd.print(str, 1, 7);
  while (Serial.available()) {
    ch = Serial.read();
    // Serial command parsing:
    switch(ch) {
    case 's':
      toggle();
      break;
    case '?':
      status();
      break;
    case '0'...'9': // Accumulates values
      v = v * 10 + ch - '0';     
      break;
    case 'i':
      iang = v;
      v = 0;
      break;
    case 'f':
      fang = v;
      v = 0;
      break;
    case 't':
      time = v;
      v = 0;
      break;  
    case 'r':
      Servo.write(iang);
      break;
    }
    m = (float)(fang-iang)/(time*time);
  }
  delay(LOOP_DELAY);
} 
