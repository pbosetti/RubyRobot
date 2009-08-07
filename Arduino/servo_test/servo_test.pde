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
 
// Serial data rate
#define BAUD 9600
 
// Constants
unsigned int const PERIOD = 5; // Period in ms
unsigned int const LOOP_DELAY = 100; // Main loop delay
unsigned int const PIN_SERVO = 30;
unsigned int const PIN_CAMERA = 13;
 
MegaServo Servo;
 
// Globals
bool running = false;
unsigned long start;
unsigned int iang = 20;
unsigned int fang = 160;
unsigned int ang  = 0;
float time = 0;
float t = 0;
float m = 0;
 
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
    Serial.println("--END--");
    running = false;
  }
}
 
void toggle() {
  if (running)
  {
    MsTimer2::stop();
    digitalWrite(PIN_CAMERA, LOW);    
    running = false;
  }
  else
  {
    digitalWrite(PIN_CAMERA, HIGH);
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
  Servo.attach(PIN_SERVO,800,2200);
  pinMode(PIN_CAMERA, OUTPUT);  // collegamento alla smartcamera
  
  MsTimer2::set(PERIOD, pulse);
 
  Serial.println("Ready");
}
 
void loop() {
  static unsigned int v = 0;
  char ch;
  if (Serial.available()) {
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
