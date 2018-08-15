  
#include "LowPower.h"
/* Photocell sensor sketch for contorlling chook gate. 
 
Connect one end of the photocell to 5V, the other end to Analog 0.
Then connect one end of a 470 Ohm resistor from Analog 0 to ground 
Connect LED from pin 11 through a resistor (eg 1kOhm) to ground 
For more information see http://learn.adafruit.com/photocells */

#define debug
//#define test
//#define sleeping
#define production

/*********  State machine!  *************
 *   STATE_NIGHT => STATE_OPENING
 *   STATE_OPENING => STATE_DAY || STATE_ERROR
 *   STATE_DAY => STATE_CLOSING
 *   STATE_CLOSING => STATE_NIGHT || STATE_ERROR
 *   STATE_UNKNOWN => ANY STATE
 */
const int  STATE_NIGHT = 0;
const int STATE_DAY = 1;
const int STATE_CLOSING = 2;
const int STATE_OPENING = 3;
const int STATE_UNKNOWN = 4;  //needed for startup
const int STATE_ERROR = 5;
int state = 4;

//Outputs
//#define ENA 6 //ONLY NEED THIS IF SPPED IS VARIED
#define IN1 11
#define IN2 12
#define LEDpin 11  // connect Red LED to pin 11 (PWM pin)

//inputs
#define PhotocellPin 4     // the cell and 10K pulldown are connected to a4
#define CLOSED 5
#define OPEN A5

//variables
int photocellReading;     // the analog reading from the sensor divider
int LEDbrightness;        // 
const int CLOSE_THRESH = 10;
const int OPEN_THRESH = 15;   //Some hysteresis required
unsigned long MOTOR_TIME = 6000; //6sec to open/close
unsigned long motor_timer;
bool motor_action = false;

void setup(void) {
  // We'll send debugging information via the Serial monitor
  Serial.begin(115200);   
  pinMode(OPEN, INPUT_PULLUP);
  pinMode(CLOSED, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}



// https://tronixlabs.com.au/news/tutorial-l298n-dual-motor-controller-module-2a-and-arduino/
void closeDoor(){
  Serial.println("closing");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
}

void openDoor(){
    Serial.println("opening");

    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
}
 
void loop() {
  closeDoor();
  delay(10000);
  openDoor();
}
