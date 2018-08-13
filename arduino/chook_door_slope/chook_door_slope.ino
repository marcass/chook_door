//To put to sleep
#include "LowPower.h"
//To help calculate slope
#include "FastRunningMedian.h"
FastRunningMedian<unsigned int,75, 200> lightMedian;
/* Photocell sensor sketch for contorlling chook gate. 
 
Connect one end of the photocell to 5V, the other end to Analog 0.
Then connect one end of a 470 Ohm resistor from Analog 0 to ground 
Connect LED from pin 11 through a resistor (eg 1kOhm) to ground 
For more information see http://learn.adafruit.com/photocells */

#define debug
//#define test
#define sleeping
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
//define slopes
const int SLOPE_SAME = 0;
const int SLOPE_INC = 1;
const int SLOPE_DEC = 2;

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
const int CLOSE_THRESH = 100;
const int OPEN_THRESH = 20;
unsigned long MOTOR_TIME = 6000; //6sec to open/close
unsigned long motor_timer;
bool motor_action = false;

#ifdef debug
 // declare an array for printing state
  char* myStates[]={"Night", "Day", "Closing", "Opening", "Unknown", "Error"};
  char* slopes[] = {"Same", "Increasing", "Decreasing"};
#endif

void setup(void) {
  // We'll send debugging information via the Serial monitor
  Serial.begin(115200);   
  pinMode(OPEN, INPUT_PULLUP);
  pinMode(CLOSED, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  #ifdef production
    if (digitalRead((CLOSED) == HIGH) and (digitalRead(OPEN) == LOW)){
      state = STATE_DAY;
    }
    else if (digitalRead((CLOSED) == LOW) and (digitalRead(OPEN) == HIGH)){
      state = STATE_NIGHT;
    }else{
      state = STATE_UNKNOWN;
    }
  #endif
  #ifdef debug
    int openS = digitalRead(OPEN);
    int closedS = digitalRead(CLOSED);
    Serial.print("Open sensor = ");
    Serial.print(openS);
    Serial.print("; Closed sensor = ");
    Serial.println(closedS);
    // get initial light reading at startup
    photocellReading = analogRead(PhotocellPin);
    // get door status
    if ((closedS == HIGH) and (openS == LOW)){
      state = STATE_DAY;
    }
    else if ((closedS == LOW) and (openS == HIGH)){
      state = STATE_NIGHT;
    }else{
      state = STATE_UNKNOWN;
    }
    Serial.print("Initial set state = ");
    Serial.println(myStates[state]);
  #endif
}

//int find_state(int light){
//  Serial.print("Passed light reading = ");
//  Serial.println(light);
//  if (light > OPEN_THRESH){
//    Serial.println("switching to opening");
//    state = STATE_OPENING;
//  }
//  else if (light < CLOSE_THRESH){
//    Serial.println("switching to closing");
//    state = STATE_CLOSING;
//  }else{
//    Serial.println("staying in unkown");    
//    state = STATE_UNKNOWN; //light intensity at hysterisi point or sensor failure
//  }
//  return state;
//}

void goToSleep(){
  #ifdef debug
    Serial.println("Going the fuck to sleep");
    delay(1000);
  #endif
  #ifdef sleeping
//    sleep
//    isInterrupted = 0;
//    attachInterrupt;
//    LowPower.powerDown(SLEEP_8S); // sleep for 8s
//    delay(75); // allows the arduino to fully wake up.
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
  #endif
}

// https://tronixlabs.com.au/news/tutorial-l298n-dual-motor-controller-module-2a-and-arduino/
void closeDoor(){
  //motor going for too long?
  if (millis() - motor_timer > MOTOR_TIME){
    state = STATE_ERROR;
  }
  if (digitalRead(CLOSED) == HIGH){
    //OR VICE VERSA
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }else{
    digitalWrite(IN1, LOW);
    if (digitalRead(OPEN) == HIGH){
      motor_action = false;    
      state = STATE_NIGHT;
    }else{
      //Sensors not working
      state = STATE_ERROR;
    }
  }
}

void openDoor(){
  //motor going for too long?
  if (millis() - motor_timer > MOTOR_TIME){
    state = STATE_ERROR;
  }
  if (digitalRead(OPEN) == HIGH){
    digitalWrite(IN2, HIGH);
    digitalWrite(IN1, LOW);
  }else{
    digitalWrite(IN2, LOW);
    if (digitalRead(CLOSED) == HIGH){
      motor_action = false;
      state = STATE_DAY;
    }else{
      //Sensors not working
      state = STATE_ERROR;
    }
  }
}

int slope(int light){
  int m = lightMedian.getMedian();
  if (m > light){
    return SLOPE_DEC;
  }
  else if (m < light){
    return SLOPE_DEC;
  }else{
    return SLOPE_SAME;
  }
}

void loop() {
  lightMedian.addValue(analogRead(PhotocellPin));
  int thisSlope = slope(analogRead(PhotocellPin));
  photocellReading = analogRead(PhotocellPin);
  #ifdef debug
    Serial.print("State = ");
    Serial.print(myStates[state]);
    Serial.print("; Slope is  = ");
    Serial.print(slopes[thisSlope]);
    Serial.print("; Analog reading = ");
    Serial.println(photocellReading);     // the raw analog reading 
    int openS = digitalRead(OPEN);
    int closedS = digitalRead(CLOSED);
    Serial.print("Open sensor = ");
    Serial.print(openS);
    Serial.print("; Closed sensor = ");
    Serial.println(closedS);
    delay(1000); 
  #endif
  #ifdef test 
    // LED gets brighter the darker it is at the sensor
    // that means we have to -invert- the reading from 0-1023 back to 1023-0
    photocellReading = 1023 - photocellReading;
    //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
    LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
    analogWrite(LEDpin, LEDbrightness);
    delay(100);
  #endif
  #ifdef production
    switch (state){
    case STATE_UNKNOWN:
//      state = find_state(photocellReading);
      if (thisSlope == SLOPE_DEC){
        state = STATE_DAY;
      }
      if (thisSlope == SLOPE_INC){
        state = STATE_NIGHT;
      }else{
        Serial.println("Still not sure what the state is");
      }
      break;
    case STATE_NIGHT:
      if (thisSlope == SLOPE_INC){
        if (photocellReading > OPEN_THRESH){
          if (!motor_action){
            motor_timer = millis();
          }
          motor_action = true;
          state = STATE_OPENING;
        }
      }else{
        goToSleep();
      }
      break;
    case STATE_DAY:
      if (thisSlope == SLOPE_DEC){    
        if (photocellReading < CLOSE_THRESH){
          if (!motor_action){
            motor_timer = millis();
          }
          motor_action = true;
          state = STATE_CLOSING;
        }
      }else{
        goToSleep();
      }
      break;
    case STATE_OPENING:
      openDoor();
      break;
    case STATE_CLOSING:
      closeDoor();
      break;
    case STATE_ERROR:
      Serial.println("AAAARRRGGHHH I'm in error");
  }
  #endif
}
