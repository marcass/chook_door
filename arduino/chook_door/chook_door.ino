/* Photocell simple testing sketch. 
 
Connect one end of the photocell to 5V, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground 
Connect LED from pin 11 through a resistor to ground 
For more information see http://learn.adafruit.com/photocells */

#define debug
//#define production

/*********  State machine!  *************
 *   STATE_NIGHT => STATE_OPENING
 *   STATE_OPENING => STATE_DAY
 *   STATE_DAY => STATE_CLOSING
 *   STATE_CLOSING => STATE_NIGHT
 *   STATE_UNKNOWN => ANY STATE
 */
const int  STATE_NIGHT = 0;
const int STATE_DAY = 1;
const int STATE_CLOSING = 2;
const int STATE_OPENING = 3;
const int STATE_UNKNOWN = 4;  //needed for startup
int state = 4;

//Outputs
#define CLOSE_REL 6
#define OPEN_REL 7

//inputs
#define PhotocellPin 0     // the cell and 10K pulldown are connected to a0
#define CLOSED 9
#define OPEN 10
#define LEDpin 11  // connect Red LED to pin 11 (PWM pin)

//variables
int photocellReading;     // the analog reading from the sensor divider
int LEDbrightness;        // 
const int CLOSE_THRESH = 10;
const int OPEN_THRESH = 25;   //Some hysteresis required

void setup(void) {
  // We'll send debugging information via the Serial monitor
  Serial.begin(115200);   
  pinMode(OPEN, INPUT_PULLUP);
  pinMode(CLOSED, INPUT_PULLUP);
  pinMode(CLOSE_REL, OUTPUT);
  pinMode(OPEN_REL, OUTPUT);
  digitalWrite(CLOSE_REL, LOW);
  digitalWrite(OPEN_REL, LOW);
  #ifdef production
    // get initial light reading at startup
    photocellReading = analogRead(PhotocellPin);
    // get door status
    if (digitalRead((CLOSED) == HIGH) and (digitalRead(OPEN) == LOW)){
      state = STATE_DAY;
    }
    elif (digitalRead((CLOSED) == LOW) and (digitalRead(OPEN) == HIGH)){
      state = STATE_NIGHT;
    }else{
      state = STATE_UNKNOWN;
    }
  #endif
}

int find_state(light){
  if (light > OPEN_THRESH){
    state = STATE_OPENING;
  }
  elif (light < CLOSE_THRESH){
    state = STATE_CLOSING;
  }else{
    state = STATE_UNKNOWN; //light intensity at hysterisi point or sensor failure
  }
  return state
}

void goToSleep(){
  #ifdef debug
    delay(1000);
  #endif
  #ifdef production
//    sleep
  #endif
}

void closeDoor(){
  if (digitalRead(CLOSED) == HIGH){
    digitalWrite(CLOSE_REL, HIGH);
  }else{
    digitalWrite(CLOSE_REL, LOW);
    state = STATE_NIGHT;
  }
}

void openDoor(){
  if (digitalRead(OPEN) == HIGH){
    digitalWrite(OPEN_REL, HIGH);
  }else{
    digitalWrite(OPEN_REL, LOW);
    state = STATE_DAY;
  }
}
}
 
void loop(void) {
  #ifdef debug
    photocellReading = analogRead(PhotocellPin);  
   
    Serial.print("Analog reading = ");
    Serial.println(photocellReading);     // the raw analog reading
   
    // LED gets brighter the darker it is at the sensor
    // that means we have to -invert- the reading from 0-1023 back to 1023-0
    photocellReading = 1023 - photocellReading;
    //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
    LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
    analogWrite(LEDpin, LEDbrightness);
   
    delay(100);
  #endif
  #ifdef producton
    photocellReading = analogRead(PhotocellPin);
    switch (state){
    case STATE_UNKNOWN:
      state = find_state(photocellReading);
      break;
    case STATE_NIGHT:
      if (photocellReading > OPEN_THRESH){
        state = STATE_OPENING;
      }else{
        goToSleep();
      }
    case STATE_DAY:
      if (photocellReading < CLOSED_THRESH){
        state = STATE_CLOSING;
      }else{
        goToSleep();
      }
    case STATE_OPENING;
      openDoor();
      break;
    case STATE_CLOSING;
      closeDoor();
      break;
  }
  #endif
}
