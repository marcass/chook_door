
//connect data pin (34 works) to side of photoresistor that is pulled to gnd with 10k (or less resistor, 680Ohm seems to work well)
//connect other side of photoresitor to 3.3V
#define PhotocellPin 4   // the cell and 10K pulldown are connected to a0 (gpi36)


//variables
int photocellReading;     // the analog reading from the sensor divider

void setup() {
  // We'll send debugging information via the Serial monitor
  Serial.begin(115200);   
}
 
void loop(void) {
  photocellReading = analogRead(PhotocellPin);
  Serial.println(photocellReading);     // the raw analog reading 
  delay(1000); 
}
