int counter = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {

  if(counter>=42) counter = 0;
  
  // print out the state of the button:
  Serial.println(++counter);
  delay(1000);        // delay in between reads for stability
}
