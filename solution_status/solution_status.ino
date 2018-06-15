/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/SerialEvent
*/

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

int greenLed = 7;
int yellowLed = 6;
int redLed = 5;
int flagLed = 4;  // blue led for indicating robot is at flag



void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(flagLed, OUTPUT);
  
}

void loop() {
  
  // print the string when a newline arrives:
  if (stringComplete) {
    Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }


  // Check for emergency stop signal from remote (32197-MI)

  
  
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // add it to the inputString:
    inputString += inChar;

    if (inputString == "fix") {
      digitalWrite(greenLed, HIGH);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, LOW);
      stringComplete = true;
    }
    else if (inputString == "float") {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, LOW);
      stringComplete = true;
    }
    else if (inputString == "single") {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, HIGH);
      stringComplete = true;
    }
    else if (inputString == "-") {
      digitalWrite(greenLed, HIGH);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, HIGH);
      stringComplete = true;
    }
    else if(inputString == "off") {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, LOW);
      digitalWrite(flagLed, LOW);
      stringComplete = true;
    }
    else if (inputString == "flag") {
      digitalWrite(flagLed, HIGH);
      stringComplete = true;
    }
    else if (inputString == "flagoff") {
      digitalWrite(flagLed, LOW);
      stringComplete = true;
    }
    

    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      Serial.println("Received return char from incoming message..");
      stringComplete = true;
    }
  }
}

