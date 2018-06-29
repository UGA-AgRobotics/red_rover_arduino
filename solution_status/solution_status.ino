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

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

char stopMessage[4] = "stop";
char solutionStatusArray[10];
char incomingSolutionStatus[10];
char flagStatusArray[10];
char incomingFlagStatus[10];

int greenLed = 7;
int yellowLed = 6;
int redLed = 5;
int flagLed = 4;  // blue led for indicating robot is at flag

int startLed = 12;
int stopLed = 11;

std_msgs::Bool start_bool_msg;
std_msgs::Bool stop_bool_msg;
std_msgs::String str_msg;



ros::NodeHandle nh;

void atFlagCallback(std_msgs::Bool flagMessage) {
  handleFlagLeds(flagMessage);
}

void emlidCallback(std_msgs::String emlidMessage) {
  handleLeds(emlidMessage);
}

// Subscribers:
ros::Subscriber<std_msgs::String> emlidSubscriber("/emlid_solution_status", emlidCallback);
ros::Subscriber<std_msgs::Bool> flagSubscriber("/at_flag", atFlagCallback);

// Publishers:
ros::Publisher rfStop("/rf_stop", &stop_bool_msg);
ros::Publisher rfStart("/rf_start", &start_bool_msg);



void setup() {

  Serial.begin(9600);

  nh.initNode();
  nh.advertise(rfStop);  // publishes to /rf_stop topic
  nh.advertise(rfStart);  // publishes to /rf_start topic
  nh.subscribe(emlidSubscriber);
  nh.subscribe(flagSubscriber);

  // Pin modes for LEDs:
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(flagLed, OUTPUT);
  pinMode(startLed, OUTPUT);
  pinMode(stopLed, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
}



void loop() {
  
  // Check for emergency stop signal from remote (32197-MI)
  checkForEmergencyStop();
  
  nh.spinOnce();

  delay(100);
  
}



void handleStartRoutine() {

  stop_bool_msg.data = false;
  start_bool_msg.data = true;

  rfStop.publish(&stop_bool_msg);
  rfStart.publish(&start_bool_msg);

  digitalWrite(startLed, HIGH);
  digitalWrite(stopLed, LOW);

}



void handleStopRoutine() {
  // Publishes True to /rf_stop topic, and
  // lights stop LED.

  stop_bool_msg.data = true;
  start_bool_msg.data = false;

  rfStop.publish(&stop_bool_msg);
  rfStart.publish(&start_bool_msg);

  digitalWrite(startLed, LOW);
  digitalWrite(stopLed, HIGH);

}



void checkForEmergencyStop() {
  
  int rfButtonA = digitalRead(A2);  // Reads RF signal for emergency stop
  int rfButtonB = digitalRead(A0);  // Reads RF signal for starting back up

  if (rfButtonA >= 1) {
    handleStopRoutine();  // Pause the rover.
  }
  else if (rfButtonB >= 1) {
    handleStartRoutine();  // Start the rover.
  }
  else {
    handleStopRoutine();  // Pause the rover.
  }
  
}



void handleFlagLeds(std_msgs::Bool flagStatus) {

  if (flagStatus.data == true) {
    digitalWrite(flagLed, HIGH);
  }
  else {
    digitalWrite(flagLed, LOW);
  }
  
}



void handleLeds(std_msgs::String solutionStatus) {

    strcpy(incomingSolutionStatus, solutionStatus.data);
    
    strcpy(solutionStatusArray, "-");
    int ret = strcmp(incomingSolutionStatus, solutionStatusArray);
    if (ret == 0) {
      digitalWrite(greenLed, HIGH);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, HIGH);
      return;
    }

    strcpy(solutionStatusArray, "single");
    ret = strcmp(incomingSolutionStatus, solutionStatusArray);
    if (ret == 0) {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, HIGH);
      return;
    }

    strcpy(solutionStatusArray, "float");
    ret = strcmp(incomingSolutionStatus, solutionStatusArray);
    if (ret == 0) {
      digitalWrite(greenLed, LOW);
      digitalWrite(yellowLed, HIGH);
      digitalWrite(redLed, LOW);
      return;
    }

    strcpy(solutionStatusArray, "fix");
    ret = strcmp(incomingSolutionStatus, solutionStatusArray);
    if (ret == 0) {
      digitalWrite(greenLed, HIGH);
      digitalWrite(yellowLed, LOW);
      digitalWrite(redLed, LOW);
      return;
    }

}


