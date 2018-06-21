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



ros::NodeHandle nh;

void atFlagCallback(std_msgs::Bool flagMessage) {
  handleFlagLeds(flagMessage);
}

void emlidCallback(std_msgs::String emlidMessage) {
  handleLeds(emlidMessage);
}

ros::Subscriber<std_msgs::String> emlidSubscriber("/emlid_solution_status", emlidCallback);
ros::Subscriber<std_msgs::Bool> flagSubscriber("/at_flag", atFlagCallback);

std_msgs::String str_msg;
ros::Publisher rfStop("rf_stop", &str_msg);



void setup() {

  nh.initNode();
  nh.advertise(rfStop);  // publish to /rf_stop topic
  nh.subscribe(emlidSubscriber);
  nh.subscribe(flagSubscriber);

  // Pin modes for LEDs:
  pinMode(greenLed, OUTPUT);
  pinMode(yellowLed, OUTPUT);
  pinMode(redLed, OUTPUT);
  pinMode(flagLed, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
}



void loop() {
  
  // Check for emergency stop signal from remote (32197-MI)
  checkForEmergencyStop();
  
  nh.spinOnce();
  
}



void checkForEmergencyStop() {
  int rfButtonA = digitalRead(A2);  // Reads RF signal for emergency stop
  if (rfButtonA >= 1) {
    str_msg.data = stopMessage;
    rfStop.publish(&str_msg);
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

