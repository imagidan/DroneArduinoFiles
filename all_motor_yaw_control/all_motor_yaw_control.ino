#include <Servo.h>

#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

Servo motA, motB, motC, motD;

int maxThrottle = 1600;
float throttle = 1000;
bool isReady = false;
unsigned long now, past, interval;

void getCommands(){
  if (Serial.available()) {
    String command = Serial.readStringUntil(',');
    Serial.println(command);
    if (command == "s"){
      Serial.readStringUntil('\n');
      isReady = false;
    } else if (command == "c"){
      Serial.readStringUntil('\n');
      sendThrottleAll(MIN_PULSE_LENGTH);
    } else if (command == "g"){
      Serial.readStringUntil('\n');
      isReady = true;
    } else if (command == "t"){
      float tempThrottle = (float) Serial.readStringUntil('\n').toInt();
      if(abs(tempThrottle) <= 600){
        throttle = tempThrottle;
        Serial.println(throttle);
      }
    } else if (command == "m"){
      Serial.readStringUntil('\n');
      sendThrottleAll(MAX_PULSE_LENGTH);
    }
  }
}

void setup() {
    Serial.begin(9600);
    
    motA.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(8, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    interval = 200;
}

void sendThrottleAll(float throttle){
  motA.writeMicroseconds(throttle);
  motB.writeMicroseconds(throttle);
  motC.writeMicroseconds(throttle);
  motD.writeMicroseconds(throttle);
}

void sendYawLeft(float throttle){
  motB.writeMicroseconds(throttle);
  motC.writeMicroseconds(throttle);
  motA.writeMicroseconds(MIN_PULSE_LENGTH);
  motD.writeMicroseconds(MIN_PULSE_LENGTH);
}

void sendYawRight(float throttle){
  motA.writeMicroseconds(throttle);
  motD.writeMicroseconds(throttle);
  motB.writeMicroseconds(MIN_PULSE_LENGTH);
  motC.writeMicroseconds(MIN_PULSE_LENGTH);
}

void loop() {
    getCommands();
    if(isReady == true){
      now = millis();
      if(now - past >= interval){
        past = now;
        int realThrottle = MIN_PULSE_LENGTH + abs(throttle);
        if(realThrottle >= MIN_PULSE_LENGTH && realThrottle <= maxThrottle){
          if(throttle >= 0){
            sendYawLeft(realThrottle);
          } else{
            sendYawRight(realThrottle);
          }
        }
      }
    } else if(isReady == false){
      sendThrottleAll(MIN_PULSE_LENGTH);
    }
}
