#include <Servo.h>
#include <Arduino_LSM9DS1.h>

#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

int batteryPin = A0;
float voltage = 0;

Servo motA, motB, motC, motD;

int stabilize = 0;
float x, y, z;
float errorZ, errorYawI, inputYawD;
float current, past;
float lastInputYaw;
float realZ;
float q, j, b, p, i, d;
float myQ, myJ, myB, myP, myI, myD;
float yawPID;
float maxPID;
float interval = 0.01;
float maxThrottle = 1600;
float throttle = 1000;
float yaw;
float pitch;
float roll;
bool isReady = false;
bool once = false;
String command;
float yawPlate, lastInputYawPlate, errorYawPlateI, inputYawPlateD;
float maxDPS = 150;

void getCommands(){
  if (Serial.available()) {
    command = Serial.readStringUntil(',');
    if (command == "s"){
      Serial.readStringUntil(',');
      isReady = false;
      throttle = MIN_PULSE_LENGTH;
      yaw = pitch = roll = 0;
      lastInputYawPlate = yawPlate;
      errorYawPlateI = 0;
      lastInputYaw = yaw;
      errorYawI = 0;
      mix(throttle, yawPID, pitch, roll);
    } else if (command == "h"){
      Serial.readStringUntil(',');
      throttle = MIN_PULSE_LENGTH;
      yaw = pitch = roll = 0;
      mix(throttle, yawPID, pitch, roll);
    } else if (command == "g"){
      Serial.readStringUntil(',');
      isReady = true;
    } else if (command == "t"){
      float temp = Serial.readStringUntil(',').toFloat();
      if(isReady == true){
        throttle = temp;
        //Serial.println(throttle);
      }
    } else if (command == "y"){
      float temp = Serial.readStringUntil(',').toFloat();
      if(isReady == true){
        lastInputYawPlate = yawPlate;
        yawPlate = -temp;
      }
    } else if (command == "f"){
      float temp = Serial.readStringUntil(',').toFloat();
      if(isReady == true){
        pitch = temp;
      }
    } else if (command == "r"){
      float temp = Serial.readStringUntil(',').toFloat();
      if(isReady == true){
        roll = temp;
      }
    } else if (command == "m"){
      Serial.readStringUntil(',');
      throttle = MAX_PULSE_LENGTH;
      yaw = pitch = roll = 0;
    } else if (command == "q"){
      q = Serial.readStringUntil(',').toFloat();
      myQ = q;
    } else if (command == "p"){
      p = Serial.readStringUntil(',').toFloat();
      myP = p;
    } else if (command == "i"){
      i = Serial.readStringUntil(',').toFloat();
      myI = i * interval;
    } else if (command == "d"){
      d = Serial.readStringUntil(',').toFloat();
      myD = d / interval;
    } else if (command == "j"){
      j = Serial.readStringUntil(',').toFloat();
      myJ = j * interval;
    } else if (command == "b"){
      b = Serial.readStringUntil(',').toFloat();
      myB = b / interval;
    } else if (command == "w"){
      int temp = Serial.readStringUntil(',').toInt();
      if(isReady == true){
        stabilize = temp;
      }
    } else {
      Serial.readStringUntil(',');
    }
  }
}

void setup() {
    Serial.begin(115200);

    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    
    pinMode(batteryPin, INPUT);
    
    motA.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(8, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    maxPID = maxThrottle - MIN_PULSE_LENGTH;
}

//+ - - + ; - - + + ; - + - +;

void mix(float throttle, float yaw, float pitch, float roll){
  motA.writeMicroseconds(constrain(throttle + yaw - pitch - roll, MIN_PULSE_LENGTH, maxThrottle));
  motB.writeMicroseconds(constrain(throttle - yaw - pitch + roll, MIN_PULSE_LENGTH, maxThrottle));
  motC.writeMicroseconds(constrain(throttle - yaw + pitch - roll, MIN_PULSE_LENGTH, maxThrottle));
  motD.writeMicroseconds(constrain(throttle + yaw + pitch + roll, MIN_PULSE_LENGTH, maxThrottle));
}

void loop() {
  current = millis();
  if(current - past >= (interval*1000) && isReady == true){
    if (IMU.gyroscopeAvailable()) {
      if(stabilize == 1){
        errorYawPlateI += myI * yawPlate;
        errorYawPlateI = constrain(errorYawPlateI, -maxDPS, maxDPS);
        inputYawPlateD = yawPlate - lastInputYawPlate;
        yaw = myP * yawPlate + errorYawPlateI + myD * inputYawPlateD;
        Serial.print(yawPlate);
        Serial.print(",");
        Serial.println(yaw);
      } else {
        yaw = yawPlate;
        Serial.print(realZ);
        Serial.print(",");
        Serial.println(yaw);
        //lastInputYawPlate = yawPlate;
        errorYawPlateI = 0;
      }
      lastInputYaw = realZ;
      IMU.readGyroscope(x, y, z);
      realZ = 0.7*realZ + 0.3*(z+0.34);
      errorZ = -yaw - realZ;
      errorYawI += myJ * errorZ;
      errorYawI = constrain(errorYawI, -maxPID, maxPID);
      inputYawD = realZ - lastInputYaw;
      yawPID = myQ * errorZ + errorYawI - myB * inputYawD;
      voltage = analogRead(batteryPin)/1016.4*16.8;
      //Serial.print(yawPID);
      //Serial.print(",");
      //Serial.println(voltage);
      mix(throttle, yawPID, pitch, roll);
    }
    past = current;
  }
  
  getCommands();
  
}
