#include <Servo.h>
#include <Arduino_LSM9DS1.h>

#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

int batteryPin = A0;
float voltage = 0;

Servo motA, motB, motC, motD;

bool stabilize = false;
float x, y, z;
float errorZ, errorYawI, inputYawD;
float current, past;
float lastInputYaw;
float yawBias = 1;
float realZ;
float q, j, b;
float qL, qR;
float myQ, myJ, myB;
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

float EarthMagnetStrength = 53.5;  //= µT

boolean magnetOK=false;
uint8_t magnetODRindex=8;  // (0..8)->{0.625,1.25,2.5,5.0,10,20,40,80,400}Hz
uint8_t magnetFSindex=0;   // 0=±400.0; 1=±800.0; 2=±1200.0 , 3=±1600.0  (µT) 

void getCommands(){
  if (Serial.available()) {
    command = Serial.readStringUntil(',');
    if (command == "s"){
      Serial.readStringUntil(',');
      isReady = false;
      throttle = MIN_PULSE_LENGTH;
      yaw = pitch = roll = 0;
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
      }
    } else if (command == "y"){
      float temp = Serial.readStringUntil(',').toFloat();
      if(isReady == true){
        yaw = temp;
      }
    } else if (command == "z"){
      float temp = Serial.readStringUntil(',').toFloat();
      if(isReady == true){
        yawBias = temp;
        qL = q * yawBias;
        qR = q / yawBias;
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
      qL = q * yawBias;
      qR = q / yawBias;
    } else if (command == "j"){
      j = Serial.readStringUntil(',').toFloat();
      myJ = j * interval;
    } else if (command == "b"){
      b = Serial.readStringUntil(',').toFloat();
      myB = b / interval;
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

    IMU.setMagnetODR(magnetODRindex);
    IMU.setMagnetFS(magnetFSindex);
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
  if(current - past >= (interval*10000) && isReady == true){
    if (IMU.gyroscopeAvailable()) {
      lastInputYaw = realZ;
      IMU.readGyroscope(x, y, z);
      realZ = 0.7*realZ + 0.3*(z+0.34);
      errorZ = -yaw - realZ;
      errorYawI += myJ * errorZ;
      errorYawI = constrain(errorYawI, -maxPID, maxPID);
      inputYawD = realZ - lastInputYaw;
      if(errorZ < 0){
        yawPID = myQ * errorZ + errorYawI - myB * inputYawD;
      } else {
        yawPID = myQ * errorZ + errorYawI - myB * inputYawD;
      }
      voltage = analogRead(batteryPin)/1016.4*16.8;
//      Serial.print(yawPID);
//      Serial.print(",");
//      Serial.print(voltage);
//      Serial.print(",");
//      Serial.print(realZ);
//      Serial.print(",");
//      Serial.println(yaw);
      calibrateMagnet();
      mix(throttle, yawPID, pitch, roll);
    }
    past = current;
  }
  
  getCommands();
  
}

void printParam(char txt[], float param[3])
{   for (int i= 0; i<=2 ; i++) 
    {  Serial.print(txt);Serial.print("[");
       Serial.print(i);Serial.print("] = "); 
       Serial.print(param[i],6);Serial.print(";");
    }
}

void printSetParam(char txt[], float param[3])
{   Serial.print(txt);Serial.print("(");
    Serial.print(param[0],6);Serial.print(", ");
    Serial.print(param[1],6);Serial.print(", ");
    Serial.print(param[2],6);Serial.print(");");
}

void calibrateMagnet()
{  float x, y, z, Xmin, Xmax, Ymin, Ymax, Zmin, Zmax  ;
   unsigned long count=0;
   IMU.setMagnetODR(8);
   raw_N_Magnet(10, Xmin, Ymin, Zmin);
   Xmax = Xmin; Ymax = Ymin; Zmax = Zmin;
   while (!Serial.available())
   {  raw_N_Magnet(10, x, y, z);
      Xmax = max (Xmax, x); Xmin = min (Xmin, x);
      Ymax = max (Ymax, y); Ymin = min (Ymin, y);
      Zmax = max (Zmax, z); Zmin = min (Zmin, z);
      count++;
      if ((count & 5)==0)
      { Serial.print(F("Xmin = "));Serial.print(Xmin); Serial.print(F("  Xmax = "));Serial.print(Xmax);
        Serial.print(F(" Ymin = "));Serial.print(Ymin); Serial.print(F("  Ymax = "));Serial.print(Ymax);
        Serial.print(F(" Zmin = "));Serial.print(Zmin); Serial.print(F("  Zmax = "));Serial.println(Zmax);
      }   
   } 
   while (Serial.available()) Serial.read();
   IMU.setMagnetOffset( (Xmax+Xmin)/2,(Ymax+Ymin)/2, (Zmax+Zmin)/2 );
   IMU.setMagnetSlope ( (2*EarthMagnetStrength)/(Xmax-Xmin),(2*EarthMagnetStrength)/(Ymax-Ymin),(2*EarthMagnetStrength)/(Zmax-Zmin));
   magnetOK=true;
   IMU.setMagnetODR(magnetODRindex);
} 


void raw_N_Magnet(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0;averZ =0;
     for (int i=1;i<=N;i++)
     {  while (!IMU.magnetAvailable());
        IMU.readRawMagnet(x, y, z);
        averX += x;    averY += y;     averZ += z;
        digitalWrite(LED_BUILTIN, (millis()/125)%2);
        if ((i%30)==0)Serial.print('.'); 
     } 
     averX /= N;    averY /= N;     averZ /= N;
     digitalWrite(LED_BUILTIN,0);
}
