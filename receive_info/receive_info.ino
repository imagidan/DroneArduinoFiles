#include <Servo.h>
#include <Arduino_LSM9DS1.h>

bool tune = false;
bool isReset = false;
bool isGo = false;
bool isConfig = false;
bool isOff = false;

float kp, ki, kd, kq, kj, kb;
float yaw, pitch, roll;
float imuYaw, imuPitch, imuRoll;
float yawPID, rollPID, pitchPID;

float limit_us = 1500;
float min_us = 1000;
float max_us = 2000;

float throttle = 1000;

class MyIMU{
  
  public:

    float x, y, z;
    float lastTime, nowTime;
    float interval = 1000;
    
    MyIMU(){}

    void init(){
      if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
      }
      this->lastTime = -1000;
    }

    int getGyroSampleRate(){
      return IMU.gyroscopeSampleRate();
    }

    void readGyro(){
      if(IMU.gyroscopeAvailable()){
        IMU.readGyroscope(this->x, this->y, this->z);
        imuPitch = x - 0.27;
        imuRoll = y + 0.225;
        imuYaw = z + 0.34;
      }
    }

    void onLoop(){
      this->nowTime = micros();
      if((this->nowTime - this->lastTime) >= this->interval){
        this->lastTime = this->nowTime;
        this->readGyro();
      }
    }
    
};

class Motors{
  public:
  
    float min_us = 1000;
    float max_us = 2000;
    float limit_us = 1500;
    float accel_time_ms = 2000;
    float interval_ms = 20;
    float maxThrottleBump = 5;
    float lastTime, nowTime;
    
    Servo motA, motB, motC, motD;
    float motADesThrottle, motBDesThrottle, motCDesThrottle, motDDesThrottle;
    float motACurThrottle, motBCurThrottle, motCCurThrottle, motDCurThrottle;
    
    Motors(){}
    
    void init(){
      this->motA.attach(6, this->min_us, this->max_us);
      this->motB.attach(8, this->min_us, this->max_us);
      this->motC.attach(10, this->min_us, this->max_us);
      this->motD.attach(12, this->min_us, this->max_us);
      this->motADesThrottle = this->motBDesThrottle = this->motCDesThrottle = this->motDDesThrottle = this->motACurThrottle = this->motBCurThrottle = this->motCCurThrottle = this->motDCurThrottle = this->min_us;
      this->interval_ms = this->accel_time_ms / ((this->limit_us - this->min_us) / this->maxThrottleBump);
    }

    void configure(){
      this->motA.writeMicroseconds(this->min_us);
      this->motB.writeMicroseconds(this->min_us);
      this->motC.writeMicroseconds(this->min_us);
      this->motD.writeMicroseconds(this->min_us);
    }

    float get_limit_us(){
      return this->limit_us;
    }

    float get_max_us(){
      return this->max_us;
    }

    float get_min_us(){
      return this->min_us;
    }

    void mix(float throttle, float yaw, float pitch, float roll){
      //Serial.println(String(throttle) + "\t" + String(yaw) + "\t" + String(pitch) + "\t" + String(roll));
      this->motADesThrottle = throttle + yaw - pitch - roll;
      this->motBDesThrottle = throttle - yaw - pitch + roll;
      this->motCDesThrottle = throttle + yaw + pitch + roll;
      this->motDDesThrottle = throttle - yaw + pitch - roll;
      this->limitThrottles();
    }

    void off(){
      this->motADesThrottle = min_us;
      this->motADesThrottle = min_us;
      this->motADesThrottle = min_us;
      this->motADesThrottle = min_us;
    }

    void changeCurThrottle(){
      if((this->motADesThrottle - this->motACurThrottle) > this->maxThrottleBump){
        this->motACurThrottle += this->maxThrottleBump;
        this->runMotor(this->motA, this->motACurThrottle);
      } else if((this->motADesThrottle - this->motACurThrottle) < -this->maxThrottleBump){
        this->motACurThrottle -= this->maxThrottleBump;
        this->runMotor(this->motA, this->motACurThrottle);
      } else if((this->motADesThrottle - this->motACurThrottle) != 0){
        this->motACurThrottle = this->motADesThrottle;
        this->runMotor(this->motA, this->motACurThrottle);
      }

      if((this->motBDesThrottle - this->motBCurThrottle) > this->maxThrottleBump){
        this->motBCurThrottle += this->maxThrottleBump;
        this->runMotor(this->motB, this->motBCurThrottle);
      } else if((this->motBDesThrottle - this->motBCurThrottle) < -this->maxThrottleBump){
        this->motBCurThrottle -= this->maxThrottleBump;
        this->runMotor(this->motB, this->motBCurThrottle);
      } else if((this->motBDesThrottle - this->motBCurThrottle) != 0){
        this->motBCurThrottle = this->motBDesThrottle;
        this->runMotor(this->motB, this->motBCurThrottle);
      }

      if((this->motCDesThrottle - this->motCCurThrottle) > this->maxThrottleBump){
        this->motCCurThrottle += this->maxThrottleBump;
        this->runMotor(this->motC, this->motCCurThrottle);
      } else if((this->motCDesThrottle - this->motCCurThrottle) < -this->maxThrottleBump){
        this->motCCurThrottle -= this->maxThrottleBump;
        this->runMotor(this->motC, this->motCCurThrottle);
      } else if((this->motCDesThrottle - this->motCCurThrottle) != 0){
        this->motCCurThrottle = this->motCDesThrottle;
        this->runMotor(this->motC, this->motCCurThrottle);
      }

      if((this->motDDesThrottle - this->motDCurThrottle) > this->maxThrottleBump){
        this->motDCurThrottle += this->maxThrottleBump;
        this->runMotor(this->motD, this->motDCurThrottle);
      } else if((this->motDDesThrottle - this->motDCurThrottle) < -this->maxThrottleBump){
        this->motDCurThrottle -= this->maxThrottleBump;
        this->runMotor(this->motD, this->motDCurThrottle);
      } else if((this->motDDesThrottle - this->motDCurThrottle) != 0){
        this->motDCurThrottle = this->motDDesThrottle;
        this->runMotor(this->motD, this->motDCurThrottle);
      }
    }

    float limit(float throttle){
      if(throttle > this->limit_us){
        return this->limit_us;
      } else if (throttle < this->min_us){
        return this->min_us;
      } else {
        return throttle;
      }
    }

    void limitThrottles(){
      this->motADesThrottle = this->limit(this->motADesThrottle);
      this->motBDesThrottle = this->limit(this->motBDesThrottle);
      this->motCDesThrottle = this->limit(this->motCDesThrottle);
      this->motDDesThrottle = this->limit(this->motDDesThrottle);
    }

    void runMotor(Servo mot, float throttle){
      mot.writeMicroseconds(throttle);
    }

    void onLoop(){
      this->nowTime = millis();
      if(isConfig == true){
        isConfig = false;
        this->configure();
      }
      if(isOff == true){
        isConfig = false;
        this->off();
      }
      if((this->nowTime - this->lastTime) >= this->interval_ms){
        this->lastTime = this->nowTime;
        this->changeCurThrottle();
        //Serial.println(String(this->motACurThrottle) + "\t" + String(this->motBCurThrottle) + "\t" + String(this->motCCurThrottle) + "\t" + String(this->motDCurThrottle));
      }
      this->mix(throttle, yawPID, pitchPID, rollPID);
    }
    
};

class Communication{
  public:
    String command;
    Motors motors;
    
    Communication(){
      
    }
    
    void init(Motors motors){
      Serial.begin(115200);
      this->motors = motors;
    }

    void onLoop(){
      if (Serial.available()) {
        this->command = Serial.readStringUntil(',');
        //Serial.println(throttle);
        if(this->command == "c"){
          isConfig = true;
        } else if(this->command == "g"){
          isGo = true;
        } else if(this->command == "s"){
          isGo = false;
          isOff = true;
        } else if(this->command == "p"){
          kp = Serial.readStringUntil(',').toFloat();
          tune = true;
        } else if(this->command == "q"){
          kq = Serial.readStringUntil(',').toFloat();
          tune = true;
        } else if(this->command == "i"){
          ki = Serial.readStringUntil(',').toFloat();
          tune = true;
        } else if(this->command == "j"){
          kj = Serial.readStringUntil(',').toFloat();
          tune = true;
        } else if(this->command == "d"){
          kd = Serial.readStringUntil(',').toFloat();
          tune = true;
        } else if(this->command == "b"){
          kb = Serial.readStringUntil(',').toFloat();
          tune = true;
        } else if(this->command == "t"){
          throttle = Serial.readStringUntil(',').toFloat();
        } else if(this->command == "y"){
          yaw = Serial.readStringUntil(',').toFloat();
        } else if(this->command == "f"){
          pitch = Serial.readStringUntil(',').toFloat();
        } else if(this->command == "r"){
          roll = Serial.readStringUntil(',').toFloat();
        }
      } //else{
        //Serial.println("a");
      //}
    }
    
};

class PID{
  
  public:

    PID(){}

    Motors motors;
    Communication comm;
    MyIMU imu;
    float errorPitch, errorRoll, errorYaw;
    float lastInputPitch, lastInputRoll, lastInputYaw;
    float errorPitchI, errorRollI, errorYawI;
    float inputPitchD, inputRollD, inputYawD;
    float interval = 0.01;
    float lastTime, nowTime;
    float myKp, myKi, myKd, myKq, myKj, myKb;
    float maxPID;
    
    void init(Motors motors, Communication comm, MyIMU imu){
      this->motors = motors;
      this->comm = comm;
      this->imu = imu;
      this->maxPID = limit_us - min_us;
    }

    void setInterval(float interval){
      if(interval > 0){
        float ratio = interval / this->interval;
        ki *= ratio;
        kd /= ratio;
        this->interval = interval;
      }
    }

    void setTunings(){
      this->myKq = kq;
      this->myKp = kp;
      this->myKj = kj * this->interval;
      this->myKi = ki * this->interval;
      this->myKb = kb / this->interval;
      this->myKd = kd / this->interval;
    }

    float limit(float errorI){
      if(errorI > maxPID){
        return maxPID;
      } else if (errorI < -maxPID){
        return -maxPID;
      } else {
        return errorI;
      }
    }
    
    void getErrors(){
      this->errorYaw = yaw - imuYaw;
      Serial.print(this->errorYaw);
      Serial.print("\t");
      this->errorPitch = pitch - imuPitch;
      Serial.print(this->errorPitch);
      Serial.print("\t");
      this->errorRoll = roll - imuRoll;
      Serial.println(this->errorRoll);
      this->errorPitchI += this->myKi * this->errorPitch;
      this->errorPitchI = this->limit(this->errorPitchI);
      this->errorRollI += this->myKi * this->errorRoll;
      this->errorRollI = this->limit(this->errorRollI);
      this->errorYawI += this->myKj * this->errorYaw;
      this->errorYawI = this->limit(this->errorYawI);
      this->inputPitchD = pitch - this->lastInputPitch;
      this->inputRollD = roll - this->lastInputRoll;
      this->inputYawD = yaw - this->lastInputYaw;
      this->lastInputPitch = pitch;
      this->lastInputRoll = roll;
      this->lastInputYaw = yaw;
    }

    void getPID(){
      yawPID = this->myKq * this->errorYaw + this->errorYawI - this->myKb * this->inputYawD;
      yawPID = this->limit(yawPID);
      pitchPID = this->myKp * this->errorPitch + this->errorPitchI - this->myKd * this->inputPitchD;
      pitchPID = this->limit(pitchPID);
      rollPID = this->myKp * this->errorRoll + this->errorRollI - this->myKd * this->inputRollD;
      rollPID = this->limit(rollPID);
    }

    void resetPID(){
      this->lastInputPitch = pitch;
      this->lastInputRoll = roll;
      this->lastInputYaw = yaw;
      this->errorRollI = 0;
      this->errorPitchI = 0;
      this->errorYawI = 0;
    }

    void onLoop(){
      if(tune == true){
        tune = false;
        this->setTunings();
        //Serial.println(this->myKq);
      }
      if(isReset == true){
        isReset = false;
        this->resetPID();
      }
      if(isGo == true){
        this->nowTime = millis();
        if((this->nowTime - this->lastTime) >= this->interval*1000){
          this->lastTime = this->nowTime;
          this->getErrors();
          this->getPID();
        }
      }
    } 
};

Motors motors;
Communication comm;
PID pid;
MyIMU imu;
//Thread motorThread, commThread, pidThread, imuThread;

void setup() {
  motors.init();
  imu.init();
  comm.init(motors);
  pid.init(motors, comm, imu);
}

void loop() {
  motors.onLoop();
  imu.onLoop();
  comm.onLoop();
  pid.onLoop();
}
