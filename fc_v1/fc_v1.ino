#include <Arduino_LSM9DS1.h>

class MyIMU{
  
  public:
    
    MyIMU(){
      
    }

    void setup(){
      if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
      }
    }

    int getGyroSampleRate(){
      return IMU.gyroscopeSampleRate();
    }

    int readGyro(){
      
      if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(this->x, this->y, this->z);
        return 1;
      } else {
        return 0;
      }
    }

    float getX(){
      return this->x;
    }

    float getY(){
      return this->y;
    }

    float getZ(){
      return this->z;
    }

  private:

    float x, y, z;
    
};

MyIMU imu;

void setup(){
  Serial.begin(9600);
  imu.setup();
  Serial.println(imu.getGyroSampleRate());
}

void loop(){
  int success = imu.readGyro();
  Serial.println(imu.getX());
}
