/***************************************************************************
  This is an example for the Adafruit SensorLab library
  It will look for a supported gyroscope and collect
  rad/s data for a few seconds to calcualte the zero rate
  calibration offsets
  
  Written by Limor Fried for Adafruit Industries.
 ***************************************************************************/

#include <Arduino_LSM9DS1.h>

#define NUMBER_SAMPLES 500

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

void setup(void) {
  Serial.begin(115200);
  calib();
}

void calib(){
  //Serial.println(F("Sensor Lab - Gyroscope Calibration!"));

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  delay(3000);

  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
  }
  
  min_x = max_x = x;
  min_y = max_y = y;
  min_z = max_z = z;
  delay(10);

  for (uint16_t sample = 0; sample < NUMBER_SAMPLES; sample++) {
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
    }
    
    Serial.print(x-0.27); Serial.print(",");
    Serial.print(y+0.225); Serial.print(",");
    Serial.println(z+0.34);

    min_x = min(min_x, x - 0.45);
    min_y = min(min_y, y + 0.46);
    min_z = min(min_z, z + 0.58);
  
    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);
  
    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    mid_z = (max_z + min_z) / 2;

//    Serial.print(F(" Zero rate offset: ("));
//    Serial.print(mid_x); Serial.print(", ");
//    Serial.print(mid_y); Serial.print(", ");
//    Serial.print(mid_z); Serial.print(")");  
//  
//    Serial.print(F(" dps noise: ("));
//    Serial.print(max_x - min_x, 3); Serial.print(", ");
//    Serial.print(max_y - min_y, 3); Serial.print(", ");
//    Serial.print(max_z - min_z, 3); Serial.println(")");   
    delay(10);
  }
//  Serial.println(F("\n\nFinal zero rate offset in dps: "));
//  Serial.print(mid_x, 4); Serial.print(", ");
//  Serial.print(mid_y, 4); Serial.print(", ");
//  Serial.println(mid_z, 4);
}

void loop() {
  if(Serial.available()){
    Serial.read();
    calib();
  }
}
