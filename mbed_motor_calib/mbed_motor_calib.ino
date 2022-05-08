#include "mbed.h"
#include "rtos.h"

using namespace mbed;
using namespace rtos;

char data;
float percent = 0;
int stage = 0;
float mot1DutyCycle, mot2DutyCycle, mot3DutyCycle, mot4DutyCycle;
float percent1, percent2, percent3, percent4;
bool go = false;

Thread threadTest;
Thread threadMultishot;

PwmOut mot1(digitalPinToPinName(6));
PwmOut mot2(digitalPinToPinName(8));
PwmOut mot3(digitalPinToPinName(10));
PwmOut mot4(digitalPinToPinName(12));

void setup()
{
  Serial.begin(115200);
  
  mot1.period_us(1000);
  mot2.period_us(1000);
  mot3.period_us(1000);
  mot4.period_us(1000);

  threadMultishot.start(multishot);
}

void loop()
{
  if(Serial.available()){
    data = Serial.read();

    switch(data){
      case 48:
        percent1 = percent2 = percent3 = percent4 = 0;
        go = true;
        Serial.println("Sending minimum throttle...");
        break;
      case 49:
        percent1 = percent2 = percent3 = percent4 = 100;
        go = true;
        Serial.println("Sending maximum throttle...");
        break;
      case 50:
        Serial.println("Running test!");
        stage=0;
        threadTest.start(test);
        break;
       default:
        break;
    }
    
  }
}

void test(){
  while(stage != 2){
    if(stage == 0){
      if(percent < 10){
        percent1 = percent2 = percent3 = percent4 = percent;
        go = true;
        ThisThread::sleep_for(200);
        percent++;
      } else {
        stage = 1;
      }
    } else if (stage == 1) {
      if(percent > 0){
        percent1 = percent2 = percent3 = percent4 = percent;
        go = true;
        ThisThread::sleep_for(200);
        percent--;
      } else {
        stage = 2;
      }
    }
  }
}

void multishot(){
  while(1){
    if(go == true){
      mot1DutyCycle = (percent1/100*12.5+12.5)/1000;
      mot2DutyCycle = (percent2/100*12.5+12.5)/1000;
      mot3DutyCycle = (percent3/100*12.5+12.5)/1000;
      mot4DutyCycle = (percent4/100*12.5+12.5)/1000;
      mot1.write(mot1DutyCycle);
      mot2.write(mot2DutyCycle);
      mot3.write(mot3DutyCycle);
      mot4.write(mot4DutyCycle);
      
      go = false;
    }
  }
}
