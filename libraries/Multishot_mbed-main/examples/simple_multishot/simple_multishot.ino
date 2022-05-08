#include <Multishot_mbed.h>

char data;

void setup()
{
  Serial.begin(115200);
  
	Multishot mot1(p6);
	Multishot mot2(p8);
	Multishot mot3(p10);
	Multishot mot4(p12);	
}

void loop()
{
		if(Serial.available()){
      data = Serial.read();

      //if User types 0 (48)
      if(data == 48){
        Serial.print("Running test in 3");
        delay(1000);
        Serial.print(" 2");
        delay(1000);
        Serial.println(" 1...");
        delay(1000);
        test();
      }
		}
}

void test(){
	for(i=0; i < 10; i++){
		mot1.setPercent(i);
		mot2.setPercent(i);
		mot3.setPercent(i);
		mot4.setPercent(i);
	}
	
	for(i=10; i > 0; i--){
		mot1.setPercent(i);
		mot2.setPercent(i);
		mot3.setPercent(i);
		mot4.setPercent(i);
	}
}
