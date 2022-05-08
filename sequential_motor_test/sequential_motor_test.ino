#include <Servo.h>

#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

Servo motA, motB, motC, motD;
char data;

int maxThrottle = 1200;

void setup() {
    Serial.begin(9600);
    
    motA.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(8, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    
    displayInstructions();
}

void loop() {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      motA.writeMicroseconds(MIN_PULSE_LENGTH);
                      motB.writeMicroseconds(MIN_PULSE_LENGTH);
                      motC.writeMicroseconds(MIN_PULSE_LENGTH);
                      motD.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      motA.writeMicroseconds(MAX_PULSE_LENGTH);
                      motB.writeMicroseconds(MAX_PULSE_LENGTH);
                      motC.writeMicroseconds(MAX_PULSE_LENGTH);
                      motD.writeMicroseconds(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;
        }
    }
    

}

void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= maxThrottle; i += 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(MIN_PULSE_LENGTH);
        motC.writeMicroseconds(MIN_PULSE_LENGTH);
        motD.writeMicroseconds(MIN_PULSE_LENGTH);
        
        delay(200);
    }

    for (int i = maxThrottle; i >= MIN_PULSE_LENGTH; i -= 5) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(MIN_PULSE_LENGTH);
        motC.writeMicroseconds(MIN_PULSE_LENGTH);
        motD.writeMicroseconds(MIN_PULSE_LENGTH);
        
        delay(200);
    }

    Serial.println("STOP");
    motA.writeMicroseconds(MIN_PULSE_LENGTH);
    motB.writeMicroseconds(MIN_PULSE_LENGTH);
    motC.writeMicroseconds(MIN_PULSE_LENGTH);
    motD.writeMicroseconds(MIN_PULSE_LENGTH);
}

void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}
