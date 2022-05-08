# Multishot MBED

Multishot using MBED PwmOut.

## Description

This library allows you to use Multishot to communicate motor speeds to your ESC. PWM not synced and only for unidirectional use. Missing features might be implemented later. Period of 100 microseconds.

## Getting Started

```
Multishot motor(p8); //Instantiate a new Multishot motor on pin 8
motor.setPercent(50); //Set rotation speed to 50% of the max speed
```

## Credits

Made possible thanks to this [OneShot125 mbed repository](https://github.com/RoToRx88/OneShot125mbed) used as a reference. 
