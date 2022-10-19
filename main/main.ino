//---- Library Includes -----//

#include <Arduino_LSM9DS1.h>


//----- Pinout -----//

const int knockSensorPin = 5;
const int vibrationPin = 4;   // only DAC pin


//----- Constants ----//

const int knockSensorThreshold = 100;
int vibrationDuration = 1500; // how long the tangible vibrates in ms


//----- Variables -----//

bool KnockSensorHit = false;  // true when the knocksensor input gets over knockThreshold, needs to be set to false when read by ble
int vibrationIntensity = 0;   // value between 0 and 255: 0 = now vibration, 255 = full vibration (needs to be calibrated)
int acceleroDegreesX = 0;     // value of the angle between -90° and 90°
int acceleroDegreesY = 0;
int acceleroDegreesZ = 0;


//----- Main Functions -----//

void setup() {
  testOutputInit();
  //accelerometerInit();
}

void loop() {
  testOutput();
  //accelerometer();
  //knockSensor();
  //vibration();
}


//----- Inputs -----//

void accelerometerInit() {
  while (!IMU.begin()) {  // try to start the accelerometer, if it does not connect try again in 100ms
    delay(100);
  }
}
void accelerometer() { // https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer
  float x, y, z;
  if (IMU.accelerationAvailable())  IMU.readAcceleration(x, y, z);
  x *= 100;
  y *= 100;
  z *= 100;

  acceleroDegreesX = map(x, -100, 97, -90, 90);
  acceleroDegreesY = map(y, -100, 97, -90, 90);
  acceleroDegreesZ = map(z, -100, 97, -90, 90);
}

void knockSensor() { // just plain ADC
  if(analogRead(knockSensorPin) >= knockSensorThreshold) KnockSensorHit = true;
}


//----- Outputs -----//

void testOutputInit() {
  pinMode(LED_BUILTIN, OUTPUT);
}
void testOutput() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

void lightsInit() {

}
void lights() { // sx1509 io expander using I2C: https://docs.arduino.cc/learn/communication/wire or https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

}

void vibration() { // just plain DAC
  analogWrite(vibrationPin, vibrationIntensity);
  delay(vibrationDuration);
  analogWrite(vibrationPin, 0);
}

void soundInit() {

}
void sound() { // Adafruit Zero I2S library: https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp/arduino-wiring-test-2

}

//----- Bluetooth -----//
void bleInit() {

}
void ble() { // https://assetstore.unity.com/packages/tools/input-management/arduino-bluetooth-plugin-98960#content

}
