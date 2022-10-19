//----- Pinout -----//
const int knockSensorPin = 5;
const int vibrationPin = 4; // only DAC pin

//----- Constants ----//
const int knockSensorThreshold = 100;
int vibrationDuration = 1500; // how long the tangible vibrates in ms

//----- Variables -----//
bool KnockSensorHit = false;  // true when the knocksensor input gets over knockThreshold, needs to be set to false when read by ble
int vibrationIntensity = 0;

void setup() {
  testOutputInit();
}

void loop() {
  testOutput();
  //knockSensor();
  //vibration();
}

//----- Inputs -----//

void accelerometerInit() {

}
void accelerometer() { // https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer

}

void knockSensor() { // just plain ADC
  if(analogRead(knockSensorPin) >= knockThreshold) {
    knockSensorHit = true;
  }
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
void ble() { // https://docs.arduino.cc/tutorials/nano-33-ble-sense/ble-device-to-device

}
