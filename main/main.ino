//---- Libraries -----//

#include <Arduino_LSM9DS1.h>
#include <Wire.h>


//----- Pinout -----//

const int knockSensorPin = 5; // A1
const int vibrationPin = 4;   // DAC0 (only DAC pin)
//pin 8: I2C SDA
//pin 9: I2C SCL


//----- Constants ----//

const int knockSensorThreshold = 100; // the value of the knocksensor above which the input is considered a valid knock (needs to be calibrated)
const int vibrationDuration = 1500; // duration of the vibration in ms
const int I2CAddress = 0x3E;  // the address of the first address of the I2C module (sx1509) (second address: 0x3F, third: 0x70, fourth: 0x71)


//----- Variables -----//

bool KnockSensorHit = false;  // true when the knocksensor input gets over knockThreshold, needs to be set to false when read by ble
int vibrationIntensity = 0;   // value between 0 and 255: 0 = now vibration, 255 = full vibration (needs to be calibrated)
float acceleroX = 0;
float acceleroY = 0;
float acceleroZ = 0;


//----- Main Functions -----//

void setup() {
  //PCInit();
  testOutputInit();
  accelerometerInit();
  //lightsInit();
}

void loop() {
  //testOutput();
  accelerometer();
  //knockSensor();
  //vibration();
  //lights(0XFF);
}

//----- PC_IO ----//

void PCInit() {       // To be able to print on the pc screen during testing (print with Serial.println())
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");
}


//----- Inputs -----//

void accelerometerInit() {
  while (!IMU.begin()) {  // try to start the accelerometer, if it does not connect try again in 100ms
    delay(100);
  }
}
void accelerometer() { // https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer
  if (IMU.accelerationAvailable()) IMU.readAcceleration(acceleroX, acceleroY, acceleroZ);

  Serial.print("x: ");
  Serial.print(acceleroX);
  Serial.print("\ty:");
  Serial.print(acceleroY);
  Serial.print("\tz: ");
  Serial.println(acceleroZ);
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
  Wire.begin(); // join i2c bus
}
void lights(int data) { // sx1509 io expander using I2C: https://docs.arduino.cc/learn/communication/wire or https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library
  Wire.beginTransmission(I2CAddress);
  Wire.write(data);
  Wire.endTransmission();
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
