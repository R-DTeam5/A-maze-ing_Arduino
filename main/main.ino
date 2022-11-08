//---- Libraries -----//

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <Scheduler.h>


//----- Pinout -----//

const int vibrationPin = A0;    // pin 4 (only DAC pin)
const int knockSensorPin = A1;  // pin 5
// pin 8: I2C SDA
// pin 9: I2C SCL
// pin 16: TX Used for UART with bluetooth module
// pin 17: RX Used for UART with bluetooth module


//----- Constants ----//

const int knockSensorThreshold = 100;   // the value of the knocksensor above which the input is considered a valid knock (needs to be calibrated)
const int vibrationDuration = 500;      // duration of the vibration in ms
const int I2CAddress = 0x3E;            // the address of the first address of the I2C module (sx1509) (second address: 0x3F, third: 0x70, fourth: 0x71)

//----- Variables -----//

bool KnockSensorHit = false;    // true when the knocksensor input gets over knockThreshold, needs to be set to false when read by ble
int vibrationIntensity = 255;   // value between 0 and 255: 0 = no vibration, 255 = full vibration (needs to be calibrated)
PinStatus buildInLED = LOW;


//----- Main Functions -----//

void setup() 
{
  serialInit();     // To be able to test with a wire
  //Serial1.begin(9600);  // UART that uses pins TX and RX to communicate with the bluetooth module

  IMUInit();
  //lightsInit();
  Scheduler.startLoop(IMU);
}

void loop() 
{
  //if (Serial.available() > 0) serialIn();     // To be able to test with a wire

  //knockSensor();      // These will probably become interrupts
  //vibration();
  //lights(0XFF);

  buildInLED = buildInLED ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, buildInLED);
  delay(1000);
  yield();
}

//----- PC_IO ----//

void serialInit() 
{
  Serial.begin(9600);
  while (!Serial);
}

void serialIn() 
{
  char inChar = (char)Serial.read();

  switch (inChar) 
  {
    case '1':
      vibration();
      break;
  }
}


//----- Inputs -----//

void IMUInit() 
{
  while (!IMU.begin())  // try to start the accelerometer, if it does not connect try again in 100ms
  {
    delay(100);
  }
}
void IMU()  // https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer
{
  float acceleroX = 0, acceleroY = 0, acceleroZ = 0, gyroX = 0, gyroY = 0, gyroZ = 0;
  if (IMU.accelerationAvailable()) IMU.readAcceleration(acceleroX, acceleroY, acceleroZ);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gyroX, gyroY, gyroZ);  // sample rate is 119 Hz

  String IMUData = "";
  IMUData.concat(acceleroX);
  IMUData.concat(",");
  IMUData.concat(acceleroY);
  IMUData.concat(",");
  IMUData.concat(acceleroZ);
  IMUData.concat(",");
  IMUData.concat(gyroZ);

  Serial.println(IMUData);
  //Serial1.println(IMUData);
}

void knockSensor()  // just plain ADC
{
  int knockValue = analogRead(knockSensorPin);
  KnockSensorHit = knockValue >= knockSensorThreshold;
  //Serial.println(knockValue);
}


//----- Outputs -----//

void lightsInit() 
{
  Wire.begin();  // join i2c bus
}
void lights(int data)  // sx1509 io expander using I2C: https://docs.arduino.cc/learn/communication/wire or https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library
{
  Wire.beginTransmission(I2CAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void vibration()  // just plain DAC
{
  analogWrite(vibrationPin, vibrationIntensity);
  delay(vibrationDuration);
  analogWrite(vibrationPin, 0);
}