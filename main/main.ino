//---- Libraries -----//

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <Scheduler.h>
#include <ArduinoBLE.h>


//----- Pinout -----//

const int knockSensorPin = A1;  // A1
const int vibrationPin = A0;    // DAC0 (only DAC pin)
//pin 8: I2C SDA
//pin 9: I2C SCL


//----- Constants ----//

const int knockSensorThreshold = 100;   // the value of the knocksensor above which the input is considered a valid knock (needs to be calibrated)
const int vibrationDuration = 500;      // duration of the vibration in ms
const int I2CAddress = 0x3E;            // the address of the first address of the I2C module (sx1509) (second address: 0x3F, third: 0x70, fourth: 0x71)

BLEService accService("a00f1c3c-5c43-11ed-9b6a-0242ac120002"); // create service with random UUID derived from https://www.uuidgenerator.net/version1
BLEByteCharacteristic accXCharacteristic("ea828f0a-5c44-11ed-9b6a-0242ac120002", BLERead | BLENotify);
BLEByteCharacteristic accYCharacteristic("ea9093f8-5c57-11ed-9b6a-0242ac120002", BLERead | BLENotify);
BLEByteCharacteristic accZCharacteristic("ef2e9176-5c57-11ed-9b6a-0242ac120002", BLERead | BLENotify);

//----- Variables -----//

bool KnockSensorHit = false;    // true when the knocksensor input gets over knockThreshold, needs to be set to false when read by ble
int vibrationIntensity = 255;   // value between 0 and 255: 0 = no vibration, 255 = full vibration (needs to be calibrated)
PinStatus buildInLED = LOW;


//----- Main Functions -----//

void setup() 
{
  //serialInit();     // To be able to test with a wire

  accelerometerInit();
  //lightsInit();
  bleInit();
  Scheduler.startLoop(accelerometerBLELoop);
}

void loop() 
{
  //accelerometerWire();      // To be able to test with a wire
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

void accelerometerInit() 
{
  while (!IMU.begin())  // try to start the accelerometer, if it does not connect try again in 100ms
  {
    delay(100);
  }
}
void accelerometerWire()  // https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer
{
  float acceleroX = 0, acceleroY = 0, acceleroZ = 0;
  if (IMU.accelerationAvailable()) IMU.readAcceleration(acceleroX, acceleroY, acceleroZ);

  String acceleroData = "";
  acceleroData.concat(acceleroX);
  acceleroData.concat(",");
  acceleroData.concat(acceleroY);
  acceleroData.concat(",");
  acceleroData.concat(acceleroZ);

  Serial.println(acceleroData);
}
void accelerometerBLELoop()
{
  float acceleroX = 0, acceleroY = 0, acceleroZ = 0;
  if (IMU.accelerationAvailable()) IMU.readAcceleration(acceleroX, acceleroY, acceleroZ);
  if(accXCharacteristic.value() != acceleroX) accXCharacteristic.writeValue(acceleroX);
  if(accYCharacteristic.value() != acceleroY) accYCharacteristic.writeValue(acceleroY);
  if(accZCharacteristic.value() != acceleroZ) accZCharacteristic.writeValue(acceleroZ);

  delay(10);
  yield();
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

//----- Bluetooth -----//

void bleInit() 
{
  while (!BLE.begin())
  {
    delay(100);
  }

  BLE.setLocalName("A-Maze-Ing");
  BLE.setAdvertisedService(accService);

  accService.addCharacteristic(accXCharacteristic);
  accService.addCharacteristic(accYCharacteristic);
  accService.addCharacteristic(accZCharacteristic);

  BLE.addService(accService);

  accXCharacteristic.writeValue(0);
  accYCharacteristic.writeValue(0);
  accZCharacteristic.writeValue(0);

  BLE.advertise();
}