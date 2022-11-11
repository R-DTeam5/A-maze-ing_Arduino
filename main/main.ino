//---- Libraries -----//

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <Scheduler.h>


//----- Pinout -----//

#define VIBRATIONPIN   A0     // pin 4 (only DAC pin)
#define KNOCKSENSORPIN A1     // pin 5
// pin 8: I2C SDA for LEDS
// pin 9: I2C SCL for LEDS
// pin 16: TX Used for UART with bluetooth module
// pin 17: RX Used for UART with bluetooth module


//----- Constants ----//

#define KNOCKSENSOR_THRESHOLD 100     // the value of the knocksensor above which the input is considered a valid knock (needs to be calibrated)
#define VIBRATION_DURATION 500        // duration of the vibration in ms
#define I2C_ADDRESS 0x3E              // the address of the first address of the I2C module (sx1509) (second address: 0x3F, third: 0x70, fourth: 0x71)


//----- Variables -----//

bool KnockSensorHit = false;    // true when the knocksensor input gets over knockThreshold, needs to be set to false when read by ble
PinStatus buildInLED = LOW;
int delay_test = 100;


//----- Main Functions -----//

void setup() 
{
  // Serial.begin(9600);     // To connect the arduino using a wire
  Serial1.begin(9600);    // UART that uses pins TX and RX to communicate with the bluetooth module

  // lightsInit();
  IMUInit();
  Scheduler.startLoop(_IMU);
}

void loop() 
{
  // if (Serial.available() > 0) serialIn(Serial1.readStringUntil('\n'));     // Read the incomming data when the arduino is connected using a wire
  if (Serial1.available() > 0) serialIn(Serial1.readStringUntil('\n'));   // Read the incomming data when the arduino is connected using a wire

  // knockSensor();      // These will probably become interrupts
  // vibration();
  // lights(0XFF);

  buildInLED = buildInLED ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, buildInLED);
  delay(1000);
  yield();
}


//----- PC_IO ----//

void serialIn(String inString) 
{
  switch (inString.charAt(0)) 
  {
    case '1':
      vibration();
      break;

    case 'd':
      inString.remove(0,2); // remove the "d,"
      delay_test = inString.toInt();
      if(delay_test == 0) delay_test = 10;
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
void _IMU()  // https://docs.arduino.cc/tutorials/nano-33-ble/imu-accelerometer
{
  float acceleroX = 0, acceleroY = 0, acceleroZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;
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

  // if(Serial) Serial.println(IMUData);
  if(Serial1) Serial1.println(IMUData);

  for(int i = 0; i < delay_test; i++) delay(1);
  //delay(100);
  yield();
}

void knockSensor()  // just plain ADC
{
  int knockValue = analogRead(KNOCKSENSORPIN);
  KnockSensorHit = knockValue >= KNOCKSENSOR_THRESHOLD;
  // Serial.println(knockValue);
}


//----- Outputs -----//

void lightsInit() 
{
  Wire.begin();  // join i2c bus
}
void lights(int data)  // sx1509 io expander using I2C: https://docs.arduino.cc/learn/communication/wire or https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(data);
  Wire.endTransmission();
}

void vibration()  // just plain DAC
{
  analogWrite(VIBRATIONPIN, 255);
  delay(VIBRATION_DURATION);
  analogWrite(VIBRATIONPIN, 0);
}