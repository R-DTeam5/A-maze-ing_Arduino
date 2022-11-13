//---- Libraries -----//

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <Scheduler.h>


//----- Pinout -----//

#define VIBRATIONPIN   A0     // pin 4 (only DAC pin)
// pin 8: I2C SDA for LEDS
// pin 9: I2C SCL for LEDS
// pin 16: TX Used for UART with bluetooth module
// pin 17: RX Used for UART with bluetooth module
#define KNOCKPIN_FRONT  20    // D2
#define KNOCKPIN_BACK   21    // D3
#define KNOCKPIN_LEFT   22    // D4
#define KNOCKPIN_RIGHT  23    // D5
#define KNOCKPIN_TOP    24    // D6
#define KNOCKPIN_BOTTOM 25    // D7


//----- Constants ----//

#define VIBRATION_DURATION 500        // duration of the vibration in ms
#define I2C_ADDRESS 0x3E              // the address of the first address of the I2C module (sx1509) (second address: 0x3F, third: 0x70, fourth: 0x71)


//----- Variables -----//

PinStatus buildInLED = LOW;
static unsigned long delay_test = 100;


//----- Main Functions -----//

void setup() 
{
  Serial.begin(9600);     // To connect the arduino using a wire
  Serial1.begin(9600);    // UART that uses pins TX and RX to communicate with the bluetooth module

  knockSetup();
  // lightsInit();
  IMUInit();
  Scheduler.startLoop(_IMU);
  Scheduler.startLoop(checkInput);
}

void loop() 
{
  buildInLED = buildInLED ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, buildInLED);
  delay(1000);
  yield();
}


//----- PC_IO ----//

void checkInput()
{
  if (Serial.available() > 0) serialIn(Serial.readStringUntil('\n'));     // Read the incomming data when the arduino is connected using a wire
  if (Serial1.available() > 0) serialIn(Serial1.readStringUntil('\n'));   // Read the incomming data when the arduino is connected using a wire

  delay(delay_test);
  yield();
}

void serialIn(String inString) 
{
  if ( inString.charAt(0) == 'd' )
  {
    inString.remove(0,2); // remove the "d,"
    delay_test = strtol(inString.c_str(), NULL, 10);
    if(delay_test == 0) delay_test = 100;
  }
  else if( inString.charAt(0) == 'l' )
  {
    inString.remove(0,2);
    int data = inString.toInt();
    lights(data);
  }
  else if( inString.charAt(0) == 'v' )
  {
    inString.remove(0,2);
    int pin = inString.substring(0, inString.indexOf(',')).toInt();
    inString.remove(0,inString.indexOf(',') + 1);

    int dutyCycle = inString.substring(0, inString.indexOf(',')).toInt();
    inString.remove(0,inString.indexOf(',') + 1);
      
    unsigned long duration = strtol(inString.c_str(), NULL, 10);

    vibration(pin, dutyCycle, duration);
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

  String IMUData = "i,";
  IMUData.concat(acceleroX);
  IMUData.concat(",");
  IMUData.concat(acceleroY);
  IMUData.concat(",");
  IMUData.concat(acceleroZ);
  IMUData.concat(",");
  IMUData.concat(gyroZ);

  if(Serial) Serial.println(IMUData);
  if(Serial1) Serial1.println(IMUData);

  delay(delay_test);
  yield();
}

void knockSetup()
{
  pinMode(KNOCKPIN_FRONT,   INPUT);
  pinMode(KNOCKPIN_BACK,    INPUT);
  pinMode(KNOCKPIN_LEFT,    INPUT);
  pinMode(KNOCKPIN_RIGHT,   INPUT);
  pinMode(KNOCKPIN_TOP,     INPUT);
  pinMode(KNOCKPIN_BOTTOM,  INPUT);

  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_FRONT),   knockFront,   LOW);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_BACK),    knockBack,    LOW);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_LEFT),    knockLeft,    LOW);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_RIGHT),   knockRight,   LOW);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_TOP),     knockTop,     LOW);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_BOTTOM),  knockBottom,  LOW);
}
void knockFront()
{
  if(Serial) Serial.println("k,front");
  if(Serial1) Serial1.println("k,front");
}
void knockBack()
{
  if(Serial) Serial.println("k,back");
  if(Serial1) Serial1.println("k,back");
}
void knockLeft()
{
  if(Serial) Serial.println("k,left");
  if(Serial1) Serial1.println("k,left");
}
void knockRight()
{
  if(Serial) Serial.println("k,right");
  if(Serial1) Serial1.println("k,right");
}
void knockTop()
{
  if(Serial) Serial.println("k,top");
  if(Serial1) Serial1.println("k,top");
}
void knockBottom()
{
  if(Serial) Serial.println("k,bottom");
  if(Serial1) Serial1.println("k,bottom");
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

void vibration(int pin, int dutyCycle, unsigned long duration)  // GPIO with PWM, pin between 4 - 7 and 10 - 11, dutyCycle between 0 and 255, 
{
  if (pin < 4 || pin == 8 || pin == 9 || pin > 11) return;
  if (dutyCycle < 0 || dutyCycle > 255) return;

  analogWrite(pin, dutyCycle);
  delay(duration);
  analogWrite(pin, 0);
}