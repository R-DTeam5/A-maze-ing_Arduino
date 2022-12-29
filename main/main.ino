//---- Libraries -----//

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <Scheduler.h>


//----- Pinout -----//

// pin 8: I2C SDA for LEDS and the vibration motors (Wire)
// pin 9: I2C SCL for LEDS and the vibration motors (Wire)
// pin 16: TX Used for UART with bluetooth module
// pin 17: RX Used for UART with bluetooth module
#define KNOCKPIN_FRONT  D8    // D2 pin : 20
#define KNOCKPIN_BACK   D9    // D3 pin : 21
#define KNOCKPIN_LEFT   D4    // D4 pin : 22
#define KNOCKPIN_RIGHT  D5    // D5 pin : 23
#define KNOCKPIN_TOP    D6    // D6 pin : 24
#define KNOCKPIN_BOTTOM D7    // D7 pin : 25


//----- Constants -----//

#define IO_ADDRESS 0x3E
SX1509 ioExpander;

//----- Variables -----//

PinStatus buildInLED = HIGH;
static unsigned long measurementDelay = 100;
int knockPinHit = 0;


//----- Main Arduino Functions -----//

void setup() 
{
  digitalWrite(LED_BUILTIN, buildInLED);  
  Serial.begin(9600);     // To connect the arduino using a wire
  Serial1.begin(9600);    // UART that uses pins TX and RX to communicate with the bluetooth module

  knockInit();
  wireInit();
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


//----- Serial Communication (with PC and Bluetooth module) ----//

void checkInput()
{
  if (Serial.available() > 0) serialIn(Serial.readStringUntil('\n'));     // Read the incomming data when the arduino is connected using a wire
  if (Serial1.available() > 0) serialIn(Serial1.readStringUntil('\n'));   // Read the incomming data when the arduino is connected using Bluetooth

  if(knockPinHit > 0) sendKnockPinData();

  delay(measurementDelay);
  yield();
}

void serialIn(String inString) 
{
  if ( inString.charAt(0) == 'd' )
  {
    inString.remove(0,2); // remove the "d,"
    measurementDelay = strtol(inString.c_str(), NULL, 10);
    if(measurementDelay == 0) measurementDelay = 100;
  }
  else if( inString.charAt(0) == 'w' )
  {
    inString.remove(0,2);

    int board = inString.substring(0, inString.indexOf(',')).toInt();
    inString.remove(0, inString.indexOf(',') + 1);

    int pin = inString.substring(0, inString.indexOf(',')).toInt();
    inString.remove(0, inString.indexOf(',') + 1);

    int value = inString.toInt();
    wire(board, pin, value);
  }
}

void sendKnockPinData()
{
  switch (knockPinHit)
  {
    case 1:
      if(Serial) 
      {
        Serial.println("k,right");
        Serial.flush();
      }
      if(Serial1) 
      {
        Serial1.println("k,right");
        Serial1.flush();
      }
      break;

    case 2:
      if(Serial) 
      {
        Serial.println("k,back");
        Serial.flush();
      }
      if(Serial1) 
      {
        Serial1.println("k,back");
        Serial1.flush();
      }
      break;

    case 3:
      if(Serial) 
      {
        Serial.println("k,left");
        Serial.flush();
      }
      if(Serial1) 
      {
        Serial1.println("k,left");
        Serial1.flush();
      }
      break;

    case 4:
      if(Serial) 
      {
        Serial.println("k,bottom");
        Serial.flush();
      }
      if(Serial1) 
      {
        Serial1.println("k,bottom");
        Serial1.flush();
      }
      break;

    case 5:
      if(Serial) 
      {
        Serial.println("k,top");
        Serial.flush();
      }
      if(Serial1) 
      {
        Serial1.println("k,top");
        Serial1.flush();
      }
      break;

    case 6:
      if(Serial) 
      {
        Serial.println("k,front");
        Serial.flush();
      }
      if(Serial1) 
      {
        Serial1.println("k,front");
        Serial1.flush();
      }
      break;
  }
  knockPinHit = 0;
}


//----- IMU data handler -----//

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
  IMUData.concat(gyroX);
  IMUData.concat(",");
  IMUData.concat(gyroY);
  IMUData.concat(",");
  IMUData.concat(gyroZ);

  if(Serial) 
  {
    Serial.println(IMUData);
    Serial.flush();
  }
  if(Serial1) 
  {
    Serial1.println(IMUData);
    Serial1.flush();
  }

  delay(measurementDelay);
  yield();
}

//----- Interrupts for the knock sensors -----//

void knockInit()
{
  pinMode(KNOCKPIN_FRONT,   INPUT_PULLUP);
  pinMode(KNOCKPIN_BACK,    INPUT_PULLUP);
  pinMode(KNOCKPIN_LEFT,    INPUT_PULLUP);
  pinMode(KNOCKPIN_RIGHT,   INPUT_PULLUP);
  pinMode(KNOCKPIN_TOP,     INPUT_PULLUP);
  pinMode(KNOCKPIN_BOTTOM,  INPUT_PULLUP);

  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_FRONT),   knockFront,   FALLING);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_BACK),    knockBack,    FALLING);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_LEFT),    knockLeft,    FALLING);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_RIGHT),   knockRight,   FALLING);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_TOP),     knockTop,     FALLING);
  attachInterrupt( digitalPinToInterrupt(KNOCKPIN_BOTTOM),  knockBottom,  FALLING);
}
void knockFront()
{
  knockPinHit = 1;
}
void knockBack()
{
  knockPinHit = 2;
}
void knockLeft()
{
  knockPinHit = 3;
}
void knockRight()
{
  knockPinHit = 4;
}
void knockTop()
{
  knockPinHit = 5;
}
void knockBottom()
{
  knockPinHit = 6;
}


//----- IO-expander using I2C -----//

void wireInit() // sx1509 io expander using I2C: https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library
{
  Wire.begin();  // join i2c bus
  ioExpander.begin(IO_ADDRESS);
  for(int i = 0; i < 16; i++) 
    ioExpander.pinMode(i, ANALOG_OUTPUT);    
}
void wire(int board, int pin, int value)
{
  if (pin >= 0 && pin <= 5)
  {
    ioExpander.analogWrite(0, 50);
    ioExpander.analogWrite(2, 50);
    delay(50);
    ioExpander.analogWrite(0, 255);
    ioExpander.analogWrite(2, 255);
  }
  else ioExpander.analogWrite(pin, value);
}