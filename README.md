# A-maze-ing_Arduino
The arduino code for the r&d project A-maze-ing (group 5)

## 1 Pinout

[![N|Solid](https://www.etechnophiles.com/wp-content/uploads/2021/01/Nano-BLE-Sense-pinout.jpg)](https://docs.arduino.cc/hardware/nano-33-ble-sense)

| Usage | Pin | Pin | Usage |
| ------ | ------ | ------ | ------ |
|  | 1 | 30 |  |
| 3.3V | 2 | 29 |  |
|  | 3 | 28 |  |
|  | 4 | 27 |  |
|  | 5 | 26 |  |
|  | 6 | 25 | knock pin bottom |
|  | 7 | 24 | knock pin top |
| I2C SDA (IO Expander) | 8 | 23 | knock pin right |
| I2C SCL (IO Expander) | 9 | 22 | knock pin left |
|  | 10 | 21 | knock pin back |
|  | 11 | 20 | knock pin front |
|  | 12 | 19 | GND |
| Reset (used for the IO Expander) | 13 | 18 | Reset (used for the IO Expander) |
| GND | 14 | 17 | RX UART (BLuetooth) |
|  | 15 | 16 | TX UART (Bluetooth) |

## 2 Features

- Uses UART to communicate with a PC or an external Bluetooth module
- Takes IMU Measurements
- Handles interrupts from knock sensors
- Uses I2C to connect with an external IO expander
- A variable delay between measurements

### 2.1 UART
The Arduino uses UART to communicate with a computer and an external Bluetooth module ([HC-05][HC-05]).
The data transmitted is a string formatted as shown:
> `i`,`value1`,`value2`

where:
- `i` : The unique identifier of the type of command
- `value1` and `value2` : A number of variables like e.g. data or variables for a funciton
- , : all values are separated with a comma

### 2.2 IMU
The Arduino measures the deta from the accelerometer and gyroscope using the [internal IMU][IMU]. This data gets send to the pc in the following format:
> i,`acceleroX`,`acceleroY`,`acceleroZ`,`gyroX`,`gyroY`,`gyroZ`
(all variables are floats)

Both the modules have a sample rate of 119Hz.

### 2.3 Knock Sensors
The Arduino handles the interrupts generated on the digital pins by a piëzo electric element. When an interrupt occurs, this gets communicated with the pc in the following format:
> k,`direction`
(direction is a string)

where direction can have the following values, depending on what side of the box gets hit:
- front
- back
- left
- right
- top
- bottom

### 2.4 IO Expanders
The arduino can connect with three IO Expanders ( [SX1509][SX1509] ). The pins of the IO Expander can be manipulated by sending a command in the following format to the Arduino:
>w,`board`,`pin`,`value`
(all variables are integers)

where:
- board : a value between 1 and 3 corresponting with which of the three IO Expanders should be used:

| Board | Address |
| ------ | ------ |
| 1 | 0x3E |
| 2 | 0x3F |
| 3 | 0x70 |

- pin : the pin on the IO Expander PCB that should be changed, ranging between 0 and 15.
- value : the pwm duty cycle that the pin should get, ranging between 0 and 255.

### 2.5 Delay
The delay between the measurements can be altered by sending a command in the following format to the Arduino:
>d,`delay`
(delay is an unsinged long)

Where delay is the time in milliseconds between measurements.

README made using [Dillinger][Dillinger]

[//]: # 

   [SX1509]: <https://learn.sparkfun.com/tutorials/sx1509-io-expander-breakout-hookup-guide#sx1509-breakout-board-overview>
   [IMU]: <https://www.arduino.cc/reference/en/libraries/arduino_lsm9ds1/>
   [HC-05]: <https://electrosome.com/hc-05-serial-bluetooth-module/>
   [Dillinger]: <https://dillinger.io/>
    

