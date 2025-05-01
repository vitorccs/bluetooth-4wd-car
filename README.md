# Bluetooth RC Car
Bluetooth Remote Controlled Car using Arduino Nano v3 board. 

### With L298N
<img src="https://github.com/user-attachments/assets/4f2cbef3-492f-4302-8aa4-057da90a90c8" width="400">
<img src="https://github.com/user-attachments/assets/fadf9f43-9b2a-44ee-9892-cfe2dd71daae" width="400">

### With TB6612
<img src="https://github.com/user-attachments/assets/05052b51-076b-4768-9a88-918afed03c8b" width="400">
<img src="https://github.com/user-attachments/assets/6a332693-4862-4871-be10-9b9c51cf525d" width="400">
<img src="https://github.com/user-attachments/assets/a88e518d-d366-4d8d-bff3-fa5197b49029" width="400">


## Recording
Note: enable the sound by clicking on the speaker icon from the video toolbar

https://github.com/user-attachments/assets/0f21e44e-0843-4f98-aa87-6b6e9641b2d8

https://github.com/user-attachments/assets/41868ca5-1b39-4b17-b1c1-4ee89e308f43


## Schematics
This is a [Platform IO IDE](https://platformio.org/platformio-ide) project coded in C++. 

**Option 1 - With L298N Bridge**
![Image](https://github.com/user-attachments/assets/65fe54e7-17b8-4145-8779-8204bebc8613)

**Option 2 - With TB6612 Bridge**
![Image](https://github.com/user-attachments/assets/55934be4-e4af-468f-8c19-61007b0bced9)

## Bluetooth controller
The car is controlled by a [free Android App](https://play.google.com/store/apps/details?id=com.electro_tex.bluetoothcar&pcampaignid=web_share) (Bluetooth connection is displayed as "HC-05"):
<img src="https://github.com/vitorccs/bluetooth-rc-car/assets/9891961/93d25180-f532-4717-aa7c-2c25e38c2414" width="550">

Once installed the app, map the buttons code as shown below:

<img src="https://github.com/vitorccs/bluetooth-rc-car/assets/9891961/4e0d08ae-a4e4-4613-90cf-5a586280fb27" width="550">

## Components
* 01 - 4WD Car Chassis with steering (see section below)
* 04 - DC Motors (3v - 6v)
* 04 - Wheels
* 01 - Arduino Nano v3
* 01 - [Arduino Nano shield](https://www.aliexpress.us/item/2251801857885983.html) (recomended, not required)
* 01 - Dual H-Bridge board (L298N or TB6612FNG)
* 01 - HC-05 Bluetooth module
* 02 - Red Leds
* 02 - White Leds
* 01 - Passive Buzzer
* 04 - 150 Ω Resistors
* 01 - 1.0K Ω Resistor
* 01 - 3.3K Ω Resistor
* 01 - 220 Ω Resistor
* 02 - 18650 batteries (3.7v - 4.2v)
* 01 - Battery support
* 01 - SG90 Servo Motor
* 02 - Electrolytic Capacitor (16v - 50v, 8,000 - 12,000 μF)

NOTES:
* TB6612FNG is a modern MOSFET driver with about 90% energy efficiency versus 40-70% for the L298N

## About PlatformIO IDE
Platform IO is a plugin for Microsoft Virtual Studio Code. It is a more robust IDE compared to official Arduino IDE. It also allows us to easily create our own private libraries and use a more object oriented code.

## About the code
The PINs can be customized in the `main.cpp` 
```c++
#include <Arduino.h>
#include <DigitalLed.h>
#include <PassiveBuzzer.h>
#include <DCMotor.h>
#include <Car.h>
#include <BluetoothJoyHandler.h>
#include <SoftwareSerial.h>
#include <ServoMotor.h>
#include <Servo.h>
#include <SerialReader.h>

#define PIN_FLED 2
#define PIN_RLED 3
#define PIN_HORN 10
#define PIN_M1_EN 5
#define PIN_M1_IN1 4
#define PIN_M1_IN2 6
#define PIN_M2_EN 11
#define PIN_M2_IN1 7
#define PIN_M2_IN2 8
#define PIN_BLUETOOTH_TX 12
#define PIN_BLUETOOTH_RX 13
#define PIN_SERVO 9
#define MIN_MOTOR_SPEED 80 // (between 0 to 255)

DigitalLed fLed(PIN_FLED);
DigitalLed rLed(PIN_RLED);
PassiveBuzzer horn(PIN_HORN);
DCMotor motor1(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
DCMotor motor2(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
Servo servo;
ServoMotor servoMotor(servo, PIN_SERVO);
Car car(motor1, motor2, fLed, rLed, servoMotor, horn);
SoftwareSerial bluetooth(PIN_BLUETOOTH_TX, PIN_BLUETOOTH_RX);
BluetoothJoyHandler joyHandler(car);
SerialReader serialReader;

void setup()
{
    Serial.begin(9600);
    bluetooth.begin(9600);

    servoMotor.attach();
    
    joyHandler.setDebug(false);

    car.stop();
    car.setMaxSpeed();
}

void loop()
{
    String command = serialReader.read(bluetooth);
    if (command != "") {
        joyHandler.handle(command);
    }
}
```

Fine-tuning customizations can be done in the individual files like `ServoMotor.h` for servo motor angle
```c++
#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H
#include <Arduino.h>
#include <Servo.h>

class ServoMotor
{
public:
  ServoMotor(Servo &servo, uint8_t pin);
  void attach();
  void turn(uint16_t angle);
  void turnLeft();
  void turnHalfLeft();
  void turnRight();
  void turnHalfRight();
  void reset();

private:
  uint8_t pin;
  Servo _servo;
  uint8_t angleReset = 90;
  uint8_t angleFull = 25;
  uint8_t angleHalf = 15;
};

#endif
```

or in the `DCMotor.h` for changing speed parameters
```c++
#ifndef DCMOTOR_H
#define DCMOTOR_H
#include <Arduino.h>

class DCMotor
{
public:
    DCMotor(uint8_t pinEn, uint8_t pinIn1, uint8_t pinIn2);
    void backward(uint8_t speed = 100);
    void forward(uint8_t speed = 100);
    void setMinAbsSpeed(uint8_t absSpeed);
    void stop();

private:
    uint8_t pinEn;
    uint8_t pinIn1;
    uint8_t pinIn2;
    uint8_t absSpeed = 0;
    uint8_t maxAbsSpeed = 255;
    uint8_t minAbsSpeed = 50;
    uint8_t ignoreAbsSpeed = 30;

    void setSpeed(uint8_t speed);
};

#endif
```

## About Car Chassis
This project was designed to have a servo motor steering the front wheels:

### Reference 1 (International):
<img src="https://github.com/vitorccs/bluetooth-rc-car/assets/9891961/fc44181b-09c9-459d-bb90-3e725720ff7c" width="300">

https://www.aliexpress.us/item/2251832846243463.html?channel=facebook

### Reference 2 (Brazil):
<img src="https://github.com/vitorccs/bluetooth-rc-car/assets/9891961/6d2626ba-d2ef-4a6d-9c85-598381214e7b" width="300">

https://www.usinainfo.com.br/kit-robotica/chassi-carrinho-arduino-mdf-com-eixo-movel-v2-manual-de-montagem-5532.html

## About the Power Supply
I recommend to use high quality 18650 batteries (3.7v - 4.2v, 2200mAh, at least 2C of discharge rate).

Most people prefer to use different power sources for Arduino (5v) and Bridge driver (7.4 - 8.4v). 

I prefer to have a single power source and thus a single power switch. However, it was required to use huge Electrolytic Capacitors (around 8,000 to 12,000µF) to prevent Arduino Nano from rebooting when the 04 DC motors drains too much power. The capacitor is also useful to filter some electric noise. Note: in the video recordings above, I used 2x 18650 batteries 2600mAh (3C) and one 10,000 µF capacitor.

It is up to you!

## About the Servo Motor
Be careful if you need to change PINs of PWM motors (`PIN_M1_EN` and `PIN_M2_EN`) and Servo (`PIN_SERVO`) since they cannot use the same Arduino timers.
* [Discussion in Arduino forum](https://forum.arduino.cc/t/problem-using-both-a-dc-motor-and-servo/1172917/13)
* [Arduino timers](https://devboards.info/boards/arduino-nano)


## Fritzing file
The eletronic schematic was created in the [Fritzing](https://fritzing.org/) software and can be downloaded at
* [BluetoothRcCar_L298N.zip](https://github.com/user-attachments/files/19987016/BluetoothRcCar_L298N.zip)
* [BluetoothRcCar_TB6612.zip](https://github.com/user-attachments/files/19987015/BluetoothRcCar_TB6612.zip)
