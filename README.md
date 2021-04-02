# :soccer::robot: SuperBall MKII
A rolling sphere

## Goal here
Making the ball roll in every direction giving by the remote whatever the orientation of the robot inside and keeping his balance when doing so.

For that i'm using an IMU (BNO055) and the two motors have encoded wheel. The remote use an 2.4GHz emitter (nRF24) to communicate.

## Arduino IDE
** Upload Settings **
Board: "Arduino Nano"
Processor: "ATmega328P (Old Bootloader)"

### esp-link

#### Flash with esptool
Download the latest [release](https://github.com/jeelabs/esp-link/releases).

```
esptool.py --port /dev/tty.wchusbserial14130 --baud 230400 write_flash -fs 32m -ff 80m \
    0x00000 boot_v1.7.bin 0x1000 user1.bin \
    0x3FC000 esp_init_data_default.bin 0x3FE000 blank.bin
```

for more infos see [esp-link](https://github.com/jeelabs/esp-link)

#### Connect to arduino
- RX: connect to TX of microcontroller
- TX: connect to RX of microcontroller
- GND: connect to GND of microcontroller

####
Add this to your programmers.txt file:
```
esplink.name=esp-link 2032
esplink.program.tool=avrdude
esplink.protocol=arduino
esplink.program.extra_params= -P net:192.168.1.59:2323  -b{upload.speed}

esplink23.name=esp-link
esplink23.program.tool=avrdude
esplink23.protocol=arduino
esplink23.program.extra_params= -P net:192.168.1.59:23  -b{upload.speed}
```
use ip of esp-link and just use upload with programmer `esp-link`.

## Utils
list port osx
`ls /dev/tty*`

# Electronics Remote
(Arduino Uno/Diecimila/Duemilanove)
```
OLED SSD1306 128X32
- GND -> GND
- VCC -> 5v
- SCL -> A5
- SDA -> A4
```

# Electronics Robot
(Arduino Uno/Diecimila/Duemilanove)
```
- Arduino NANO V3
- nRF24 l01 PA LNA
- BNO055
- L298N
- Motor 19:1 Metal Gearmotor 37Dx68L 12V 540 RPM

Connections you will need.

RIGHT MOTOR
* MOTOR 1 RED    -> L298N OUT1
* MOTOR 1 BLACK  -> L298N OUT2
* MOTOR 1 GREEN  -> GND
* MOTOR 1 BLUE   -> VCC (3.5 – 20 V) -> 5v
* MOTOR 1 YELLOW -> encoder A output -> INT1/3
* MOTOR 1 WHITE  -> encoder B output -> A3/17

LEFT MOTOR
* MOTOR 2 RED    -> L298N OUT4
* MOTOR 2 BLACK  -> L298N OUT3
* MOTOR 2 GREEN  -> GND
* MOTOR 2 BLUE   -> VCC (3.5 – 20 V) -> 5v
* MOTOR 2 YELLOW -> encoder A output -> INT0/2
* MOTOR 2 WHITE  -> encoder B output -> A0/14

* L298N +12v -> BATT +
* L298N GND  -> BATT - & Arduino GND
* L298N +5v  -> Arduino 5v
* L298N ENA -> 5
* L298N IN1 -> 4
* L298N IN2 -> 7
* L298N IN3 -> 8
* L298N IN4 -> A1/15
* L298N ENB -> 6

* RF24 GND -> GND & 10uf cap GND
* RF24 VCC -> +3.3v & 10uf cap +
* RF24 CE  -> 9
* RF24 CSN -> 10
* RF24 SCK -> 13
* RF24 MOSI -> 11
* RF24 MISO -> 12

* BNO055 VIN -> 5v
* BNO055 GND -> GND
* BNO055 SDA -> A4
* BNO055 SCL -> A5

* BATT + -> 100K resistor -> A2 -> 10K resistor + (100nf cap) -> GND
```

## Test

Tune PID motor encoder position
No load:
```
  | Ok   | Good
P | 15   | 40
I | 2.5  | 2.5
D | 3000 | 5000
```

## :thumbsup: Inspired by some nice project on the internet		
[nRF24-Esk8-Remote by @SolidGeek](https://github.com/SolidGeek/nRF24-Esk8-Remote)

[Arduino voltage divider 0V to 30V](http://www.electroschematics.com/9351/arduino-digital-voltmeter/)

[James Bruton Star Wars BB-8 V3 series](https://www.youtube.com/playlist?list=PLpwJoq86vov8gnKpQkZUH4szapX1jxcmC)

[Matt Denton Droid Build D-O](https://youtu.be/zplirkxl6iM)

[Encoded motor](https://github.com/NikodemBartnik/ArduinoTutorials/tree/master/Encoded%20motor)

## :heart: Thanks for your support		
[@spectre](https://github.com/spectrenoir06), Antony, Mom & Dad
