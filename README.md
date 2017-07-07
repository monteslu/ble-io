# ble-io

Ble-io is a library for writing javascript programs that interact with Bluetooth Low-Energy (BLE) devices. It can also be used an [IO plugin](https://github.com/rwaldron/io-plugins) for use with the [johnny-five](https://github.com/rwaldron/johnny-five) robotics framework.

The intention is to be able to use a simple and generic API for interacting with the GPIO pins across many different types of BLE peripherals.

Ble-io was built at [IcedDev](http://iceddev.com/)


## Installation

`npm install ble-io`

## Getting Started

In order to use the ble-io library, you will need to load a firmware onto the BLE peripheral that supports the ble-io Service and Characteristics described [here](https://github.com/monteslu/ble-io/blob/master/service.md).

One implementation of the ble-io service is for Intel curie based boards including the Arduino 101 and TinyTile here: [firmware](https://github.com/monteslu/ble-io/tree/master/arduino/curie)

## Usage

```js
const BleIO = require("ble-io");
const io = new BleIO();

io.on("ready", function() {
  //turn on pin 13
  this.digitalWrite(13, 1);
});
```


## With johnny-five

```js
const BleIO = require("ble-io");
const five = require("johnny-five");

const board = new five.Board({
  io: new BleIO()
});

board.on("ready", function() {
  cosnt led = new five.Led(13);
  led.blink(500);
});
```

## TODO

* We need a way to query pin capabilities. Right now the code is assuming 20 pins all able to accept any pin mode.
* i2C support needs to be added as a new Characteristic to the reference firmware
