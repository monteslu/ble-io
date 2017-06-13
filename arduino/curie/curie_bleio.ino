#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>

#include <Arduino.h>
#if defined(_VARIANT_ARDUINO_101_X_)
#include <CurieBLE.h>
#define _MAX_ATTR_DATA_LEN_ BLE_MAX_ATTR_DATA_LEN
#else
#include <BLEPeripheral.h>
#define _MAX_ATTR_DATA_LEN_ BLE_ATTRIBUTE_MAX_VALUE_LENGTH
#endif

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1


struct PinState {
  uint8_t mode;
  uint16_t value;
  bool reportDigital;
  bool reportAnalog;
};

uint16_t samplingInterval = 500;      // how often (milliseconds) to report analog data
long currentMillis;     // store the current value from millis()
long previousMillis;    // for comparison with currentMillis

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[64];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

BLEPeripheral blePeripheral; // create peripheral instance

BLEService gpioService("bada5555-e91f-1337-a49b-8675309fb099"); // create service

// create switch characteristic and allow remote device to read and write
BLECharacteristic digitalChar("2a56", BLEWriteWithoutResponse | BLENotify, (unsigned short) 20);
BLECharacteristic analogChar("2a58", BLEWriteWithoutResponse | BLENotify, (unsigned short) 20);
BLECharacteristic configChar("2a59", BLEWriteWithoutResponse, (unsigned short) 20);

PinState states[TOTAL_PINS];

void setup() {
  Serial.begin(57600);

  for (int i=0; i < TOTAL_PINS; i++) {
    states[i].value = 0;
    states[i].reportAnalog = 0;
    states[i].reportDigital = 0;


    if(IS_PIN_DIGITAL(i) && !IS_PIN_ANALOG(i)){
      pinMode(i, OUTPUT);
      states[i].mode = OUTPUT;
    }
    else{
      pinMode(i, INPUT);
      states[i].mode = INPUT;
    }
  }


  // set the local name peripheral advertises
  // will likely default to board name + some of mac address
  //blePeripheral.setLocalName("BLEIO");

  // set the UUID for the service this peripheral advertises
  blePeripheral.setAdvertisedServiceUuid(gpioService.uuid());

  // add service and characteristic
  blePeripheral.addAttribute(gpioService);


  blePeripheral.addAttribute(digitalChar);
  blePeripheral.addAttribute(analogChar);
  blePeripheral.addAttribute(configChar);


  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristics
  digitalChar.setEventHandler(BLEWritten, digitalCharWritten);

  // assign event handlers for characteristics
  analogChar.setEventHandler(BLEWritten, analogCharWritten);

  // assign event handlers for characteristics
  configChar.setEventHandler(BLEWritten, configCharWritten);



  // advertise the service
  blePeripheral.begin();
  Serial.println(("Bluetooth device active, waiting for connections..."));

  currentMillis = millis();
}

void loop() {
  // poll peripheral
  blePeripheral.poll();

  bool notify = 0;

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    notify = 1;
    previousMillis += samplingInterval;
  }

  for (int i=0; i < TOTAL_PINS; i++) {
    if(states[i].reportDigital){
      int val = digitalRead(i);
      if(states[i].value != val) {
        unsigned char report[2] = {(unsigned char)i, (unsigned char)val};
        digitalChar.setValue(report, 2);
      }
      states[i].value = val;
    }

    if(notify == 1 && states[i].reportAnalog == 1){
      Serial.print("notify analog: ");
      int val = analogRead(PIN_TO_ANALOG(i));
      Serial.println((int) val);
      unsigned char report[3] = {(unsigned char)i, lowByte(val), highByte(val)};
      analogChar.setValue(report, 3);

    }

  }

}




/*==============================================================================
 * FUNCTIONS
 *============================================================================*/




void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  Serial.println("starting");

}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}



void digitalCharWritten(BLECentral& central, BLECharacteristic& characteristic) {

  Serial.println("digitalWrite: ");
  uint8_t p = digitalChar.value()[0];
  Serial.println(p);
  uint8_t v = digitalChar.value()[1];

  if(IS_PIN_DIGITAL(p)){
    if(v > 0){
      digitalWrite(p, HIGH);
    }
    else{
      digitalWrite(p, LOW);
    }
    Serial.print("digital written: ");
    Serial.print(v);
  }


}

void analogCharWritten(BLECentral& central, BLECharacteristic& characteristic) {

  Serial.print("analogWrite bytes: ");
  uint8_t len = analogChar.valueLength();
  Serial.println(len);

  uint8_t p = analogChar.value()[0];
  uint8_t v1= analogChar.value()[1];
  uint8_t v2 = analogChar.value()[2];

  Serial.print("p: ");
  Serial.println(p);
  Serial.print("v1: ");
  Serial.println(v1);
  Serial.print("v2: ");
  Serial.println(v2);


  if (p < TOTAL_PINS) {
    uint16_t val = 0;
    val+= v1;
    if(len > 2){
      val+= v2 << 8;
    }
    Serial.print("pwm/servo pin mode ");
    Serial.println(Firmata.getPinMode(p));

    if(Firmata.getPinMode(p) == PIN_MODE_SERVO){
      if (IS_PIN_DIGITAL(p)){
          servos[servoPinMap[p]].write(val);
        }
        Serial.print("servo wrote ");
        Serial.println(val);

    }
    else if(IS_PIN_PWM(p)){
      analogWrite(PIN_TO_PWM(p), val);

      Serial.print("pwm wrote ");
      Serial.println(val);
    }
    Firmata.setPinState(p, val);
  }


}


void configCharWritten(BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print("configWrite bytes: ");
  uint8_t len = configChar.valueLength();
  Serial.println(len);
  uint8_t cmd = configChar.value()[0];
  uint8_t p = configChar.value()[1];
  uint8_t val = configChar.value()[2];


  Serial.print("cmd: ");
  Serial.println(cmd);

  Serial.print("p: ");
  Serial.println(p);

  Serial.print("val: ");
  Serial.println(val);

  switch (cmd) {
    case SET_PIN_MODE:
      Serial.println("SET_PIN_MODE");
//      pinMode(p, val);
      setPinModeCallback(p, val);
      break;

    case REPORT_ANALOG:
      reportAnalogCallback(p, val);
      break;

    case REPORT_DIGITAL:
      Serial.println("REPORT_DIGITAL");
      if(IS_PIN_DIGITAL(p)){
        states[p].reportDigital = (val == 0) ? 0 : 1;
        states[p].reportAnalog = !states[p].reportDigital;
        Serial.print("report digital setting: ");
        Serial.println(states[p].reportDigital);
        Serial.println("REPORT_DIGITAL END");
      }else{
        Serial.print("pin NOT digital: ");
        Serial.println(p);
      }
      break;

    case SAMPLING_INTERVAL:
      Serial.print("SAMPLING_INTERVAL start ");
      Serial.println(samplingInterval);
      samplingInterval = (uint16_t) p;
      if(len > 2){
        samplingInterval+= val << 8;
      }
      Serial.print("SAMPLING_INTERVAL end ");
      Serial.println(samplingInterval);
      break;

    case SERVO_CONFIG:
      Serial.println("SERVO_CONFIG");
      if (len > 5) {
        // these vars are here for clarity, they'll optimized away by the compiler
        int minPulse = val + (configChar.value()[3] << 8);
        int maxPulse = configChar.value()[4] + (configChar.value()[5] << 8);

        if (IS_PIN_DIGITAL(p)) {
          if (servoPinMap[p] < MAX_SERVOS && servos[servoPinMap[p]].attached()) {
            detachServo(p);
          }
          attachServo(p, minPulse, maxPulse);
          setPinModeCallback(p, PIN_MODE_SERVO);
        }
      }
      break;

  }

}



void attachServo(byte pin, int minPulse, int maxPulse)
{
  Serial.print("attachServo ");
  Serial.println(pin);
  if (servoCount < MAX_SERVOS) {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}



void setPinModeCallback(byte pin, int mode)
{

  Serial.print("setPinModeCallback: ");
  Serial.println(pin);
  Serial.print("mode: ");
  Serial.println(mode);


  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO) {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }

  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
    case PIN_MODE_SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        Firmata.setPinMode(pin, PIN_MODE_SERVO);
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          // pass -1 for min and max pulse values to use default values set
          // by Servo library
          attachServo(pin, -1, -1);
        }
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
      break;

  }
}


void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  isI2CEnabled = true;

  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

void reportAnalogCallback(byte p, int val)
{
    Serial.println("REPORT_ANALOG");
    if(IS_PIN_ANALOG(p)){
      Serial.print("current report analog setting: ");
      Serial.println(states[p].reportAnalog);
      states[p].reportAnalog = (val == 0) ? 0 : 1;
      states[p].reportDigital = !states[p].reportAnalog;
      Serial.print("report analog setting: ");
      Serial.println(states[p].reportAnalog);
    }else{
      Serial.print("pin NOT analog: ");
      Serial.println(p);
    }
    Serial.println("REPORT_ANALOG END");
}
