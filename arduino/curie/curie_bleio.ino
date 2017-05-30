#include "Firmata.h"

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


struct PinState {
  uint8_t mode;
  uint16_t value;
  bool reportDigital;
  bool reportAnalog;
};

uint16_t samplingInterval = 500;      // how often (milliseconds) to report analog data
long currentMillis;     // store the current value from millis()
long previousMillis;    // for comparison with currentMillis


BLEPeripheral blePeripheral; // create peripheral instance

BLEService gpioService("bada5555-e91f-1337-a49b-8675309fb099"); // create service

// create switch characteristic and allow remote device to read and write
BLECharacteristic digitalChar("2a56", BLEWriteWithoutResponse | BLENotify, (unsigned short) 20);
BLECharacteristic analogChar("2a58", BLEWriteWithoutResponse | BLENotify, (unsigned short) 20);
BLECharacteristic configChar("2a59", BLEWriteWithoutResponse, (unsigned short) 20);

PinState states[TOTAL_PINS];

void setup() {
  Serial.begin(9600);

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

  if(IS_PIN_PWM(p)){
    uint16_t val = 0;

    val+= v1;
    Serial.print("pwm writing val ");
    Serial.println(val);
    if(len > 2){
      val+= v2 << 8;
    }

    analogWrite(p, val);


    Serial.print("pwm wrote ");
    Serial.println(val);

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

  if(cmd == SET_PIN_MODE) {
    //TODO This should use Firmata's SYSEX implementation
    Serial.println("SET_PIN_MODE");
    pinMode(p, val);
  }

  else if(cmd == REPORT_ANALOG) {
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

  else if(cmd == REPORT_DIGITAL) {
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

  }

  else if (cmd == SAMPLING_INTERVAL){
    Serial.print("SAMPLING_INTERVAL start ");
    Serial.println(samplingInterval);
    uint16_t newInterval = (uint16_t) p;
    if(len > 2){
      newInterval+= val << 8;
    }
    samplingInterval = newInterval;
    Serial.print("SAMPLING_INTERVAL end ");
    Serial.println(samplingInterval);
  }

}
