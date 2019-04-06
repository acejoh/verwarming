#include <Wire.h>
#include <Adafruit_AM2315.h>

// Connect RED of the AM2315 sensor to 5.0V
// Connect BLACK to Ground
// Connect WHITE to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect YELLOW to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4

// Include the libraries we need
//#include <OneWire.h>
//#include <DallasTemperature.h>
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <LowPower.h>
#include <PinChangeInterrupt.h>
#include <Time.h>
#include <TimeZone.h>

Adafruit_AM2315 sensor;

// Data wire is plugged into port 2 on the Arduino
//#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature sensors(&oneWire);

// switch state pin should be from one port for pin change interrupts
// Each pin row (0-7, 8-13, A0-A5) represents a port.
const int PIN_SWITCH_OFF = 8;
const int PIN_SWITCH_ON = 9;
const int PIN_SWITCH_AUTO = 10;

const unsigned long DEBOUNCE_DELAY = 50;

const int PIN_RELAY_1 = 10;


int switchState;
int lastSwitchState;

unsigned long debounceTimer;


void setup() {
  Serial.begin(9600);
  Serial.println("Verwarming Test!");

  // Start up the sensor
  if (!sensor.begin()) {
     Serial.println("Sensor not found, check wiring & pullups!");
  }
  //sensors.begin();

  // set the time
  setSyncProvider(RTC.get);   // the function to get the time from the RTC // @suppress("Invalid arguments")
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  // init pins
  pinMode(PIN_SWITCH_OFF, INPUT);
  pinMode(PIN_SWITCH_ON, INPUT);
  pinMode(PIN_SWITCH_AUTO, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  switchState = getSwitchState();
  lastSwitchState = switchState;
  handleSwitchState(switchState);

}

void loop() {
  float humidity;
  float temperature;
  //sensors.requestTemperatures(); // Send the command to get temperatures
  //temperature = sensors.getTempCByIndex(0);
  //if (temperature == DEVICE_DISCONNECTED_C) {
  if (!sensor.readTemperatureAndHumidity(temperature, humidity)) {
    Serial.println("Sensor fail");
    humidity = 0;
    temperature = 0;
  }

  //Serial.print("Hum: "); Serial.println(humidity);
  //Serial.print("Temp: "); Serial.println(temperature);

  int reading = getSwitchState();
  if (reading != lastSwitchState) {
    Serial.println("Debounce start");
    debounceTimer = millis();
  }

  if (millis() > debounceTimer + DEBOUNCE_DELAY
      && reading != switchState) {
    Serial.println("Switching state");
    handleSwitchState(reading);
    switchState = reading;
  }

  lastSwitchState = reading;

  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

int getSwitchState() {
  int switchState;
  //Get starting switch state
  if (digitalRead(PIN_SWITCH_ON) == HIGH) {
    switchState = PIN_SWITCH_ON;
  }
  else if (digitalRead(PIN_SWITCH_AUTO) == HIGH) {
    switchState = PIN_SWITCH_AUTO;
  }
  else {
    switchState = PIN_SWITCH_OFF;
  }
  return switchState;
}

void handleSwitchState(int switchState) {

  switch (switchState) {
    case PIN_SWITCH_ON:
      Serial.println("State=ON");
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(PIN_SWITCH_OFF, LOW);
      break;
    case PIN_SWITCH_AUTO:
      Serial.println("State=AUTO");
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(PIN_SWITCH_OFF, LOW);
      break;
    default:
      Serial.println("State=OFF");
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(PIN_SWITCH_OFF, HIGH);
      break;
  }

}
