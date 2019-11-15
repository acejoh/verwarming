// Connect RED of the AM2315 sensor to 5.0V
// Connect BLACK to Ground
// Connect WHITE to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect YELLOW to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4

// Include the libraries we need
#include <Wire.h>
#include <Adafruit_AM2315.h>
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
#include <Time.h>
#include <TimeZone.h>
#include <advancedSerial.h>
#include <MemoryFree.h>
#include <avr/wdt.h>

Adafruit_AM2315 sensor;

// constants
// PIN numbers & states
const int PIN_LED = 7;
const int PIN_RELAY = 8;

// switch #3 = pin 9
// switch #2 = GND = switch #4 (=GND)
// switch #1 = pin 10
const int PIN_SWITCH_START = 9;
const int PIN_SWITCH_AUTO = 10;
const int PIN_SWITCH_OFF = 0; // not used as pin

const int HEATER_STATE_OFF = 0;
const int HEATER_STATE_ON = 1;
const int HEATER_STATE_AUTO = 2;

// time constants
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 60 }; // Central European Summer Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 0 }; // Central European Standard Time
Timezone CE(CEST, CET);

const int SATURDAY = 7;
const int SUNDAY = 1;
const int DAL_UUR_START = 3;
const int DAL_UUR_END = 6;

// temperature and humidity settings
const float MIN_TEMP_OFF = 1; // Antivorstbeveiliging in AUTO tijdens piekuren
const float MAX_HUM_OFF = 999;
const float MIN_TEMP_AUTO = 10; // Onder deze temp, springt verwarming op tijdens daluren
const float MAX_HUM_AUTO = 999;
const float MAX_TEMP_START = 20;  // Boven deze temperatuur zal de verwarming nooit aanspringen

// switch debounce and state constants
const unsigned int DEBOUNCE_DELAY = 50;
const unsigned int DEBOUNCE_DELAY_START = 400;
const unsigned int DEBOUNCE_DELAY_AUTO = 750;
const unsigned int DEBOUNCE_DELAY_OFF = 750;

const int DOWN_TIME_SECONDS_OFF = 30 * 60;
const int DOWN_TIME_SECONDS_AUTO = 30 * 60; // Tijd dat de verwarming uit staat in AUTO
const int UP_TIME_SECONDS_OFF = 30 * 60;
const int UP_TIME_SECONDS_AUTO = 30 * 60; // Tijd dat de verwarming aan staat in AUTO
const int UP_TIME_SECONDS_ON = 10 * 60; // Tijd dat de verwarming mag aanstaan in HAND

// debug led constants
const int ERROR_SENSOR = 1;
const int ERROR_RTC = 2;
const int ERROR_NAN = 3;
const int ERROR_NAN_COUNT = 99;
const int ERROR_STATE = 4;

const int HOUR_LED_ON_UNTIL = 23;
const int HOUR_LED_ON_FROM = 6;


// variables
int switchState;
int heaterState;

time_t heaterStateStart;
bool isDownTime = false;
bool hasStarted = false;
bool hasDownTimePassed = false;

unsigned long readSensorStart;
float temperature = MIN_TEMP_AUTO + 1;
float humidity = 0;

unsigned long debugLogStart;
int errorCode = 0;
unsigned long debugLedStart;


void setup() {
	// init pins
	digitalWrite(PIN_RELAY, LOW);
	pinMode(PIN_RELAY, OUTPUT);

	digitalWrite(PIN_LED, LOW);
	pinMode(PIN_LED, OUTPUT);

	pinMode(PIN_SWITCH_START, INPUT_PULLUP);
	pinMode(PIN_SWITCH_AUTO, INPUT_PULLUP);

	Serial.begin(9600);
	aSerial.setPrinter(Serial);
	aSerial.setFilter(Level::vvv);
	/* Uncomment the following line to disable the output. By default the ouput is on. */
	// aSerial.off();

	aSerial.v().println(F("Verwarming start"));
	aSerial.v().println(F("================"));

	// Start the temperature sensor
	if (!sensor.begin()) {
		errorCode = ERROR_SENSOR;
		aSerial.v().println(F("Error: Sensor not found, check wiring & pullups"));
	}

	// set/get the RTC time
	//RTC.set(compileTime());	// COMMENT THIS LINE ONCE IN PRODUCTION
	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if (timeStatus() != timeSet) {
		errorCode = ERROR_RTC;
		aSerial.v().println(F("Error: failed to sync with RTC"));
	}
	printDateTime(now());

	// get initial state of switch & heater
	switchState = getSwitchState();
	heaterState = setHeaterState(switchState);

	wdt_enable(WDTO_8S);
	aSerial.v().println(F("Setup is done"));
	aSerial.v().println();
}

void loop() {
	wdt_reset();

	// get temperature & humidity from sensor
	//  update frequency = .5hz
	if (millis() - readSensorStart > 2000) {
		temperature = readTemperature();
		humidity = readHumidity();

		readSensorStart = millis();
	}

	// debug log
	if (millis() - debugLogStart > 5000) {
		printDateTime(now());
		if (isNowDalUur())
			aSerial.vvv().println(F("Daluur"));

		aSerial.vvv().print(F("Temp: ")).println(temperature);

		aSerial.vvv().print(F("Current heater state: "));
		switch(heaterState) {
		case HEATER_STATE_AUTO:
			aSerial.vvv().println(F("AUTO"));
			break;
		case HEATER_STATE_OFF:
			aSerial.vvv().println(F("OFF"));
			break;
		case HEATER_STATE_ON:
			aSerial.vvv().println(F("ON"));
			break;
		default:
			aSerial.vvv().println(F("UNDEFINED"));
		}

		aSerial.vvv().print(F("Current cycle: "));
		if (isDownTime)
			aSerial.vvv().println(F("DOWN"));
		else
			aSerial.vvv().println(F("UP"));

		time_t running = now() - heaterStateStart;
		aSerial.vvv().print(F(" - runtime: ")).print(minute(running)).print(':').println(
				second(running));

		aSerial.vvv().print(F("Free RAM: ")).println(freeMemory());

		aSerial.vvv().println();
		debugLogStart = millis();
	}


	// check for switch state changes
	// 1. Debounce
	int reading = getSwitchState();
	if (reading != switchState) {
		// debounce switch
		aSerial.vvv().println(F("Debounce start"));
		unsigned long debounceStart = millis();
		unsigned int debounceDelay = DEBOUNCE_DELAY + addSwitchDelay(reading);
		while (millis() - debounceStart < debounceDelay)
			;
		reading = getSwitchState();
	}

	// 2. perform state change
	if (reading != switchState) {
		// switch to new state
		aSerial.vvv().println(F("Switching state"));
		heaterState = setHeaterState(reading);
		switchState = reading;
	}

	// determine if heater should start or not
	bool startHeater = getStartHeater(heaterState, humidity, temperature);
	if (startHeater)
		digitalWrite(PIN_RELAY, HIGH);
	else
		digitalWrite(PIN_RELAY, LOW);

	if (hasStarted != startHeater) {
		aSerial.vvv().println(F("Heater state change"));
		aSerial.vvv().print(F("Current temp: ")).println(temperature);
		hasStarted = startHeater;
		heaterStateStart = now();
	}

	// debug led
	if (hour() < HOUR_LED_ON_UNTIL && hour() > HOUR_LED_ON_FROM)
		handleDebugLed();

}

bool getStartHeater(int &state, float humidity, float temperature) {
	bool start = false;

	int upTime = 0;
	int downTime = 0;

	// set parameters depending on state
	switch (state) {
	case HEATER_STATE_ON:
		hasDownTimePassed = false;
		upTime = UP_TIME_SECONDS_ON;
		break;

	case HEATER_STATE_AUTO:
		upTime = UP_TIME_SECONDS_AUTO;
		downTime = DOWN_TIME_SECONDS_AUTO;
		break;

	default:
		upTime = UP_TIME_SECONDS_OFF;
		downTime = DOWN_TIME_SECONDS_OFF;
		hasStarted = false;
		hasDownTimePassed = false;
	}

	// heater should not run constantly ->
	//  for every x seconds it runs it should stop y seconds
	if (isDownTime && isTimeToSwitchState(heaterStateStart, downTime)) {
		aSerial.vvv().println(F("Up time..."));
		isDownTime = false;
		hasDownTimePassed = true;
		heaterStateStart = now();
	} else if (!isDownTime && isTimeToSwitchState(heaterStateStart, upTime)) {
		aSerial.vvv().println(F("Down time..."));
		if (state == HEATER_STATE_ON) {
			// return heater to AUTO state after START
			aSerial.vvv().println(F("Heater state set to AUTO"));
			state = heaterState;
		}
		isDownTime = true;
		heaterStateStart = now();
	}

	// if down time -> do not start heater
	if (isDownTime && !hasDownTimePassed)
		return false;

	// keep heater going once started, even if temp rises above required values
	// = avoid hysteresis
	if (hasStarted)
		return true;

	switch (state) {
	case HEATER_STATE_ON:
		start = (temperature < MAX_TEMP_START);
		break;

	case HEATER_STATE_AUTO:
		start = ((temperature < MIN_TEMP_AUTO || humidity > MAX_HUM_AUTO)
				&& isNowDalUur()) ||
				(temperature < MIN_TEMP_OFF || humidity > MAX_HUM_OFF);
		break;

	default:
		start = false;

	}

	if (hasDownTimePassed && start) {
		// force uptime if both:
		//  + heater has passed one complete downtime cycle without powering up due to low temp/hum
		//  + temperature/humidity conditions fall below required values
		hasDownTimePassed = false;
		isDownTime = false;
	}

	return start;
}

bool isTimeToSwitchState(time_t start, int seconds) {
	return (now() > start + seconds);
}

//Get starting switch state
int getSwitchState() {
	int newSwitchState;
	if (digitalRead(PIN_SWITCH_START) == LOW
			&& digitalRead(PIN_SWITCH_AUTO) == LOW) {
		newSwitchState = PIN_SWITCH_START;
	} else if (digitalRead(PIN_SWITCH_START) == LOW) {
		newSwitchState = PIN_SWITCH_AUTO;
	} else {
		newSwitchState = PIN_SWITCH_OFF;
	}
	return newSwitchState;
}

// Set heater state based on switch
int setHeaterState(int newSwitchState) {
	int newHeaterState;
	switch (newSwitchState) {
	case PIN_SWITCH_START:
		aSerial.vvv().println(F("Pin state=START"));
		newHeaterState = HEATER_STATE_ON;
		heaterStateStart = now();
		isDownTime = false;
		break;

	case PIN_SWITCH_AUTO:
		aSerial.vvv().println(F("Pin state=AUTO"));
		if (switchState == PIN_SWITCH_START) {
			// ignore switch from START to AUTO due to spring-loaded switch
			aSerial.vvv().println(F("Delayed heater state"));
			newHeaterState = HEATER_STATE_ON;
		} else {
			newHeaterState = HEATER_STATE_AUTO;
			heaterStateStart = now();
			isDownTime = true;	// do not start heater at once
		}
		break;

	default:
		aSerial.vvv().println(F("Pin state=OFF"));
		newHeaterState = HEATER_STATE_OFF;
		heaterStateStart = now();
		isDownTime = true;
	}

	return newHeaterState;
}

// https://www.engie-electrabel.be/nl/support/faq/energie/meters/daltarief
bool isNowDalUur() {

	TimeChangeRule *tcr;
	time_t t = CE.toLocal(now(), &tcr);

//	return ((dayOfWeek(t) == SATURDAY || dayOfWeek(t) == SUNDAY)
//			|| (hour(t) >= DAL_UUR_START && hour(t) < DAL_UUR_END));

	return (hour(t) >= DAL_UUR_START && hour(t + UP_TIME_SECONDS_AUTO) < DAL_UUR_END);

}

// given a Timezone object, UTC and a string description, convert and print local time with time zone
void printDateTime(time_t dateTime) {
	char buf[40];
	char m[4]; // temporary storage for month string (DateStrings.cpp uses shared buffer)
	TimeChangeRule *tcr; // pointer to the time change rule, use to get the TZ abbrev

	time_t t = CE.toLocal(dateTime, &tcr);
	strcpy(m, monthShortStr(month(t)));
	sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s", hour(t), minute(t),
			second(t), dayShortStr(weekday(t)), day(t), m, year(t),
			tcr->abbrev);
	aSerial.v().println(buf);
}

// function to return the compile date and time as a time_t value
time_t compileTime() {
	const time_t FUDGE(10); //fudge factor to allow for upload time, etc. (seconds, YMMV)
	const char *compDate = __DATE__, *compTime = __TIME__, *months =
			"JanFebMarAprMayJunJulAugSepOctNovDec";
	char compMon[3], *m;

	aSerial.v().println(F("Setting RTC clock"));

	strncpy(compMon, compDate, 3);
	compMon[3] = '\0';
	m = strstr(months, compMon);

	tmElements_t tm;
	tm.Month = ((m - months) / 3 + 1);
	tm.Day = atoi(compDate + 4);
	tm.Year = atoi(compDate + 7) - 1970;
	tm.Hour = atoi(compTime);
	tm.Minute = atoi(compTime + 3);
	tm.Second = atoi(compTime + 6);

	time_t t = CE.toUTC(makeTime(tm));
	return t + FUDGE;        //add fudge factor to allow for compile time
}

int addSwitchDelay(int state) {
	int delay;
	switch (state) {
	case PIN_SWITCH_START:
		delay = DEBOUNCE_DELAY_START;
		break;
	case PIN_SWITCH_AUTO:
		delay = DEBOUNCE_DELAY_AUTO;
		break;
	default:
		delay = DEBOUNCE_DELAY_OFF;
	}
	return delay;
}

float readTemperature() {
	int readCount = 0;
	float temperature = NAN;

	while (isnan(temperature) && readCount <= ERROR_NAN_COUNT) {
		temperature = sensor.readTemperature();
		readCount++;
	}
	if (readCount >= ERROR_NAN_COUNT) {
		// too many read attempts -> probable sensor fault
		aSerial.v().println(F("Error: failed to read temperature"));
		errorCode = ERROR_NAN;
		// disable AUTO on
		return MIN_TEMP_AUTO + 1;
	}
	// read OK
	resetErrorCode(ERROR_NAN);
	return temperature;
}

float readHumidity() {
	return 0;
}

void resetErrorCode(const int errorToReset) {
	if (errorCode == errorToReset)
		errorCode = 0;
}

void handleDebugLed() {
	int state = LOW;

	switch (errorCode) {
	case ERROR_NAN:
	case ERROR_RTC:
	case ERROR_SENSOR:
	case ERROR_STATE: {
		// blink led in case of error
		int even = second() % 2;
		if (even == 0 &&
				millis() - debugLedStart > 250) {
			state = HIGH;
			debugLedStart = millis();
		}
		break;
	}
	default:
		// led is on first minute of every hour
		if (minute() < 1)
			state = HIGH;

	}

	digitalWrite(PIN_LED, state);
}
