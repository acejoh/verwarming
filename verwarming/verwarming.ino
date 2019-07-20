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

Adafruit_AM2315 sensor;

const int PIN_RELAY = 8;
const int PIN_SWITCH_OFF = 1; // not used
const int PIN_SWITCH_START = 9;
const int PIN_SWITCH_AUTO = 10;

const int HEATER_STATE_OFF = 0;
const int HEATER_STATE_ON = 1;
const int HEATER_STATE_AUTO = 2;

TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 60 }; // Central European Summer Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 0 }; // Central European Standard Time
Timezone CE(CEST, CET);

const int SATURDAY = 7;
const int SUNDAY = 1;
const int DAL_UUR_START = 21;
const int DAL_UUR_END = 6;

const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long DEBOUNCE_DELAY_START = 400;
const unsigned long DEBOUNCE_DELAY_AUTO = 750;
const unsigned long DEBOUNCE_DELAY_OFF = 750;
unsigned long debounceTimer;

int switchState;
int heaterState;

const int DELAY_START_SECONDS = 1;
const int DOWN_TIME_SECONDS_OFF = 60; //60 * 60;
const int DOWN_TIME_SECONDS_AUTO = 60; //60 * 60;
const int UP_TIME_SECONDS_OFF = 1 * 60;
const int UP_TIME_SECONDS_AUTO = 3 * 60;
const int UP_TIME_SECONDS_ON = 1 * 60;
time_t heaterStateStart;
bool isDownTime = false;
bool hasStarted = false;
bool hasDownTimePassed = false;

const float MIN_TEMP_OFF = 5;
const float MAX_HUM_OFF = 999;
const float MIN_TEMP_AUTO = 26;
const float MAX_HUM_AUTO = 999;
const float MAX_TEMP_START = 30;

unsigned long debugLogTimer;

void setup() {
	Serial.begin(9600);
	Serial.println("Verwarming Test!");

	// Start up the sensor
	if (!sensor.begin()) {
		Serial.println("Sensor not found, check wiring & pullups!");
	}

	// set the time
	RTC.set(compileTime());	// COMMENT THIS LINE ONCE IN PRODUCTION
	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if (timeStatus() != timeSet)
		Serial.println("Unable to sync with the RTC");
	else
		Serial.println("RTC has set the system time");
	printDateTime(now());

	// init pins
	pinMode(PIN_SWITCH_START, INPUT_PULLUP);
	pinMode(PIN_SWITCH_AUTO, INPUT_PULLUP);

	digitalWrite(PIN_RELAY, LOW);
	pinMode(PIN_RELAY, OUTPUT);

	digitalWrite(LED_BUILTIN, LOW);
	pinMode(LED_BUILTIN, OUTPUT);

	// get initial state
	switchState = getSwitchState();
	heaterState = setHeaterState(switchState);

	Serial.println("Start");
	delay(1000);
}

void loop() {

	float humidity = sensor.readHumidity();
	float temperature = sensor.readTemperature();
	if (millis() - 5000 > debugLogTimer) {
		//Serial.print("Hum: ");
		//Serial.println(humidity);
		Serial.print("Temp: ");
		Serial.println(temperature);

		Serial.print("Current state: ");
		Serial.println(heaterState);

		Serial.print("Current cycle: ");
		if (isDownTime)
			Serial.println("DOWN");
		else
			Serial.println("UP");

		Serial.print(" - runtime: ");
		time_t running = now() - heaterStateStart;
		Serial.print(minute(running));
		Serial.print(':');
		Serial.println(second(now() - heaterStateStart));

		debugLogTimer = millis();
	}

	// check for switch state changes
	// 1. Debounce
	int reading = getSwitchState();
	if (reading != switchState) {
		// debounce switch
		Serial.println("Debounce start");
		debounceTimer = millis();
		debounceTimer += addSwitchDelay(reading);
		while (millis() < debounceTimer + DEBOUNCE_DELAY);
		reading = getSwitchState();
	}

	// 2. perform state change
	if (reading != switchState) {
		// switch to new state
		Serial.println("Switching state");
		heaterState = setHeaterState(reading);
		switchState = reading;
	}

	// determine if heater should start or not
	bool startHeater = getStartHeater(heaterState, humidity, temperature);
	// relay logic
	if (startHeater)
		digitalWrite(PIN_RELAY, HIGH);
	else
		digitalWrite(PIN_RELAY, LOW);

	if (hasStarted != startHeater) {
		Serial.println("Heater state change");
		hasStarted = startHeater;
		heaterStateStart = now();
	}

}

bool getStartHeater(int state, float humidity, float temperature) {

	bool start = false;

	int upTime = 0;
	int downTime = 0;

	// set parameters depending on state
	switch (state) {
	case HEATER_STATE_OFF:
		upTime = UP_TIME_SECONDS_OFF;
		downTime = DOWN_TIME_SECONDS_OFF;
		hasStarted = false;
		hasDownTimePassed = false;
		break;
	case HEATER_STATE_ON:
		upTime = UP_TIME_SECONDS_ON;
		break;
	default:
		upTime = UP_TIME_SECONDS_AUTO;
		downTime = DOWN_TIME_SECONDS_AUTO;
	}

	// heater should not run constantly ->
	//  for every x seconds it runs it should stop y seconds
	if (isDownTime && isTimeToSwitchState(heaterStateStart, downTime)) {
		Serial.println("Up time...");
		isDownTime = false;
		hasDownTimePassed = true;
		heaterStateStart = now();
	} else if (!isDownTime && isTimeToSwitchState(heaterStateStart, upTime)) {
		Serial.println("Down time...");
		if (state == HEATER_STATE_ON) {
			// return heater to AUTO state after START
			Serial.println("Heater state set to AUTO");
			heaterState = HEATER_STATE_AUTO;
			state = heaterState;
		}
		isDownTime = true;
		heaterStateStart = now();
	}

	// if down time -> do not start heater
	if (isDownTime && !hasDownTimePassed)
		return false;

	// keep heater going once started
	if (hasStarted)
		return true;

	switch (state) {
	case HEATER_STATE_ON:
		start = (temperature < MAX_TEMP_START);
		break;

	case HEATER_STATE_AUTO:
		start = ((temperature < MIN_TEMP_AUTO || humidity > MAX_HUM_AUTO)
				&& isNowDalUur());
		break;

	default:
		start = (temperature < MIN_TEMP_OFF || humidity > MAX_HUM_OFF);

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

	int newHeaterState = 0;
	//delayedAuto = false;
	switch (newSwitchState) {
	case PIN_SWITCH_START:
		Serial.println("Pin state=START");
		newHeaterState = HEATER_STATE_ON;
		heaterStateStart = now();
		isDownTime = false;
		break;
	case PIN_SWITCH_AUTO:
		Serial.println("Pin state=AUTO");
		if (switchState == PIN_SWITCH_START) {
			// ignore switch from START to AUTO due to spring-loaded switch
			Serial.println("Delayed heater state");
			newHeaterState = HEATER_STATE_ON;
			isDownTime = true;
		} else {
			newHeaterState = HEATER_STATE_AUTO;
			heaterStateStart = now();
		}
		isDownTime = true;
		break;
	default:
		Serial.println("Pin state=OFF");
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

	return ((dayOfWeek(t) == SATURDAY || dayOfWeek(t) == SUNDAY)
			|| (hour(t) >= DAL_UUR_START && hour(t) < DAL_UUR_END));

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
	Serial.println(buf);
}

// function to return the compile date and time as a time_t value
time_t compileTime() {
	const time_t FUDGE(10); //fudge factor to allow for upload time, etc. (seconds, YMMV)
	const char *compDate = __DATE__, *compTime = __TIME__, *months =
			"JanFebMarAprMayJunJulAugSepOctNovDec";
	char compMon[3], *m;

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

	int delay = 0;
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
