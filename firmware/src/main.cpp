#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h> // ESP32 non-volatile storage (NVS, in flash)

#define ADC_PIN 4
#define DIGI_PIN 10

const int defaultPolePairs = 2; // Used until a value was saved, can be changed via Commander ("P<pole pairs>\n")
const int maxPolePairs = 64; // Upper sanity limit for the Commander command

// Persistent settings (NVS)
const char* const prefsNamespace = "flexdrive";
const char* const prefsKeyPolePairs = "pp";
const float supplyVoltage = 24.0;
const float motorVoltage = 16.0; // Can be changed via Commander ("MLU<voltage>\n")

// Firmware version
typedef struct {
	uint8_t major;
	uint8_t minor;
} FWVersion;
FWVersion fwVer = {1, 1};

// Motor instance
BLDCMotor motor(defaultPolePairs);

// Driver instance
// AH, AL, BH, BL, CH, CL
/*
Important!!! Fix .pio/libdeps/esp32-c3-devkitm-1/Simple FOC/src/drivers/hardware_specific/esp32/esp32_ledc_mcu.cpp
by moving `group_channels_used[group]++;` after `params->groups[...;`
*/
BLDCDriver6PWM driver(9, 3, 8, 1, 2, 0);

// Hall sensor instance
// A, B, C, pole pairs
HallSensor sensor(5, 6, 7, defaultPolePairs);

// Change the pole pairs (better not call when motor is running!)
bool setPolePairs(int pp) {
	if (pp < 1 || pp > maxPolePairs) return false;
	if (pp == motor.pole_pairs) return true;

	// Update both users of the pole pairs together
	motor.pole_pairs = pp;
	sensor.cpr = pp * 6;

	return true;
}

// Load the saved pole pairs, falls back to the default if nothing (valid) was saved yet
int loadPolePairs() {
	Preferences prefs;
	int pp = defaultPolePairs;
	if (prefs.begin(prefsNamespace, true)) { // read-only
		pp = prefs.getUChar(prefsKeyPolePairs, defaultPolePairs);
		prefs.end();
	}
	if (pp < 1 || pp > maxPolePairs) pp = defaultPolePairs; // corrupt / out of range
	return pp;
}

// Save the pole pairs to flash, skips the write if the stored value is already identical (flash wear)
bool savePolePairs(int pp) {
	Preferences prefs;
	if (!prefs.begin(prefsNamespace, false)) return false;
	bool ok = true;
	if (prefs.getUChar(prefsKeyPolePairs, 0) != pp) {
		ok = prefs.putUChar(prefsKeyPolePairs, pp) == sizeof(uint8_t);
	}
	prefs.end();
	return ok;
}

// Commander interface
/*
E.g. setup closed loop velocity mode, spin with 10 rad/s and output current angle:
"MC1\n"
"M10\n"
"MMS0000001\n"

Pole pairs (saved to flash, restored at boot):
"P\n"  - print the current pole pairs
"P7\n"  - set 7 pole pairs
*/
Commander command(Serial);
void doTarget(char* cmd) { command.motor(&motor, cmd); }
void doPolePairs(char* cmd) {
	if (command.isSentinel(cmd[0])) {
		// Query only
		Serial.printf("Pole pairs: %d\n", motor.pole_pairs);
		return;
	}
	int pp = atoi(cmd);
	if (setPolePairs(pp)) {
		bool saved = savePolePairs(motor.pole_pairs);
		Serial.printf("Pole pairs: %d%s\n", motor.pole_pairs, saved ? " (saved)" : " (saving failed!)");
	} else {
		Serial.printf("Invalid pole pairs, allowed: 1...%d\n", maxPolePairs);
	}
}

// Analog input
const uint16_t adcThresh = 20;

// PWM pulse measurement
volatile uint32_t pulseStart = 0;
volatile uint32_t pulseWidth = 0;

void IRAM_ATTR handlePWM() {
	uint32_t currentTime = micros();

	if (digitalRead(DIGI_PIN) == HIGH) {
		// Pulse started
		pulseStart = currentTime;
	} else {
		// Pulse ended
		if (pulseStart != 0) {
			pulseWidth = currentTime - pulseStart;
		}
	}
}

void setup() {
	Serial.begin(115200);

	// GPIO
	pinMode(DIGI_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(DIGI_PIN), handlePWM, CHANGE);

	// Monitor
	motor.useMonitoring(Serial);
	motor.monitor_variables = 0; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE
	motor.monitor_downsample = 1000; // default 10
	SimpleFOCDebug::enable(NULL); // uncomment to show details while setting up

	// Restore the saved pole pairs. The motor and sensor objects were constructed with the default,
	// and nothing is initialized yet, so both values can simply be assigned here
	int savedPolePairs = loadPolePairs();
	setPolePairs(savedPolePairs);

	// Hall Sensor setup
	// sensor.pullup = Pullup::USE_INTERN;
	sensor.init();

	// Driver & Motor setup
	driver.voltage_power_supply = supplyVoltage;
	driver.init();

	motor.torque_controller = TorqueControlType::voltage; // set the torque control type
	motor.controller = MotionControlType::torque;
	// motor.controller = MotionControlType::velocity;
	motor.updateVoltageLimit(motorVoltage); // for open loop, keep very low (2...6 V)!
	motor.voltage_sensor_align = 3.0;
	motor.velocity_limit = 20.0; // for position control
	// For closed loop operation
	motor.PID_velocity.P = 0.1;
	motor.PID_velocity.I = 0.2;
	// motor.LPF_velocity.Tf = 0.02;
	// motor.P_angle.P = 20;

	motor.linkSensor(&sensor);
	motor.linkDriver(&driver);

	motor.init();
	motor.initFOC();

	// Commander Setup
	command.add('M', doTarget, "motor control");
	command.add('P', doPolePairs, "pole pairs");

	Serial.printf("FlexDrive V%d.%d, pole pairs: %d\n", fwVer.major, fwVer.minor, motor.pole_pairs);
	Serial.println("Commands: https://docs.simplefoc.com/commander_motor");
	Serial.println("Pole pairs: P<1...64>");
}

void loop() {
	// Iterative FOC function
	motor.loopFOC();

	// Iterative function setting and calculating the angle/position loop
	// This function can be run at much lower frequency than loopFOC function
	motor.move();

	// Commander interface with the user
	command.run();

	// Output configured variable values
	motor.monitor();

	// Use hardware interface
	static uint32_t oldTime = 0;
	uint32_t newTime = millis();
	if (newTime - oldTime > 50) {
		oldTime = newTime;

		float duty;
		uint16_t adcVal = analogRead(ADC_PIN);
		if (pulseWidth >= 1000 && pulseWidth <= 2000) {
			// PWM
			duty = (static_cast<float>(pulseWidth) - 1500.0) / 500.0;
			// Serial.printf("PWM: %d\n", pulseWidth);
			motor.target = motor.voltage_limit * duty;
		} else if (adcVal >= adcThresh) {
			// Analog
			// Serial.printf("ADC: %d\n", adcVal);
			duty = static_cast<float>(adcVal - adcThresh) / (4095.0 - adcThresh);
			if (digitalRead(DIGI_PIN) == LOW) duty = -duty;
			motor.target = motor.voltage_limit * duty;
		}
	}
}
