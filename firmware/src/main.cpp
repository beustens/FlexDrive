#include <Arduino.h>
#include <SimpleFOC.h>

#define ADC_PIN 4
#define DIGI_PIN 10

const int polePairs = 2; // Set for your motor
const float supplyVoltage = 24.0;
const float motorVoltage = 16.0; // Can be changed via Commander ("MLU<voltage>\n")

// Firmware version
typedef struct {
	uint8_t major;
	uint8_t minor;
} FWVersion;
FWVersion fwVer = {1, 0};

// Motor instance
BLDCMotor motor(polePairs);

// Driver instance
// AH, AL, BH, BL, CH, CL
/*
Important!!! Fix .pio/libdeps/esp32-c3-devkitm-1/Simple FOC/src/drivers/hardware_specific/esp32/esp32_ledc_mcu.cpp
by moving `group_channels_used[group]++;` after `params->groups[...;`
*/
BLDCDriver6PWM driver(9, 3, 8, 1, 2, 0);

// Hall sensor instance
// A, B, C, pole pairs
HallSensor sensor(5, 6, 7, polePairs);

// (Optional) interrupt routine initialization
// void doA() { sensor.handleA(); Serial.println("Hall A"); }
// void doB() { sensor.handleB(); Serial.println("Hall B"); }
// void doC() { sensor.handleC(); Serial.println("Hall C"); }

// Commander interface
/*
E.g. setup closed loop velocity mode, spin with 10 rad/s and output current angle:
"MC1\n"
"M10\n"
"MMS0000001\n"
*/
Commander command(Serial);
void doTarget(char* cmd) { command.motor(&motor, cmd); }

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

	// Hall Sensor setup
	// sensor.pullup = Pullup::USE_INTERN;
	sensor.init();
	// sensor.enableInterrupts(doA, doB, doC);

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

	Serial.printf("FlexDrive V%d.%d\n", fwVer.major, fwVer.minor);
	Serial.println("Commands: https://docs.simplefoc.com/commander_motor");
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