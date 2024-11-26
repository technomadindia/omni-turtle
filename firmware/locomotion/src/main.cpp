#include <Arduino.h>
#include <drv8833.hpp>
#include <pidcontroller.hpp>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// #define _DEBUG

// N20 motor pin connections
#define MOTOR_ID 1
#define ENC1_PIN 3
#define ENC2_PIN 4
#define MOTOR_PIN1 5
#define MOTOR_PIN2 6
#define SLEEP_PIN 7
#define FAULT_PIN 2

// PID constants for N20 motor
const float KP = 1.0;
const float KI = 0.05;
const float KD = 0.1;
const float DZ = 0.0;

Drv8833 driver;
PIDController controller;

// specify interrupt variable as volatile
volatile int g_positionInterrupt = 0;

// Encoder interrupt routine
void readEncoder() { // on rising edge of Encode 1
    int encBState = digitalRead(ENC2_PIN);
    if (encBState > 0) { // if Encoder 2 is ahead implies CW rotation
        g_positionInterrupt++;
    } else { // if Encoder 2 is lagging implies CCW rotation
        g_positionInterrupt--;
    }
}

void setup() {
    // put your setup code here, to run once:
#if _DEBUG
    Serial.begin(9600);
#endif

    // set motor driver configuration
    driver.config_control(SLEEP_PIN, FAULT_PIN);
    driver.config_motor(MOTOR_ID, MOTOR_PIN1, MOTOR_PIN2);
    // driver.sleep();

    // set encoder configuration
    pinMode(ENC1_PIN, INPUT);
    pinMode(ENC2_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC1_PIN), readEncoder, RISING);

    controller.tune(KP, KI, KD);
}

// Timing loop state
long prevTime = 0;

void loop() {
    // put your main code here, to run repeatedly:

    // set target position
    int targetPosition = 350 * sin(prevTime / 1e6);

    // time difference
    long currTime = micros();
    float deltaTime = ((float)(currTime - prevTime)) / (1.0e6);
    prevTime = currTime;

    // Read the position in an atomic block to avoid a potential
    // misread if the interrupt coincides with this code running
    int currentPosition = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition = g_positionInterrupt;
    }

    // update feedback control loop
    float controlSignal = controller.update(targetPosition, currentPosition, deltaTime);

    driver.set_motor(MOTOR_ID, (controlSignal >= 0.0) ? 1 : -1, (int)fabs(controlSignal));

#if _DEBUG
    // debug: Teleplot monitoring
    Serial.print(">targetPosition:");
    Serial.println(targetPosition);
    // Serial.print(" ");
    Serial.print(">currentPosition:");
    Serial.println(currentPosition);
    // Serial.print(" ");
    Serial.print(">motorPower:");
    Serial.println(motorPower);
#endif
}
