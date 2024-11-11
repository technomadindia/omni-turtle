#include <Arduino.h>

#include "drv8833.hpp"

void Drv8833::init_motor(int motor_id, int pin1, int pin2) {
    if (1 == motor_id) {
        motor1_pin1_ = pin1;
        motor1_pin2_ = pin2;
    } else if (2 == motor_id) {
        motor2_pin1_ = pin1;
        motor2_pin2_ = pin2;
    }
}

void Drv8833::init_control(int sleep_pin, int fault_pin) {
    Drv8833::sleep_pin_ = sleep_pin;
    Drv8833::fault_pin_ = fault_pin;
}

void Drv8833::set_motor(int motor_id, int direction, int power) {
    // select motor pins to actuate
    int pin1, pin2;
    if (1 == motor_id) {
        pin1 = motor1_pin1_;
        pin2 = motor1_pin2_;
    } else if (2 == motor_id) {
        pin1 = motor2_pin1_;
        pin2 = motor2_pin2_;
    } else {
        return;
    }

    // motor power clipped to 8-bit PWM range
    if (power > 255) {
        power = 255;
    } else if (power < 0) {
        power = 0;
    }

    // set motor polarity and pwm dutycycle
    if (direction == 1) { // CW rotation
        analogWrite(pin1, power);
        digitalWrite(pin2, LOW);
    } else if (direction == -1) { // CCW rotation
        digitalWrite(pin1, LOW);
        analogWrite(pin2, power);
    } else { // braking
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, LOW);
    }
}

void Drv8833::sleep() {
    digitalWrite(Drv8833::sleep_pin_, HIGH);
}

void Drv8833::wake() {
    digitalWrite(Drv8833::sleep_pin_, LOW);
}
