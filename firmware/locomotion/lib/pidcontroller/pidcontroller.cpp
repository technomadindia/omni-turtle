#include <Arduino.h>

#include "pidcontroller.hpp"

PIDController::PIDController() {
    kp_ = 0.0;
    ki_ = 0.0;
    kd_ = 0.0;
    dead_zone_ = 0;
    prev_error_ = 0;
    error_integral_ = 0;
}

PIDController::~PIDController() {
}

void PIDController::tune(float kp, float ki, float kd, float dead_zone) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    dead_zone_ = dead_zone;
}

// PID control feedback loop
float PIDController::update(float desired, float measured, float delta_time) {
    // error
    float error = desired - measured;

    // integral
    error_integral_ += error * delta_time;

    // derivative
    float errorDerivative = (error - prev_error_) / (delta_time);

    // control signal
    float controlSignal = kp_ * error + kd_ * errorDerivative + ki_ * error_integral_;

    // store previous error
    prev_error_ = error;

    return controlSignal;
}

void PIDController::reset() {
    prev_error_ = 0;
    error_integral_ = 0;
}
