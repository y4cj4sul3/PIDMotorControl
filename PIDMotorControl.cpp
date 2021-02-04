/*
    PIDMotorController.cpp - Library for controlling DC motor by PID control.
    Created by y4cj4sul3, February, 4, 2021.
    Released into the public domain.
*/

#include <Encoder.h>
#include "PIDMotorControl.h"

PIDMotorController::PIDMotorController(uint8_t pinIN1, uint8_t pinIN2, uint8_t pinPWM, uint8_t pinSTBY, uint8_t pinENA, uint8_t pinENB, float unitConv) //: _pinIN1(pinIN2), _pinIN2(pinIN2), _pinPWM(pinPWM), _pinSTBY(pinSTBY), _pinENA(pinENA), _pinENB(pinENB)
{
    // set pin mode
    _pinIN1 = pinIN1;
    _pinIN2 = pinIN2;
    _pinPWM = pinPWM;
    _pinSTBY = pinSTBY;

    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN1, OUTPUT);
    pinMode(_pinPWM, OUTPUT);
    pinMode(_pinSTBY, OUTPUT);

    // encoder
    _enc = new Encoder(pinENA, pinENB);

    // unit convertion
    // encoder count -> user defined displacement unit
    // e.g. cpr (encoder count / motor revolution) * gearRatio (motor revolution / gearbox revolution) * user defined displace unit (unit / gearbox revolution)
    _unitConv = unitConv;

    // time
    _prevTime = micros();
}

void PIDMotorController::move(float displacement)
{
    // rotate motor to move certain displacement (unit: user defined unit)
    rotateCount(displacement * _unitConv);
}

void PIDMotorController::rotateCount(long countToMove)
{
    // rotate motor with certain count (unit encoder count)
    long targetCount = _curCount + countToMove;
    setTarget(targetCount);
}

void PIDMotorController::setTarget(long targetCount)
{
    _targetCount = targetCount;
    _integral = 0;
    _prevError = 0;
}

int PIDMotorController::_computePID()
{
    float P, I, D;

    // Proportional
    long error = _targetCount - _curCount;
    P = _kp * error;

    // Integral
    unsigned long deltaTime = _getDeltaTime();
    _integral = _integral * _integralDecay + error * deltaTime;
    I = _ki * _integral;

    // Derivative
    D = _kd * (error - _prevError) / deltaTime;
    _prevError = error;

    // derive driving voltage
    long voltageFeed = P + I + D;
    voltageFeed = max(min(voltageFeed, 255), -255);

    return voltageFeed;
}

void PIDMotorController::update()
{
    // observe current position
    _readEncoder();
    // actions to revise position
    int voltageFeed = _computePID();
    // execute action
    drive(voltageFeed);
}

/* Encoder */
void PIDMotorController::_readEncoder()
{
    // read encoder
    _curCount = _enc->read();
}

float PIDMotorController::_getDeltaTime()
{
    unsigned long curTime = micros();
    float deltaTime;
    if (curTime >= _prevTime)
    {
        deltaTime = (curTime - _prevTime) * 0.000001;
    }
    else
    {
        // overflow
        unsigned long maxVal = -1;
        deltaTime = (maxVal - _prevTime + curTime) * 0.000001;
    }
    _prevTime = curTime;

    // return delta time (second)
    return deltaTime;
}

/* Basic Motor Driver Functions */
void PIDMotorController::drive(int speed)
{
    // drive motor in certain speed and direction

    // stop if speed is zero
    if (speed == 0)
    {
        stop();
        return;
    }

    // set motor is true
    _isActive = true;
    // motor rotate direction
    bool pinIN1, pinIN2;
    if (speed > 0)
    {
        // clockwise
        pinIN1 = HIGH;
        pinIN2 = LOW;
    }
    else
    {
        // counter-colckwise
        pinIN1 = LOW;
        pinIN2 = HIGH;
    }
    // clip speed (duty cycle, voltage)
    speed = max(min(abs(speed), 255), 0);
    // set the direction and the voltage
    digitalWrite(_pinIN1, pinIN1);
    digitalWrite(_pinIN2, pinIN2);
    analogWrite(_pinPWM, speed);

    // Finally , make sure STBY is disabled - pull it HIGH
    digitalWrite(_pinSTBY, HIGH);
}

void PIDMotorController::brake()
{
    // short brake
    analogWrite(_pinPWM, 0);
}

void PIDMotorController::stop()
{
    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, LOW);

    // try standby, just for saving power
    _isActive = false;
    standBy();
}

void PIDMotorController::standBy()
{
    // TODO: use DualMotorDriver
    digitalWrite(_pinSTBY, LOW);
}

void PIDMotorController::reset()
{
    _targetCount = _curCount;
    _integral = 0;
    _prevError = 0;
    _isActive = false;
}

bool PIDMotorController::checkActive()
{
    return _isActive;
}
