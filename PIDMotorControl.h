/*
    PIDMotorController.h - Library for controlling DC motor by PID control.
    Created by y4cj4sul3, February, 4, 2021.
    Released into the public domain.
*/

#ifndef PIDMotorController_h
#define PIDMotorController_h

#include <Arduino.h>
#include <Encoder.h>

class PIDMotorController {
    public:
        PIDMotorController(uint8_t pinIN1, uint8_t pinIN2, uint8_t pinPWM, uint8_t pinSTBY, uint8_t pinENA, uint8_t pinENB, float unitConv);
        
        void move(float displacement);
        void rotateCount(long countToMove);
        void setTarget(long targetCount);

        void update();

        void drive(int speed);
        void brake();
        void stop();
        void standBy();

        void reset();

        bool checkActive();


    private:
        // encoder
        Encoder* _enc;

        // pins 
        uint8_t _pinIN1;
        uint8_t _pinIN2;
        uint8_t _pinPWM;
        uint8_t _pinSTBY;

        // unit convertion
        // encoder count -> user defined displacement unit
        float _unitConv;

        // motor revolution (in encoder count)
        long _curCount = 0;
        long _targetCount = 0;

        // PID parameters
        float _kp;
        float _ki;
        float _kd;
        long _integral = 0;
        long _prevError = 0;
        float _integralDecay = 1;

        bool _isActive = false;

        unsigned long _prevTime;

        int _computePID();
        void _readEncoder();
        float _getDeltaTime();

};

#endif