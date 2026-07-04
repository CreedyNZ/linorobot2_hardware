#ifndef DRI0054_MOTOR_H
#define DRI0054_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include "motor_interface.h"

class DRI0054Motor : public MotorInterface
{
public:
    DRI0054Motor(uint8_t motor_channel, bool invert)
    {
        motor_channel_ = motor_channel;
        invert_ = invert;
    }

    void begin()
    {
        Wire.begin();
        // initialise PCA9685 / DRI0054 here
    }

    void spin(int pwm)
    {
        pwm = constrain(pwm, -255, 255);

        if (invert_) {
            pwm = -pwm;
        }

        if (pwm > 0) {
            setMotorForward(abs(pwm));
        } else if (pwm < 0) {
            setMotorReverse(abs(pwm));
        } else {
            brake();
        }
    }

    void brake()
    {
        setMotorPWM(0);
    }

private:
    uint8_t motor_channel_;
    bool invert_;

    void setMotorForward(int pwm)
    {
        setMotorPWM(pwm);
        // set DRI0054 direction forward
    }

    void setMotorReverse(int pwm)
    {
        setMotorPWM(pwm);
        // set DRI0054 direction reverse
    }

    void setMotorPWM(int pwm)
    {
        // convert 0-255 to PCA9685 0-4095
        uint16_t pca_pwm = map(pwm, 0, 255, 0, 4095);

        // send I2C command to DRI0054 here
    }
};

#endif
