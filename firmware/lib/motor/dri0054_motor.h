#ifndef DRI0054_MOTOR_H
#define DRI0054_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include "motor_interface.h"

// DFRobot DRI0054 is controlled like a PCA9685 PWM expander at I2C address 0x60.
class DRI0054Motor : public MotorInterface
{
private:
    static const uint8_t I2C_ADDRESS = 0x60;
    static const uint8_t MODE1 = 0x00;
    static const uint8_t MODE2 = 0x01;
    static const uint8_t LED0_ON_L = 0x06;
    static const uint8_t PRESCALE = 0xFE;
    static const uint8_t RESTART = 0x80;
    static const uint8_t SLEEP = 0x10;
    static const uint8_t ALLCALL = 0x01;
    static const uint8_t OUTDRV = 0x04;

    static bool initialized_;
    static uint16_t pwm_max_;
    uint8_t in1_channel_;
    uint8_t in2_channel_;
    int pwm_bits_;
    float pwm_frequency_;

    static void write8(uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }

    static uint8_t read8(uint8_t reg)
    {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(I2C_ADDRESS, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0;
    }

    static void setPwm(uint8_t channel, uint16_t on, uint16_t off)
    {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(LED0_ON_L + 4 * channel);
        Wire.write(on & 0xFF);
        Wire.write(on >> 8);
        Wire.write(off & 0xFF);
        Wire.write(off >> 8);
        Wire.endTransmission();
    }

    static void setPin(uint8_t channel, bool value)
    {
        if (value) {
            setPwm(channel, 4096, 0);
        } else {
            setPwm(channel, 0, 4096);
        }
    }

    static void setPwmFrequency(float frequency)
    {
        frequency = constrain(frequency, 24.0f, 1526.0f);
        float prescale_value = 25000000.0f;
        prescale_value /= 4096.0f;
        prescale_value /= frequency;
        prescale_value -= 1.0f;
        uint8_t prescale = (uint8_t)(prescale_value + 0.5f);

        uint8_t old_mode = read8(MODE1);
        uint8_t sleep_mode = (old_mode & 0x7F) | SLEEP;
        write8(MODE1, sleep_mode);
        write8(PRESCALE, prescale);
        write8(MODE1, old_mode);
        delay(5);
        write8(MODE1, old_mode | RESTART);
    }

    static void initialize(float frequency)
    {
        if (initialized_) {
            return;
        }

        for (uint8_t channel = 0; channel < 16; channel++) {
            setPin(channel, false);
        }
        write8(MODE2, OUTDRV);
        write8(MODE1, ALLCALL);
        delay(5);
        uint8_t mode1 = read8(MODE1) & ~SLEEP;
        write8(MODE1, mode1);
        delay(5);
        setPwmFrequency(frequency);
        initialized_ = true;
    }

    static uint16_t scalePwm(int pwm, uint16_t pwm_max)
    {
        pwm = constrain(abs(pwm), 0, (int)pwm_max);
        return map(pwm, 0, pwm_max, 0, 4095);
    }

    static void mapMotorChannels(uint8_t motor_channel, uint8_t &in1, uint8_t &in2)
    {
        switch (motor_channel) {
            case 1:
                in1 = 0;
                in2 = 1;
                break;
            case 2:
                in1 = 3;
                in2 = 2;
                break;
            case 3:
                in1 = 4;
                in2 = 5;
                break;
            case 4:
                in1 = 7;
                in2 = 6;
                break;
            default:
                in1 = 0;
                in2 = 1;
                break;
        }
    }

protected:
    void forward(int pwm) override
    {
        uint16_t duty = scalePwm(pwm, pwm_max_);
        setPin(in2_channel_, false);
        setPwm(in1_channel_, 0, duty);
    }

    void reverse(int pwm) override
    {
        uint16_t duty = scalePwm(pwm, pwm_max_);
        setPin(in1_channel_, false);
        setPwm(in2_channel_, 0, duty);
    }

public:
    DRI0054Motor(float pwm_frequency, int pwm_bits, bool invert, int motor_channel, int unused=-1, int unused2=-1):
        MotorInterface(invert),
        pwm_bits_(pwm_bits),
        pwm_frequency_(pwm_frequency)
    {
        (void)unused;
        (void)unused2;
        mapMotorChannels((uint8_t)motor_channel, in1_channel_, in2_channel_);
    }

    void begin()
    {
        pwm_max_ = (1 << pwm_bits_) - 1;
        initialize(pwm_frequency_);
        brake();
    }

    void brake() override
    {
        setPin(in1_channel_, false);
        setPin(in2_channel_, false);
    }
};

bool DRI0054Motor::initialized_ = false;
uint16_t DRI0054Motor::pwm_max_ = 255;

#endif
