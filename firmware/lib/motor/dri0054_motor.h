#ifndef DRI0054_MOTOR_H
#define DRI0054_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include "motor_interface.h"

#ifndef DRI0054_I2C_ADDRESS
#define DRI0054_I2C_ADDRESS 0x40
#endif

// DFRobot DRI0054 uses a PCA9685 over I2C. Each DC motor has one speed
// channel and two direction channels.
class DRI0054 : public MotorInterface
{
private:
    static const uint8_t I2C_ADDRESS = DRI0054_I2C_ADDRESS;
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
    uint8_t pwm_channel_;
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

        write8(MODE1, ALLCALL);
        write8(MODE2, OUTDRV);
        delay(5);
        write8(MODE1, read8(MODE1) & ~SLEEP);
        delay(5);
        setPwmFrequency(frequency);

        for (uint8_t channel = 0; channel < 16; channel++) {
            setPin(channel, false);
        }

        initialized_ = true;
    }

    static uint16_t scalePwm(int pwm, uint16_t pwm_max)
    {
        pwm = constrain(abs(pwm), 0, (int)pwm_max);
        return map(pwm, 0, pwm_max, 0, 4095);
    }

    static void mapMotorChannels(uint8_t motor_channel, uint8_t &pwm, uint8_t &in1, uint8_t &in2)
    {
        switch (motor_channel) {
            case 1:
                pwm = 8;
                in1 = 10;
                in2 = 9;
                break;
            case 2:
                pwm = 13;
                in1 = 11;
                in2 = 12;
                break;
            case 3:
                pwm = 2;
                in1 = 4;
                in2 = 3;
                break;
            case 4:
                pwm = 7;
                in1 = 5;
                in2 = 6;
                break;
            default:
                pwm = 8;
                in1 = 10;
                in2 = 9;
                break;
        }
    }

protected:
    void forward(int pwm) override
    {
        uint16_t duty = scalePwm(pwm, pwm_max_);
        setPin(in2_channel_, false);
        setPin(in1_channel_, true);
        setPwm(pwm_channel_, 0, duty);
    }

    void reverse(int pwm) override
    {
        uint16_t duty = scalePwm(pwm, pwm_max_);
        setPin(in1_channel_, false);
        setPin(in2_channel_, true);
        setPwm(pwm_channel_, 0, duty);
    }

public:
    DRI0054(float pwm_frequency, int pwm_bits, bool invert, int motor_channel, int unused=-1, int unused2=-1):
        MotorInterface(invert),
        pwm_bits_(pwm_bits),
        pwm_frequency_(pwm_frequency)
    {
        (void)unused;
        (void)unused2;
        mapMotorChannels((uint8_t)motor_channel, pwm_channel_, in1_channel_, in2_channel_);
    }

    void begin()
    {
        pwm_max_ = (1 << pwm_bits_) - 1;
        initialize(pwm_frequency_);
        brake();
    }

    void brake() override
    {
        setPwm(pwm_channel_, 0, 0);
        setPin(in1_channel_, false);
        setPin(in2_channel_, false);
    }
};

// Keep the old name available for any code that still references it directly.
using DRI0054Motor = DRI0054;

bool DRI0054::initialized_ = false;
uint16_t DRI0054::pwm_max_ = 255;

#endif
