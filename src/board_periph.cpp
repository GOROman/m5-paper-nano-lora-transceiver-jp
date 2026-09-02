#include "board_periph.h"

#include <M5Unified.h>

static constexpr uint8_t PMIC_I2C_ADDR        = 0x6E;
static constexpr uint8_t IO_EXPANDER_I2C_ADDR = 0x4F;
static constexpr uint32_t PERIPH_I2C_FREQ_HZ  = 100000;

// RGB LED の配線 (公式デモ main/hal/hal_board.cpp と同じ)
static constexpr uint16_t RGB_PWM_FREQ_HZ = 5000;
static constexpr uint8_t RGB_GREEN_PIN    = M5IOE1_PIN_8;
static constexpr uint8_t RGB_BLUE_PIN     = M5IOE1_PIN_9;
static constexpr uint8_t RGB_GREEN_PWM_CH = M5IOE1_PWM_CH2;
static constexpr uint8_t RGB_BLUE_PWM_CH  = M5IOE1_PWM_CH1;

static M5PM1 s_pm1;
static M5IOE1 s_ioe1;
static bool s_ready = false;

M5PM1& periphPm1()
{
    return s_pm1;
}

M5IOE1& periphIoe1()
{
    return s_ioe1;
}

bool periphBegin()
{
    if (s_ready) {
        return true;
    }

    if (s_pm1.begin(&M5.In_I2C, PMIC_I2C_ADDR, PERIPH_I2C_FREQ_HZ) != M5PM1_OK) {
        Serial.println("[periph] M5PM1 begin failed");
        return false;
    }
    if (s_ioe1.begin(&M5.In_I2C, IO_EXPANDER_I2C_ADDR, PERIPH_I2C_FREQ_HZ, M5IOE1_INT_MODE_DISABLED) != M5IOE1_OK) {
        Serial.println("[periph] M5IOE1 begin failed");
        return false;
    }

    s_ioe1.pinMode(RGB_GREEN_PIN, OUTPUT);
    s_ioe1.pinMode(RGB_BLUE_PIN, OUTPUT);
    s_ioe1.setDriveMode(RGB_GREEN_PIN, M5IOE1_DRIVE_PUSHPULL);
    s_ioe1.setDriveMode(RGB_BLUE_PIN, M5IOE1_DRIVE_PUSHPULL);
    s_ioe1.setPwmFrequency(RGB_PWM_FREQ_HZ);

    s_ready = true;
    ledOff();
    return true;
}

bool ledSet(bool red_on, uint8_t green_percent, uint8_t blue_percent)
{
    if (!s_ready) {
        return false;
    }
    if (green_percent > 100) {
        green_percent = 100;
    }
    if (blue_percent > 100) {
        blue_percent = 100;
    }

    const bool red_ok   = s_pm1.setLedEnLevel(red_on) == M5PM1_OK;
    const bool green_ok = s_ioe1.setPwmDuty(RGB_GREEN_PWM_CH, green_percent, false, green_percent > 0) == M5IOE1_OK;
    const bool blue_ok  = s_ioe1.setPwmDuty(RGB_BLUE_PWM_CH, blue_percent, false, blue_percent > 0) == M5IOE1_OK;
    return red_ok && green_ok && blue_ok;
}

void ledOff()
{
    ledSet(false, 0, 0);
}
