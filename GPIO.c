/* Copyright (c) Codethink Ltd. All rights reserved.
   Licensed under the MIT License. */

#include <stdbool.h>
#include <stdint.h>

#include "GPIO.h"
#include "mt3620/gpio.h"
#include "mt3620/adc.h"

static uint32_t pinToIndex(int pin)
{
    if (pin < MT3620_GPIO_ADC_BLOCK_START && pin >= 0) {
        return (pin / MT3620_GPIO_BLOCK_PIN_COUNT);
    }
    else if (pin >= MT3620_GPIO_ADC_BLOCK_START &&
        pin <= MT3620_GPIO_ADC_BLOCK_START + MT3620_GPIO_ADC_BLOCK_PIN_COUNT) {
        return MT3620_GPIO_ADC_BLOCK_INDEX;
    } else {
        return MT3620_GPIO_MAX_INDEX + 1;
    }
}

static uint32_t pinToPinMask(int pin, int index)
{
    if (pin < MT3620_GPIO_ADC_BLOCK_START) {
        return (1U << (pin - (index * MT3620_GPIO_BLOCK_PIN_COUNT) ) );
    } else {
        return (1U << (pin - (MT3620_GPIO_ADC_BLOCK_START) ) );
    }
}

static int32_t ConfigurePin(uint32_t pin, bool asInput)
{
    uint32_t index = pinToIndex(pin);
    if (index > MT3620_GPIO_MAX_INDEX) {
        return ERROR_GPIO_NOT_A_PIN;
    }

    uint32_t pinMask = pinToPinMask(pin, index);

    if (index == MT3620_GPIO_ADC_BLOCK_INDEX) {
        if (asInput == true) {
            mt3620_adc->gpio_adc_ies_set = pinMask;
        } else {
            mt3620_adc->gpio_adc_oe_set = pinMask;
        }

    } else {
        if (asInput == true) {
            mt3620_gpio[index]->gpio_pwm_grp_ies_set = pinMask;
        } else {
            mt3620_gpio[index]->gpio_pwm_grp_oe_set = pinMask;
        }
    }

    return 0;
}

int32_t GPIO_ConfigurePinForOutput(uint32_t pin)
{
    return ConfigurePin(pin, false);
}

int32_t GPIO_ConfigurePinForInput(uint32_t pin)
{
    return ConfigurePin(pin, true);
}

int32_t GPIO_Write(uint32_t pin, bool state)
{
    uint32_t index = pinToIndex(pin);
    if (index > MT3620_GPIO_MAX_INDEX) {
        return ERROR_GPIO_NOT_A_PIN;
    }

    uint32_t pinMask = pinToPinMask(pin, index);

    if (index == MT3620_GPIO_ADC_BLOCK_INDEX) {
        if (state == true) {
            mt3620_adc->gpio_adc_dout_set = pinMask;
        } else {
            mt3620_adc->gpio_adc_dout_reset = pinMask;
        }
    } else {
        if (state == true) {
            mt3620_gpio[index]->gpio_pwm_grp_dout_set = pinMask;
        } else {
            mt3620_gpio[index]->gpio_pwm_grp_dout_reset = pinMask;
        }
}

    return 0;
}

int32_t GPIO_Read(uint32_t pin, bool *state)
{
    uint32_t index = pinToIndex(pin);
    if (index > MT3620_GPIO_MAX_INDEX) {
        return ERROR_GPIO_NOT_A_PIN;
    }

    uint32_t pinMask = pinToPinMask(pin, index);

    if (index == MT3620_GPIO_ADC_BLOCK_INDEX) {
        *state = pinMask & mt3620_adc->gpio_adc_din;
    } else {
        *state = pinMask & mt3620_gpio[index]->gpio_pwm_grp_din;
    }

    return 0;
}

#define PWM_MAX_DUTY_CYCLE 65535
#define PWM_CLOCK_SEL_DEADZONE 5

int32_t PWM_ConfigurePin(uint32_t pin, uint32_t clockFrequency, uint32_t onTime, uint32_t offTime)
{
    uint32_t index = pinToIndex(pin);

    if (index > MT3620_PWM_MAX_INDEX) {
        return ERROR_PWM_NOT_A_PIN;
    }

    uint32_t pinMask = pinToPinMask(pin, index);

    if (onTime > PWM_MAX_DUTY_CYCLE || offTime > PWM_MAX_DUTY_CYCLE) {
        return ERROR_PWM_UNSUPPORTED_DUTY_CYCLE;
    }

    uint8_t clockSel;
    unsigned frequencyLow  = (clockFrequency * (100ULL - PWM_CLOCK_SEL_DEADZONE)) / 100U;
    unsigned frequencyHigh = (clockFrequency * (100ULL + PWM_CLOCK_SEL_DEADZONE)) / 100U;

    if (frequencyLow <= MT3620_PWM_32k && frequencyHigh >= MT3620_PWM_32k ) {
        clockSel = MT3620_PWM_CLK_SEL_32K;
    }
    else if (frequencyLow <= MT3620_PWM_2M && frequencyHigh >= MT3620_PWM_2M) {
        clockSel = MT3620_PWM_CLK_SEL_2M;
    }
    else if (frequencyLow <= MT3620_PWM_XTAL && frequencyHigh >= MT3620_PWM_XTAL) {
        clockSel = MT3620_PWM_CLK_SEL_XTAL;
    } else {
        return ERROR_PWM_UNSUPPORTED_CLOCK_SEL;
    }

    //Write default values of the registers before starting as recommended in the datasheet
    mt3620_pwm[index]->pwm_glo_ctrl = MT3620_PWM_GLO_CTRL_DEF;
    //Switch statement starts from 1 since pinMask = pwm index + 1
    switch(pinMask) {
        case 1:
            mt3620_pwm[index]->pwm0_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[index]->pwm0_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[index]->pwm0_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        case 2:
            mt3620_pwm[index]->pwm1_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[index]->pwm1_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[index]->pwm1_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        case 4:
            mt3620_pwm[index]->pwm2_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[index]->pwm2_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[index]->pwm2_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        case 8:
            mt3620_pwm[index]->pwm3_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[index]->pwm3_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[index]->pwm3_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        default:
            break;
    }

    MT3620_PWM_FIELD_WRITE(index, pwm_glo_ctrl, pwm_tick_clock_sel, clockSel);

    uint32_t pwm_param = ((offTime << 16) | onTime);
    switch (pinMask) {
        case 1:
            MT3620_PWM_FIELD_WRITE(index, pwm0_ctrl, pwm_clock_en, 1);
            mt3620_pwm[index]->pwm0_param_s0 = pwm_param;
            mt3620_pwm[index]->pwm0_param_s1 = 0;
            mt3620_pwm0_ctrl_t pwm0_ctrl = {.mask = mt3620_pwm[index]->pwm0_ctrl};
            pwm0_ctrl.S0_stay_cycle = 1;
            pwm0_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[index]->pwm0_ctrl = pwm0_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(index, pwm0_ctrl, kick, 1);
            break;
        case 2:
            MT3620_PWM_FIELD_WRITE(index, pwm1_ctrl, pwm_clock_en, 1);
            mt3620_pwm[index]->pwm1_param_s0 = pwm_param;
            mt3620_pwm[index]->pwm1_param_s1 = 0;
            mt3620_pwm1_ctrl_t pwm1_ctrl = { .mask = mt3620_pwm[index]->pwm1_ctrl };
            pwm1_ctrl.S0_stay_cycle = 1;
            pwm1_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[index]->pwm1_ctrl = pwm1_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(index, pwm1_ctrl, kick, 1);
            break;
        case 4:
            MT3620_PWM_FIELD_WRITE(index, pwm2_ctrl, pwm_clock_en, 1);
            mt3620_pwm[index]->pwm2_param_s0 = pwm_param;
            mt3620_pwm[index]->pwm2_param_s1 = 0;
            mt3620_pwm2_ctrl_t pwm2_ctrl = { .mask = mt3620_pwm[index]->pwm2_ctrl };
            pwm2_ctrl.S0_stay_cycle = 1;
            pwm2_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[index]->pwm2_ctrl = pwm2_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(index, pwm2_ctrl, kick, 1);
            break;
        case 8:
            MT3620_PWM_FIELD_WRITE(index, pwm3_ctrl, pwm_clock_en, 1);
            mt3620_pwm[index]->pwm3_param_s0 = pwm_param;
            mt3620_pwm[index]->pwm3_param_s1 = 0;
            mt3620_pwm3_ctrl_t pwm3_ctrl = { .mask = mt3620_pwm[index]->pwm3_ctrl };
            pwm3_ctrl.S0_stay_cycle = 1;
            pwm3_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[index]->pwm3_ctrl = pwm3_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(index, pwm3_ctrl, kick, 1);
            break;
        default:
            break;
    }

    return ERROR_NONE;
}
