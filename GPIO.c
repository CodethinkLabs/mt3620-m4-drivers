/* Copyright (c) Codethink Ltd. All rights reserved.
   Licensed under the MIT License. */

#include <stdbool.h>
#include <stdint.h>

#include "GPIO.h"
#include "mt3620/gpio.h"
#include "mt3620/adc.h"

static mt3620_gpio_block_e pinToBlock(uint32_t pin)
{
    if (pin > MT3620_GPIO_COUNT) {
        return MT3620_GPIO_BLOCK_NOT_MAPPED;
    }
    return mt3620_gpioPinMap[pin];
}

static uint32_t getPinMask(uint32_t pin, mt3620_gpio_block_e block)
{
    if (block > MT3620_GPIO_BLOCK_NOT_MAPPED) {
        return 0U;
    }

    return (1U << (pin - mt3620_gpioBlockStart[block]));
}

static int32_t ConfigurePin(uint32_t pin, bool asInput)
{
    mt3620_gpio_block_e block = pinToBlock(pin);
    if (block >= MT3620_GPIO_BLOCK_NOT_MAPPED) {
        return ERROR_GPIO_NOT_A_PIN;
    }

    uint32_t pinMask = getPinMask(pin, block);

    if (asInput) {
        mt3620_gpio[block]->gpio_pwm_grp_ies_set  = pinMask;
        mt3620_gpio[block]->gpio_pwm_grp_oe_reset = pinMask;
    } else {
        mt3620_gpio[block]->gpio_pwm_grp_oe_set    = pinMask;
        mt3620_gpio[block]->gpio_pwm_grp_ies_reset = pinMask;
    }

    return ERROR_NONE;
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
    mt3620_gpio_block_e block = pinToBlock(pin);
    if (block >= MT3620_GPIO_BLOCK_NOT_MAPPED) {
        return ERROR_GPIO_NOT_A_PIN;
    }

    uint32_t pinMask = getPinMask(pin, block);

    if (state) {
        mt3620_gpio[block]->gpio_pwm_grp_dout_set   = pinMask;
    } else {
        mt3620_gpio[block]->gpio_pwm_grp_dout_reset = pinMask;
    }

    return ERROR_NONE;
}

int32_t GPIO_Read(uint32_t pin, bool *state)
{
    mt3620_gpio_block_e block = pinToBlock(pin);
    if (block >= MT3620_GPIO_BLOCK_NOT_MAPPED) {
        return ERROR_GPIO_NOT_A_PIN;
    }

    uint32_t pinMask = getPinMask(pin, block);

    *state = pinMask & mt3620_gpio[block]->gpio_pwm_grp_din;

    return ERROR_NONE;
}

#define PWM_MAX_DUTY_CYCLE 65535
#define PWM_CLOCK_SEL_DEADZONE 5

int32_t PWM_ConfigurePin(uint32_t pin, uint32_t clockFrequency, uint32_t onTime, uint32_t offTime)
{
    mt3620_gpio_block_e block = pinToBlock(pin);

    if ((block < MT3620_GPIO_BLOCK_0) || (block > MT3620_GPIO_BLOCK_2)) {
        return ERROR_PWM_NOT_A_PIN;
    }

    uint32_t pinMask  = getPinMask(pin, block);
    uint32_t pwmBlock = block - MT3620_GPIO_BLOCK_0;

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
    mt3620_pwm[pwmBlock]->pwm_glo_ctrl = MT3620_PWM_GLO_CTRL_DEF;
    //Switch statement starts from 1 since pinMask = pwm pwmBlock + 1
    switch(pinMask) {
        case 1:
            mt3620_pwm[pwmBlock]->pwm0_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[pwmBlock]->pwm0_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[pwmBlock]->pwm0_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        case 2:
            mt3620_pwm[pwmBlock]->pwm1_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[pwmBlock]->pwm1_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[pwmBlock]->pwm1_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        case 4:
            mt3620_pwm[pwmBlock]->pwm2_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[pwmBlock]->pwm2_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[pwmBlock]->pwm2_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        case 8:
            mt3620_pwm[pwmBlock]->pwm3_ctrl = MT3620_PWM_CTRL_DEF;
            mt3620_pwm[pwmBlock]->pwm3_param_s0 = MT3620_PWM_PARAM_S0_DEF;
            mt3620_pwm[pwmBlock]->pwm3_param_s1 = MT3620_PWM_PARAM_S1_DEF;
            break;
        default:
            break;
    }

    MT3620_PWM_FIELD_WRITE(pwmBlock, pwm_glo_ctrl, pwm_tick_clock_sel, clockSel);

    uint32_t pwm_param = ((offTime << 16) | onTime);
    switch (pinMask) {
        case 1:
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm0_ctrl, pwm_clock_en, 1);
            mt3620_pwm[pwmBlock]->pwm0_param_s0 = pwm_param;
            mt3620_pwm[pwmBlock]->pwm0_param_s1 = 0;
            mt3620_pwm0_ctrl_t pwm0_ctrl = {.mask = mt3620_pwm[pwmBlock]->pwm0_ctrl};
            pwm0_ctrl.S0_stay_cycle = 1;
            pwm0_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[pwmBlock]->pwm0_ctrl = pwm0_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm0_ctrl, kick, 1);
            break;
        case 2:
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm1_ctrl, pwm_clock_en, 1);
            mt3620_pwm[pwmBlock]->pwm1_param_s0 = pwm_param;
            mt3620_pwm[pwmBlock]->pwm1_param_s1 = 0;
            mt3620_pwm1_ctrl_t pwm1_ctrl = { .mask = mt3620_pwm[pwmBlock]->pwm1_ctrl };
            pwm1_ctrl.S0_stay_cycle = 1;
            pwm1_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[pwmBlock]->pwm1_ctrl = pwm1_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm1_ctrl, kick, 1);
            break;
        case 4:
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm2_ctrl, pwm_clock_en, 1);
            mt3620_pwm[pwmBlock]->pwm2_param_s0 = pwm_param;
            mt3620_pwm[pwmBlock]->pwm2_param_s1 = 0;
            mt3620_pwm2_ctrl_t pwm2_ctrl = { .mask = mt3620_pwm[pwmBlock]->pwm2_ctrl };
            pwm2_ctrl.S0_stay_cycle = 1;
            pwm2_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[pwmBlock]->pwm2_ctrl = pwm2_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm2_ctrl, kick, 1);
            break;
        case 8:
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm3_ctrl, pwm_clock_en, 1);
            mt3620_pwm[pwmBlock]->pwm3_param_s0 = pwm_param;
            mt3620_pwm[pwmBlock]->pwm3_param_s1 = 0;
            mt3620_pwm3_ctrl_t pwm3_ctrl = { .mask = mt3620_pwm[pwmBlock]->pwm3_ctrl };
            pwm3_ctrl.S0_stay_cycle = 1;
            pwm3_ctrl.pwm_io_ctrl = 0;
            mt3620_pwm[pwmBlock]->pwm3_ctrl = pwm3_ctrl.mask;
            MT3620_PWM_FIELD_WRITE(pwmBlock, pwm3_ctrl, kick, 1);
            break;
        default:
            break;
    }

    return ERROR_NONE;
}
