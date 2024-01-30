/**
 ******************************************************************************
 * @file    Project/STM32F10x_StdPeriph_Template/main.c
 * @author  MCD Application Team
 * @version V3.6.0
 * @date    20-September-2021
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2011 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "soft_i2c_dev.h"
#include <stdio.h>
#include <string.h>

void block_delay(void) {
    uint32_t delayCnt = 0x119400;
    while (delayCnt > 0) {
        delayCnt--;
    }
}

void led_init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO_LED clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void led_on(void) { 
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void led_off(void) {
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
}

#define SLAVE_ADDR 0xA0 ///< 从机地址
#define REG_ADDR 0x00   ///< 寄存器地址

uint8_t w_buf[8] = {0xAA, 0xA5, 0x5A, 0xFF, 0xFA, 0xAF, 0xDD, 0xEE};
uint8_t r_buf[8] = {0};
uint8_t flag = 1;
bool equal = false;
soft_i2c_err_t ret = SOFT_I2C_ERR_OK;

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void) {
    led_init();
    led_on();

    ret = soft_i2c_init(I2C_DEV);

    while (1) {
        led_on();
        block_delay();
        {
            memset(r_buf, 0, 8);
            if (flag) {
                ret = soft_i2c_write(I2C_DEV, SLAVE_ADDR, REG_ADDR, 1, w_buf, 8);
                // if (ret != SOFT_I2C_ERR_OK) {
                //     while(1);
                // }
                equal = false;
            } else {
                ret = soft_i2c_read(I2C_DEV, SLAVE_ADDR, REG_ADDR, 1, r_buf, 8);
                // if (ret != SOFT_I2C_ERR_OK) {
                //     while(1);
                // }
                equal = (memcmp(r_buf, w_buf, 8) == 0) ? true : false;
            }
            flag = !flag;
        }
        led_off();
        block_delay();
    }
}
