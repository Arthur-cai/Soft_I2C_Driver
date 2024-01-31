#include "gd32f30x.h"
#include "systick.h"
#include <string.h>
// 引入软件模拟I2C头文件
#include "soft_i2c_dev.h"

void led_init(void) {
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
}

void led_on(void) {
    gpio_bit_set(GPIOC, GPIO_PIN_13);
}

void led_off(void) {
    gpio_bit_reset(GPIOC, GPIO_PIN_13);
}

#define SLAVE_ADDR 0xA0 ///< 从机地址
#define REG_ADDR 0x00   ///< 寄存器地址

uint8_t w_buf[8] = {0xAA, 0xA5, 0x5A, 0xFF, 0xFA, 0xAF, 0xDD, 0xEE};
uint8_t r_buf[8] = {0};
uint8_t flag = 1;
soft_i2c_err_t ret = SOFT_I2C_ERR_OK;
bool equal = false;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void) {
    systick_config();
    led_init();
    led_on();

    // 初始化结构体对象
    ret = soft_i2c_init(I2C_DEV, SOFT_I2C_DEFAULT_WAITCNT);

    while (1) {
        /* turn on LED1 */
        led_on();
        /* insert 200 ms delay */
        delay_1ms(200);
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
        /* turn off LEDs */
        led_off();
        /* insert 200 ms delay */
        delay_1ms(200);
    }
}
