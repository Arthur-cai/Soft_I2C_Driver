/**
 * @brief 软件模拟I2C驱动
 * @file bsp_soft_i2c.c
 * @version 1.0
 * @author arthur.cai (arthurcai_c@163.com)
 * @date 2024-01-21 12:17:15
 * @copyright Copyright (c) ArthurCai 2023 - 2024. All rights reserved.
 * 
 * @details 
 * @par 修改日志:
 * <table>
 * <tr><th>Date                     <th>Author          <th>Description
 * <tr><td>2024-01-21 12:17:15      <td>arthur.cai      <td>创建文件
 */
#include "bsp_soft_i2c_private.h"
#include "bsp_soft_i2c.h"

// ---------------------------------------- 内部调用函数 ---------------------------------------- //

/**
 * @brief 内联 使能GPIO时钟
 * 
 * @param [in] p_gpio        GPIO结构体指针
 * 
 * @return void 
 */
SOFT_I2C_STATIC_INLINE void soft_i2c_gpio_clk_enable(P_SOFT_I2C_GPIO_COMM_T p_gpio) {
#if defined(SOFT_I2C_GD32F3_USED) || defined(SOFT_I2C_STM32F1_USED)
    SOFT_I2C_ENABLE_GPIO_CLK(p_gpio->gpioClk);
#else
    return;
#endif
}

/**
 * @brief 内部调用 配置开漏输出函数
 * 
 * @param [in] p_gpio        GPIO结构体指针
 * 
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static void soft_i2c_set_od_mode(P_SOFT_I2C_GPIO_COMM_T p_gpio) {
#if defined(SOFT_I2C_GD32F3_USED)
    // GD32F30x芯片
    gpio_init(p_gpio->gpioPort, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, p_gpio->gpioPin);
#elif defined(SOFT_I2C_STM32F1_USED)
    // STM32F10x芯片
    GPIO_InitTypeDef initStruct;
    initStruct.GPIO_Pin = (uint16_t)p_gpio->gpioPin;
    initStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    initStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init((GPIO_TypeDef *)p_gpio->gpioPort, &initStruct);
#else
    return;
#endif
}

#pragma push
#pragma O0
/**
 * @brief 内联 设置GPIO电平
 * 
 * @param [in] p_gpio        GPIO结构体指针
 * @param [in] val           电平值, 0:低电平, 1:高电平
 * 
 * @return void 
 */
SOFT_I2C_STATIC_INLINE void soft_i2c_write_gpio(P_SOFT_I2C_GPIO_COMM_T p_gpio, uint8_t val) {
#if defined(SOFT_I2C_GD32F3_USED) || defined(SOFT_I2C_STM32F1_USED)
    if (val) {
        SOFT_I2C_SET_PIN(p_gpio->gpioPort, p_gpio->gpioPin);
    } else {
        SOFT_I2C_RESET_PIN(p_gpio->gpioPort, p_gpio->gpioPin);
    }
#else
    return;
#endif
}

/**
 * @brief 内联 读取GPIO电平
 * 
 * @param [in] p_gpio        GPIO结构体指针
 * 
 * @return uint8_t
 *  @retval 0 低电平, 1 高电平
 */
SOFT_I2C_STATIC_INLINE uint8_t soft_i2c_read_gpio(P_SOFT_I2C_GPIO_COMM_T p_gpio) {
#if defined(SOFT_I2C_GD32F3_USED) || defined(SOFT_I2C_STM32F1_USED)
    return (SOFT_I2C_READ_PIN(p_gpio->gpioPort, p_gpio->gpioPin)) ? 1 : 0;
#else
    return 0xFF;
#endif
}

// ---------------------------------------- 时序函数 ---------------------------------------- //

/**
 * @brief 内部调用 微秒延时函数
 */
static void soft_i2c_delay_us(void) {
    uint32_t i, j = 0;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < SOFT_I2C_DELAY_CYCLE; j++) {
            __asm("NOP");
        }
    }
}

/**
 * @brief 内部调用 800纳秒延时函数
 */
static void soft_i2c_delay_800ns(void) {
    uint32_t i = 0;
#if defined(SOFT_I2C_GD32F3_USED)
    uint32_t j = 0;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < (SOFT_I2C_DELAY_CYCLE * 4) / 5; j++) {
            __asm("NOP");
        }
    }
#elif defined(SOFT_I2C_STM32F1_USED)
    for (i = 0; i < 2; i++) {
        __asm("NOP");
    }
#else
    return;
#endif
}
#pragma pop

/**
 * @brief 内部调用 I2C判忙函数
 * 
 * @param [in] p_i2c         I2C结构体指针
 * 
 * @return bool
 *  @retval true 忙, false 空闲
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static bool soft_i2c_is_busy(P_SOFT_I2C_T p_i2c) {
    uint32_t waitCnt = 100; ///< 等待计数
    while (waitCnt > 0 &&
           soft_i2c_read_gpio(&p_i2c->sda) == SOFT_I2C_LEVEL_LOW) {
        waitCnt--;
    }
    return (waitCnt > 0) ? false : true;
}

/**
 * @brief 内部调用 I2C开始信号
 * 
 * @param [in] p_i2c         I2C结构体指针
 * 
 * @details 特殊说明:
 * @par eg:
 * @code
 *    
 * @endcode
 */
static void soft_i2c_start(P_SOFT_I2C_T p_i2c) {
    soft_i2c_delay_800ns();
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_HIGH);
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH);
    soft_i2c_delay_us();
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_LOW);
    soft_i2c_delay_us();
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_LOW);
}

/**
 * @brief 内部调用 I2C停止信号
 * 
 * @param [in] p_i2c         I2C结构体指针
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static soft_i2c_err_t soft_i2c_stop(P_SOFT_I2C_T p_i2c) {
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_LOW);
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_LOW);
    soft_i2c_delay_us();
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH);
#if defined(__SOFT_I2C_CLOCK_STRECH_EN__)
    uint32_t waitCnt = SOFT_I2C_WAIT_CNT;   ///< 等待计数
    SOFT_I2C_WAIT_SCL_RELEASE(p_i2c, waitCnt); // 等待SCL释放
    if (waitCnt == 0) {
        return SOFT_I2C_ERR_TIMEOUT;
    }
#endif
    soft_i2c_delay_us();
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_HIGH);
    return SOFT_I2C_ERR_OK;
}

/**
 * @brief 内部调用 I2C等待应答信号
 * 
 * @param [in] p_i2c         I2C结构体指针
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static soft_i2c_err_t soft_i2c_wait_ack(P_SOFT_I2C_T p_i2c) {
    uint32_t waitCnt = SOFT_I2C_WAIT_CNT;   ///< 等待计数
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_HIGH); // 开启线与
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH); // 拉起SCL
#if defined(__SOFT_I2C_CLOCK_STRECH_EN__)
    SOFT_I2C_WAIT_SCL_RELEASE(p_i2c, waitCnt); // 等待SCL释放
    if (waitCnt == 0) {
        return SOFT_I2C_ERR_TIMEOUT;
    }
#endif
    waitCnt = 100;
    // 等待应答信号
    while (soft_i2c_read_gpio(&p_i2c->sda) == SOFT_I2C_LEVEL_HIGH) {
        waitCnt--;
        if (waitCnt == 0) {
            return SOFT_I2C_ERR_NACK;
        }
    }
    soft_i2c_delay_800ns();
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_LOW);
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_LOW);   // 关闭线与
    return SOFT_I2C_ERR_OK;
}

/**
 * @brief 内部调用 I2C发送应答信号
 * 
 * @param [in] p_i2c         I2C结构体指针
 * @param [in] ackVal        SOFT_I2C_ACK 或 SOFT_I2C_NACK
 * 
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static void soft_i2c_ack_respond(P_SOFT_I2C_T p_i2c, uint8_t ackVal) {
    soft_i2c_write_gpio(&p_i2c->sda, ackVal);
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH);
    soft_i2c_delay_us();
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_LOW);
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_LOW);
    soft_i2c_delay_800ns();
}

/**
 * @brief 内部调用 I2C发送一个字节
 * 
 * @param [in] p_i2c         I2C结构体指针
 * @param [in] byte          要发送的字节
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static soft_i2c_err_t soft_i2c_send_byte(P_SOFT_I2C_T p_i2c, uint8_t byte) {
    uint8_t  loopCnt    = 0;      ///< 循环计数
#if defined(__SOFT_I2C_CLOCK_STRECH_EN__)
    uint32_t waitCnt    = SOFT_I2C_WAIT_CNT;  ///< 等待计数
    bool     isFirstBit = true;               ///< 是否第一位
#endif
    for (loopCnt = 0; loopCnt < 8; loopCnt++) {
        soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_GET_MSB_BIT(byte));   // 写入数据
        byte <<= 1;
        soft_i2c_delay_us();
        soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH);          // 拉起SCL
#if defined(__SOFT_I2C_CLOCK_STRECH_EN__)
        if (isFirstBit) {
            SOFT_I2C_WAIT_SCL_RELEASE(p_i2c, waitCnt); // 等待SCL释放
            if (waitCnt == 0) {
                return SOFT_I2C_ERR_TIMEOUT;
            }
            isFirstBit = false;
        }
#endif
        soft_i2c_delay_us();
        soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_LOW);           // 拉低SCL
    }
    soft_i2c_delay_us();
    return SOFT_I2C_ERR_OK;
}

/**
 * @brief 内部调用 I2C读取一个字节
 * 
 * @param [in] p_i2c         I2C结构体指针
 * @param [in] p_data        读取的数据
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static soft_i2c_err_t soft_i2c_read_byte(P_SOFT_I2C_T p_i2c, uint8_t *p_data) {
    uint8_t  loopCnt    = 0;      ///< 循环计数
#if defined(__SOFT_I2C_CLOCK_STRECH_EN__)
    uint32_t waitCnt    = SOFT_I2C_WAIT_CNT;  ///< 等待计数
    bool     isFirstBit = true;               ///< 是否第一位
#endif
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_HIGH); // 开启线与
    for (loopCnt = 0; loopCnt < 8; loopCnt++) {
        soft_i2c_delay_us();
        *p_data <<= 1;
        soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH);  // 拉起SCL
#if defined(__SOFT_I2C_CLOCK_STRECH_EN__)
        if (isFirstBit) {
            SOFT_I2C_WAIT_SCL_RELEASE(p_i2c, waitCnt); // 等待SCL释放
            if (waitCnt == 0) {
                return SOFT_I2C_ERR_TIMEOUT;
            }
            isFirstBit = false;
        }
#endif
        *p_data |= soft_i2c_read_gpio(&p_i2c->sda); // 读取数据
        soft_i2c_delay_800ns();
        soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_LOW);   // 拉低SCL
    }
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_LOW);   // 关闭线与
    soft_i2c_delay_800ns();
    return SOFT_I2C_ERR_OK;
}

/**
 * @brief 内部调用 I2C通信函数
 * 
 * @param [in] p_i2c         I2C结构体指针
 * @param [in] p_msg         消息结构体指针
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
static soft_i2c_err_t soft_i2c_comm(P_SOFT_I2C_T p_i2c, P_SOFT_I2C_MSG_T p_msg) {
    uint16_t loopCnt = 0;             ///< 循环计数
    uint8_t  ackVal  = SOFT_I2C_ACK;  ///< 应答值
    // 1. 判忙
    if (soft_i2c_is_busy(p_i2c)) {
        p_i2c->i2cSta = SOFT_I2C_BUSY;
        return SOFT_I2C_ERR_BUSY;
    }
    // 2. 发送开始信号
    p_i2c->i2cSta = SOFT_I2C_WRITE_START;
    soft_i2c_start(p_i2c);
    // 3. 发送从机地址
    if (soft_i2c_send_byte(p_i2c, p_msg->slaveAddr | SOFT_I2C_WRITE) != SOFT_I2C_ERR_OK ||
        soft_i2c_wait_ack(p_i2c) != SOFT_I2C_ERR_OK) {
        soft_i2c_stop(p_i2c);
        return SOFT_I2C_ERR_WRITE_ADDR; // 发送写地址失败
    }
    // 4. 发送寄存器地址
    for (loopCnt = 0; loopCnt < p_msg->regAddrLen; loopCnt++) {
        if (soft_i2c_send_byte(p_i2c, SOFT_I2C_GET_LOW_BYTE(p_msg->regAddr >> (loopCnt * 8))) != SOFT_I2C_ERR_OK ||
            soft_i2c_wait_ack(p_i2c) != SOFT_I2C_ERR_OK) {
            soft_i2c_stop(p_i2c);
            return SOFT_I2C_ERR_WRITE_REG_ADDR; // 发送寄存器地址失败
        }
    }
    // 5. 如果是写操作
    if (p_msg->rwFlag == SOFT_I2C_WRITE) {
        // 发送数据
        for (loopCnt = 0; loopCnt < p_msg->dataLen; loopCnt++) {
            if (soft_i2c_send_byte(p_i2c, p_msg->pdata[loopCnt]) != SOFT_I2C_ERR_OK ||
                soft_i2c_wait_ack(p_i2c) != SOFT_I2C_ERR_OK) {
                soft_i2c_stop(p_i2c);
                return SOFT_I2C_ERR_WRITE_DATA; // 发送数据失败
            }
        }
    }
    p_i2c->i2cSta = SOFT_I2C_WRITE_END;
    // 6. 如果是读操作
    if (p_msg->rwFlag == SOFT_I2C_READ) {
        p_i2c->i2cSta = SOFT_I2C_WRITE_START;
        soft_i2c_start(p_i2c);
        // 发送读地址
        if (soft_i2c_send_byte(p_i2c, p_msg->slaveAddr | SOFT_I2C_READ) != SOFT_I2C_ERR_OK ||
            soft_i2c_wait_ack(p_i2c) != SOFT_I2C_ERR_OK) {
            soft_i2c_stop(p_i2c);
            return SOFT_I2C_ERR_READ_ADDR; // 发送读地址失败
        }
        soft_i2c_delay_800ns();
        p_i2c->i2cSta = SOFT_I2C_READ_START;
        for (loopCnt = 0; loopCnt < p_msg->dataLen; loopCnt++) {
            // 最后一个字节发送NACK
            ackVal = (loopCnt >= p_msg->dataLen - 1) ? SOFT_I2C_NACK : SOFT_I2C_ACK;
            // 读取数据
            if (soft_i2c_read_byte(p_i2c, &p_msg->pdata[loopCnt]) != SOFT_I2C_ERR_OK) {
                soft_i2c_stop(p_i2c);
                return SOFT_I2C_ERR_READ_DATA; // 读取数据失败
            }
            soft_i2c_ack_respond(p_i2c, ackVal);
        }
        p_i2c->i2cSta = SOFT_I2C_READ_END;
    }
    // 7. 发送停止信号
    soft_i2c_stop(p_i2c);
    p_i2c->i2cSta = SOFT_I2C_IDLE;
    return SOFT_I2C_ERR_OK;
}

// ---------------------------------------- 外部调用函数 ---------------------------------------- //

/**
 * @brief 软件模拟 I2C 初始化
 * 
 * @param [in] p_i2c         I2C结构体指针
 * 
 * @return uint32_t
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
soft_i2c_err_t soft_i2c_init(P_SOFT_I2C_T p_i2c) {
    // 1. 参数检查
    if (p_i2c == NULL) {
        return SOFT_I2C_ERR_PARAM;
    }
    // 2. 初始化结构体内监控变量
#if defined(SOFT_I2C_GD32F3_USED) || defined(SOFT_I2C_STM32F1_USED)
    p_i2c->isValid = true;  // 芯片有效
#else
    p_i2c->isValid = false; // 芯片无效
#endif
    p_i2c->isInit = false;
    p_i2c->i2cSta = SOFT_I2C_IDLE;
    if (!(p_i2c->isValid)) {
        return SOFT_I2C_ERR_PARAM;
    }
    // 3. 强制设置为开漏输出高电平
    soft_i2c_gpio_clk_enable(&p_i2c->scl);
    soft_i2c_gpio_clk_enable(&p_i2c->sda);
    soft_i2c_set_od_mode(&p_i2c->scl);
    soft_i2c_set_od_mode(&p_i2c->sda);
    soft_i2c_write_gpio(&p_i2c->scl, SOFT_I2C_LEVEL_HIGH);
    soft_i2c_write_gpio(&p_i2c->sda, SOFT_I2C_LEVEL_HIGH);
    p_i2c->isInit = true;
    return SOFT_I2C_ERR_OK;
}

/**
 * @brief 软件模拟 I2C 写数据
 * 
 * @param [in] p_i2c         I2C结构体指针
 * @param [in] slaveAddr     从机地址
 * @param [in] regAddr       寄存器地址
 * @param [in] regAddrLen    寄存器地址长度 @ref SOFT_I2C_REG_ADDR_LEN_1 , @ref SOFT_I2C_REG_ADDR_LEN_2
 * @param [in] p_data        数据指针
 * @param [in] dataLen       数据长度(字节)
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
soft_i2c_err_t soft_i2c_write(P_SOFT_I2C_T p_i2c, uint32_t slaveAddr, uint32_t regAddr, uint32_t regAddrLen, uint8_t *p_data, uint32_t dataLen) {
    if (p_i2c == NULL || p_data == NULL || p_i2c->isValid == false || p_i2c->isInit == false || \
        dataLen == 0 || regAddrLen < SOFT_I2C_REG_ADDR_LEN_1 || regAddrLen > SOFT_I2C_REG_ADDR_LEN_2) {
        return SOFT_I2C_ERR_PARAM;
    }
    SOFT_I2C_DEF_MSG(w_msg, slaveAddr, regAddr, regAddrLen, p_data, dataLen, SOFT_I2C_WRITE);
    return soft_i2c_comm(p_i2c, &w_msg);
}

/**
 * @brief 软件模拟 I2C 读数据
 * 
 * @param [in] p_i2c         I2C结构体指针
 * @param [in] slaveAddr     从机地址
 * @param [in] regAddr       寄存器地址
 * @param [in] regAddrLen    寄存器地址长度 @ref SOFT_I2C_REG_ADDR_LEN_1 , @ref SOFT_I2C_REG_ADDR_LEN_2
 * @param [in] p_data        数据指针
 * @param [in] dataLen       数据长度(字节)
 * 
 * @return soft_i2c_err_t 
 *  @retval 0 成功, 其他值 失败 @ref SOFT_I2C_ERR_CODE
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
soft_i2c_err_t soft_i2c_read(P_SOFT_I2C_T p_i2c, uint32_t slaveAddr, uint32_t regAddr, uint32_t regAddrLen, uint8_t *p_data, uint32_t dataLen) {
    if (p_i2c == NULL || p_data == NULL || p_i2c->isValid == false || p_i2c->isInit == false || \
        dataLen == 0 || regAddrLen < SOFT_I2C_REG_ADDR_LEN_1 || regAddrLen > SOFT_I2C_REG_ADDR_LEN_2) {
        return SOFT_I2C_ERR_PARAM;
    }
    SOFT_I2C_DEF_MSG(r_msg, slaveAddr, regAddr, regAddrLen, p_data, dataLen, SOFT_I2C_READ);
    return soft_i2c_comm(p_i2c, &r_msg);
}
