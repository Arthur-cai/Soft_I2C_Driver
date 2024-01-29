/**
 * @brief 软件模拟I2C驱动私有头文件
 * @file bsp_soft_i2c_private.h
 * @version 1.0
 * @author arthur.cai (arthurcai_c@163.com)
 * @date 2024-01-27 01:48:48
 * @copyright Copyright (c) ArthurCai 2023 - 2024. All rights reserved.
 * 
 * @details 
 * @par 修改日志:
 * <table>
 * <tr><th>Date                     <th>Author          <th>Description
 * <tr><td>2024-01-27 01:48:48      <td>arthur.cai      <td>创建文件
 */
#ifndef BSP_SOFT_I2C_PRIVATE_H
#define BSP_SOFT_I2C_PRIVATE_H

#include <stdint.h>

/**
 * @brief 软件模拟 I2C 通用宏定义
 * @addtogroup SOFT_I2C_COMM_DEF
 * @{
 */
#if defined(SOFT_I2C_GD32F3_USED)
#define SOFT_I2C_DELAY_CYCLE     (10U)                  ///< 延时周期数
#elif defined(SOFT_I2C_STM32F1_USED)
#define SOFT_I2C_DELAY_CYCLE     (1U)                   ///< 延时周期数
#else
#define SOFT_I2C_DELAY_CYCLE     (1U)                   ///< 延时周期数
#endif
#define SOFT_I2C_LEVEL_HIGH      (1U)                   ///< 高电平
#define SOFT_I2C_LEVEL_LOW       (0U)                   ///< 低电平
#define SOFT_I2C_ACK             SOFT_I2C_LEVEL_LOW     ///< I2C应答信号(低电平)
#define SOFT_I2C_NACK            SOFT_I2C_LEVEL_HIGH    ///< I2C非应答信号(高电平)
#define SOFT_I2C_WRITE           (0U)                   ///< 写操作
#define SOFT_I2C_READ            (1U)                   ///< 读操作
#define SOFT_I2C_WAIT_CNT        (0xFFFU)               ///< 等待计数(用于时钟延展)
/*
 * @}
*/

/**
 * @brief 软件模拟 I2C 引脚置高
 *
 * @param [in]  port         端口
 * @param [in]  pin          引脚
 */
#define SOFT_I2C_SET_PIN(port, pin)     \
        (*(volatile uint32_t *)(uint32_t)((port) + 0x10U)) = (uint32_t)(pin)

/**
 * @brief 软件模拟 I2C 引脚置低
 * 
 * @param [in]  port         端口
 * @param [in]  pin          引脚
 */
#define SOFT_I2C_RESET_PIN(port, pin)   \
        (*(volatile uint32_t *)(uint32_t)((port) + 0x14U)) = (uint32_t)(pin)

/**
 * @brief 软件模拟 I2C 读引脚输入电平
 * 
 * @param [in]  port         端口
 * @param [in]  pin          引脚
 */
#define SOFT_I2C_READ_PIN(port, pin)    \
        (*(volatile uint32_t *)(uint32_t)((port) + 0x08U)) & (uint32_t)(pin)

/**
 * @brief 软件模拟 I2C 使能引脚时钟
 * 
 * @param [in]  gpioClk      引脚时钟 @ref SOFT_I2C_GPIO_CLK_E
 */
#define SOFT_I2C_ENABLE_GPIO_CLK(gpioClk)   \
        (*(volatile uint32_t *)(uint32_t)(0x40021000U + 0x18U)) |= (uint32_t)(gpioClk)

/**
 * @brief 软件模拟 I2C 位操作
 * @addtogroup SOFT_I2C_BIT_OPERATION
 * @{
*/

/**
 * @brief 软件模拟 I2C 获取最高位
 *
 * @param [in]  data         数据
 */
#define SOFT_I2C_GET_MSB_BIT(data)   (((data) >> 7) & 0x01)

/**
 * @brief 软件模拟 I2C 获取最低字节
 *
 * @param [in]  data         数据
 */
#define SOFT_I2C_GET_LOW_BYTE(data)  (uint8_t)((data) & 0xFF)

/**
 * @brief 软件模拟 I2C 等待 SCL 释放
 *
 * @param [in]  p_i2c          I2C结构体指针
 * @param [in]  waitCnt        等待次数, 必须在前文声明
 *
 * @details 用于开启时钟延展功能, 等待从机释放时钟线
 */
#define SOFT_I2C_WAIT_SCL_RELEASE(p_i2c, waitCnt)                       \
        while (waitCnt > 0 &&                                           \
               soft_i2c_read_gpio(&p_i2c->scl) == SOFT_I2C_LEVEL_LOW) { \
            waitCnt--;                                                  \
        }

/**
 * @brief 软件模拟 I2C 定义消息
 *
 * @param [in]  msgName        消息名称
 * @param [in]  slaveAddr      从机地址
 * @param [in]  regAddr        寄存器地址
 * @param [in]  regAddrLen     寄存器地址长度(字节)
 * @param [in]  pdata          数据指针
 * @param [in]  dataLen        数据长度(字节)
 *
 * @details 
 * @par 示例:
 * @code
 *   
 * @endcode
 */
#define SOFT_I2C_DEF_MSG(msgName, slaveAddr, regAddr, regAddrLen, pdata, dataLen, rwFlag)   \
        SOFT_I2C_MSG_T msgName = {slaveAddr, regAddr, regAddrLen, rwFlag, dataLen, pdata}

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 软件模拟 I2C 消息结构体
 */
typedef struct _SOFT_I2C_MSG_T {
    uint32_t slaveAddr;   ///< 从机地址(8位地址)
    uint32_t regAddr;     ///< 寄存器地址
    uint32_t regAddrLen;  ///< 寄存器地址长度(字节)
    uint32_t rwFlag;      ///< 读写标志(SOFT_I2C_WRITE 或 SOFT_I2C_READ)
    uint32_t dataLen;     ///< 数据长度(字节)
    uint8_t  *pdata;      ///< 数据指针
} SOFT_I2C_MSG_T, *P_SOFT_I2C_MSG_T;

#ifdef __cplusplus
}
#endif

#endif  // BSP_SOFT_I2C_PRIVATE_H
