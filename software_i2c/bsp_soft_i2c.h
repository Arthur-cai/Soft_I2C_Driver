/**
 * @brief 软件模拟I2C驱动
 * @file bsp_soft_i2c.h
 * @version 1.0
 * @author arthur.cai (arthurcai_c@163.com)
 * @date 2024-01-21 12:16:49
 * @copyright Copyright (c) ArthurCai 2023 - 2024. All rights reserved.
 * 
 * @details 
 * @par 修改日志:
 * <table>
 * <tr><th>Date                     <th>Author          <th>Description
 * <tr><td>2024-01-21 12:16:49      <td>arthur.cai      <td>创建文件
 */
#ifndef BSP_SOFT_I2C_H
#define BSP_SOFT_I2C_H

/* 根据不同的芯片包含不同的文件 */
#if defined(GD32F30X_HD) || defined(GD32F30X_CL) || defined(GD32F30X_XD)
#define SOFT_I2C_GD32F3_USED        ///< 使用GD32F3芯片
#elif defined(STM32F10X_HD) || defined(STM32F10X_MD) || defined(STM32F10X_CL)
#define SOFT_I2C_STM32F1_USED       ///< 使用STM32F1芯片
#else
#error "Please define GD32F30X_XX or STM32F10X_XX"
#endif

#if defined(SOFT_I2C_GD32F3_USED)
#include "gd32f30x.h"
#elif defined(SOFT_I2C_STM32F1_USED)
#include "stm32f10x.h"
#endif
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// #define __SOFT_I2C_CLOCK_STRECH_EN__         ///< 时钟延展使能(从机会钳住时钟线, 使主机无法继续发送数据)

#if defined(__CC_ARM)
#define SOFT_I2C_STATIC_INLINE   static __inline        ///< 静态内联函数
#elif defined(__GNUC__)
#define SOFT_I2C_STATIC_INLINE   static inline          ///< 静态内联函数
#else
#define SOFT_I2C_STATIC_INLINE
#endif

/**
 * @brief 软件模拟 I2C 错误码
 * @addtogroup SOFT_I2C_ERR_CODE
 * @{
 */
typedef uint32_t                       soft_i2c_err_t;  ///< 返回值类型
#define SOFT_I2C_ERR_OK                (0U)             ///< 正常
#define SOFT_I2C_ERR_NACK              (1U)             ///< 无应答
#define SOFT_I2C_ERR_BUSY              (2U)             ///< 总线忙
#define SOFT_I2C_ERR_PARAM             (3U)             ///< 参数错误
#define SOFT_I2C_ERR_TIMEOUT           (4U)             ///< 超时
#define SOFT_I2C_ERR_WRITE_ADDR        (5U)             ///< 发送写地址失败
#define SOFT_I2C_ERR_WRITE_REG_ADDR    (6U)             ///< 发送寄存器地址失败
#define SOFT_I2C_ERR_WRITE_DATA        (7U)             ///< 写数据失败
#define SOFT_I2C_ERR_READ_ADDR         (8U)             ///< 发送读地址失败
#define SOFT_I2C_ERR_READ_DATA         (9U)             ///< 读数据失败
/*
 * @}
*/

/**
 * @brief 软件模拟 I2C 寄存器地址长度
 * @addtogroup SOFT_I2C_REG_ADDR_LEN
 * @{
 */
#define SOFT_I2C_REG_ADDR_LEN_1        (1U)             ///< 1字节寄存器地址长度
#define SOFT_I2C_REG_ADDR_LEN_2        (2U)             ///< 2字节寄存器地址长度
/**
 * @}
 */

#if defined(SOFT_I2C_GD32F3_USED) || defined(SOFT_I2C_STM32F1_USED)
/**
 * @brief 软件模拟 I2C 外部声明
 *
 * @param [in]  name        I2C结构体名称
 */
#define SOFT_I2C_EXT(name)  \
        extern void *const name
#else
#define SOFT_I2C_EXT(name)
#endif

#if defined(SOFT_I2C_GD32F3_USED)
/**
 * @brief 软件模拟 I2C 定义结构体
 *
 * @param [in]  name        I2C结构体名称
 * @param [in]  scl_port    时钟线端口, A ~ G
 * @param [in]  scl_pin     时钟线引脚, 0 ~ 15
 * @param [in]  sda_port    数据线端口, A ~ G
 * @param [in]  sda_pin     数据线引脚, 0 ~ 15
 */
#define SOFT_I2C_DEF(name, scl_port, scl_pin, sda_port, sda_pin)                 \
        SOFT_I2C_T soft_i2c_##name = {                                           \
            false, false,                                                        \
            {GPIO##scl_port, GPIO_PIN_##scl_pin, SOFT_I2C_GPIO##scl_port##_CLK}, \
            {GPIO##sda_port, GPIO_PIN_##sda_pin, SOFT_I2C_GPIO##sda_port##_CLK}, \
            SOFT_I2C_IDLE                                                        \
        };                                                                       \
        void *const name = &soft_i2c_##name                                      \

#elif defined(SOFT_I2C_STM32F1_USED)
/**
 * @brief 软件模拟 I2C 定义结构体
 *
 * @param [in]  name        I2C结构体名称
 * @param [in]  scl_port    时钟线端口, A ~ G
 * @param [in]  scl_pin     时钟线引脚, 0 ~ 15
 * @param [in]  sda_port    数据线端口, A ~ G
 * @param [in]  sda_pin     数据线引脚, 0 ~ 15
 */
#define SOFT_I2C_DEF(name, scl_port, scl_pin, sda_port, sda_pin)                        \
        SOFT_I2C_T soft_i2c_##name = {                                                  \
            false, false,                                                               \
            {GPIO##scl_port##_BASE, GPIO_Pin_##scl_pin, SOFT_I2C_GPIO##scl_port##_CLK}, \
            {GPIO##sda_port##_BASE, GPIO_Pin_##sda_pin, SOFT_I2C_GPIO##sda_port##_CLK}, \
            SOFT_I2C_IDLE                                                               \
        };                                                                              \
        void *const name = &soft_i2c_##name                                             \

#else
#define SOFT_I2C_DEF(name, scl_port, scl_pin, sda_port, sda_pin)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 软件模拟 I2C GPIO时钟枚举
 */
typedef enum _SOFT_I2C_GPIO_CLK_E {
    SOFT_I2C_GPIOA_CLK = 0x00000004U,   ///< GPIOA_CLK
    SOFT_I2C_GPIOB_CLK = 0x00000008U,   ///< GPIOB_CLK
    SOFT_I2C_GPIOC_CLK = 0x00000010U,   ///< GPIOC_CLK
    SOFT_I2C_GPIOD_CLK = 0x00000020U,   ///< GPIOD_CLK
    SOFT_I2C_GPIOE_CLK = 0x00000040U,   ///< GPIOE_CLK
    SOFT_I2C_GPIOF_CLK = 0x00000080U,   ///< GPIOF_CLK
    SOFT_I2C_GPIOG_CLK = 0x00000100U,   ///< GPIOG_CLK
} SOFT_I2C_GPIO_CLK_E;

/**
 * @brief 软件模拟 I2C 监控信息枚举
 */
typedef enum _SOFT_I2C_STA_E {
    SOFT_I2C_IDLE        = 0,   ///< 空闲
    SOFT_I2C_BUSY        = 1,   ///< 忙
    SOFT_I2C_WRITE_START = 2,   ///< 写开始
    SOFT_I2C_WRITE_END   = 3,   ///< 写结束
    SOFT_I2C_READ_START  = 4,   ///< 读开始
    SOFT_I2C_READ_END    = 5,   ///< 读结束
} SOFT_I2C_STA_E;

/**
 * @brief 软件模拟 I2C GPIO通用结构体
 */
typedef struct _SOFT_I2C_GPIO_COMM_T {
    uint32_t            gpioPort;  ///< GPIO端口, GD: GPIOx, STM32: GPIOx_BASE
    uint32_t            gpioPin;   ///< GPIO引脚, GD: GPIO_PIN_x, STM32: GPIO_Pin_x
    SOFT_I2C_GPIO_CLK_E gpioClk;   ///< GPIO时钟 @ref SOFT_I2C_GPIO_CLK_E
} SOFT_I2C_GPIO_COMM_T, *P_SOFT_I2C_GPIO_COMM_T;

/**
 * @brief 软件模拟 I2C 结构体
 */
typedef struct _SOFT_I2C_T {
    bool                 isValid; ///< 芯片是否有效
    bool                 isInit;  ///< 是否初始化
    SOFT_I2C_GPIO_COMM_T scl;     ///< 时钟线
    SOFT_I2C_GPIO_COMM_T sda;     ///< 数据线
    SOFT_I2C_STA_E       i2cSta;  ///< I2C监控状态
} SOFT_I2C_T, *P_SOFT_I2C_T;

/**
 * @brief 内联 软件模拟 I2C 获取状态
 * 
 * @param [in] p_i2c         I2C结构体指针
 * 
 * @return SOFT_I2C_STA_E 
 *  @retval SOFT_I2C_IDLE       , 空闲
 *  @retval SOFT_I2C_BUSY       , 忙
 *  @retval SOFT_I2C_WRITE_START, 写开始
 *  @retval SOFT_I2C_WRITE_END  , 写结束
 *  @retval SOFT_I2C_READ_START , 读开始
 *  @retval SOFT_I2C_READ_END   , 读结束
 * @details 特殊说明: 
 * @par eg:
 * @code
 *    
 * @endcode
 */
SOFT_I2C_STATIC_INLINE SOFT_I2C_STA_E soft_i2c_get_sta(P_SOFT_I2C_T p_i2c) {
    return p_i2c->i2cSta;
}

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
extern soft_i2c_err_t soft_i2c_init(P_SOFT_I2C_T p_i2c);

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
extern soft_i2c_err_t soft_i2c_write(P_SOFT_I2C_T p_i2c, uint32_t slaveAddr, uint32_t regAddr, uint32_t regAddrLen, uint8_t *p_data, uint32_t dataLen);

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
extern soft_i2c_err_t soft_i2c_read(P_SOFT_I2C_T p_i2c, uint32_t slaveAddr, uint32_t regAddr, uint32_t regAddrLen, uint8_t *p_data, uint32_t dataLen);

#ifdef __cplusplus
}
#endif

#endif  // BSP_SOFT_I2C_H
