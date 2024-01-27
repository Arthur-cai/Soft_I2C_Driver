软件模拟I2C驱动

【特性】
1. 支持时钟延展
2. 支持400KHz快速模式
3. 支持GD32F30x平台和STM32F10x平台

【移植步骤】】
1. 将 bsp_soft_i2c.c, bsp_soft_i2c.h, bsp_soft_i2c_private.h 文件添加到工程中
2. GD工程需要有全局宏 GD32F30X_HD 或 GD32F30X_XD 或 GD32F30X_CL
3. STM工程需要有全局宏 STM32F10X_HD 或 STM32F10X_MD 或 STM32F10X_CL 以及 USE_STDPERIPH_DRIVER

【使用方法】
1. 在一个驱动注册.c文件中使用 SOFT_I2C_DEF 接口, 参照其注释，注册 I2C 驱动
2. 如果需要外部声明, 则需要在驱动注册.h文件中使用 SOFT_I2C_EXT 接口
3. 其他文件使用 I2C 驱动时, 需包含h头文件
4. 调用 soft_i2c_init() 函数初始化 I2C 驱动, 此函数会初始化监控变量, 并强制配置SCL和SDA为开漏高电平
5. 调用 soft_i2c_write() 函数进行 I2C 数据发送
6. 调用 soft_i2c_read() 函数进行 I2C 数据读取

【开启时钟延展】
1. 在 bsp_soft_i2c.h 文件中, 取消注释 __SOFT_I2C_CLOCK_STRECH_EN__
2. 或者在工程中, 添加全局宏 __SOFT_I2C_CLOCK_STRECH_EN__

【联系方式】
如有BUG, 请联系 arthurcai_c@163.com
