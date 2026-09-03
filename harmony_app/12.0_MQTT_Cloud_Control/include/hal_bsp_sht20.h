#ifndef __HAL_BSP_SHT20_H__
#define __HAL_BSP_SHT20_H__

#include "cmsis_os2.h"

#define SHT20_I2C_ADDR (0x80)   // 器件的I2C从机地址
#define SHT20_I2C_IDX  0             // 模块的I2C总线号
#define SHT20_I2C_SPEED 100000       // 100KHz

/**
 * @brief SHT20 读取器件的温湿度
 * @param temp 温度值 
 * @param humi 湿度值 
 * @return Returns {@link IOT_SUCCESS} 成功;
 *         Returns {@link IOT_FAILURE} 失败.
 */
uint32_t SHT20_ReadData(float *temp, float *humi);
/**
 * @brief SHT20 初始化
 * @return Returns {@link IOT_SUCCESS} 成功;
 *         Returns {@link IOT_FAILURE} 失败.
 */
uint32_t SHT20_Init(void);





#endif // !__HAL_BSP_SHT20_H__
