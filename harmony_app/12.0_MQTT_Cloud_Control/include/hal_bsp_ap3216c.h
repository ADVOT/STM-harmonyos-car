#ifndef __HAL_BSP_AP3216C_H__
#define __HAL_BSP_AP3216C_H__

#include "cmsis_os2.h"


#define AP3216C_I2C_ADDR (0x3C)   // 器件的I2C从机地址
#define AP3216C_I2C_IDX  WIFI_IOT_I2C_IDX_0             // 模块的I2C总线号
#define AP3216C_I2C_SPEED 400000       // 100KHz




/**
 * @brief AP3216C 读 接近传感器、光照强度传感器、人体红外传感器的数值
 * @param irData 人体红外传感器 
 * @param alsData 光强传感器
 * @param psData 接近传感器
 * @return Returns {@link IOT_SUCCESS} 成功;
 *         Returns {@link IOT_FAILURE} 失败.
 */
uint32_t AP3216C_ReadData(uint16_t *irData, uint16_t *alsData, uint16_t *psData);

/**
 * @brief AP3216C 初始化
 * @return Returns {@link IOT_SUCCESS} 成功;
 *         Returns {@link IOT_FAILURE} 失败.
 */
uint32_t AP3216C_Init(void);

#endif // !__HAL_BSP_AP3216C_H__
