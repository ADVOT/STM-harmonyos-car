#ifndef __HAL_BSP_SSD1306_H
#define __HAL_BSP_SSD1306_H

#include "cmsis_os2.h"

#define SSD1306_I2C_ADDR (0x78)   // 器件的I2C从机地址
#define SSD1306_I2C_IDX  0             // 模块的I2C总线号
#define SSD1306_I2C_SPEED 100000       // 100KHz

/**
 * @brief  设置垂直滚动方式
 * @note   
 * @retval None
 */
void OLED_Set_Vertical_Rol(void);
/**
 * @brief SSD1306 初始化
 * @return Returns {@link HI_ERR_SUCCESS} 成功;
 *         Returns {@link HI_ERR_FAILURE} 失败.
 */
uint32_t SSD1306_Init(void);
/**
 * @brief SSD1306 设置坐标的起始点坐标
 * @param x X轴坐标
 * @param y Y轴坐标
 * @return none
 */
void SSD1306_SetPos(uint8_t x, uint8_t y);
/**
 * @brief SSD1306 全屏填充
 * @return none
 */
void SSD1306_Fill(uint8_t fill_Data);
/**
 * @brief SSD1306 清屏
 * @return none
 */
void SSD1306_CLS(void);
/**
 * @brief SSD1306 打开OLED显示
 * @return none
 */
void SSD1306_ON(void);
/**
 * @brief SSD1306 关闭OLED显示
 * @return none
 *         
 */
void SSD1306_OFF(void);
/**
 * @brief SSD1306 OLED显示屏显示字符串
 * @param x X轴坐标 0~128
 * @param y Y轴坐标 0~7
 * @param ch 显示的字符串
 * @param TextSize 显示的字体大小  8：6*8   16：8*16
 * @return none
 */
void SSD1306_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize);

/**
 * @brief  显示BMP图片
 * @note   
 * @param  x0: 左上角坐标
 * @param  y0: 
 * @param  x1: 右下角坐标
 * @param  y1: 
 * @param  BMP[]: 图片数组
 * @retval None
 */
void SSD1306_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[]);

#endif