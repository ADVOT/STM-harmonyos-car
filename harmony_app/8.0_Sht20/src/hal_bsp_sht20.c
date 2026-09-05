#include "hal_bsp_sht20.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"
#include <unistd.h>
#include "ohos_init.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#define SHT20_HoldMaster_Temp_REG_ADDR 0xE3 // 主机模式会阻塞其他IIC设备的通信
#define SHT20_HoldMaster_Humi_REG_ADDR 0xE5
#define SHT20_NoHoldMaster_Temp_REG_ADDR 0xF3
#define SHT20_NoHoldMaster_Humi_REG_ADDR 0xF5
#define SHT20_W_USER_REG_ADDR 0xE6
#define SHT20_R_USER_REG_ADDR 0xE7
#define SHT20_SW_REG_ADDR 0xFE

// 读从机设备的数据
static uint32_t SHT20_RecvData(uint8_t *data, size_t size)
{
  WifiIotI2cData i2cData = {0};
  i2cData.receiveBuf = data;
  i2cData.receiveLen = size;

  return I2cRead(SHT20_I2C_IDX, SHT20_I2C_ADDR, &i2cData);
}
// 向从机设备 发送数据
static uint32_t SHT20_WiteByteData(uint8_t byte)
{
  uint8_t buffer[] = {byte};
  WifiIotI2cData i2cData = {0};
  i2cData.sendBuf = buffer;
  i2cData.sendLen = 1;

  return I2cWrite(SHT20_I2C_IDX, SHT20_I2C_ADDR, &i2cData);
}

// 读温湿度的值
uint32_t SHT20_ReadData(float *temp, float *humi)
{
  uint32_t result;
  uint8_t buffer[4] = {0};

  /* 发送检测温度命令 */
  result = SHT20_WiteByteData(SHT20_NoHoldMaster_Temp_REG_ADDR);
  if (result != 0)
  {
    printf("I2C SHT20 status = 0x%x!!!", result);
    return result;
  }

  usleep(85 * 1000); /* datasheet: typ=66, max=85 */

  // 读数据
  result = SHT20_RecvData(buffer, 3);
  if (result != 0)
  {
    printf("I2C SHT20 status = 0x%x!!!", result);
    return result;
  }

  *temp = 175.72 * (((((int)buffer[0]) << 8) + buffer[1]) / 65536.0) - 46.85;

  memset(buffer, 0, sizeof(buffer));

  /* 发送检测湿度命令 */
  result = SHT20_WiteByteData(SHT20_NoHoldMaster_Humi_REG_ADDR);
  if (result != 0)
  {
    printf("I2C SHT20 status = 0x%x!!!", result);
    return result;
  }

  usleep(50 * 1000); /* datasheet: typ=22, max=29 */

  // 读数据
  result = SHT20_RecvData(buffer, 3);
  if (result != 0)
  {
    printf("I2C SHT20 status = 0x%x!!!", result);
    return result;
  }

  *humi = 125 * (((((int)buffer[0]) << 8) + buffer[1]) / 65536.0) - 6;

  return 0;
}

// 传感器 SHT20 的初始化
uint32_t SHT20_Init(void)
{
  uint32_t result;

    GpioInit();

    //GPIO_10复用为I2C1_SDA
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);

    //GPIO_9复用为I2C1_SCL
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);

    //baudrate: 400kbps
    result = I2cInit(WIFI_IOT_I2C_IDX_0, 400000);

    result = I2cSetBaudrate(WIFI_IOT_I2C_IDX_0, 400000);
  if (result != 0)
  {
    printf("I2C sht20 Init status is 0x%x!!!", result);
    return result;
  }
  // 软复位
  result = SHT20_WiteByteData(SHT20_SW_REG_ADDR);
  if (result != 0)
  {
    printf("I2C SHT20 status = 0x%x!!!", result);
    return result;
  }

  usleep(100 * 1000);
  printf("I2C SHT20 Init is succeeded!!!");
  return 0;
}
