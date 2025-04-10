/**************************************************************************/
/*! 
 * 2016年由Sam C. Lin从Adafruit_MCP9808.cpp修改而来

    @file     Adafruit_MCP9808.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (见license.txt)

	这是Microchip的MCP9808 I2C温度传感器的I2C驱动程序。

	这是Adafruit MCP9808模块的库
	----> http://www.adafruit.com/products/1782

	Adafruit投入时间和资源提供开源代码，
	请通过购买Adafruit的产品支持Adafruit和开源硬件！

	@section  HISTORY

    v1.0 - 第一次发布
*/
#include "open_evse.h"

#ifdef MCP9808_IS_ON_I2C

// 从I2C总线读取一个字节
inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

// 向I2C总线发送一个字节
inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

// 初始化MCP9808传感器，检查设备是否连接
int8_t MCP9808::begin()
{
  // 读取制造商ID和设备ID来确认是否连接了正确的设备
  if ((read16(MCP9808_REG_MANUF_ID) == 0x0054) &&
      (read16(MCP9808_REG_DEVICE_ID) == 0x0400))
    isPresent = 1;  // 如果设备ID匹配，则设备已连接
  else
    isPresent = 0;  // 如果设备ID不匹配，则设备未连接
  return isPresent;  // 返回设备状态（1：设备存在，0：设备未连接）
}

// 从I2C读取16位数据
int16_t MCP9808::read16(uint8_t reg) {
  int16_t val;
  Wire.beginTransmission(MCP9808_ADDRESS);  // 开始与MCP9808的I2C通信
  wiresend(reg);  // 发送寄存器地址
  Wire.endTransmission();  // 结束通信

  Wire.requestFrom((uint8_t)MCP9808_ADDRESS, (uint8_t)2);  // 请求2个字节的数据
  val = wirerecv();  // 读取第一个字节
  val <<= 8;  // 左移8位，为第二个字节腾出空间
  val |= wirerecv();  // 读取第二个字节并合并到val中

  return val;  // 返回16位数据
}

// 读取环境温度并返回温度值（单位为C*10）
int16_t MCP9808::readAmbient()
{
  if (isPresent) {  // 如果设备存在
    int16_t temp = read16(MCP9808_REG_AMBIENT_TEMP) & 0x1FFF;  // 读取环境温度
    if (temp & 0x1000) temp |= 0xF000;  // 如果温度为负数，则进行符号扩展
    return (temp * 10) / 16;  // 返回温度（单位：摄氏度 * 10）
  }
  else {
    return TEMPERATURE_NOT_INSTALLED;  // 如果设备不存在，返回未安装温度传感器的标记
  }
}
#endif // MCP9808_IS_ON_I2C
