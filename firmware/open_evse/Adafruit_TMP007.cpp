/*************************************************** 
  这是 TMP007 红外温度传感器的驱动库

  专为 Adafruit TMP007 模块设计
  ----> https://www.adafruit.com/products/2023

  此模块使用 I2C 协议通信，只需要两个引脚即可连接
  Adafruit 花费了大量时间和资源来开发此开源代码，
  如果你喜欢，请支持 Adafruit 和开源硬件，购买他们的产品！

  作者：Limor Fried（Ladyada）为 Adafruit Industries 编写
  BSD 许可证，以上文字在任何分发中必须保留
 ****************************************************/

#include "open_evse.h"  // 可能包含项目中其他传感器或控制相关定义

// 如果定义了 TMP007_IS_ON_I2C 宏，表示启用了 TMP007 功能
#ifdef TMP007_IS_ON_I2C

#include <util/delay.h>  // AVR 上的延迟库

// 测试时使用的伪造数据（注释掉了）
// #define TESTDIE 0x0C78
// #define TESTVOLT 0xFEED

// TMP007 类构造函数，传入 I2C 地址
Adafruit_TMP007::Adafruit_TMP007(uint8_t i2caddr) {
  _addr = i2caddr;  // 保存设备地址
}

// 初始化 TMP007 传感器
boolean Adafruit_TMP007::begin(uint16_t samplerate) {
  // 启用测量模式、警报功能、转换模式，并设置采样率
  write16(TMP007_CONFIG, TMP007_CFG_MODEON | TMP007_CFG_ALERTEN |
          TMP007_CFG_TRANSC | samplerate);

  // 配置状态寄存器：启用 ALERT 和 CRITICAL 中断
  write16(TMP007_STATMASK, TMP007_STAT_ALERTEN | TMP007_STAT_CRTEN);

  // 读取设备 ID，验证是否为 TMP007
  uint16_t did;
  did = read16(TMP007_DEVID);

  // 如果启用了调试模式，则输出设备 ID
#ifdef TMP007_DEBUG
  Serial.print("did = 0x"); Serial.println(did, HEX);
#endif

  // 如果读取的设备 ID 不是 0x78，说明不是 TMP007，返回 false
  if (did != 0x78) return false;

  // 初始化成功
  return true;
}

/*
// 已注释：读取芯片自身温度（摄氏度），未启用
int16_t Adafruit_TMP007::readDieTempC(void) {
  double Tdie = readRawDieTemperature();
  Tdie *= 0.03125; // 每单位等于 0.03125°C
#ifdef TMP007_DEBUG
  Serial.print("Tdie = "); Serial.print(Tdie); Serial.println(" C");
#endif
  return Tdie;
}
*/

// 读取物体温度（单位：摄氏度 * 10）
int16_t Adafruit_TMP007::readObjTempC10(void) {
  int16_t raw = read16(TMP007_TOBJ);  // 从 TOBJ 寄存器读取温度原始值

  // 检查最低位是否为 1，表示没有传感器安装（或数据无效）
  if (raw & 0x1) return (int16_t)TEMPERATURE_NOT_INSTALLED;

  // 将原始数据转换为摄氏度 × 10（固定点转换）
  int32_t temp = ((int32_t)raw) * 78125;  // 原始数据 * 0.078125°C = 78125 / 1000000
  return (int16_t)(temp / 1000000);       // 除以 1,000,000 得到°C × 10
}


/*
// 以下两个函数被注释掉了，但可以在调试模式下用于读取芯片温度和电压原始值

// 读取芯片温度原始值
int16_t Adafruit_TMP007::readRawDieTemperature(void) {
  int16_t raw = read16(TMP007_TDIE);

#if TMP007_DEBUG == 1
#ifdef TESTDIE
  raw = TESTDIE;  // 如果定义了测试数据，则使用它
#endif

  Serial.print("Raw Tambient: 0x"); Serial.print (raw, HEX);
  float v = raw / 4;
  v *= 0.03125;
  Serial.print(" ("); Serial.print(v); Serial.println(" *C)");
#endif

  raw >>= 2;  // 右移两位，去除低位无效位
  return raw;
}

// 读取原始电压（用于发射红外的热电堆电压）
int16_t Adafruit_TMP007::readRawVoltage(void) {
  int16_t raw;

  raw = read16(TMP007_VOBJ);

#if TMP007_DEBUG == 1
#ifdef TESTVOLT
  raw = TESTVOLT;
#endif

  Serial.print("Raw voltage: 0x"); Serial.print (raw, HEX);
  float v = raw;
  v *= 156.25;  // 转换为微伏
  v /= 1000;    // 转换为毫伏
  Serial.print(" ("); Serial.print(v); Serial.println(" uV)");
#endif

  return raw;
}
*/


/*********************************************************************
 * 下面两个函数是 I2C 通信的核心，负责从寄存器读取和写入 16 位数据
 *********************************************************************/

// 从指定寄存器读取 16 位（2 字节）数据
uint16_t Adafruit_TMP007::read16(uint8_t a) {
  uint16_t ret;

  // 第一次发送寄存器地址
  Wire.beginTransmission(_addr);  // 开始传输
#if (ARDUINO >= 100)
  Wire.write(a);  // 新版 Arduino 使用 write
#else
  Wire.send(a);   // 旧版使用 send
#endif
  Wire.endTransmission();  // 结束地址传输

  // 第二次读取数据
  Wire.beginTransmission(_addr);    // 开始传输
  Wire.requestFrom(_addr, (uint8_t)2); // 请求 2 字节数据

#if (ARDUINO >= 100)
  ret = Wire.read();   // 读取高字节
  ret <<= 8;
  ret |= Wire.read();  // 读取低字节
#else
  ret = Wire.receive();
  ret <<= 8;
  ret |= Wire.receive();
#endif

  Wire.endTransmission();  // 结束通信
  return ret;
}

// 向指定寄存器写入 16 位（2 字节）数据
void Adafruit_TMP007::write16(uint8_t a, uint16_t d) {
  Wire.beginTransmission(_addr);  // 开始传输
#if (ARDUINO >= 100)
  Wire.write(a);        // 寄存器地址
  Wire.write(d >> 8);   // 写高字节
  Wire.write(d);        // 写低字节
#else
  Wire.send(a);
  Wire.send((uint8_t)(d >> 8));
  Wire.send((uint8_t)d);
#endif
  Wire.endTransmission();  // 结束通信
}

#endif // TMP007_IS_ON_I2C
