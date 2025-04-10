// Copyright (C) 2015 Sam C. Lin
//
// 本程序是自由软件：你可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改它，
// 无论是版本3，还是许可证的其他版本。
//
// 本程序希望能够有用，但不提供任何担保；甚至没有对适销性或特定用途的隐含担保。详见
// GNU通用公共许可证的详细说明。
//
// 可以在LICENSE文件中找到GNU通用公共许可证的副本，或者在线查看：<http://www.gnu.org/licenses/>。

#include "avrstuff.h"

// 数字引脚初始化
void DigitalPin::init(volatile uint8_t* _reg, uint8_t idx, PinMode _mode)
{
  reg = _reg;  // 设置寄存器
  bit = 1 << idx;  // 设置位位置
  mode(_mode);  // 设置模式
}

// 设置引脚模式
void DigitalPin::mode(PinMode mode)
{
  switch(mode) {
  case INP_PU:  // 输入并启用上拉
    *port() |= bit;  // 启用上拉电阻
    // 然后继续设置为输入
  case INP:  // 输入
    *ddr() &= ~bit;  // 设置为输入模式
    break;
  default:  // 输出
    *ddr() |= bit;  // 设置为输出模式
  }
}

// ADC 引脚相关功能
#include "wiring_private.h"
#include "pins_arduino.h"

// 默认的参考电压模式
uint8_t AdcPin::refMode = DEFAULT;

// 初始化 ADC 引脚
void AdcPin::init(uint8_t _adcNum)
{
  channel = _adcNum;  // 设置 ADC 通道

  // 处理不同平台的通道编号
#if defined(analogChannelToChannel)
 #if defined(__AVR_ATmega32U4__)
  if (channel >= 18) channel -= 18;  // 如果是 ATmega32U4，调整通道编号
 #endif
  channel = analogChannelToChannel(channel);  // 转换通道编号
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (channel >= 54) channel -= 54;  // 对于 ATmega1280 和 ATmega2560，进行通道调整
#elif defined(__AVR_ATmega32U4__)
  if (channel >= 18) channel -= 18;  // 对于 ATmega32U4，调整通道编号
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  if (channel >= 24) channel -= 24;  // 对于 ATmega1284 和其他系列的调整
#else
  if (channel >= 14) channel -= 14;  // 默认的通道调整
#endif
}

// 读取 ADC 值
uint16_t AdcPin::read()
{
  uint8_t low, high;  // 存储低字节和高字节的值

  // 如果存在 ADCSRB 和 MUX5，设置通道范围（0-7 或 8-15）
#if defined(ADCSRB) && defined(MUX5)
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((channel >> 3) & 0x01) << MUX5);
#endif

  // 设置 ADC 的参考电压（高两位）并选择通道（低 4 位）
  // 同时也将 ADLAR 设置为 0（默认）
#if defined(ADMUX)
  ADMUX = (refMode << 6) | (channel & 0x07);
#endif

  // 进行转换前不加延时可能会读取到错误的通道数据
  //delay(1);

  // 启动 ADC 转换，等待转换完成
#if defined(ADCSRA) && defined(ADCL)
  sbi(ADCSRA, ADSC);  // 设置 ADSC 启动转换

  // 等待 ADSC 清零，表示转换完成
  while (bit_is_set(ADCSRA, ADSC));

  // 读取 ADC 结果，首先读取低字节，再读取高字节
  low  = ADCL;
  high = ADCH;
#else
  // 如果没有 ADC，则返回 0
  low  = 0;
  high = 0;
#endif

  // 合并高低字节并返回 16 位的 ADC 值
  return (high << 8) | low;
}
