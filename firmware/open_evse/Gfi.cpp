/*
 * 该文件是 Open EVSE 的一部分。
 *
 * 版权所有 (c) 2011-2019 Sam C. Lin
 *
 * Open EVSE 是自由软件；你可以在 GNU 通用公共许可证（由自由软件基金会发布）的条款下重新分发和/或修改它；无论是版本 3，还是（你选择的）任何更高版本。
 *
 * Open EVSE 被分发的目的是希望它能有用，但不提供任何担保；甚至没有对适销性或特定用途的隐含担保。详见 GNU 通用公共许可证的详细说明。
 *
 * 你应该已收到一份 GNU 通用公共许可证副本；与 Open EVSE 一起，查看文件 COPYING。如果没有，请写信给自由软件基金会，地址为：
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA。
 */

#include "open_evse.h"

#ifdef GFI  // 如果启用了 GFI（漏电保护）功能

// 漏电保护中断服务函数
void gfi_isr()
{
  g_EvseController.SetGfiTripped();  // 设置漏电保护跳闸
}

// Gfi 类的初始化函数
void Gfi::Init(uint8_t v6)
{
  pin.init(GFI_REG, GFI_IDX, DigitalPin::INP);  // 初始化 GFI 引脚，设置为输入
  // GFI 触发时，在上升沿产生中断
  attachInterrupt(GFI_INTERRUPT, gfi_isr, RISING);  // 为 GFI 引脚设置中断，触发时调用 gfi_isr

#ifdef GFI_SELFTEST  // 如果启用了自检功能
  volatile uint8_t *reg = GFITEST_REG;  // 自检寄存器地址
  volatile uint8_t idx = GFITEST_IDX;  // 自检寄存器索引
#ifdef OEV6  // 如果是版本 V6
  if (v6) {
    reg = V6_GFITEST_REG;  // 设置 V6 的自检寄存器地址
    idx = V6_GFITEST_IDX;  // 设置 V6 的自检寄存器索引
  }
#endif // OEV6
  pinTest.init(reg, idx, DigitalPin::OUT);  // 初始化自检引脚，设置为输出
#endif

  Reset();  // 重置 GFI
}

// 重置 GFI 逻辑
void Gfi::Reset()
{
  WDT_RESET();  // 重置看门狗计时器

#ifdef GFI_SELFTEST  // 如果启用了自检功能
  testInProgress = 0;  // 清除测试进行标志
  testSuccess = 0;  // 清除测试成功标志
#endif // GFI_SELFTEST

  if (pin.read()) m_GfiFault = 1;  // 如果 GFI 引脚为高电平，则设置漏电保护故障标志
  else m_GfiFault = 0;  // 如果 GFI 引脚为低电平，则清除故障标志
}

#ifdef GFI_SELFTEST  // 如果启用了自检功能

// 自检函数
uint8_t Gfi::SelfTest()
{
  int i;
  // 等待 GFI 引脚清零
  for (i = 0; i < 20; i++) {
    WDT_RESET();  // 重置看门狗计时器
    if (!pin.read()) break;  // 如果 GFI 引脚为低电平，跳出循环
    delay(50);  // 延迟 50 毫秒
  }
  if (i == 20) return 2;  // 如果超时未清除，则返回错误代码 2

  testInProgress = 1;  // 设置测试进行标志
  testSuccess = 0;  // 清除测试成功标志
  for (int i = 0; !testSuccess && (i < GFI_TEST_CYCLES); i++) {
    pinTest.write(1);  // 使自检引脚为高电平
    delayMicroseconds(GFI_PULSE_ON_US);  // 延迟一定时间（高电平时间）
    pinTest.write(0);  // 使自检引脚为低电平
    delayMicroseconds(GFI_PULSE_OFF_US);  // 延迟一定时间（低电平时间）
  }

  // 等待 GFI 引脚清零
  for (i = 0; i < 40; i++) {
    WDT_RESET();  // 重置看门狗计时器
    if (!pin.read()) break;  // 如果 GFI 引脚为低电平，跳出循环
    delay(50);  // 延迟 50 毫秒
  }
  if (i == 40) return 3;  // 如果超时未清除，则返回错误代码 3

#ifndef OPENEVSE_2  // 如果不是 OPENEVSE 2 版本
  // 在测试接近闭合继电器之前，偶尔会出现虚假的 GFI 故障。
  // 等待更多时间让系统稳定下来
  // 如果电路中有 10uF 电容，延迟是必要的，直到电容放电完成，系统才会稳定
  wdt_delay(1000);  // 延迟 1000 毫秒
#endif // OPENEVSE_2

  m_GfiFault = 0;  // 清除漏电保护故障标志
  testInProgress = 0;  // 清除测试进行标志

  return !testSuccess;  // 返回测试是否成功（如果成功返回 0，否则返回 1）
}

#endif // GFI_SELFTEST  // 结束自检功能的条件编译
#endif // GFI  // 结束 GFI 功能的条件编译
