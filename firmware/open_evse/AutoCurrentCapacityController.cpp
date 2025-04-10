// Copyright (C) 2016 Sam C. Lin
//
// 本程序是自由软件：你可以根据自由软件基金会发布的GNU通用公共许可证的条款重新分发和/或修改它，
// 无论是版本3，还是许可证的其他版本。
//
// 本程序希望能够有用，但不提供任何担保；甚至没有对适销性或特定用途的隐含担保。详见
// GNU通用公共许可证的详细说明。
//
// 可以在LICENSE文件中找到GNU通用公共许可证的副本，或者在线查看：<http://www.gnu.org/licenses/>。

#include "open_evse.h"

#ifdef PP_AUTO_AMPACITY  // 如果启用了自动电流容量功能

// 定义不同 ADC 值和对应的电流值（以安培为单位）
static PP_AMPS s_ppAmps[] = {
  {0,0},  // 电流为 0
  {93,63},  // ADC值 100 对应 93 安培
  {185,32}, // ADC值 220 对应 185 安培
  {415,20}, // ADC值 680 对应 415 安培
  {615,13}, // ADC值 1.5K 对应 615 安培
  {1023,0}  // 最大 ADC值对应 0 安培
};

// 自动电流容量控制器类的构造函数
AutoCurrentCapacityController::AutoCurrentCapacityController() :
  adcPP(PP_PIN)  // 初始化 adcPP 引脚
{
}

// 读取最大电流（安培值）
uint8_t AutoCurrentCapacityController::ReadPPMaxAmps()
{
  // 注意：应该多次采样并取平均值
  uint16_t adcval = adcPP.read();  // 读取 ADC 值

  uint8_t amps = 0;
  // 遍历预定义的 ADC 和电流值的数组
  for (uint8_t i = 1; i < sizeof(s_ppAmps) / sizeof(s_ppAmps[0]); i++) {
    // 根据 ADC 值判断对应的电流值
    if (adcval <= (s_ppAmps[i].adcVal - (s_ppAmps[i].adcVal - s_ppAmps[i - 1].adcVal) / 2)) {
      amps = s_ppAmps[i - 1].amps;  // 设置电流值
      break;
    }
  }

  // Serial.print("pp: ");Serial.print(adcval);Serial.print(" amps: ");Serial.println(amps);  // 可以用于调试输出

  ppMaxAmps = amps;  // 设置最大电流
  return amps;  // 返回电流值
}

// 自动设置电流容量
uint8_t AutoCurrentCapacityController::AutoSetCurrentCapacity()
{
  uint8_t amps = ReadPPMaxAmps();  // 读取最大电流值

  if (amps) {
    // 如果最大电流值不为零，并且 EVSE 控制器的电流容量大于最大电流值，则进行调整
    if (g_EvseController.GetCurrentCapacity() > ppMaxAmps) {
      g_EvseController.SetCurrentCapacity(ppMaxAmps, 1, 1);  // 设置新的电流容量
    }
    return 0;  // 成功设置
  }
  else {
    return 1;  // 读取电流值失败
  }
}

#endif //PP_AUTO_AMPACITY  // 结束自动电流容量功能的条件编译
