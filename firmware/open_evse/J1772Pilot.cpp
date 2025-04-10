/*
 * 此文件是 Open EVSE 项目的一部分。
 *
 * 版权所有 (c) 2011-2019 Sam C. Lin
 *
 * Open EVSE 是自由软件；你可以根据自由软件基金会发布的 GNU 通用公共许可证（GPL）进行重新分发和/或修改；
 * 要么使用版本 3，要么使用（根据你的选择）任何更高版本。

 * Open EVSE 被分发的目的是希望它能有用，
 * 但不提供任何担保；甚至没有适销性或适合特定目的的暗示性担保。请参见
 * GNU 通用公共许可证以获取更多详细信息。

 * 你应该已经收到了 GNU 通用公共许可证的副本
 * 随 Open EVSE 一起发放；请参见文件 COPYING。如果没有，请写信给
 * 自由软件基金会，地址为：59 Temple Place - Suite 330,
 * 波士顿，MA 02111-1307，美国。
 */
#include "open_evse.h"

#define TOP ((F_CPU / 2000000) * 1000) // 1KHz频率，周期为1000微秒

// J1772Pilot 类的初始化函数
void J1772Pilot::Init()
{
#ifdef PAFC_PWM
  // 设置定时器用于相位与频率正确的 PWM 输出
  TCCR1A = 0;  // 设置控制寄存器 A
  ICR1 = TOP;  // 设置定时器的上限
  // WGM13 -> 选择 P&F 模式 CS10 -> 分频器设置为 1
  TCCR1B = _BV(WGM13) | _BV(CS10);

#if (PILOT_IDX == 1) // PB1
  DDRB |= _BV(PORTB1);  // 设置 PB1 引脚为输出
  TCCR1A |= _BV(COM1A1);  // 启用 PWM 输出到 PB1
#else // PB2
  DDRB |= _BV(PORTB2);  // 设置 PB2 引脚为输出
  TCCR1A |= _BV(COM1B1);  // 启用 PWM 输出到 PB2
#endif // PILOT_IDX
#else // 快速 PWM
  pin.init(PILOT_REG, PILOT_IDX, DigitalPin::OUT);  // 初始化引脚为输出模式
#endif

  SetState(PILOT_STATE_P12); // 设置 Pilot 信号为 12V 稳态
}

// 设置 Pilot 信号的状态
// PILOT_STATE_P12 = 稳态 12V (EVSE_STATE_A - 车辆未连接)
// PILOT_STATE_N12 = 稳态 -12V (EVSE_STATE_F - 故障)
void J1772Pilot::SetState(PILOT_STATE state)
{
  AutoCriticalSection asc;  // 临界区，确保线程安全

#ifdef PAFC_PWM
  // 如果状态是 PILOT_STATE_P12，则设置 PWM 输出为 12V
  if (state == PILOT_STATE_P12) {
#if (PILOT_IDX == 1)
    OCR1A = TOP;  // 设置 PWM 输出到 PB1
#else
    OCR1B = TOP;  // 设置 PWM 输出到 PB2
#endif
  }
  else {
    // 设置为 0，关闭 PWM 输出
#if (PILOT_IDX == 1) // PB1
    OCR1A = 0;
#else // PB2
    OCR1B = 0;
#endif
  }
#else // 快速 PWM
  TCCR1A = 0;  // 关闭 PWM，通过禁用 COM1A1, COM1A0, COM1B1, COM1B0
  pin.write((state == PILOT_STATE_P12) ? 1 : 0);  // 设置引脚的高低电平
#endif // PAFC_PWM

  m_State = state;  // 更新当前状态
}

// 设置 EVSE 当前容量（单位：安培），并输出 1KHz 方波到数字引脚 10，通过定时器 1
int J1772Pilot::SetPWM(int amps)
{
#ifdef PAFC_PWM
  // 计算占空比：OCR1A(B) / ICR1 * 100%

  unsigned cnt;
  if ((amps >= 6) && (amps <= 51)) {
    // 安培值 = (占空比 %) X 0.6
    cnt = amps * (TOP / 60);  // 计算对应的占空比计数值
  } else if ((amps > 51) && (amps <= 80)) {
    // 安培值 = (占空比 % - 64) X 2.5
    cnt = (amps * (TOP / 250)) + (64 * (TOP / 100));  // 计算对应的占空比计数值
  }
  else {
    return 1;  // 返回 1 表示无效的安培值
  }

#if (PILOT_IDX == 1) // PB1
  OCR1A = cnt;  // 设置 PWM 输出占空比
#else // PB2
  OCR1B = cnt;  // 设置 PWM 输出占空比
#endif

  m_State = PILOT_STATE_PWM;  // 设置状态为 PWM

  return 0;  // 返回 0 表示成功
#else // 快速 PWM
  uint8_t ocr1b = 0;
  if ((amps >= 6) && (amps <= 51)) {
    ocr1b = 25 * amps / 6 - 1;  // J1772 协议中定义：可用电流 = (占空比 %) X 0.6
  } else if ((amps > 51) && (amps <= 80)) {
    ocr1b = amps + 159;  // J1772 协议中定义：可用电流 = (占空比 % - 64) X 2.5
  }
  else {
    return 1;  // 返回 1 表示无效的安培值
  }

  if (ocr1b) {
    AutoCriticalSection asc;  // 临界区，确保线程安全
    // 初始化定时器 1：
    // 16MHz / 64 / (OCR1A+1) / 2 输出到数字引脚 9
    // 16MHz / 64 / (OCR1A+1) 输出到数字引脚 10
    // 数字引脚 10 输出 1KHz 可调占空比，数字引脚 9 输出固定 50% 频率为 500Hz
    // 引脚 10 的占空比 = (OCR1B+1)/(OCR1A+1)

    TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);  // 设置 PWM 模式
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);  // 设置时钟源和预分频
    OCR1A = 249;  // 设置周期

    // 10% = 24 , 96% = 239
    OCR1B = ocr1b;  // 设置占空比

    m_State = PILOT_STATE_PWM;  // 设置状态为 PWM
    return 0;  // 返回 0 表示成功
  }
  else {  // 如果占空比为 0
    // 无效的安培值
    return 1;  // 返回 1 表示无效
  }
#endif // PAFC_PWM
}
