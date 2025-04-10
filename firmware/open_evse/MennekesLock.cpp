// -*- C++ -*-
/*
 * Open EVSE 固件
 *
 * 版权所有 (c) 2013-2021 Sam C. Lin <lincomatic@gmail.com>
 *
 * 本文件是Open EVSE的一部分。

 * Open EVSE 是自由软件；你可以根据自由软件基金会发布的GNU通用公共许可证的条款进行重新分发和/或修改；
 * 无论是版本3，还是（根据你的选择）任何更高版本。

 * Open EVSE以“按现状提供”的形式分发，旨在对你有所帮助，
 * 但不提供任何明示或暗示的担保；包括但不限于适销性或适合特定用途的担保。详情请参阅
 * GNU通用公共许可证。

 * 你应该已经收到了GNU通用公共许可证的副本
 * 随着Open EVSE一起提供；请参阅文件COPYING。如果没有，请写信给
 * 自由软件基金会，地址为59 Temple Place - Suite 330，
 * 波士顿，MA 02111-1307，美国。
 */
#include "open_evse.h"

#ifdef MENNEKES_LOCK

// MennekesLock类的初始化函数
void MennekesLock::Init()
{
  pinA.init(MENNEKES_LOCK_PINA_REG, MENNEKES_LOCK_PINA_IDX, DigitalPin::OUT);  // 初始化PinA为输出
  pinB.init(MENNEKES_LOCK_PINB_REG, MENNEKES_LOCK_PINB_IDX, DigitalPin::OUT);  // 初始化PinB为输出

  Unlock(1);  // 在初始化时解锁
}

// 锁定操作，force参数决定是否强制执行
void MennekesLock::Lock(int8_t force)
{
  // 如果force为真，或者当前没有锁定，则执行锁定操作
  if (force || !isLocked) {
    pinA.write(1);  // 设置PinA为高
    pinB.write(0);  // 设置PinB为低
    delay(300);  // 延迟300毫秒
    pinA.write(0);  // 设置PinA为低
    pinB.write(0);  // 设置PinB为低
    isLocked = 1;  // 更新状态为已锁定
  }
}

// 解锁操作，force参数决定是否强制执行
void MennekesLock::Unlock(int8_t force)
{
  // 如果force为真，或者当前已经锁定，则执行解锁操作
  if (force || isLocked) {
    pinA.write(0);  // 设置PinA为低
    pinB.write(1);  // 设置PinB为高
    delay(300);  // 延迟300毫秒
    pinA.write(0);  // 设置PinA为低
    pinB.write(0);  // 设置PinB为低
    isLocked = 0;  // 更新状态为未锁定
  }
}

#endif // MENNEKES_LOCK
