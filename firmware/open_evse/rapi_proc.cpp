// -*- C++ -*-
/*
 * Open EVSE 固件
 *
 * 版权所有 (c) 2013-2023 Sam C. Lin <lincomatic@gmail.com>
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

#if defined(ARDUINO) && (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h" // 这个应该不需要，但有时Arduino会搞错，把它放在了# ifdef里面
#endif // ARDUINO
#include "open_evse.h"

#ifdef RAPI
// 定义RAPI协议版本
const char RAPI_VER[] PROGMEM = RAPIVER;

#ifdef MCU_ID_LEN
// 如果MCU_ID_LEN定义了，我们需要获取MCU的ID
#include <avr/boot.h>
void getMcuId(uint8_t *mcuid)
{
  for (int i = 0; i < MCU_ID_LEN; i++) {
    mcuid[i] = boot_signature_byte_get(0x0E + i); // 获取MCU的ID
  }
}
#endif // MCU_ID_LEN

// 将2位十六进制字符串转换为uint8_t
uint8_t htou8(const char *s)
{
  uint8_t u = 0;
  for (int i = 0; i < 2; i++) {
    char c = s[i];
    if (c != '\0') {
      if (i == 1) u <<= 4;
      if ((c >= '0') && (c <= '9')) {
        u += c - '0';
      }
      else if ((c >= 'A') && (c <= 'F')) {
        u += c - 'A' + 10;
      }
      else if ((c >= 'a') && (c <= 'f')) {
        u += c - 'a' + 10;
      }
      else {
        // 如果字符无效，返回0
        return 0;
      }
    }
  }
  return u;
}

// 将十进制字符串转换为uint32_t
uint32_t dtou32(const char *s)
{
  uint32_t u = 0;
  while (*s) {
    u *= 10;
    u += *(s++) - '0'; // 转换每个数字
  }
  return u;
}

#ifdef RAPI_I2C
// 从主机接收数据 - 提示：这是一个中断服务例程（ISR）调用！
// 提示2：不要在这里处理数据，数据收集仅在主循环中进行处理！
void receiveEvent(int numBytes)
{
  // 在此不做任何处理
}
#endif // RAPI_I2C

// RAPI处理器构造函数
EvseRapiProcessor::EvseRapiProcessor()
{
  curReceivedSeqId = INVALID_SEQUENCE_ID; // 当前接收的序列ID无效
#ifdef RAPI_SENDER
  curSentSeqId = INVALID_SEQUENCE_ID; // 当前发送的序列ID无效
#endif
}

// 初始化RAPI处理器
void EvseRapiProcessor::init()
{
  echo = 0;  // 不回显
  reset();   // 重置
}

// 处理命令
int EvseRapiProcessor::doCmd()
{
  int rc = 1;

  int bcnt = available();  // 检查是否有数据可用
  if (bcnt) {
    for (int i = 0; i < bcnt; i++) {
      char c = read();  // 读取数据
      if (echo) write(c);  // 如果启用了回显，则写回数据

      if (c == ESRAPI_SOC) {
        buffer[0] = ESRAPI_SOC;
        bufCnt = 1;
      }
      else if (buffer[0] == ESRAPI_SOC) {
        if (bufCnt < ESRAPI_BUFLEN) {
          if (c == ESRAPI_EOC) {
            buffer[bufCnt++] = 0;  // 结束字符
            if (!tokenize(buffer)) {
              rc = processCmd();  // 处理命令
            }
            else {
              reset();  // 重置
              curReceivedSeqId = INVALID_SEQUENCE_ID; // 接收序列ID无效
              response(0);  // 响应
            }
          }
          else {
            buffer[bufCnt++] = c;  // 将字符添加到缓冲区
          }
        }
        else { // 如果字符太多
          reset();  // 重置
        }
      }
    }
  }

  return rc;
}

// 发送启动通知
void EvseRapiProcessor::sendBootNotification()
{
  sprintf(g_sTmp, "%cAB %02x ", ESRAPI_SOC, g_EvseController.GetState());
  char *s = g_sTmp + strlen(g_sTmp);
  GetVerStr(s);  // 获取版本信息
  appendChk(g_sTmp);  // 追加校验和
  writeStart();  // 开始写数据
  write(g_sTmp);  // 写入数据
  writeEnd();  // 结束写数据
}

// 发送EVSE状态信息
void EvseRapiProcessor::sendEvseState()
{
  sprintf(g_sTmp, "%cAT %02x %02x %d %04x", ESRAPI_SOC, g_EvseController.GetState(),
          g_EvseController.GetPilotState(), g_EvseController.GetCurrentCapacity(),
          g_EvseController.GetVFlags());
  appendChk(g_sTmp);  // 追加校验和
  writeStart();  // 开始写数据
  write(g_sTmp);  // 写入数据
  writeEnd();  // 结束写数据
}

#ifdef RAPI_WF
// 设置WiFi模式
void EvseRapiProcessor::setWifiMode(uint8_t mode)
{
  sprintf(g_sTmp, "%cWF %02x", ESRAPI_SOC, (int)mode);
  appendChk(g_sTmp);  // 追加校验和
  writeStart();  // 开始写数据
  write(g_sTmp);  // 写入数据
  writeEnd();  // 结束写数据
}
#endif // RAPI_WF

#ifdef RAPI_BTN
// 发送按钮按压事件，传入参数 long_press 表示是否为长按
void EvseRapiProcessor::sendButtonPress(uint8_t long_press)
{
  // 格式化字符串为 AN <long_press>，并计算校验和
  sprintf(g_sTmp,"%cAN %d", ESRAPI_SOC, long_press);
  appendChk(g_sTmp); // 添加校验和
  writeStart(); // 开始发送数据
  write(g_sTmp); // 发送数据
  writeEnd(); // 结束数据发送
}
#endif // RAPI_BTN

// 解析接收到的命令并验证校验和
int EvseRapiProcessor::tokenize(char *buf)
{
  // 设定第一个令牌为缓冲区中的第一个字符（跳过SOC标记）
  tokens[0] = &buf[1];
  char *s = &buf[2]; // 从第二个字符开始
  tokenCnt = 1; // 初始化令牌计数
  uint8_t achkSum = ESRAPI_SOC + buf[1]; // 初始化加法校验和
  uint8_t xchkSum = ESRAPI_SOC ^ buf[1]; // 初始化XOR校验和
  uint8_t hchkSum; // 存储接收到的校验和
  uint8_t chktype = 0; // 校验和类型，0 = 无，1 = 加法，2 = XOR

  // 遍历字符串直到末尾
  while (*s) {
    if (*s == ' ') { // 如果遇到空格，表示一个新的令牌开始
      // 如果令牌数量已经达到最大值，退出解析
      if (tokenCnt >= ESRAPI_MAX_ARGS) {
        chktype = 255; // 标记为无效
        break;
      }
      else {
        achkSum += *s; // 更新加法校验和
        xchkSum ^= *s; // 更新XOR校验和
        *s = '\0'; // 用空字符结束当前令牌
        tokens[tokenCnt++] = ++s; // 设置下一个令牌
      }
    }
    else if ((*s == '*') || (*s == '^')) { // 检查是否为校验和类型指示符
      if (*s == '*') chktype = 1; // 加法校验和
      else if (*s == '^') chktype = 2; // XOR校验和
      *(s++) = '\0'; // 结束令牌
      hchkSum = htou8(s); // 提取接收到的校验和
      break;
    }
    else {
      achkSum += *s; // 更新加法校验和
      xchkSum ^= *(s++); // 更新XOR校验和
    }
  }

  // 校验接收到的校验和是否匹配
  int rc = ((chktype == 0) || // 如果没有校验和
           ((chktype == 1) && (hchkSum == achkSum)) || // 加法校验和匹配
           ((chktype == 2) && (hchkSum == xchkSum))) ? 0 : 1; // XOR校验和匹配
  if (rc) tokenCnt = 0; // 如果校验失败，重置令牌计数
  // 可以取消注释下面的调试信息
  // sprintf(g_sTmp,"trc: %d",rc);
  // g_EIRP.writeStr(g_sTmp);

  return rc; // 返回校验结果
}

uint8_t g_inRapiCommand = 0; // 标记当前是否在处理RAPI命令

// 处理接收到的RAPI命令
int EvseRapiProcessor::processCmd()
{
  g_inRapiCommand = 1; // 标记正在处理RAPI命令

  UNION4B u1,u2,u3,u4; // 定义联合体用于存储数据
  int rc = -1; // 默认返回值为-1，表示命令处理失败

#ifdef RAPI_SENDER
  // 丢弃不需要的响应，这些可能是来自我们已经超时的命令
  if (isRespToken()) { // 如果是响应令牌
    g_inRapiCommand = 0; // 处理完毕，重置标志
    return rc; // 返回失败的结果
  }
#endif // RAPI_SENDER

  // 初始化当前接收的序列ID为无效值
  curReceivedSeqId = INVALID_SEQUENCE_ID;
  const char *seqtoken = tokens[tokenCnt-1]; // 获取当前令牌数组中的最后一个元素
  if ((tokenCnt > 1) && (*seqtoken == ESRAPI_SOS)) { // 如果令牌数量大于1并且最后一个令牌是 SOS
      curReceivedSeqId = htou8(++seqtoken); // 解析序列ID
      tokenCnt--; // 减少令牌数量
  }

  // 用bufCnt作为标志，在response()中表示有数据需要写入
  bufCnt = 0;

  char *s = tokens[0]; // 获取第一个令牌
  switch(*(s++)) { // 处理第一个令牌字符
  case 'F': // 功能设置
      switch(*s) { // 根据第二个字符处理不同的功能
      case '0': // 启用/禁用LCD更新
          if (tokenCnt == 2) {
              g_OBD.DisableUpdate((*tokens[1] == '0') ? 1 : 0); // 根据令牌值决定是否禁用更新
              if (*tokens[1] != '0') g_OBD.Update(OBD_UPD_FORCE); // 如果不是禁用，则强制更新
              rc = 0;
          }
          break;
  #ifdef BTN_MENU
      case '1': // 模拟前面板短按
          g_BtnHandler.DoShortPress(g_EvseController.InFaultState()); // 模拟按钮按下
          g_OBD.Update(OBD_UPD_FORCE); // 强制更新
          rc = 0;
          break;
  #endif // BTN_MENU
  #ifdef LCD16X2
      case 'B': // LCD背光控制
          if (tokenCnt == 2) {
              g_OBD.LcdSetBacklightColor(dtou32(tokens[1])); // 设置LCD背光颜色
              rc = 0;
          }
          break;
  #endif // LCD16X2
      case 'D': // 禁用EVSE
          g_EvseController.Disable(); // 禁用EVSE
          rc = 0;
          break;
      case 'E': // 启用EVSE
          g_EvseController.Enable(); // 启用EVSE
          rc = 0;
          break;
      case 'F': // 启用/禁用特性
          if (tokenCnt == 3) {
              u1.u8 = (uint8_t)(*tokens[2] - '0'); // 获取令牌值并转换为数字
              if (u1.u8 <= 1) { // 确保值是0或1
                  rc = 0;
                  switch(*tokens[1]) { // 根据第二个令牌决定执行的操作
  #ifdef BTN_MENU
                  case 'B': // 启用前按钮
                      g_EvseController.ButtonEnable(u1.u8); // 启用按钮功能
                      break;
  #endif // BTN_MENU
                  case 'D': // 二极管检查
                      g_EvseController.EnableDiodeCheck(u1.u8); // 启用二极管检查
                      break;
                  case 'E': // 命令回显
                      echo = ((u1.u8 == '0') ? 0 : 1); // 设置回显标志
                      break;
  #ifdef ADVPWR
                  case 'F': // GFI自检
                      g_EvseController.EnableGfiSelfTest(u1.u8); // 启用GFI自检
                      break;
                  case 'G': // 地面检查
                      g_EvseController.EnableGndChk(u1.u8); // 启用地面检查
                      break;
                  case 'R': // 卡住继电器检查
                      g_EvseController.EnableStuckRelayChk(u1.u8); // 启用继电器卡住检查
                      break;
  #endif // ADVPWR
  #ifdef TEMPERATURE_MONITORING
                  case 'T': // 温度监控
                      g_EvseController.EnableTempChk(u1.u8); // 启用温度检查
                      break;
  #endif // TEMPERATURE_MONITORING
                  case 'V': // 通风要求检查
                      g_EvseController.EnableVentReq(u1.u8); // 启用通风检查
                      break;
                  default: // 未知操作
                      rc = -1;
                  }
              }
          }
          break;
  #ifdef LCD16X2
      case 'P': // 打印到LCD
          if ((tokenCnt >= 4) && !g_EvseController.InHardFault()) {
              u1.u = dtou32(tokens[1]); // 获取x坐标
              u2.u = dtou32(tokens[2]); // 获取y坐标
              // 恢复由于分割令牌时替换成空字符的空格
              for (u3.i = 4; u3.i < tokenCnt; u3.i++) {
                  *(tokens[u3.i] - 1) = ' ';
              }
              g_OBD.LcdPrint(u1.u, u2.u, tokens[3]); // 打印到LCD
              rc = 0;
          }
          break;
  #endif // LCD16X2
      case 'R': // 重置EVSE
          g_EvseController.Reboot(); // 重启EVSE
          rc = 0;
          break;
      case 'S': // 睡眠模式
          g_EvseController.Sleep(); // 使EVSE进入睡眠状态
          rc = 0;
          break;
      }
      break;

  case 'S': // 设置参数
      switch(*s) { // 根据第二个字符处理不同的设置
  #ifdef LCD16X2
      case '0': // 设置LCD类型
          if (tokenCnt == 2) {
  #ifdef RGBLCD
              rc = g_EvseController.SetBacklightType((*tokens[1] == '0') ? BKL_TYPE_MONO : BKL_TYPE_RGB); // 设置LCD背光类型
  #endif // RGBLCD
          }
          break;
  #endif // LCD16X2
  #ifdef RTC
      case '1': // 设置RTC（实时时钟）
          if (tokenCnt == 7) {
              extern void SetRTC(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mn, uint8_t s);
              SetRTC(dtou32(tokens[1]), dtou32(tokens[2]), dtou32(tokens[3]),
                     dtou32(tokens[4]), dtou32(tokens[5]), dtou32(tokens[6])); // 设置RTC时间
              rc = 0;
          }
          break;
  #endif // RTC
  #if defined(AMMETER) && defined(ECVF_AMMETER_CAL)
      case '2': // 电表校准模式
          if (tokenCnt == 2) {
              g_EvseController.EnableAmmeterCal((*tokens[1] == '1') ? 1 : 0); // 启用或禁用电表校准
              rc = 0;
          }
          break;
  #endif // AMMETER && ECVF_AMMETER_CAL
  #ifdef TIME_LIMIT
      case '3': // 设置时间限制
          if (tokenCnt == 2) {
              if (g_EvseController.LimitsAllowed()) { // 如果允许设置限制
                  g_EvseController.SetTimeLimit15(dtou32(tokens[1])); // 设置时间限制
                  if (!g_OBD.UpdatesDisabled()) g_OBD.Update(OBD_UPD_FORCE); // 如果更新没有禁用，强制更新
                  rc = 0;
              }
          }
          break;
  #endif // TIME_LIMIT
  #if defined(AUTH_LOCK) && !defined(AUTH_LOCK_REG)
      case '4': // 授权锁定
          if (tokenCnt == 2) {
              g_EvseController.AuthLock((int8_t)dtou32(tokens[1]), 1); // 设置授权锁定
              rc = 0;
          }
          break;
  #endif // AUTH_LOCK && !AUTH_LOCK_REG
  #ifdef MENNEKES_LOCK
      case '5': // Mennekes设置
          if (tokenCnt == 2) {
              rc = 0;
              switch(*tokens[1]) { // 根据令牌值选择不同操作
              case '0':
                  g_EvseController.UnlockMennekes(); // 解锁Mennekes
                  break;
              case '1':
                  g_EvseController.LockMennekes(); // 锁定Mennekes
                  break;
              case 'A':
                  g_EvseController.ClrMennekesManual(); // 清除手动设置
                  break;
              case 'M':
                  g_EvseController.SetMennekesManual(); // 设置为手动模式
                  break;
              default:
                  rc = 1;
              }
          }
          break;
  #endif // MENNEKES_LOCK
  #ifdef AMMETER
      case 'A': // 设置电流系数
          if (tokenCnt == 3) {
              g_EvseController.SetCurrentScaleFactor(dtou32(tokens[1])); // 设置电流系数
              g_EvseController.SetAmmeterCurrentOffset(dtou32(tokens[2])); // 设置电流偏移
              rc = 0;
          }
          break;
  #endif // AMMETER
  #ifdef BOOTLOCK
      case 'B': // 启动锁定
          if (g_EvseController.InFaultState()) { // 如果处于故障状态
              strcpy(buffer, "1"); // 返回1表示有故障
          } else {
              g_EvseController.ClearBootLock(); // 清除启动锁定
              strcpy(buffer, "0"); // 返回0表示无故障
          }
          bufCnt = 1; // 标记响应文本输出
          rc = 0;
          break;
  #endif // BOOTLOCK
      case 'C': // 设置电流容量
          if ((tokenCnt == 2) || (tokenCnt == 3)) {
              u2.u8 = dtou32(tokens[1]); // 获取电流容量
              if ((tokenCnt == 3) && (*tokens[2] == 'M')) { // 如果有M标志，则设置最大电流容量
                  rc = g_EvseController.SetMaxHwCurrentCapacity(u2.u8); // 设置最大硬件电流容量
                  sprintf(buffer, "%d", (int)g_EvseController.GetMaxHwCurrentCapacity()); // 输出最大电流容量
              }
              else {
                  if (tokenCnt == 3) {
                      u1.u8 = 1; // 设置为volatile
                  }
                  else {
                      u1.u8 = 0; // 设置为非volatile
                  }
  #ifdef TEMPERATURE_MONITORING
                  if (g_TempMonitor.OverTemperature() &&
                      (u2.u8 > g_EvseController.GetCurrentCapacity())) {
                      // 如果当前温度过高，不允许增加电流容量
                      rc = 1;
                  }
                  else {
                      rc = g_EvseController.SetCurrentCapacity(u2.u8, 1, u1.u8); // 设置电流容量
                  }
  #else // !TEMPERATURE_MONITORING
                  rc = g_EvseController.SetCurrentCapacity(u2.u8, 1, u1.u8); // 设置电流容量
  #endif // TEMPERATURE_MONITORING

                  sprintf(buffer, "%d", (int)g_EvseController.GetCurrentCapacity()); // 输出当前电流容量
              }
              bufCnt = 1; // 标记响应文本输出
          }
          break;
#ifdef CHARGE_LIMIT
    case 'H': // 设置充电限制
      if (tokenCnt == 2) {
        // 如果允许设置限制
        if (g_EvseController.LimitsAllowed()) {
          // 设置充电限制（以千瓦时为单位）
          g_EvseController.SetChargeLimitkWh(dtou32(tokens[1]));
          // 如果更新没有被禁用，则强制更新
          if (!g_OBD.UpdatesDisabled()) g_OBD.Update(OBD_UPD_FORCE);
          rc = 0; // 成功
        }
      }
      break;
#endif // CHARGE_LIMIT

#ifdef KWH_RECORDING
    case 'K': // 设置累计的千瓦时
      g_EnergyMeter.SetTotkWh(dtou32(tokens[1])); // 设置累计的千瓦时
      g_EnergyMeter.SaveTotkWh(); // 保存累计千瓦时
      rc = 0; // 成功
      break;
#endif // KWH_RECORDING

    case 'L': // 设置服务级别
      if (tokenCnt == 2) {
        switch(*tokens[1]) {
          case '1': // 服务级别 1
          case '2': // 服务级别 2
            g_EvseController.SetSvcLevel(*tokens[1] - '0', 1); // 设置服务级别
            #if defined(ADVPWR) && defined(AUTOSVCLEVEL)
            g_EvseController.EnableAutoSvcLevel(0); // 禁用自动服务级别
            #endif
            rc = 0; // 成功
            break;
          #if defined(ADVPWR) && defined(AUTOSVCLEVEL)
          case 'A': // 启用自动服务级别
            g_EvseController.EnableAutoSvcLevel(1);
            rc = 0; // 成功
            break;
          #endif // ADVPWR && AUTOSVCLEVEL
        }
      }
      break;

#ifdef VOLTMETER
    case 'M': // 设置电压计
      if (tokenCnt == 3) {
        g_EvseController.SetVoltmeter(dtou32(tokens[1]), dtou32(tokens[2])); // 设置电压计的值
        rc = 0; // 成功
      }
      break;
#endif // VOLTMETER

#ifdef DELAYTIMER
    case 'T': // 设置定时器
      if (tokenCnt == 5) {
        extern DelayTimer g_DelayTimer; // 延迟定时器实例
        u1.u8 = (uint8_t)dtou32(tokens[1]); // 设置开始定时器的开始时间
        u2.u8 = (uint8_t)dtou32(tokens[2]); // 设置开始定时器的结束时间
        u3.u8 = (uint8_t)dtou32(tokens[3]); // 设置停止定时器的开始时间
        u4.u8 = (uint8_t)dtou32(tokens[4]); // 设置停止定时器的结束时间
        // 如果所有的定时器时间均为零，则禁用定时器
        if ((u1.u8 == 0) && (u2.u8 == 0) && (u3.u8 == 0) && (u4.u8 == 0)) {
          g_DelayTimer.Disable();
        } else {
          // 启用并设置定时器
          g_DelayTimer.SetStartTimer(u1.u8, u2.u8);
          g_DelayTimer.SetStopTimer(u3.u8, u4.u8);
          g_DelayTimer.Enable();
        }
        rc = 0; // 成功
      }
      break;
#endif // DELAYTIMER

#if defined(KWH_RECORDING) && !defined(VOLTMETER)
    case 'V': // 设置电压（当不使用电压计时）
      if (tokenCnt == 2) {
        g_EvseController.SetMV(dtou32(tokens[1])); // 设置电压
        rc = 0; // 成功
      }
      break;
#endif // defined(KWH_RECORDING) && !defined(VOLTMETER)

#ifdef HEARTBEAT_SUPERVISION
    case 'Y': // 心跳监控
      if (tokenCnt == 1)  { // 这是一个心跳请求
        rc = g_EvseController.HsPulse(); // 发送心跳信号
      }
      else if (tokenCnt == 3) { // 这是一个完整的心跳监控命令，包含两个参数
        rc = 0;
        u1.u16 = (uint16_t)dtou32(tokens[1]); // HS间隔，单位：秒。0表示禁用
        u2.u8 = (uint8_t)dtou32(tokens[2]);  // HS回退电流，单位：安培
        if (u1.u16 == 0) { // 检查是否禁用
          rc = g_EvseController.HsRestoreAmpacity(); // 恢复电流容量
        }
        rc |= g_EvseController.HeartbeatSupervision(u1.u16, u2.u8); // 配置心跳监控
      }
      else if (tokenCnt == 2) { // 这是一个心跳监控丢失的确认命令
        u1.u8 = (uint8_t)dtou32(tokens[1]); // 魔法值（标识丢失）
        rc = g_EvseController.HsAckMissedPulse(u1.u8); // 确认丢失的心跳信号
      }
      else { // 参数数量无效，返回错误
        rc = 1; // 无效的参数数量
      }
      sprintf(buffer,"%d %d %d", g_EvseController.GetHearbeatInterval(), g_EvseController.GetHearbeatCurrent(), g_EvseController.GetHearbeatTrigger());
      bufCnt = 1; // 标记响应文本输出
      break;
#endif // HEARTBEAT_SUPERVISION

    }
    break;

  case 'G': // 获取参数
    switch(*s) {
    case '0': // 获取EV连接状态
      {
	uint8_t connstate;
	if (g_EvseController.GetPilot()->GetState() == PILOT_STATE_N12) {
	  connstate = 2; // 未知
	}
	else {
	  if (g_EvseController.EvConnected()) connstate = 1; // EV已连接
	  else connstate = 0; // EV未连接
	}
	sprintf(buffer,"%d",(int)connstate); // 输出连接状态
      }
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#ifdef TIME_LIMIT
    case '3': // 获取时间限制
      sprintf(buffer,"%d",(int)g_EvseController.GetTimeLimit15()); // 输出15分钟时间限制
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#endif // TIME_LIMIT
#if defined(AUTH_LOCK) && !defined(AUTH_LOCK_REG)
    case '4': // 获取认证锁状态
      sprintf(buffer,"%d",(int)g_EvseController.AuthLockIsOn() ? 1 : 0); // 输出认证锁是否启用
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#endif // AUTH_LOCK && !AUTH_LOCK_REG
#ifdef MENNEKES_LOCK
    case '5': // 获取Mennekes设置
      sprintf(buffer,"%d %c",g_EvseController.MennekesIsLocked(), g_EvseController.MennekesIsManual() ? 'M' : 'A');
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#endif // MENNEKES_LOCK
#ifdef AMMETER
    case 'A': // 获取电表设置
      u1.i = g_EvseController.GetCurrentScaleFactor(); // 获取当前电流比例因子
      u2.i = g_EvseController.GetAmmeterCurrentOffset(); // 获取电流偏移量
      sprintf(buffer,"%d %d",u1.i,u2.i); // 输出比例因子和偏移量
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#endif // AMMETER
    case 'C': // 获取电流容量范围
      u1.i = MIN_CURRENT_CAPACITY_J1772; // 最小电流容量（J1772）
      if (g_EvseController.GetCurSvcLevel() == 2) { // 服务级别2
	u2.i = g_EvseController.GetMaxHwCurrentCapacity(); // 获取最大硬件电流容量
      }
      else {
	u2.i = MAX_CURRENT_CAPACITY_L1; // 获取最大L1电流容量
      }
      u3.i = g_EvseController.GetCurrentCapacity(); // 获取当前电流容量
      u4.i = g_EvseController.GetMaxCurrentCapacity(); // 获取最大电流容量
      sprintf(buffer,"%d %d %d %d",u1.i,u2.i,u3.i,u4.i); // 输出四个电流容量参数
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#ifdef DELAYTIMER
    case 'D': // 获取延时计时器设置
      extern DelayTimer g_DelayTimer;
      if (g_DelayTimer.IsTimerEnabled()) { // 如果计时器启用
	u1.i = g_DelayTimer.GetStartTimerHour(); // 获取启动小时
	u2.i = g_DelayTimer.GetStartTimerMin(); // 获取启动分钟
	u3.i = g_DelayTimer.GetStopTimerHour(); // 获取停止小时
	u4.i = g_DelayTimer.GetStopTimerMin(); // 获取停止分钟
      }
      else { // 如果计时器未启用，返回0
	u1.i = 0;
	u2.i = 0;
	u3.i = 0;
	u4.i = 0;
      }
      sprintf(buffer,"%d %d %d %d",u1.i,u2.i,u3.i,u4.i); // 输出计时器设置
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#endif // DELAYTIMER
    case 'E': // 获取设置
      u1.u = g_EvseController.GetCurrentCapacity(); // 获取当前容量
      u2.u = g_EvseController.GetFlags(); // 获取标志位
      sprintf(buffer,"%d %04x",u1.u,u2.u); // 输出容量和标志位
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
    case 'F': // 获取故障计数器
#ifdef GFI
      u1.u = g_EvseController.GetGfiTripCnt(); // 获取GFI跳闸计数器
#else
      u1.u = 0; // 未启用GFI时返回0
#endif // GFI
#ifdef ADVPWR
      u2.u = g_EvseController.GetNoGndTripCnt(); // 获取无地面跳闸计数
      u3.u = g_EvseController.GetStuckRelayTripCnt(); // 获取卡住继电器跳闸计数
#else
	  u2.u = 0;
	  u3.u = 0;
#endif // ADVPWR
      sprintf(buffer,"%x %x %x",u1.u,u2.u,u3.u); // 输出故障计数器
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#if defined(AMMETER)||defined(VOLTMETER)
    case 'G': // 获取充电电流和电压
      u1.i32 = g_EvseController.GetChargingCurrent(); // 获取充电电流
      u2.i32 = (int32_t)g_EvseController.GetVoltage(); // 获取电压
      sprintf(buffer,"%ld %ld",u1.i32,u2.i32); // 输出充电电流和电压
      bufCnt = 1; // 标记响应文本输出
      rc = 0;
      break;
#endif // AMMETER || VOLTMETER
#ifdef CHARGE_LIMIT
    case 'H': // 获取充电限制
      sprintf(buffer,"%d",(int)g_EvseController.GetChargeLimitkWh()); // 获取充电限制，单位为kWh
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // CHARGE_LIMIT

#ifdef MCU_ID_LEN
    case 'I': // 获取MCU ID
      {
        uint8_t mcuid[MCU_ID_LEN]; // 定义一个数组来存储MCU ID
        getMcuId(mcuid); // 获取MCU ID
        char *s = buffer;
        *(s++) = ' '; // 在输出中添加空格
        for (int i=0; i < 6; i++) {
          *(s++) = mcuid[i]; // 将前6个字节的MCU ID写入缓冲区
        }
        for (int i=6; i < MCU_ID_LEN; i++) {
          sprintf(s, "%02X", mcuid[i]); // 将剩余的字节按十六进制格式写入缓冲区
          s += 2; // 移动指针
        }
        bufCnt = 1; // 设置标志，表示输出响应文本
        rc = 0; // 设置返回值为0，表示处理成功
      }
      break;
#endif // MCU_ID_LEN

#ifdef VOLTMETER
    case 'M': // 获取电压信息
      u1.i = g_EvseController.GetVoltScaleFactor(); // 获取电压的比例因子
      u2.i32 = g_EvseController.GetVoltOffset(); // 获取电压的偏移量
      sprintf(buffer,"%d %ld",u1.i, u2.i32); // 将电压比例因子和偏移量格式化为字符串
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // VOLTMETER

#ifdef TEMPERATURE_MONITORING
#ifdef TEMPERATURE_MONITORING_NY
    case 'O': // 获取温度阈值
      u1.i = g_TempMonitor.m_ambient_thresh; // 获取环境温度阈值
      u2.i = g_TempMonitor.m_ir_thresh; // 获取红外温度阈值
      sprintf(buffer,"%d %d",u1.i,u2.i); // 将两个温度阈值格式化为字符串
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // TEMPERATURE_MONITORING_NY
    case 'P': // 获取温度传感器的读数
      sprintf(buffer,"%d %d %d", (int)g_TempMonitor.m_DS3231_temperature,
              (int)g_TempMonitor.m_MCP9808_temperature,
              (int)g_TempMonitor.m_TMP007_temperature); // 获取并格式化各个温度传感器的读数
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // TEMPERATURE_MONITORING

    case 'S': // 获取当前状态
      u1.u8 = g_EvseController.GetState(); // 获取设备状态
      u2.u8 = g_EvseController.GetPilotState(); // 获取引导状态
      u3.u16 = g_EvseController.GetVFlags(); // 获取电压标志
      sprintf(buffer,"%02x %ld %02x %04x", u1.u8, g_EvseController.GetElapsedChargeTime(), u2.u8, u3.u16); // 格式化状态信息
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;

#ifdef RTC
    case 'T': // 获取RTC时间
      extern void GetRTC(char *buf); // 声明外部函数来获取RTC时间
      GetRTC(buffer); // 获取RTC时间
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // RTC

#ifdef KWH_RECORDING
    case 'U': // 获取能量计量信息
      sprintf(buffer,"%lu %lu", g_EnergyMeter.GetSessionWs(), g_EnergyMeter.GetTotkWh()); // 获取当前会话的瓦特秒数和总kWh
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // KWH_RECORDING

    case 'V': // 获取版本信息
      GetVerStr(buffer); // 获取版本字符串
      strcat(buffer," "); // 添加空格
      strcat_P(buffer,RAPI_VER); // 连接API版本号
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;

#ifdef HEARTBEAT_SUPERVISION
    case 'Y': // 获取心跳监控信息
      sprintf(buffer,"%d %d %d", g_EvseController.GetHearbeatInterval(), g_EvseController.GetHearbeatCurrent(), g_EvseController.GetHearbeatTrigger()); // 获取心跳监控的间隔、当前电流和触发条件
      bufCnt = 1; // 设置标志，表示输出响应文本
      rc = 0; // 设置返回值为0，表示处理成功
      break;
#endif // HEARTBEAT_SUPERVISION

    }
    break;

#ifdef RAPI_T_COMMANDS
  case 'T': // 测试操作
    switch(*s) {
#ifdef FAKE_CHARGING_CURRENT
    case '0': // 设置虚拟充电电流
      if (tokenCnt == 2) { // 如果令牌数量为2
        // 设置充电电流，单位是mA
        g_EvseController.SetChargingCurrent(dtou32(tokens[1])*1000);
        g_OBD.SetAmmeterDirty(1); // 设置电表为脏数据
        g_OBD.Update(OBD_UPD_FORCE); // 强制更新OBD数据
        rc = 0; // 设置返回码为0，表示成功
      }
      break;
#endif // FAKE_CHARGING_CURRENT
    }
    break;
#endif // RAPI_T_COMMANDS

#if defined(RELAY_HOLD_DELAY_TUNING)
  case 'Z': // 保留操作
    switch(*s) {
    case '0': // 设置继电器闭合时间
      if (tokenCnt == 3) { // 如果令牌数量为3
        u1.u8 = dtou32(tokens[1]); // 读取第一个参数并转换为无符号8位整数
        u2.u8 = dtou32(tokens[2]); // 读取第二个参数并转换为无符号8位整数
        g_EvseController.setPwmPinParms(u1.u8,u2.u8); // 设置PWM引脚参数
        sprintf(g_sTmp,"\nZ0 %u %u",(unsigned)u1.u8,(unsigned)u2.u8); // 格式化输出字符串
        Serial.println(g_sTmp); // 打印调试信息
        eeprom_write_byte((uint8_t*)EOFS_RELAY_CLOSE_MS,u1.u8); // 将继电器闭合时间保存到EEPROM
        eeprom_write_byte((uint8_t*)EOFS_RELAY_HOLD_PWM,u2.u8); // 将PWM保持时间保存到EEPROM
      }
      rc = 0; // 设置返回码为0，表示成功
      break;
    }
    break;
#endif // RELAY_HOLD_DELAY_TUNING

  default:
    ; // 默认情况，不做任何操作
  }

  if (bufCnt != -1){ // 如果bufCnt不等于-1
    response((rc == 0) ? 1 : 0); // 调用response函数，传递成功或失败标志
  }

  reset(); // 重置状态

  g_inRapiCommand = 0; // 清除命令标志

  // 命令可能已更改EVSE状态
  RapiSendEvseState(); // 发送EVSE的当前状态

  return rc; // 返回操作结果
}

// 追加校验和
void EvseRapiProcessor::appendChk(char *buf)
{
  char *s = buf;
  uint8_t chk = 0;
  while (*s) {
    chk ^= *(s++); // 计算校验和
  }
  sprintf(s,"^%02X",(unsigned)chk); // 格式化校验和为16进制
  s[3] = ESRAPI_EOC; // 追加结束符
  s[4] = '\0'; // 字符串结束
}

// 响应函数
void EvseRapiProcessor::response(uint8_t ok)
{
  writeStart(); // 开始写入数据

  sprintf(g_sTmp,"%c%s",ESRAPI_SOC,ok ? "OK" : "NK"); // 根据状态决定返回OK还是NK
  if (bufCnt) { // 如果有数据
    strcat(g_sTmp," "); // 添加空格分隔
    strcat(g_sTmp,buffer); // 添加数据
  }
  if (curReceivedSeqId != INVALID_SEQUENCE_ID) {
    appendSequenceId(g_sTmp,curReceivedSeqId); // 如果有接收到的序列ID，追加序列ID
  }
  appendChk(g_sTmp); // 追加校验和
  write(g_sTmp); // 写入响应数据
  if (echo) write('\n'); // 如果启用了回显，写入换行符

  writeEnd(); // 结束写入数据
}

// 追加序列ID
void EvseRapiProcessor::appendSequenceId(char *s,uint8_t seqId)
{
  sprintf(s+strlen(s)," %c%02X",ESRAPI_SOS,seqId); // 格式化并追加序列ID
}

#ifdef RAPI_SENDER
// 获取发送序列ID
uint8_t EvseRapiProcessor::getSendSequenceId()
{
  if (++curSentSeqId == INVALID_SEQUENCE_ID) ++curSentSeqId; // 如果序列ID达到无效值，则重置
  return curSentSeqId; // 返回当前发送的序列ID
}

// 检查是否为异步令牌
int8_t EvseRapiProcessor::isAsyncToken()
{
  if ((*tokens[0] == 'A') || // 如果第一个令牌是A
      !strcmp(tokens[0],"WF") || // 或者令牌是WF
      (!strcmp(tokens[0],"ST") && (tokenCnt == 2))) { // 或者令牌是ST且令牌数量为2
    return 1; // 返回1，表示异步令牌
  }
  else {
    return 0; // 否则返回0
  }
}

// 检查响应令牌是否为OK或NK
int8_t EvseRapiProcessor::isRespToken()
{
  const char *token = tokens[0];  // 获取第一个令牌
  // 判断令牌的长度为2，并且第二个字符是'K'，且第一个字符是'O'或'N'
  if ((strlen(token) == 2) && (token[1] == 'K') &&
      ((*token == 'O') || (*token == 'N'))) {
    return 1;  // 如果是OK或NK，返回1
  }
  else {
    return 0;  // 否则返回0
  }
}

// 发送命令
void EvseRapiProcessor::_sendCmd(const char *cmdstr)
{
  *sendbuf = ESRAPI_SOC;  // 设置命令的开始符
  strcpy(sendbuf+1, cmdstr);  // 将命令字符串复制到sendbuf中
  appendSequenceId(sendbuf, getSendSequenceId());  // 追加序列ID
  appendChk(sendbuf);  // 追加校验和
  writeStart();  // 开始写入数据
  write(sendbuf);  // 写入命令数据
  writeEnd();  // 结束写入数据
}

// 接收响应
int8_t EvseRapiProcessor::receiveResp(unsigned long msstart)
{
  tokenCnt = 0;  // 重置令牌计数
  *sendbuf = 0;  // 清空发送缓冲区
  int bufpos = 0;  // 缓冲区位置指针

  // 等待响应
  do {
    WDT_RESET();  // 重置看门狗定时器
    int bytesavail = available();  // 检查是否有可用的字节
    if (bytesavail) {
      for (int i = 0; i < bytesavail; i++) {
        char c = read();  // 读取一个字节

        if (!bufpos && c != ESRAPI_SOC) {
          // 如果缓冲区为空且字符不是开始符，继续等待开始符
          continue;
        }
        else if (c == ESRAPI_EOC) {
          sendbuf[bufpos] = '\0';  // 如果遇到结束符，终止字符串
          if (!tokenize(sendbuf)) return 0;  // 解析令牌失败，返回0
          else return 1;  // 解析成功，返回1
        }
        else {
          sendbuf[bufpos++] = c;  // 将字符添加到缓冲区
          if (bufpos >= (RAPIS_BUFLEN - 1)) return 2;  // 如果缓冲区满，返回2
        }
      }
    }
  } while (!tokenCnt && ((millis() - msstart) < RAPIS_TIMEOUT_MS));  // 超过超时时间则退出

  return -1;  // 如果没有接收到响应，返回-1
}

// 发送命令并等待响应
int8_t EvseRapiProcessor::sendCmd(const char *cmdstr)
{
  _sendCmd(cmdstr);  // 调用内部的发送命令函数

  unsigned long msstart = millis();  // 获取当前时间戳
start:
  while (receiveResp(msstart) > 0) WDT_RESET();  // 接收响应并重置看门狗定时器
  if (tokenCnt) {  // 如果令牌计数不为0，表示接收到了响应
    uint8_t seqId = INVALID_SEQUENCE_ID;  // 默认无效的序列ID
    const char *seqtoken = tokens[tokenCnt - 1];  // 获取最后一个令牌（序列ID）
    if ((tokenCnt > 1) && isRespToken() && (*seqtoken == ESRAPI_SOS)) {  // 如果是有效的响应令牌
      seqId = htou8(++seqtoken);  // 获取序列ID
      tokenCnt--;  // 减少令牌计数
    }
    // 如果令牌是"OK"并且序列ID匹配，返回0表示成功
    if (!strcmp(tokens[0], "OK") && (seqId == curSentSeqId)) {
      return 0;  // 命令成功
    }
    // 如果令牌是"NK"并且序列ID匹配，返回1表示失败
    else if (!strcmp(tokens[0], "NK") && (seqId == curSentSeqId)) {
      return 1;  // 命令失败
    }
    else {  // 如果收到命令或异步通知，处理它
      processCmd();  // 处理命令
      msstart = millis();  // 更新时间戳
      goto start;  // 重新开始接收响应
    }
  }
  else {
    return -1;  // 如果没有接收到任何令牌，返回-1
  }
}

#endif // RAPI_SENDER


#ifdef RAPI_SERIAL
// EvseSerialRapiProcessor类构造函数
EvseSerialRapiProcessor::EvseSerialRapiProcessor()
{
  // 构造函数为空，主要用于初始化
}

// 初始化函数
void EvseSerialRapiProcessor::init()
{
  // 调用父类EvseRapiProcessor的初始化函数
  EvseRapiProcessor::init();
}
#endif // RAPI_SERIAL

#ifdef RAPI_I2C

// EvseI2cRapiProcessor类构造函数
EvseI2cRapiProcessor::EvseI2cRapiProcessor()
{
  // 构造函数为空，主要用于初始化
}

// 初始化函数
void EvseI2cRapiProcessor::init()
{
  // 启动I2C通信，使用RAPI_I2C_LOCAL_ADDR作为设备地址
  Wire.begin(RAPI_I2C_LOCAL_ADDR);
  // 定义接收数据的函数，接收来自主设备的数据
  Wire.onReceive(receiveEvent);

  // 调用父类EvseRapiProcessor的初始化函数
  EvseRapiProcessor::init();
}

#endif // RAPI_I2C

// 创建EvseSerialRapiProcessor的全局对象g_ESRP
#ifdef RAPI_SERIAL
EvseSerialRapiProcessor g_ESRP;
#endif

// 创建EvseI2cRapiProcessor的全局对象g_EIRP
#ifdef RAPI_I2C
EvseI2cRapiProcessor g_EIRP;
#endif

// 初始化RAPI命令处理器
void RapiInit()
{
#ifdef RAPI_SERIAL
  // 初始化g_ESRP对象
  g_ESRP.init();

  // 如果定义了GPPBUGKLUDGE宏，设置特定的缓冲区
#ifdef GPPBUGKLUDGE
  static char g_rapiSerialBuffer[ESRAPI_BUFLEN];
  g_ESRP.setBuffer(g_rapiSerialBuffer);
#endif // GPPBUGKLUDGE
#endif // RAPI_SERIAL

#ifdef RAPI_I2C
  // 初始化g_EIRP对象
  g_EIRP.init();

  // 如果定义了GPPBUGKLUDGE宏，设置特定的缓冲区
#ifdef GPPBUGKLUDGE
  static char g_rapiI2ClBuffer[ESRAPI_BUFLEN];
  g_ESRP.setBuffer(g_rapiI2CBuffer);
#endif // GPPBUGKLUDGE
#endif // RAPI_I2C
}

// 执行RAPI命令
void RapiDoCmd()
{
#ifdef RAPI_SERIAL
  // 如果使用串行接口，调用g_ESRP的doCmd函数
  g_ESRP.doCmd();
#endif

#ifdef RAPI_I2C
  // 如果使用I2C接口，在紧密循环中运行RapiDoCmd时，需要延时
  // 这是因为在硬故障时，I2C无法接收字符。没有延时时，I2C会丢失字符
#define RDCDELAY 6
#ifdef RDCDELAY
  static unsigned long lastdocmd;  // 上次执行命令的时间
  unsigned long msnow = millis();  // 获取当前时间
  // 如果两次执行命令的时间间隔小于RDCDELAY，执行延时
  if ((msnow - lastdocmd) < RDCDELAY) {
    delay(RDCDELAY - (msnow - lastdocmd));
  }
  lastdocmd = msnow;  // 更新上次执行命令的时间
#endif // RDCDELAY

  // 调用g_EIRP的doCmd函数
  g_EIRP.doCmd();
#endif // RAPI_I2C
}

// 返回值:
//  0 = 已发送
//  1 = 没有变化，未发送
//  2 = 在processCmd()中，未发送
uint8_t RapiSendEvseState(uint8_t force)
{
  // 静态变量用于保存上次发送的状态信息
  static uint8_t evseStateSent = EVSE_STATE_UNKNOWN;   // 记录上次发送的EVSE状态
  static uint8_t pilotStateSent = EVSE_STATE_UNKNOWN;  // 记录上次发送的Pilot状态
  static uint8_t currentCapacitySent = 0;               // 记录上次发送的当前容量
  static uint16_t vFlagsSent = 0;                       // 记录上次发送的标志

  // 判断当前是否在执行RAPI命令
  if (!g_inRapiCommand) {
    // 获取EVSE的状态信息
    uint8_t evseState = g_EvseController.GetState();
    uint8_t pilotState = g_EvseController.GetPilotState();
    uint8_t currentCapacity = g_EvseController.GetCurrentCapacity();
    uint16_t vFlags = g_EvseController.GetVFlags() & ECVF_CHANGED_TEST;

    // 如果强制发送或者状态发生变化，则发送新的EVSE状态
    if (force ||
        ((evseState != EVSE_STATE_UNKNOWN) &&
        !((evseState == EVSE_STATE_A) && (vFlags & ECVF_EV_CONNECTED)) &&
        !((evseState == EVSE_STATE_B) && !(vFlags & ECVF_EV_CONNECTED)) &&
        !((evseState == EVSE_STATE_C) && !(vFlags & ECVF_EV_CONNECTED)) &&
        ((evseStateSent != evseState) ||
         (pilotStateSent != pilotState) ||
         (currentCapacitySent != currentCapacity) ||
         (vFlagsSent != vFlags)))) {

      // 如果使用串行接口，发送EVSE状态
#ifdef RAPI_SERIAL
      g_ESRP.sendEvseState();
#endif
      // 如果使用I2C接口，发送EVSE状态
#ifdef RAPI_I2C
      g_EIRP.sendEvseState();
#endif

      // 更新已发送的状态信息
      evseStateSent = evseState;
      pilotStateSent = pilotState;
      currentCapacitySent = currentCapacity;
      vFlagsSent = vFlags;
      return 0;  // 返回已发送状态
    }

    return 1;  // 返回没有变化，未发送
  }
  else return 2;  // 返回正在processCmd()中，未发送
}

// 发送启动通知
void RapiSendBootNotification()
{
#ifdef RAPI_SERIAL
  g_ESRP.sendBootNotification();  // 使用串行接口发送启动通知
#endif
#ifdef RAPI_I2C
  g_EIRP.sendBootNotification();  // 使用I2C接口发送启动通知
#endif
}

#ifdef RAPI_WF
// 设置WiFi模式
void RapiSetWifiMode(uint8_t mode)
{
#ifdef RAPI_SERIAL
  g_ESRP.setWifiMode(mode);  // 使用串行接口设置WiFi模式
#endif
#ifdef RAPI_I2C
  g_EIRP.setWifiMode(mode);  // 使用I2C接口设置WiFi模式
#endif
}
#endif // RAPI_WF

#ifdef RAPI_BTN
// 发送按钮按压事件
void RapiSendButtonPress(uint8_t long_press)
{
#ifdef RAPI_SERIAL
  g_ESRP.sendButtonPress(long_press);  // 使用串行接口发送按钮按压事件
#endif
#ifdef RAPI_I2C
  g_EIRP.sendButtonPress(long_press);  // 使用I2C接口发送按钮按压事件
#endif
}
#endif // RAPI_BTN
#endif // RAPI
