/*
 * This file is part of Open EVSE.
 *
 * Copyright (c) 2011-2023 Sam C. Lin
 *
 * Open EVSE is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.

 * Open EVSE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with Open EVSE; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
#include "open_evse.h"

#ifdef FT_ENDURANCE
int g_CycleCnt = -1;
long g_CycleHalfStart;
uint8_t g_CycleState;
#endif

//                                               A/B B/C C/D D DS
THRESH_DATA J1772EVSEController::m_ThreshData = {875,780,690,0,260};

J1772EVSEController g_EvseController;

#ifdef AMMETER
static inline unsigned long ulong_sqrt(unsigned long in)
{
  unsigned long out = 0;
  unsigned long bit = 0x40000000ul;

  // "bit" starts at the highest power of four <= the argument.
  while (bit > in)
    bit >>= 2;

  while (bit) {
    unsigned long sum = out + bit;
    if (in >= sum) {
      in -= sum;
      out = (out >> 1) + bit;
    }
    else
      out >>= 1;
    bit >>= 2;
  }

  return out;
}

void J1772EVSEController::readAmmeter()
{
  WDT_RESET();

  unsigned long sum = 0;
  uint8_t zero_crossings = 0;
  unsigned long last_zero_crossing_time = 0, now_ms;
  uint8_t is_first_sample = 1;
  uint16_t last_sample;
  unsigned int sample_count = 0;
  for(unsigned long start = millis(); ((now_ms = millis()) - start) < CURRENT_SAMPLE_INTERVAL; ) {
    // the A/d is 0 to 1023.
    uint16_t sample = adcCurrent.read();
    // If this isn't the first sample, and if the sign of the value differs from the
    // sign of the previous value, then count that as a zero crossing.
    if (!is_first_sample && ((last_sample > 512) != (sample > 512))) {
      // Once we've seen a zero crossing, don't look for one for a little bit.
      // It's possible that a little noise near zero could cause a two-sample
      // inversion.
      if ((now_ms - last_zero_crossing_time) > CURRENT_ZERO_DEBOUNCE_INTERVAL) {
        zero_crossings++;
        last_zero_crossing_time = now_ms;
      }
    }
    is_first_sample = 0;
    last_sample = sample;
    switch(zero_crossings) {
    case 0:
      continue; // Still waiting to start sampling
    case 1:
    case 2:
      // Gather the sum-of-the-squares and count how many samples we've collected.
      sum += (unsigned long)(((long)sample - 512) * ((long)sample - 512));
      sample_count++;
      continue;
    case 3:
      // The answer is the square root of the mean of the squares.
      // But additionally, that value must be scaled to a real current value.
      // we will do that elsewhere
      m_AmmeterReading = ulong_sqrt(sum / sample_count);
      return;
    }
  }
  // ran out of time. Assume that it's simply not oscillating any.
  m_AmmeterReading = 0;

  WDT_RESET();
}

#define MA_PTS 32 // # points in moving average MUST BE power of 2
#define MA_BITS 5 // log2(MA_PTS)
/*
uint32_t MovingAverage(uint32_t samp)
{
  static uint32_t samps[MA_PTS] = {0};
  uint32_t tot = samp;
  samps[0] = samp;

  for (int8_t c=MA_PTS-1;c > 0;c--) {
  samps[c] = samps[c-1];
  tot += samps[c];
  }
  
  return tot >> MA_BITS;
}
*/
// 为了节省内存
// 我们不执行真正的滑动平均，而是执行非重叠的滑动窗口
// 每 MA_PTS 个样本输出一个平均值
uint32_t MovingAverage(uint32_t samp)
{
  static uint32_t tot = 0;       // 用于累加样本值
  static int8_t curidx = 0;      // 当前索引位置

  if (curidx == 0) {
    tot = 0;                     // 每次新一组开始时，清空总和
  }

  tot += samp;                   // 累加样本值

  if (++curidx == MA_PTS) {      // 达到窗口大小
    curidx = 0;                  // 重置索引
    return tot >> MA_BITS;       // 返回平均值 (等同于 tot / MA_PTS)
  }

  return 0xffffffff;             // 尚未满一个窗口，返回无效值
}

#endif // AMMETER

// J1772EVSE 控制器构造函数
J1772EVSEController::J1772EVSEController() :
  adcPilot(PILOT_PIN)            // 初始化 Pilot 引脚
#ifdef CURRENT_PIN
  , adcCurrent(CURRENT_PIN)      // 如果定义了 CURRENT_PIN，初始化电流监测引脚
#endif
#ifdef VOLTMETER_PIN
  , adcVoltMeter(VOLTMETER_PIN)  // 如果定义了 VOLTMETER_PIN，初始化电压监测引脚
#endif
{
#ifdef STATE_TRANSITION_REQ_FUNC
  m_StateTransitionReqFunc = NULL;  // 初始化状态转移请求函数为空
#endif
}

// 保存当前设置到 EEPROM
void J1772EVSEController::SaveSettings()
{
  uint8_t *dest;
  // 注意：可以考虑使用 dirty bits 优化写入，或者按需写入

  // 根据服务级别决定存储地址
  if (GetCurSvcLevel() == 1) {
    dest = (uint8_t *)EOFS_CURRENT_CAPACITY_L1;
  } else {
    dest = (uint8_t *)EOFS_CURRENT_CAPACITY_L2;
  }

  // 写入最大电流容量
  eeprom_write_byte(dest, GetMaxCurrentCapacity());

  // 保存标志位
  SaveEvseFlags();
}

#ifdef AUTH_LOCK
// 授权锁定功能
void J1772EVSEController::AuthLock(uint8_t tf,uint8_t update)
{
  if (tf) setVFlags(ECVF_AUTH_LOCKED);  // 设置授权锁定标志
  else clrVFlags(ECVF_AUTH_LOCKED);     // 清除标志

  if (update && !InFaultState()) {
    if (m_EvseState != EVSE_STATE_A)
      Update(1);  // 更新状态（会中断）
    else
      Update(0);  // 更新状态（不中断）
    g_OBD.Update(OBD_UPD_FORCE);  // 强制更新显示
  }
}
#endif // AUTH_LOCK

// 通过看门狗重启系统
void J1772EVSEController::Reboot()
{
  m_Pilot.SetState(PILOT_STATE_P12);  // 设置 Pilot 为 P12 状态，通知车辆断开

#ifdef LCD16X2
  g_OBD.LcdPrint_P(1,g_psResetting);  // LCD 显示“正在重启”
#endif

  if (chargingIsOn()) {
    wdt_delay(3000);  // 如果正在充电，等待 EV 接触器断开
  }

  // 启用看门狗，并等待超时以重启系统
  wdt_enable(WDTO_1S);
  delay(1500);
}

#ifdef SHOW_DISABLED_TESTS
// 显示禁用的测试项目信息（LCD）
void J1772EVSEController::DisabledTest_P(PGM_P message)
{
#ifdef LCD16X2
  g_OBD.LcdMsg_P(g_psDisabledTests, message);
#endif
#ifndef NOCHECKS
  delay(SHOW_DISABLED_DELAY); // 延时展示
#endif
}

// 依次检查并显示禁用的安全测试项目
void J1772EVSEController::ShowDisabledTests()
{
  if (m_wFlags & (
    ECF_DIODE_CHK_DISABLED |
    ECF_VENT_REQ_DISABLED |
    ECF_GND_CHK_DISABLED |
    ECF_STUCK_RELAY_CHK_DISABLED |
    ECF_GFI_TEST_DISABLED |
    ECF_TEMP_CHK_DISABLED)) {

#ifdef LCD16X2
    g_OBD.LcdSetBacklightColor(YELLOW); // 设置 LCD 背光为黄色
#endif

    if (!DiodeCheckEnabled()) DisabledTest_P(g_psDiodeCheck);
    if (!VentReqEnabled()) DisabledTest_P(g_psVentReqChk);

#ifdef ADVPWR
    if (!GndChkEnabled()) DisabledTest_P(g_psGndChk);
    if (!StuckRelayChkEnabled()) DisabledTest_P(g_psRlyChk);
#endif

#ifdef GFI_SELFTEST
    if (!GfiSelfTestEnabled()) DisabledTest_P(g_psGfiTest);
#endif

#ifdef TEMPERATURE_MONITORING
    if (!TempChkEnabled()) DisabledTest_P(g_psTempChk);
#endif

#ifdef LCD16X2
    g_OBD.LcdSetBacklightColor(WHITE); // 恢复 LCD 背光
#endif
  }
}
#endif // SHOW_DISABLED_TESTS

// 开启充电
void J1772EVSEController::chargingOn()
{
#ifdef OEV6
  if (isV6()) {
#ifdef RELAY_PWM
    Serial.print("\nrelayCloseMs: "); Serial.println(m_relayCloseMs);
    Serial.print("relayHoldPwm: "); Serial.println(m_relayHoldPwm);

    digitalWrite(V6_CHARGING_PIN, HIGH);     // 关闭继电器
    digitalWrite(V6_CHARGING_PIN2, HIGH);
    delay(m_relayCloseMs);

    analogWrite(V6_CHARGING_PIN, m_relayHoldPwm);  // 保持继电器闭合
    analogWrite(V6_CHARGING_PIN2, m_relayHoldPwm);
#else
    digitalWrite(V6_CHARGING_PIN, HIGH);
    digitalWrite(V6_CHARGING_PIN2, HIGH);
#endif
  } else {
#endif
#ifdef CHARGING_REG
    pinCharging.write(1); // 打开充电输出
#endif
#ifdef CHARGING2_REG
    pinCharging2.write(1);
#endif
#ifdef OEV6
  }
#endif
#ifdef CHARGINGAC_REG
    pinChargingAC.write(1); // 交流充电信号输出
#endif

  setVFlags(ECVF_CHARGING_ON); // 设置充电标志位

  // 如果上次会话已经结束，重置累计时间；否则累加
  if (vFlagIsSet(ECVF_SESSION_ENDED)) {
    m_AccumulatedChargeTime = 0;
    clrVFlags(ECVF_SESSION_ENDED);
  } else {
    m_AccumulatedChargeTime += m_ElapsedChargeTime;
    m_ElapsedChargeTime = 0;
  }

  m_ChargeOnTimeMS = millis(); // 记录充电开始时间
}

// 关闭充电
void J1772EVSEController::chargingOff()
{
#ifdef OEV6
  if (isV6()) {
#ifdef RELAY_AUTO_PWM_PIN
    digitalWrite(RELAY_AUTO_PWM_PIN, LOW); // PWM 控制继电器断开
#else
    digitalWrite(V6_CHARGING_PIN, LOW);    // 断开继电器
    digitalWrite(V6_CHARGING_PIN2, LOW);
#endif
  } else {
#endif
#ifdef CHARGING_REG
    pinCharging.write(0); // 停止充电
#endif
#ifdef CHARGING2_REG
    pinCharging2.write(0);
#endif
#ifdef OEV6
  }
#endif
#ifdef CHARGINGAC_REG
  pinChargingAC.write(0);
#endif

  clrVFlags(ECVF_CHARGING_ON); // 清除充电标志

  m_ChargeOffTimeMS = millis(); // 记录断电时间

#ifdef AMMETER
  m_ChargingCurrent = 0; // 清零充电电流
#endif
}

// 设置硬件故障
void J1772EVSEController::HardFault(int8_t recoverable)
{
  SetHardFault(); // 设置故障状态
  g_OBD.Update(OBD_UPD_HARDFAULT); // 更新显示

#ifdef RAPI
  RapiSendEvseState(); // 发送状态
#endif

  while (1) {
    ProcessInputs(); // 处理输入，进入死循环

    if (m_Pilot.GetState() != PILOT_STATE_N12) {
      ReadPilot(); // 读取 pilot 状态
      if (!EvConnected() && recoverable) {
        m_EvseState = EVSE_STATE_UNKNOWN; // 重置状态
        break;
      }
    }
  }

  ClrHardFault(); // 清除故障状态
}

#ifdef GFI
// 设置 GFI（漏电）触发状态
void J1772EVSEController::SetGfiTripped()
{
#ifdef GFI_SELFTEST
  if (m_Gfi.SelfTestInProgress()) {
    m_Gfi.SetTestSuccess(); // 自检成功
    return;
  }
#endif
  setVFlags(ECVF_GFI_TRIPPED); // 设置 GFI 触发标志

  chargingOff();               // 立即关闭充电
  m_Pilot.SetState(PILOT_STATE_P12); // 切换 PWM 为 P12，通知 EV 停止

  m_Gfi.SetFault();            // 设置故障

  // 后续将在 Update 中处理
}
#endif // GFI

// 启用或禁用二极管检查
void J1772EVSEController::EnableDiodeCheck(uint8_t tf)
{
  if (tf)
    m_wFlags &= ~ECF_DIODE_CHK_DISABLED;
  else
    m_wFlags |= ECF_DIODE_CHK_DISABLED;
  SaveEvseFlags(); // 保存标志
}

#ifdef GFI_SELFTEST
// 启用或禁用漏电自检
void J1772EVSEController::EnableGfiSelfTest(uint8_t tf)
{
  if (tf)
    m_wFlags &= ~ECF_GFI_TEST_DISABLED;
  else
    m_wFlags |= ECF_GFI_TEST_DISABLED;
  SaveEvseFlags(); // 保存标志
}
#endif

#ifdef TEMPERATURE_MONITORING
// 启用或禁用温度检测功能
void J1772EVSEController::EnableTempChk(uint8_t tf)
{
  if (tf) {
    // 清除禁用温度检测标志
    m_wFlags &= ~ECF_TEMP_CHK_DISABLED;
  }
  else {
    // 设置禁用温度检测标志
    m_wFlags |= ECF_TEMP_CHK_DISABLED;
  }
  // 保存标志位
  SaveEvseFlags();
}
#endif // TEMPERATURE_MONITORING

// 启用或禁用通风请求功能
void J1772EVSEController::EnableVentReq(uint8_t tf)
{
  if (tf) {
    // 清除禁用通风请求标志
    m_wFlags &= ~ECF_VENT_REQ_DISABLED;
  }
  else {
    // 设置禁用通风请求标志
    m_wFlags |= ECF_VENT_REQ_DISABLED;
  }
  // 保存标志位
  SaveEvseFlags();
}

#ifdef ADVPWR
// 启用或禁用接地检测功能
void J1772EVSEController::EnableGndChk(uint8_t tf)
{
  if (tf) {
    // 清除禁用接地检测标志
    m_wFlags &= ~ECF_GND_CHK_DISABLED;
  }
  else {
    // 清零无接地重试计数器
    m_NoGndRetryCnt = 0;
    // 清零无接地起始时间
    m_NoGndStart = 0;
    // 设置禁用接地检测标志
    m_wFlags |= ECF_GND_CHK_DISABLED;
  }
  // 保存标志位
  SaveEvseFlags();
}

// 启用或禁用继电器粘连检测
void J1772EVSEController::EnableStuckRelayChk(uint8_t tf)
{
  if (tf) {
    // 清除禁用粘连检测标志
    m_wFlags &= ~ECF_STUCK_RELAY_CHK_DISABLED;
  }
  else {
    // 设置禁用粘连检测标志
    m_wFlags |= ECF_STUCK_RELAY_CHK_DISABLED;
  }
  // 保存标志位
  SaveEvseFlags();
}

#ifdef AUTOSVCLEVEL
// 启用或禁用自动服务等级
uint8_t J1772EVSEController::EnableAutoSvcLevel(uint8_t tf)
{
  int rc = 0;
  if (tf) {
    // 如果CGMI功能启用，则不进行设置
    if (CGMIisEnabled()) rc = 1;
    else clrFlags(ECF_AUTO_SVC_LEVEL_DISABLED); // 启用自动服务等级
  }
  else {
    setFlags(ECF_AUTO_SVC_LEVEL_DISABLED); // 禁用自动服务等级
  }
  if (!rc) SaveEvseFlags(); // 如果未跳过，则保存标志位
  return rc;
}
#endif // AUTOSVCLEVEL

#endif // ADVPWR

// 启用或禁用串口调试信息输出
void J1772EVSEController::EnableSerDbg(uint8_t tf)
{
  if (tf) {
    // 启用调试标志
    m_wFlags |= ECF_SERIAL_DBG;
  }
  else {
    // 禁用调试标志
    m_wFlags &= ~ECF_SERIAL_DBG;
  }
  // 保存标志位
  SaveEvseFlags();
}

#ifdef RGBLCD
// 设置LCD背光类型
int J1772EVSEController::SetBacklightType(uint8_t t,uint8_t update)
{
#ifdef RGBLCD
  // 设置LCD背光颜色类型
  g_OBD.LcdSetBacklightType(t,update);
  // 如果是单色LCD，则设置相应标志
  if (t == BKL_TYPE_MONO) m_wFlags |= ECF_MONO_LCD;
  else m_wFlags &= ~ECF_MONO_LCD;
  // 保存标志位
  SaveEvseFlags();
#endif // RGBLCD
  return 0;
}
#endif // RGBLCD

// 启用EVSE控制器（从睡眠或禁用状态恢复）
void J1772EVSEController::Enable()
{
  if ((m_EvseState == EVSE_STATE_DISABLED)||
      (m_EvseState == EVSE_STATE_SLEEPING)) {
#ifdef SLEEP_STATUS_REG
    if (m_EvseState == EVSE_STATE_SLEEPING) {
      // 关闭睡眠状态引脚
      pinSleepStatus.write(0);
    }
#endif // SLEEP_STATUS_REG
#if defined(TIME_LIMIT) || defined(CHARGE_LIMIT)
	SetLimitSleep(0); // 禁用限时睡眠
#endif //defined(TIME_LIMIT) || defined(CHARGE_LIMIT)

    // 保存上一个状态为禁用
    m_PrevEvseState = EVSE_STATE_DISABLED;
    // 设置当前状态为未知
    m_EvseState = EVSE_STATE_UNKNOWN;
    // 设置pilot为+12V，准备连接EV
    m_Pilot.SetState(PILOT_STATE_P12);
  }
}

// 禁用EVSE（立即停止充电）
void J1772EVSEController::Disable()
{
  if (m_EvseState != EVSE_STATE_DISABLED) {
    // 设置pilot为-12V，通知EV断开
    m_Pilot.SetState(PILOT_STATE_N12);
    // 设置EVSE状态为禁用
    m_EvseState = EVSE_STATE_DISABLED;
    // 强制断开充电，不等待EV响应
    chargingOff();
#ifdef MENNEKES_LOCK
    if (!MennekesIsManual()) m_MennekesLock.Unlock(1); // 解锁插头
#endif // MENNEKES_LOCK

    // 更新OBD信息
    g_OBD.Update(OBD_UPD_FORCE);
#ifdef RAPI
    RapiSendEvseState(); // 通知远程状态变化
#endif // RAPI
  }
}

// 进入睡眠状态（保持pilot为+12V，但不供电）
void J1772EVSEController::Sleep()
{
  if (m_EvseState != EVSE_STATE_SLEEPING) {
    // 设置pilot为+12V
    m_Pilot.SetState(PILOT_STATE_P12);
    // 设置EVSE状态为睡眠
    m_EvseState = EVSE_STATE_SLEEPING;
#ifdef SLEEP_STATUS_REG
    // 打开睡眠状态引脚
    pinSleepStatus.write(1);
#endif // SLEEP_STATUS_REG

    // 为避免电弧，在此处计时等待EV断开
    m_ChargeOffTimeMS = millis();

    // 强制刷新LCD显示
    g_OBD.Update(OBD_UPD_FORCE);

#ifdef RAPI
    RapiSendEvseState(); // 发送EVSE状态
#endif // RAPI
  }
}

// 设置服务等级（L1或L2）
void J1772EVSEController::SetSvcLevel(uint8_t svclvl,uint8_t updatelcd)
{
#ifdef SERDBG
  if (SerDbgEnabled()) {
    Serial.print("SetSvcLevel: ");Serial.println((int)svclvl); // 输出调试信息
  }
#endif //#ifdef SERDBG
  if (svclvl == 2) {
    m_wFlags |= ECF_L2; // 设置为Level 2
    m_Voltage = MV_FOR_L2; // 设置L2电压
  }
  else {
    svclvl = 1; // 无效等级强制为L1
    m_wFlags &= ~ECF_L2; // 设置为Level 1
    m_Voltage = MV_FOR_L1; // 设置L1电压
  }

  SaveEvseFlags(); // 保存服务等级状态

  uint8_t ampacity = GetMaxCurrentCapacity(); // 获取最大电流容量

  // 设置当前电流容量
  SetCurrentCapacity(ampacity,0,1);

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE); // 更新LCD显示
  }
}

// 获取当前服务等级下的最大电流容量
uint8_t J1772EVSEController::GetMaxCurrentCapacity()
{
  uint8_t svclvl = GetCurSvcLevel(); // 当前服务等级
  // 从EEPROM读取电流容量设置
  uint8_t ampacity =  eeprom_read_byte((uint8_t*)((svclvl == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2));

  // 如果读取失败或为0，则使用默认值
  if ((ampacity == 0xff) || (ampacity == 0)) {
    ampacity = (svclvl == 1) ? DEFAULT_CURRENT_CAPACITY_L1 : DEFAULT_CURRENT_CAPACITY_L2;
  }

#ifdef PP_AUTO_AMPACITY
  // 如果车辆连接，自动检测PP引脚允许的最大电流
  if ((m_EvseState >= EVSE_STATE_B) && (m_EvseState <= EVSE_STATE_C)) {
    uint8_t ppamps =  g_ACCController.GetPPMaxAmps(); // 获取PP电流限制
    if (ppamps < ampacity) {
      ampacity = ppamps;
    }
  }
#endif // PP_AUTO_AMPACITY

  // 最小电流容量保护
  if (ampacity < MIN_CURRENT_CAPACITY_J1772) {
    ampacity = MIN_CURRENT_CAPACITY_J1772;
  }
  else {
    if (svclvl == 1) { // L1
      if (ampacity > MAX_CURRENT_CAPACITY_L1) {
        ampacity = MAX_CURRENT_CAPACITY_L1;
      }
    }
    else {
      if (ampacity > m_MaxHwCurrentCapacity) {
        ampacity = m_MaxHwCurrentCapacity;
      }
    }
  }

  return ampacity;
}


// J1772EVSEController.cpp

// 初始化 EVSE 控制器
void J1772EVSEController::Init()
{
#ifdef OEV6
  // 设置 V6 ID 引脚为输入模式
  DPIN_MODE_INPUT(V6_ID_REG, V6_ID_IDX);
#ifdef INVERT_V6_DETECTION
  // 如果启用反转检测，则取反读入值判断是否为 V6
  m_isV6 = !DPIN_READ(V6_ID_REG, V6_ID_IDX);
#else
  m_isV6 = DPIN_READ(V6_ID_REG, V6_ID_IDX);
#endif
#endif

#ifdef MENNEKES_LOCK
  // 初始化 Mennekes 插头锁机制
  m_MennekesLock.Init();
#endif

  // 初始化 EVSE 状态
  m_EvseState = EVSE_STATE_UNKNOWN;
  m_PrevEvseState = EVSE_STATE_UNKNOWN;

  // 从 EEPROM 读取设定标志位
  uint16_t rflgs = eeprom_read_word((uint16_t*)EOFS_FLAGS);

#ifdef RGBLCD
  // 根据 EEPROM 设定设置 LCD 背光类型
  if ((rflgs != 0xffff) && (rflgs & ECF_MONO_LCD)) {
    g_OBD.LcdSetBacklightType(BKL_TYPE_MONO);
  }
#endif

#ifdef RELAY_PWM
  // 读取继电器控制参数
  m_relayCloseMs = eeprom_read_byte((uint8_t*)EOFS_RELAY_CLOSE_MS);
  m_relayHoldPwm = eeprom_read_byte((uint8_t*)EOFS_RELAY_HOLD_PWM);
  // 若值无效则恢复默认值
  if (!m_relayCloseMs || (m_relayCloseMs == 255)) {
    m_relayCloseMs = DEFAULT_RELAY_CLOSE_MS;
    m_relayHoldPwm = DEFAULT_RELAY_HOLD_PWM;
  }
  Serial.print("\nrelayCloseMs: "); Serial.println(m_relayCloseMs);
  Serial.print("relayHoldPwm: "); Serial.println(m_relayHoldPwm);
#endif

#ifdef OEV6
  if (isV6()) {
    // 如果是 V6，设置充电控制引脚为输出
    pinMode(V6_CHARGING_PIN, OUTPUT);
    pinMode(V6_CHARGING_PIN2, OUTPUT);
  } else {
#endif
#ifdef CHARGING_REG
    // 初始化充电控制引脚
    pinCharging.init(CHARGING_REG, CHARGING_IDX, DigitalPin::OUT);
#endif
#ifdef CHARGING2_REG
    pinCharging2.init(CHARGING2_REG, CHARGING2_IDX, DigitalPin::OUT);
#endif
#ifdef OEV6
  }
#endif

#ifdef CHARGINGAC_REG
  // 初始化 AC 充电控制引脚
  pinChargingAC.init(CHARGINGAC_REG, CHARGINGAC_IDX, DigitalPin::OUT);
#endif
#ifdef ACLINE1_REG
  // 初始化 AC 电源线路检测引脚
  pinAC1.init(ACLINE1_REG, ACLINE1_IDX, DigitalPin::INP_PU);
#endif
#ifdef ACLINE2_REG
  pinAC2.init(ACLINE2_REG, ACLINE2_IDX, DigitalPin::INP_PU);
#endif
#ifdef SLEEP_STATUS_REG
  pinSleepStatus.init(SLEEP_STATUS_REG, SLEEP_STATUS_IDX, DigitalPin::OUT);
#endif
#ifdef AUTH_LOCK_REG
  pinAuthLock.init(AUTH_LOCK_REG, AUTH_LOCK_IDX, DigitalPin::INP_PU);
#endif

#ifdef AUTH_LOCK
  // 执行授权锁初始化
  AuthLock(AUTH_LOCK, 0);
#endif

#ifdef GFI
#ifdef OEV6
  // 初始化 GFI 模块，判断是否为 V6
  m_Gfi.Init(isV6());
#else
  m_Gfi.Init();
#endif
#endif

  // 确保初始状态为未充电
  chargingOff();

  // 初始化 Pilot 信号控制器
  m_Pilot.Init();

  // 默认服务等级
  uint8_t svclvl = (uint8_t)DEFAULT_SERVICE_LEVEL;

  if (rflgs == 0xffff) {
    // EEPROM 未初始化，使用默认设置
    m_wFlags = ECF_DEFAULT;
#ifdef RGBLCD
    if (DEFAULT_LCD_BKL_TYPE == BKL_TYPE_MONO) {
      m_wFlags |= ECF_MONO_LCD;
    }
#endif
  } else {
    m_wFlags = rflgs;
    svclvl = GetCurSvcLevel();  // 从标志位中获取服务等级
  }

#ifndef AUTOSVCLEVEL
  m_wFlags |= ECF_AUTO_SVC_LEVEL_DISABLED;
#endif

#ifdef ENABLE_CGMI
  m_wFlags |= ECF_CGMI;
#endif

  // CGMI 启用时禁止自动服务等级
  if (CGMIisEnabled() && !flagIsSet(ECF_AUTO_SVC_LEVEL_DISABLED)) {
    setFlags(ECF_AUTO_SVC_LEVEL_DISABLED);
    svclvl = DEFAULT_SERVICE_LEVEL;
  }

#ifdef NOCHECKS
  // 禁用所有检测功能
  m_wFlags |= ECF_DIODE_CHK_DISABLED | ECF_VENT_REQ_DISABLED | ECF_GND_CHK_DISABLED |
              ECF_STUCK_RELAY_CHK_DISABLED | ECF_GFI_TEST_DISABLED | ECF_TEMP_CHK_DISABLED;
#endif

#ifdef SERDBG
  // 启用串口调试
  EnableSerDbg(1);
#endif

#ifdef AMMETER
  // 读取电流检测偏移量和比例因子
  m_AmmeterCurrentOffset = eeprom_read_word((uint16_t*)EOFS_AMMETER_CURR_OFFSET);
  m_CurrentScaleFactor = eeprom_read_word((uint16_t*)EOFS_CURRENT_SCALE_FACTOR);

  if (m_AmmeterCurrentOffset == (int16_t)0xffff) {
    m_AmmeterCurrentOffset = DEFAULT_AMMETER_CURRENT_OFFSET;
  }
  if (m_CurrentScaleFactor == (int16_t)0xffff) {
    m_CurrentScaleFactor = DEFAULT_CURRENT_SCALE_FACTOR;
  }

  m_AmmeterReading = 0;
  m_ChargingCurrent = 0;
#endif

#ifdef VOLTMETER
  m_VoltOffset = eeprom_read_dword((uint32_t*)EOFS_VOLT_OFFSET);
  m_VoltScaleFactor = eeprom_read_word((uint16_t*)EOFS_VOLT_SCALE_FACTOR);

  if (m_VoltOffset == 0xffffffff) {
    m_VoltOffset = DEFAULT_VOLT_OFFSET;
  }
  if (m_VoltScaleFactor == 0xffff) {
    m_VoltScaleFactor = DEFAULT_VOLT_SCALE_FACTOR;
  }
#endif

#ifndef RGBLCD
  m_wFlags |= ECF_MONO_LCD;
#endif

  m_wVFlags = ECVF_DEFAULT;

#ifdef BOOTLOCK
  m_wVFlags |= ECVF_BOOT_LOCK;
#endif

  // 最大硬件电流能力
  m_MaxHwCurrentCapacity = eeprom_read_byte((uint8_t*)EOFS_MAX_HW_CURRENT_CAPACITY);
  if (!m_MaxHwCurrentCapacity || (m_MaxHwCurrentCapacity == 0xff)) {
    m_MaxHwCurrentCapacity = MAX_CURRENT_CAPACITY_L2;
  }

#ifdef GFI
  m_GfiRetryCnt = 0;
  m_GfiTripCnt = eeprom_read_byte((uint8_t*)EOFS_GFI_TRIP_CNT);
#endif

#ifdef ADVPWR
  m_NoGndRetryCnt = 0;
  m_NoGndTripCnt = eeprom_read_byte((uint8_t*)EOFS_NOGND_TRIP_CNT);
  m_StuckRelayStartTimeMS = 0;
  m_StuckRelayTripCnt = eeprom_read_byte((uint8_t*)EOFS_STUCK_RELAY_TRIP_CNT);
  m_NoGndRetryCnt = 0;
  m_NoGndStart = 0;

  uint8_t fault;
  do {
    fault = 0;
    uint8_t psvclvl = doPost(); // 执行上电自检（POST）

#ifdef AUTOSVCLEVEL
    if ((AutoSvcLevelEnabled()) && ((psvclvl == L1) || (psvclvl == L2))) {
      svclvl = psvclvl;
    }
#endif

    if ((GndChkEnabled()) && (psvclvl == OG)) {
      m_EvseState = EVSE_STATE_NO_GROUND;
      fault = 1;
    }
    if ((StuckRelayChkEnabled()) && (psvclvl == SR)) {
      m_EvseState = EVSE_STATE_STUCK_RELAY;
      fault = 1;
    }
#ifdef GFI_SELFTEST
    if ((GfiSelfTestEnabled()) && (psvclvl == FG)) {
      m_EvseState = EVSE_STATE_GFI_TEST_FAILED;
      fault = 1;
    }
#endif

    if (fault) {
#ifdef UL_COMPLIANT
      while (1) {
        ProcessInputs(); // 无限循环，直到断电
      }
#else
      unsigned long faultms = millis();
      while ((millis() - faultms) < 2 * 60000ul) {
        ProcessInputs(); // 2 分钟内重试 POST
      }
#endif
    }
  } while (fault &&
           (m_EvseState == EVSE_STATE_GFI_TEST_FAILED ||
            m_EvseState == EVSE_STATE_NO_GROUND ||
            m_EvseState == EVSE_STATE_STUCK_RELAY));
#endif

  SetSvcLevel(svclvl);

#ifdef DELAYTIMER
  if (g_DelayTimer.IsTimerEnabled()) {
    Sleep(); // 若启用延迟计时器，设备进入休眠状态
  }
#endif

  g_OBD.SetGreenLed(0); // 设置绿色 LED 指示灯熄灭

#ifdef RAPI
  RapiSendBootNotification(); // 通知主机已启动
#endif

#ifdef HEARTBEAT_SUPERVISION
  m_HsInterval = eeprom_read_word((uint16_t*)EOFS_HEARTBEAT_SUPERVISION_INTERVAL);
  m_IFallback = eeprom_read_byte((uint8_t*)EOFS_HEARTBEAT_SUPERVISION_CURRENT);

  if (m_HsInterval == 0xffff) {
    m_HsInterval = HS_INTERVAL_DEFAULT;
    m_IFallback = HS_IFALLBACK_DEFAULT;
  }

  m_HsLastPulse = millis(); // 初始化心跳检测时间戳
#endif
}

// 读取 Pilot 信号电压值范围
void J1772EVSEController::ReadPilot(uint16_t *plow, uint16_t *phigh)
{
  uint16_t pl = 1023; // 初始最小值设为最大
  uint16_t ph = 0;    // 初始最大值设为最小

  for (int i = 0; i < PILOT_LOOP_CNT; i++) {
    uint16_t reading = adcPilot.read(); // 读取 pilot 引脚模拟值
    if (reading > ph) ph = reading;
    else if (reading < pl) pl = reading;
  }

  // 非 -12V 状态下处理连接状态
  if (m_Pilot.GetState() != PILOT_STATE_N12) {
    if (EvConnected()) SetEvConnectedPrev();
    else ClrEvConnectedPrev();

    if (ph >= m_ThreshData.m_ThreshAB) {
      ClrEvConnected(); // 断开连接状态
#ifdef MENNEKES_LOCK
      if (!MennekesIsManual()) m_MennekesLock.Unlock(0);
#endif
    } else {
      SetEvConnected(); // 连接状态
#ifdef MENNEKES_LOCK
      if (!MennekesIsManual()) m_MennekesLock.Lock(0);
#endif
    }
  }

  *plow = pl;
  *phigh = ph;
}


// 表 A1 - 引导线电压范围（推荐...根据需要调整）
//                             最小值      标称值       最大值
// 正电压，状态 A        11.40      12.00      12.60
// 正电压，状态 B        8.36       9.00       9.56
// 正电压，状态 C        5.48       6.00       6.49
// 正电压，状态 D        2.62       3.00       3.25
// 负电压 - 状态 B，C，D 和 F   -11.40   -12.00   -12.60
void J1772EVSEController::Update(uint8_t forcetransition)
{
  uint16_t plow;
  uint16_t phigh = 0xffff;

  unsigned long curms = millis();

  // 如果当前状态是禁用状态，取消状态转换并返回
  if (m_EvseState == EVSE_STATE_DISABLED) {
    m_PrevEvseState = m_EvseState; // 取消状态转换
    return;
  }

  ReadPilot(&plow,&phigh); // 始终读取以便更新 EV 连接状态

  if (EvConnectedTransition()) {
    if (EvConnected()) {
      m_AccumulatedChargeTime = 0;
      m_ElapsedChargeTime = 0;
    }
  }

  // 如果当前状态为休眠状态
  if (m_EvseState == EVSE_STATE_SLEEPING) {
    int8_t cancelTransition = 1;
    if (chargingIsOn()) {
      // 等待引导电压高于状态 C。如果发生以下情况：
      // a) EV 做出反应并回到状态 B（打开接触点）
      // b) 用户拔掉充电连接器
      // 如果 3 秒内未发生此操作，则我们将强制打开继电器
      // c) 没有电流意味着 EV 即使保持在状态 C 也已打开接触点
      //    允许 3A 的误差以补偿电表不准确
#ifdef AMMETER
      readAmmeter();
      long instantma = m_AmmeterReading * m_CurrentScaleFactor - m_AmmeterCurrentOffset;
      if (instantma < 0) instantma = 0;
#endif // AMMETER
      // 如果电压高于阈值或者电流小于指定值，则打开充电
      if ((phigh >= m_ThreshData.m_ThreshBC)
#ifdef AMMETER
	 || (instantma <= 1000L)
#endif // AMMETER
	  || ((curms - m_ChargeOffTimeMS) >= 3000UL)) {
#ifdef FT_SLEEP_DELAY
	// sprintf(g_sTmp,"SLEEP OPEN %d",(int)phigh);
	// g_OBD.LcdMsg(g_sTmp,(phigh >= m_ThreshData.m_ThreshBC) ? "THRESH" : "TIMEOUT");
	sprintf(g_sTmp,"%d %lu %lu",phigh,instantma,(curms - m_ChargeOffTimeMS));
	g_OBD.LcdMsg(g_sTmp,(phigh >= m_ThreshData.m_ThreshBC) ? "THRESH" : "TIMEOUT");
	chargingOff();
	for(;;)
	wdt_delay(2000);
#endif // FT_SLEEP_DELAY
	chargingOff();
      }
    }
    else { // 如果没有正在充电
#if defined(TIME_LIMIT) || defined(CHARGE_LIMIT)
      if (LimitSleepIsSet()) {
	if (!EvConnected()) {
	  // 如果由于时间/充电限制而进入休眠状态，那么在拔掉车时自动取消休眠
	  cancelTransition = 0;
	  SetLimitSleep(0);
	  m_EvseState = EVSE_STATE_UNKNOWN;
	}
      }
#endif //defined(TIME_LIMIT) || defined(CHARGE_LIMIT)

      if (EvConnectedTransition()) {
#ifdef DELAYTIMER
	if (!EvConnected()) {
	  g_DelayTimer.ClrManualOverride();
      }
#endif // DELAYTIMER
	g_OBD.Update(OBD_UPD_FORCE);
#ifdef RAPI
	RapiSendEvseState();
#endif // RAPI
      }
    }

    if (cancelTransition) {
      m_PrevEvseState = m_EvseState; // 取消状态转换
      return;
    }
  }

  uint8_t prevevsestate = m_EvseState;
  uint8_t tmpevsestate = EVSE_STATE_UNKNOWN;
  uint8_t nofault = 1;

#ifdef ADVPWR
  uint8_t acpinstate = ReadACPins();

  // 如果启用了 CGMI 和接地检查并且接地测试引脚打开
  if (CGMIisEnabled() && GndChkEnabled() && (acpinstate & GND_TEST_PIN_OPEN)) {
    // 接地故障
    tmpevsestate = EVSE_STATE_NO_GROUND;
    m_EvseState = EVSE_STATE_NO_GROUND;
    chargingOff(); // 打开继电器
    if ((prevevsestate != EVSE_STATE_NO_GROUND) &&
        (((uint8_t)(m_NoGndTripCnt+1)) < 254)) {
      m_NoGndTripCnt++;
      eeprom_write_byte((uint8_t*)EOFS_NOGND_TRIP_CNT,m_NoGndTripCnt);
    }
    nofault = 0;
  }

  if (nofault) {
    if (chargingIsOn()) { // 如果继电器闭合
      if (StuckRelayChkEnabled() && CGMIisEnabled() && ((curms - m_ChargeOnTimeMS) > GROUND_CHK_DELAY) && (acpinstate & RLY_TEST_PIN_OPEN)) {
        // 如果继电器未闭合
        chargingOff(); // 打开继电器
        tmpevsestate = EVSE_STATE_RELAY_CLOSURE_FAULT;
        m_EvseState = EVSE_STATE_RELAY_CLOSURE_FAULT;
        nofault = 0;
      }
      else if (!CGMIisEnabled() && ((curms - m_ChargeOnTimeMS) > GROUND_CHK_DELAY)) {
        // 地面检查 - 只能在继电器闭合时进行
        if (GndChkEnabled() && (acpinstate == ACPINS_OPEN)) {
          // 接地故障
          tmpevsestate = EVSE_STATE_NO_GROUND;
          m_EvseState = EVSE_STATE_NO_GROUND;

          chargingOff(); // 打开继电器
          if ((prevevsestate != EVSE_STATE_NO_GROUND) && (((uint8_t)(m_NoGndTripCnt+1)) < 254)) {
            m_NoGndTripCnt++;
            eeprom_write_byte((uint8_t*)EOFS_NOGND_TRIP_CNT,m_NoGndTripCnt);
          }
          m_NoGndStart = curms;

          nofault = 0;
        }

#ifdef AUTOSVCLEVEL
        // 如果 EV 在 POST 期间插入，我们无法进行 AutoSvcLevel 检测，
        // 所以必须硬编码为 L1。在第一次充电会话时，我们可以探测并在必要时设置为 L2
        if (AutoSvcLvlSkipped() && (m_EvseState == EVSE_STATE_C)) {
          if (!acpinstate) {
            // 设置为 L2
            SetSvcLevel(2,1);
          }
          SetAutoSvcLvlSkipped(0);
        }
#endif // AUTOSVCLEVEL
      }
    }
    else { // 如果没有正在充电 - 继电器打开
      if (!CGMIisEnabled() && (prevevsestate == EVSE_STATE_NO_GROUND)) {
        // 检查是否拔掉 EV
        if (!EvConnected()) {
          // EV 断开 - 取消故障
          m_EvseState = EVSE_STATE_UNKNOWN;
          return;
        }

        if (((m_NoGndRetryCnt < GFI_RETRY_COUNT) || (GFI_RETRY_COUNT == 255)) &&
            ((curms - m_NoGndStart) > GFI_TIMEOUT)) {
          m_NoGndRetryCnt++;
        }
        else {
          tmpevsestate = EVSE_STATE_NO_GROUND;
          m_EvseState = EVSE_STATE_NO_GROUND;

          nofault = 0;
        }
      }
      else if (StuckRelayChkEnabled()) {    // 检查卡住的继电器 - 只能在继电器打开时测试
        if ((CGMIisEnabled() && !(acpinstate & RLY_TEST_PIN_OPEN)) ||
            (!CGMIisEnabled() && (acpinstate != ACPINS_OPEN))) { // 检查卡住的继电器
          if ((prevevsestate != EVSE_STATE_STUCK_RELAY) && !m_StuckRelayStartTimeMS) { // 检查第一次发生
            m_StuckRelayStartTimeMS = curms; // 标记开始状态
          }
          if ( ( ((curms - m_ChargeOffTimeMS) > STUCK_RELAY_DELAY) && // 充电关闭去抖动
                 ((curms - m_StuckRelayStartTimeMS) > STUCK_RELAY_DELAY) ) ||  // 开始去抖动延迟
               (prevevsestate == EVSE_STATE_STUCK_RELAY) ) { // 已经在故障状态
            // 卡住的继电器
            tmpevsestate = EVSE_STATE_STUCK_RELAY;
            m_EvseState = EVSE_STATE_STUCK_RELAY;
            chargingOff(); // 打开继电器
            nofault = 0;
          }
        }
      }
    }
  }
#endif // ADVPWR
   
#ifdef GFI
  // 如果检测到GFI故障
  if (m_Gfi.Fault()) {
    tmpevsestate = EVSE_STATE_GFCI_FAULT; // 设置临时状态为GFCI故障
    m_EvseState = EVSE_STATE_GFCI_FAULT;  // 更新EVSE状态为GFCI故障

    // 如果之前的状态不是GFCI故障，表示状态转换
    if (prevevsestate != EVSE_STATE_GFCI_FAULT) {
      // 检查GFI触发计数是否小于254，如果小于，则增加
      if (((uint8_t)(m_GfiTripCnt+1)) < 254) {
        m_GfiTripCnt++;
        eeprom_write_byte((uint8_t*)EOFS_GFI_TRIP_CNT,m_GfiTripCnt); // 保存触发计数到EEPROM
      }
      m_GfiRetryCnt = 0; // 重试计数归零
      m_GfiFaultStartMs = curms; // 记录故障开始的时间戳
    }
    else { // 如果已经处于GFCI故障状态
      if (!EvConnected()) {
        // 如果电动车断开连接，则取消故障状态
        m_EvseState = EVSE_STATE_UNKNOWN;
        m_Gfi.Reset(); // 重置GFI故障状态
        return;
      }

      // 如果故障已持续超过指定时间（GFI_TIMEOUT）
      if ((curms - m_GfiFaultStartMs) >= GFI_TIMEOUT) {
#ifdef FT_GFI_RETRY
        g_OBD.LcdMsg("Reset","GFI"); // 在LCD上显示"GFI重置"
        delay(250); // 等待250ms
#endif // FT_GFI_RETRY
        m_GfiRetryCnt++; // 增加重试计数

        // 如果重试次数超过最大值，触发硬件故障
        if ((GFI_RETRY_COUNT != 255) && (m_GfiRetryCnt > GFI_RETRY_COUNT)) {
          HardFault(1); // 触发硬故障
          return;
        }
        else {
          m_Gfi.Reset(); // 重置GFI
          m_GfiFaultStartMs = 0; // 重置故障开始时间
        }
      }
    }

    nofault = 0; // 设置无故障标志为0
  }
#endif // GFI

#ifdef TEMPERATURE_MONITORING // 温度监控状态
  if (TempChkEnabled()) {
    // 如果任何温度传感器的温度超过了阈值，进入过温状态
    if ((g_TempMonitor.m_TMP007_temperature >= TEMPERATURE_INFRARED_PANIC) ||
        (g_TempMonitor.m_MCP9808_temperature >= TEMPERATURE_AMBIENT_PANIC) ||
        (g_TempMonitor.m_DS3231_temperature >= TEMPERATURE_AMBIENT_PANIC)) {
      tmpevsestate = EVSE_STATE_OVER_TEMPERATURE; // 设置临时状态为过温
      m_EvseState = EVSE_STATE_OVER_TEMPERATURE; // 更新EVSE状态为过温
      nofault = 0; // 设置无故障标志为0
    }
  }
#endif // TEMPERATURE_MONITORING

  uint8_t prevpilotstate = m_PilotState; // 保存上一个Pilot状态
  uint8_t tmppilotstate = EVSE_STATE_UNKNOWN; // 临时Pilot状态初始化为未知

  if (nofault) {
    // 如果当前没有故障
    if ((prevevsestate >= EVSE_FAULT_STATE_BEGIN) && (prevevsestate <= EVSE_FAULT_STATE_END)) {
      // 如果之前的状态在故障状态范围内，表示刚刚退出故障状态，Pilot信号恢复
      m_Pilot.SetState(PILOT_STATE_P12); // 设置Pilot信号为P12
      prevevsestate = EVSE_STATE_UNKNOWN; // 恢复EVSE状态为未知
      m_EvseState = EVSE_STATE_UNKNOWN;   // 恢复EVSE状态为未知
    }

    // 如果启用了二极管检查，并且Pilot状态为PWM且电压低于阈值，表示二极管检查失败
    if (DiodeCheckEnabled() && (m_Pilot.GetState() == PILOT_STATE_PWM) && (plow >= m_ThreshData.m_ThreshDS)) {
      tmpevsestate = EVSE_STATE_DIODE_CHK_FAILED; // 设置临时状态为二极管检查失败
      tmppilotstate = EVSE_STATE_DIODE_CHK_FAILED; // 设置Pilot状态为二极管检查失败
    }
    // 判断不同电压状态，并设置对应的EVSE状态
    else if (phigh >= m_ThreshData.m_ThreshAB) {
      tmpevsestate = EVSE_STATE_A; // 12V EV未连接
      tmppilotstate = EVSE_STATE_A;
    }
    else if (phigh >= m_ThreshData.m_ThreshBC) {
      tmpevsestate = EVSE_STATE_B; // 9V EV连接，等待准备充电
      tmppilotstate = EVSE_STATE_B;
    }
    else if (phigh >= m_ThreshData.m_ThreshCD) {
      tmppilotstate = EVSE_STATE_C; // 6V EV准备充电
      if (m_Pilot.GetState() == PILOT_STATE_PWM) {
        tmpevsestate = EVSE_STATE_C; // PWM开启，充电准备好
      }
      else {
        // 如果PWM关闭，则无法充电，强制转到状态B
        tmpevsestate = EVSE_STATE_B;
      }
    }
    else if (phigh > m_ThreshData.m_ThreshD) {
      tmppilotstate = EVSE_STATE_D; // 3V充电通风要求
      if (VentReqEnabled()) {
        tmpevsestate = EVSE_STATE_D; // 如果启用了通风要求，则进入D状态
      }
      else {
        tmpevsestate = EVSE_STATE_C; // 否则保持C状态
      }
    }
    else {
      tmpevsestate = EVSE_STATE_UNKNOWN; // 未知状态
    }

#ifdef FT_ENDURANCE
    if (nofault) {
      // 如果状态是A或B，并且循环计数小于0，初始化循环计数和状态
      if (((tmpevsestate == EVSE_STATE_A)||(tmpevsestate == EVSE_STATE_B)) && (g_CycleCnt < 0)) {
        g_CycleCnt = 0;
        g_CycleHalfStart = curms;
        g_CycleState = EVSE_STATE_B;
      }

      // 如果循环计数大于等于0，处理循环状态
      if (g_CycleCnt >= 0) {
        if (g_CycleState == EVSE_STATE_B) {
          if ((curms - g_CycleHalfStart) >= 9000) {
            g_CycleCnt++;
            g_CycleHalfStart = curms;
            tmpevsestate = EVSE_STATE_C; // 进入C状态
            g_CycleState = EVSE_STATE_C;
          }
          else tmpevsestate = EVSE_STATE_B; // 保持B状态
        }
        else if (g_CycleState == EVSE_STATE_C) {
          if ((curms - g_CycleHalfStart) >= 1000) {
            g_CycleHalfStart = curms;
            tmpevsestate = EVSE_STATE_B; // 进入B状态
            g_CycleState = EVSE_STATE_B;
          }
          else tmpevsestate = EVSE_STATE_C; // 保持C状态
        }
      }
    }
#endif // FT_ENDURANCE

    // 防抖动状态转换
    if (tmpevsestate != prevevsestate) {
      if (tmpevsestate != m_TmpEvseState) {
        m_TmpEvseStateStart = curms; // 记录状态开始时间
      }
      else if ((curms - m_TmpEvseStateStart) >= ((tmpevsestate == EVSE_STATE_A) ? DELAY_STATE_TRANSITION_A : DELAY_STATE_TRANSITION)) {
        m_EvseState = tmpevsestate; // 更新EVSE状态
      }
    }
  } // 如果没有故障

  // 防抖动Pilot状态转换
  if (tmppilotstate != prevpilotstate) {
    if (tmppilotstate != m_TmpPilotState) {
      m_TmpPilotStateStart = curms; // 记录Pilot状态开始时间
    }
    else if ((curms - m_TmpPilotStateStart) >= DELAY_STATE_TRANSITION) {
      m_PilotState = tmppilotstate; // 更新Pilot状态
    }
  }

  m_TmpPilotState = tmppilotstate; // 保存当前Pilot状态
  m_TmpEvseState = tmpevsestate; // 保存当前EVSE状态

#ifdef FT_GFI_RETRY
  // 如果没有故障，且处于C状态超过10秒，进行GFI故障诱导测试
  if (nofault && (prevevsestate == EVSE_STATE_C) && ((curms - m_ChargeOnTimeMS) > 10000)) {
    g_OBD.LcdMsg("Induce","Fault"); // 在LCD上显示"诱导故障"
    for(int i = 0; i < GFI_TEST_CYCLES; i++) {
      m_Gfi.pinTest.write(1); // 发送测试脉冲
      delayMicroseconds(GFI_PULSE_ON_US);
      m_Gfi.pinTest.write(0); // 关闭脉冲
      delayMicroseconds(GFI_PULSE_OFF_US);
      if (m_Gfi.Fault()) break; // 如果检测到故障，则跳出循环
    }
  }
#endif // FT_GFI_RETRY

  //
    // 检查请求的状态转换到 A/B/C
    //

  #ifdef AUTH_LOCK
  #ifdef AUTH_LOCK_REG
    {
      int8_t locked;
      // 读取是否锁定
      if (pinAuthLock.read()) locked = 1;
      else locked = 0;

      if (m_EvseState == EVSE_STATE_A) {
        // 在 STATE_A 状态下忽略 PIN 密码，总是锁定
        locked = 1;
      }

      // 如果锁定状态和预期不同，则更新锁定状态
      if (locked != AuthLockIsOn()) {
        AuthLock(locked, 0);  // 更新锁定状态
        g_OBD.Update(OBD_UPD_FORCE);  // 强制更新 OBD
        forcetransition = 1;  // 强制状态转换
      }
    }
  #endif // AUTH_LOCK_REG

    // 如果锁定并且当前状态是 C，则强制转换到 B 状态
    if (AuthLockIsOn() && (m_EvseState == EVSE_STATE_C)) {
      m_EvseState = EVSE_STATE_B;
    }
  #endif // AUTH_LOCK

    if (
  #ifdef STATE_TRANSITION_REQ_FUNC
        m_StateTransitionReqFunc &&
  #endif // STATE_TRANSITION_REQ_FUNC
        (m_EvseState != prevevsestate) &&
        ((m_EvseState >= EVSE_STATE_A) && (m_EvseState <= EVSE_STATE_C))) {
      m_PilotState = tmppilotstate;
  #ifdef STATE_TRANSITION_REQ_FUNC
      uint8_t newstate = (*m_StateTransitionReqFunc)(prevpilotstate, m_PilotState, prevevsestate, m_EvseState);
      if (newstate) {
        m_EvseState = newstate;  // 如果返回新的状态，则更新状态
      }
  #endif // STATE_TRANSITION_REQ_FUNC
    }

    // 状态转换
    if (forcetransition || (m_EvseState != prevevsestate)) {
      if (m_EvseState == EVSE_STATE_A) { // EV 未连接
        chargingOff(); // 关闭充电电流
        m_Pilot.SetState(PILOT_STATE_P12); // 设置到 P12 状态
  #if defined(AUTH_LOCK) && ((AUTH_LOCK != 0) || !defined(AUTH_LOCK_REG))
        // 如果默认是锁定的，转换到 STATE_A 时进行锁定
        AuthLock(1, 0);
  #endif
  #ifdef CHARGE_LIMIT
        ClrChargeLimit();  // 清除充电限制
  #endif // CHARGE_LIMIT
  #ifdef TIME_LIMIT
        ClrTimeLimit();  // 清除时间限制
  #endif // TIME_LIMIT
  #ifdef DELAYTIMER
        g_DelayTimer.ClrManualOverride();  // 清除手动覆盖
  #endif // DELAYTIMER
  #ifdef TEMPERATURE_MONITORING
        g_TempMonitor.ClrOverTemperatureLogged();  // 清除过温记录
  #endif
      }
      else if (m_EvseState == EVSE_STATE_B) { // 已连接
        chargingOff(); // 关闭充电电流
  #ifdef AUTH_LOCK
        // 如果已锁定，不开启 PWM
        if (AuthLockIsOn()) {
          m_Pilot.SetState(PILOT_STATE_P12); // 锁定状态下设置 P12 状态
        }
        else {
          m_Pilot.SetPWM(m_CurrentCapacity);  // 否则设置 PWM
        }
  #else
        m_Pilot.SetPWM(m_CurrentCapacity);  // 设置 PWM
  #endif // AUTH_LOCK
      }
      else if (m_EvseState == EVSE_STATE_C) {
        m_Pilot.SetPWM(m_CurrentCapacity);  // 设置 PWM
  #if defined(UL_GFI_SELFTEST) && !defined(NOCHECKS)
        // 如果启用了 GFI 自测，测试 GFI
        if (GfiSelfTestEnabled() && m_Gfi.SelfTest()) {
          // GFI 测试失败 - 硬件故障
          m_EvseState = EVSE_STATE_GFI_TEST_FAILED;
          m_Pilot.SetState(PILOT_STATE_P12); // 设置 P12 状态
          HardFault(1);  // 发生硬件故障
          return;
        }
  #endif // UL_GFI_SELFTEST

  #ifdef FT_GFI_LOCKOUT
        for(int i = 0; i < GFI_TEST_CYCLES; i++) {
          m_Gfi.pinTest.write(1);  // 激活 GFI 引脚
          delayMicroseconds(GFI_PULSE_ON_US);  // 延迟
          m_Gfi.pinTest.write(0);  // 关闭 GFI 引脚
          delayMicroseconds(GFI_PULSE_OFF_US);  // 延迟
          if (m_Gfi.Fault()) break;  // 检查故障
        }
        g_OBD.LcdMsg("Closing", "Relay");  // LCD 显示关断继电器
        delay(150);  // 延迟
  #endif // FT_GFI_LOCKOUT

        chargingOn(); // 启动充电电流
      }
      else if (m_EvseState == EVSE_STATE_D) {
        // 不支持通风
        chargingOff(); // 关闭充电电流
        m_Pilot.SetState(PILOT_STATE_P12); // 设置 P12 状态
        HardFault(1);  // 发生硬件故障
      }
      else if (m_EvseState == EVSE_STATE_GFCI_FAULT) {
        // 车辆状态 F
        chargingOff(); // 关闭充电电流
        m_Pilot.SetState(PILOT_STATE_P12); // 设置 P12 状态
      }
  #ifdef TEMPERATURE_MONITORING
      else if (m_EvseState == EVSE_STATE_OVER_TEMPERATURE) {
        // EVSE 内部过温
        m_Pilot.SetState(PILOT_STATE_P12);  // 暂停 EV，5 秒内应停止高电流

        // 等待 5 秒钟
        while ((millis() - curms) < 5000) {
          wdt_reset();  // 重置看门狗定时器
        }
        chargingOff();  // 打开 EVSE 继电器，期望 EV 已经断开
        HardFault(1);  // 发生硬件故障
      }
  #endif //TEMPERATURE_MONITORING
      else if (m_EvseState == EVSE_STATE_DIODE_CHK_FAILED) {
        chargingOff(); // 关闭充电电流
        // 必须保持 pilot 引脚以继续检查
        // 注意：J1772 规范要求进入状态 F (-12V)，但我们不能这么做，同时保持检查
        m_Pilot.SetPWM(m_CurrentCapacity);  // 设置 PWM
        m_Pilot.SetState(PILOT_STATE_P12);  // 设置 P12 状态
        HardFault(1);  // 发生硬件故障
      }
      else if (m_EvseState == EVSE_STATE_NO_GROUND) {
        // 未检测到接地
        chargingOff(); // 关闭充电电流
        m_Pilot.SetState(PILOT_STATE_P12); // 设置 P12 状态
      }
      else if (m_EvseState == EVSE_STATE_STUCK_RELAY) {
        // 检测到继电器卡住
        chargingOff(); // 关闭充电电流
  #ifdef UL_COMPLIANT
        // 根据与 UL 的讨论，永远硬件故障卡住的继电器
        HardFault(0);  // 发生硬件故障
  #endif // UL_COMPLIANT
      }
      else {
        m_Pilot.SetState(PILOT_STATE_P12); // 设置 P12 状态
        chargingOff(); // 关闭充电电流
      }
  #ifdef SERDBG
      if (SerDbgEnabled()) {
        // 串口调试信息
        Serial.print("state: ");
        switch (m_Pilot.GetState()) {
        case PILOT_STATE_P12: Serial.print("P12"); break;
        case PILOT_STATE_PWM: Serial.print("PWM"); break;
        case PILOT_STATE_N12: Serial.print("N12"); break;
        }
        Serial.print(" ");
        Serial.print((int)prevevsestate);
        Serial.print("->");
        Serial.print((int)m_EvseState);
        Serial.print(" p ");
        Serial.print(plow);
        Serial.print(" ");
        Serial.println(phigh);
      }
  #endif //#ifdef SERDBG
    } // 状态转换

  #ifdef AUTH_LOCK
    // 如果状态或 pilot 状态改变，则强制更新 OBD
    if ((m_EvseState != prevevsestate) ||
        (m_PilotState != prevpilotstate)) {
      g_OBD.Update(OBD_UPD_FORCE);  // 强制更新 OBD
    }
  #endif // AUTH_LOCK


#ifdef UL_COMPLIANT
  if (!nofault && (prevevsestate == EVSE_STATE_C)) {
    // 如果充电开始后 2 秒内发生故障，认为是硬故障
    if ((curms - m_ChargeOnTimeMS) <= 2000) {
      HardFault(1);  // 触发硬故障处理
      return;  // 结束当前函数
    }
  }
#endif // UL_COMPLIANT

  m_PrevEvseState = prevevsestate;  // 记录之前的 EVSE 状态

#ifdef VOLTMETER
  ReadVoltmeter();  // 读取电压表
#endif // VOLTMETER

#ifdef AMMETER
  // 如果 EVSE 状态是 C，并且电流比例因子大于 0
  if (((m_EvseState == EVSE_STATE_C) && (m_CurrentScaleFactor > 0))
#ifdef ECVF_AMMETER_CAL
      || AmmeterCalEnabled()  // 如果电流表校准启用
#endif
      ) {

#ifndef FAKE_CHARGING_CURRENT
    readAmmeter();  // 读取电流表
    uint32_t ma = MovingAverage(m_AmmeterReading);  // 计算电流表的移动平均值
    if (ma != 0xffffffff) {
      m_ChargingCurrent = ma * m_CurrentScaleFactor - m_AmmeterCurrentOffset;  // 计算充电电流并扣除偏移
      if (m_ChargingCurrent < 0) {
        m_ChargingCurrent = 0;  // 防止电流为负数
      }
      g_OBD.SetAmmeterDirty(1);  // 标记电流表数据为脏数据
    }
#endif // !FAKE_CHARGING_CURRENT
  }

#ifdef OVERCURRENT_THRESHOLD
  if (m_EvseState == EVSE_STATE_C) {
    // 如果充电电流超过设定的过流阈值
    if (m_ChargingCurrent >= ((m_CurrentCapacity + OVERCURRENT_THRESHOLD) * 1000L)) {
      if (m_OverCurrentStartMs) {  // 如果已经进入过流状态
        if ((millis() - m_OverCurrentStartMs) >= OVERCURRENT_TIMEOUT) {
          // 如果过流时间过长，停止充电并触发硬故障
          m_EvseState = EVSE_STATE_OVER_CURRENT;
          m_Pilot.SetState(PILOT_STATE_P12);  // 发送信号让 EV 暂停充电
          curms = millis();
          while ((millis() - curms) < 1000) {  // 等待 1 秒钟让 EV 停止充电
            wdt_reset();
          }
          chargingOff();  // 关闭 EVSE 继电器，希望 EV 已断开连接

          // 等待 EV 断开连接
          HardFault(1);  // 触发硬故障处理

          m_OverCurrentStartMs = 0;  // 清除过流状态
        }
      }
      else {
        m_OverCurrentStartMs = millis();  // 记录过流开始时间
      }
    }
    else {
      m_OverCurrentStartMs = 0;  // 清除过流状态
    }
  }
  else {
    m_OverCurrentStartMs = 0;  // 清除过流状态
  }
#endif // OVERCURRENT_THRESHOLD
#endif // AMMETER


#ifdef HEARTBEAT_SUPERVISION
    this->HsExpirationCheck();  // 检查心跳是否丢失，若丢失则执行相应处理
#endif //HEARTBEAT_SUPERVISION

#ifdef TEMPERATURE_MONITORING
    if(TempChkEnabled()) {
      uint8_t currcap = GetMaxCurrentCapacity();  // 获取当前最大电流容量
      uint8_t setit = 0;

      if (g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature <= TEMPERATURE_INFRARED_RESTORE_AMPERAGE) &&  // 所有传感器的温度需要恢复到较低水平
                                              (g_TempMonitor.m_MCP9808_temperature <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE) &&
                                              (g_TempMonitor.m_DS3231_temperature <= TEMPERATURE_AMBIENT_RESTORE_AMPERAGE))) {  // 恢复为用户的原始电流设置
        setit = 1;  // 设置为原始电流
      }
      else if (g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature <= TEMPERATURE_INFRARED_THROTTLE_DOWN) &&  // 所有传感器的温度需要恢复到较低水平
                                                          (g_TempMonitor.m_MCP9808_temperature <= TEMPERATURE_AMBIENT_THROTTLE_DOWN) &&
                                                          (g_TempMonitor.m_DS3231_temperature <= TEMPERATURE_AMBIENT_THROTTLE_DOWN))) {   // 恢复降流设置
        currcap /= 2;  // 设置为降流后的电流容量
        setit = 3;
      }
      if (setit) {
        if (setit <= 2) {
          g_TempMonitor.SetOverTemperature(setit - 1);  // 设置过温状态
        }
        else {
          g_TempMonitor.SetOverTemperatureShutdown(setit - 3);  // 设置过温关机状态
        }
        SetCurrentCapacity(currcap, 0, 1);  // 设置电流容量
        if (m_Pilot.GetState() != PILOT_STATE_PWM) {
          m_Pilot.SetPWM(m_CurrentCapacity);  // 设置 PWM 信号
        }
      }
    }
#endif // TEMPERATURE_MONITORING

  if (m_EvseState == EVSE_STATE_C) {
    m_ElapsedChargeTimePrev = m_ElapsedChargeTime;  // 记录之前的充电时间
    m_ElapsedChargeTime = (millis() - m_ChargeOnTimeMS) / 1000;  // 计算已充电时间


#ifdef TEMPERATURE_MONITORING
  if(TempChkEnabled()) {
    if (m_ElapsedChargeTime != m_ElapsedChargeTimePrev) {
      uint8_t currcap = GetMaxCurrentCapacity();
      uint8_t setit = 0;

      if (!g_TempMonitor.OverTemperature() && ((g_TempMonitor.m_TMP007_temperature >= TEMPERATURE_INFRARED_THROTTLE_DOWN) ||  // 任一传感器超过阈值会触发动作
                                               (g_TempMonitor.m_MCP9808_temperature >= TEMPERATURE_AMBIENT_THROTTLE_DOWN) ||
                                               (g_TempMonitor.m_DS3231_temperature >= TEMPERATURE_AMBIENT_THROTTLE_DOWN))) {   // 降低充电电流
        currcap /= 2;  // 设置为降流后的电流容量
        setit = 2;
      }
      else if (!g_TempMonitor.OverTemperatureShutdown() && ((g_TempMonitor.m_TMP007_temperature >= TEMPERATURE_INFRARED_SHUTDOWN) ||  // 任一传感器超过阈值会触发关机动作
                                                              (g_TempMonitor.m_MCP9808_temperature >= TEMPERATURE_AMBIENT_SHUTDOWN) ||
                                                              (g_TempMonitor.m_DS3231_temperature >= TEMPERATURE_AMBIENT_SHUTDOWN))) {   // 强制关机
        currcap /= 4;  // 设置为更低的电流容量
        setit = 4;
      }
      if (setit) {
        if (setit <= 2) {
          g_TempMonitor.SetOverTemperature(setit - 1);  // 设置过温状态
        }
        else {
          g_TempMonitor.SetOverTemperatureShutdown(setit - 3);  // 设置过温关机状态
        }
        SetCurrentCapacity(currcap, 0, 1);  // 设置电流容量
        if (m_Pilot.GetState() != PILOT_STATE_PWM) {
          m_Pilot.SetPWM(m_CurrentCapacity);  // 设置 PWM 信号
        }
      }
    }
  }
#endif // TEMPERATURE_MONITORING

#ifdef CHARGE_LIMIT
    if (m_chargeLimitTotWs && (g_EnergyMeter.GetSessionWs() >= m_chargeLimitTotWs)) {
      ClrChargeLimit(); // 清除充电限制
#ifdef TIME_LIMIT
      ClrTimeLimit(); // 清除时间限制
#endif // TIME_LIMIT
      SetLimitSleep(1); // 设置进入休眠状态
      Sleep(); // 休眠
    }
#endif
#ifdef TIME_LIMIT
    if (m_timeLimitEnd) {
      // 必须调用 millis()，因为 curms 在切换到 C 状态之前已被采样，所以 m_ChargeOnTimeMS 从一开始就会大于 curms
      if (GetElapsedChargeTime() >= m_timeLimitEnd) {
        ClrTimeLimit(); // 清除时间限制
#ifdef CHARGE_LIMIT
        ClrChargeLimit(); // 清除充电限制
#endif // CHARGE_LIMIT
        SetLimitSleep(1); // 设置进入休眠状态
        Sleep(); // 休眠
      }
    }
#endif // TIME_LIMIT
  }

#ifdef RAPI
    RapiSendEvseState(); // 发送 EVSE 状态
#endif // RAPI

  return;
}

#ifdef CALIBRATE
// 读取 ADC 值并获取 pilot 稳定高/低状态的最小/最大/平均值
void J1772EVSEController::Calibrate(PCALIB_DATA pcd)
{
  uint16_t pmax,pmin,pavg,nmax,nmin,navg;

  for (int l=0;l < 2;l++) {
    int reading;
    uint32_t tot = 0;
    uint16_t plow = 1023;
    uint16_t phigh = 0;
    uint16_t avg = 0;
    m_Pilot.SetState(l ? PILOT_STATE_N12 : PILOT_STATE_P12);

    delay(250); // 等待稳定

    // 1x = 114us 20x = 2.3ms 100x = 11.3ms
    int i;
    for (i=0;i < 1000;i++) {
      reading = adcPilot.read();  // 测量 pilot 电压

      if (reading > phigh) {
        phigh = reading;
      }
      else if (reading < plow) {
        plow = reading;
      }

      tot += reading;
    }
    avg = tot / i;

    if (l) {
      nmax = phigh;
      nmin = plow;
      navg = avg;
    }
    else {
      pmax = phigh;
      pmin = plow;
      pavg = avg;
    }
  }
  pcd->m_pMax = pmax;
  pcd->m_pAvg = pavg;
  pcd->m_pMin = pmin;
  pcd->m_nMax = nmax;
  pcd->m_nAvg = navg;
  pcd->m_nMin = nmin;
}
#endif // CALIBRATE

int J1772EVSEController::SetCurrentCapacity(uint8_t amps,uint8_t updatelcd,uint8_t nosave)
{
  int rc = 0;
  uint8_t maxcurrentcap = (GetCurSvcLevel() == 1) ? MAX_CURRENT_CAPACITY_L1 : m_MaxHwCurrentCapacity;

  if (nosave) {
    // 临时设置的电流不能超过 EEPROM 中设置的最大值
    maxcurrentcap = GetMaxCurrentCapacity();
  }

#ifdef PP_AUTO_AMPACITY
  if ((GetState() >= EVSE_STATE_B) && (GetState() <= EVSE_STATE_C)) {
    uint8_t mcc = g_ACCController.ReadPPMaxAmps();
    if (mcc && (mcc < maxcurrentcap)) {
      maxcurrentcap = mcc;
    }
  }
#endif // PP_AUTO_AMPACITY

  if ((amps >= MIN_CURRENT_CAPACITY_J1772) && (amps <= maxcurrentcap)) {
    m_CurrentCapacity = amps;
  }
  else if (amps < MIN_CURRENT_CAPACITY_J1772) {
    m_CurrentCapacity = MIN_CURRENT_CAPACITY_J1772;
    rc = 1;
  }
  else {
    m_CurrentCapacity = maxcurrentcap;
    rc = 2;
  }

  if (!nosave) {
    #ifdef DEBUG_HS
      Serial.println(F("SetCurrentCapacity: Writing to EEPROM!"));
    #endif
    eeprom_write_byte((uint8_t*)((GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2),(byte)m_CurrentCapacity);
  }

  if (m_Pilot.GetState() == PILOT_STATE_PWM) {
    m_Pilot.SetPWM(m_CurrentCapacity); // 设置 PWM 电流
  }

  if (updatelcd) {
    g_OBD.Update(OBD_UPD_FORCE); // 强制更新显示
  }

  return rc;
}

#ifdef HEARTBEAT_SUPERVISION
// 设置心跳监控间隔为 0 以暂停心跳监控
int J1772EVSEController::HeartbeatSupervision(uint16_t interval, uint8_t amps) {
  #ifdef DEBUG_HS
    Serial.println(F("HeartbeatSupervision called"));
    Serial.print(F("m_HsInterval was: "));
    Serial.println(m_HsInterval);
    Serial.print(F("m_IFallback was: "));
    Serial.println(m_IFallback);
  #endif
  m_HsInterval = interval;    // 设置间隔为 0 以暂停心跳监控
  m_IFallback = amps;
  m_HsTriggered = 0;         // 每次启动 HEARTBEAT_SUPERVISION 时，清除 "Triggered" 标志
  m_HsLastPulse = millis();  // 更新 m_HsLastPulse
  #ifdef DEBUG_HS
    Serial.print(F("m_HsInterval is: "));
    Serial.println(m_HsInterval);
    Serial.print(F("m_IFallback is: "));
    Serial.println(m_IFallback);
  #endif
  if (eeprom_read_word((uint16_t*)EOFS_HEARTBEAT_SUPERVISION_INTERVAL) != m_HsInterval) { // 如果需要才写入 EEPROM
    #ifdef DEBUG_HS
      Serial.print(F("Writing new m_HsInterval to EEPROM: "));
      Serial.println(m_HsInterval);
    #endif
    eeprom_write_word((uint16_t*)EOFS_HEARTBEAT_SUPERVISION_INTERVAL, m_HsInterval);
  }
  if (eeprom_read_byte((uint8_t*)EOFS_HEARTBEAT_SUPERVISION_CURRENT) != m_IFallback) { // 如果需要才写入 EEPROM
    #ifdef DEBUG_HS
      Serial.print(F("Writing new m_IFallback to EEPROM: "));
      Serial.println(m_IFallback);
    #endif
    eeprom_write_byte((uint8_t*)EOFS_HEARTBEAT_SUPERVISION_CURRENT, m_IFallback);
  }
  return 0; // 无错误码
}

int J1772EVSEController::HsPulse() {
  int rc = 1;
  #ifdef DEBUG_HS
	Serial.print(F("HsPulse called.  Time interval before reset: "));
	Serial.println((millis() - m_HsLastPulse)/1000);
  #endif
  if ((m_HsTriggered == HS_MISSEDPULSE_NOACK)) { // 如果之前丢失了心跳脉冲
    rc = 1; // 触发了心跳超时但没有收到确认，因此返回 "NK"（未确认）
  }
  else { // 如果已经被触发，说明问题已处理（m_HsTriggered = HS_MISSEDPULSE 或 0）
    rc = 0;
  }
  m_HsLastPulse = millis(); // 刚刚收到了一个心跳信号，因此重置心跳超时间隔
  #ifdef DEBUG_HS
	Serial.print(F("                 Time interval  after reset: "));
	Serial.println((millis() - m_HsLastPulse)/1000);
  #endif
  return rc;
}

int J1772EVSEController::HsRestoreAmpacity() {
  #ifdef DEBUG_HS
	Serial.println(F("HsRestoreAmpacity called"));
  #endif
  int rc=1;
  if(m_HsTriggered) { // 如果心跳超时曾被触发
    #ifdef DEBUG_HS
	  Serial.println(F("HEARTBEAT_SUPERVISION was previously triggered - checking if OK to restore ampacity"));
    #endif
    uint8_t maxCapacity = g_EvseController.GetMaxCurrentCapacity();  // 获取当前可设置的最大电流容量
    #ifdef TEMPERATURE_MONITORING
      if (!g_TempMonitor.OverTemperature()) { // 确保温度监控未触发过温保护
	    #ifdef DEBUG_HS
	      Serial.print(F("Not over temp - OK to restore ampacity to: "));
		  Serial.println(maxCapacity);
        #endif
        rc = g_EvseController.SetCurrentCapacity(maxCapacity,1,1);  // 设置当前电流为最大容量，不写入 EEPROM
      }
      else {
        #ifdef DEBUG_HS
	      Serial.println(F("Over temp still in force - not restoring ampacity"));
        #endif
        rc = 1;  // 由于温度过高，无法恢复电流容量
      }
    #else // !TEMPERATURE_MONITORING
      rc = g_EvseController.SetCurrentCapacity(maxCapacity,1,1);   // 不考虑温度，直接设置最大电流容量
    #endif // TEMPERATURE_MONITORING
  }
  else {
    rc = 0; // 如果没有触发心跳超时，直接返回
  }
  return rc;
}

int J1772EVSEController::HsExpirationCheck() {
  unsigned long sinceLastPulse = millis() - m_HsLastPulse;
  int rc=1;
  if (m_HsInterval != 0 && sinceLastPulse > (m_HsInterval * 1000)) { // 如果心跳超时
  	#ifdef DEBUG_HS
	  Serial.println(F("HsExpirationCheck: Heartbeat timer expired account late or no pulse"));
	#endif
    if(m_IFallback < GetCurrentCapacity()) { // 如果当前电流容量不符合心跳超时的限制
	  #ifdef DEBUG_HS
	    Serial.println(F("HsExpirationCheck: Reducing Current capacity"));
	  #endif
      rc=SetCurrentCapacity(m_IFallback,1,1);  // 降低当前电流容量，不写入 EEPROM
    }
    // 心跳超时，可能需要调整电流容量设置
    m_HsTriggered = HS_MISSEDPULSE_NOACK; // 标记心跳超时并等待确认
	m_HsLastPulse = millis(); // 重置计时器，等待下一个心跳
  }
  else {  // 如果心跳正常且计时器未超时
    rc = 0; // 无需调整，正常返回
  }
  return rc;
}

int J1772EVSEController::HsAckMissedPulse(uint8_t ack) {
  int rc = 1;
  #ifdef DEBUG_HS
    Serial.println(F("HsAckMissedPulse called"));
  #endif
  if (ack == HS_ACK_COOKIE) { // 检查是否为有效的丢失心跳确认
    if(m_HsTriggered == HS_MISSEDPULSE_NOACK) { // 如果心跳丢失且未确认
	  #ifdef DEBUG_HS
        Serial.println(F("HsAckMissedPulse cookie legit"));
      #endif
      rc = HsRestoreAmpacity(); // 尝试恢复电流容量
      if (rc == 0) {
		#ifdef DEBUG_HS
        Serial.println(F("HsAckMissedPulse HsRestoreAmpacity success"));
        #endif
        m_HsTriggered = HS_MISSEDPULSE;       // 恢复电流容量成功，记录丢失心跳并调整电流容量
      }
      else {
		  // 恢复电流容量失败
	  }
    }
	else {
	  rc = 0;  // 心跳丢失确认有效且没有问题，返回正常
	}
  }
  else {
	  // 确认值不合法，返回失败
  }
  return rc;
}

int J1772EVSEController::GetHearbeatInterval() {
  return (int)m_HsInterval; // 返回心跳间隔
}

int J1772EVSEController::GetHearbeatCurrent() {
  return (int)m_IFallback; // 返回心跳的电流值
}

int J1772EVSEController::GetHearbeatTrigger() {
  return (int)m_HsTriggered; // 返回心跳触发状态
}

#endif //HEARTBEAT_SUPERVISION

#if defined(GFI) || defined(ADVPWR)
// 获取重置时间（毫秒）
unsigned long J1772EVSEController::GetResetMs()
{
#ifdef GFI
  // 如果启用了GFI，返回剩余的重置时间
  return GFI_TIMEOUT - (millis() - ((m_EvseState == EVSE_STATE_GFCI_FAULT) ? m_GfiFaultStartMs : m_NoGndStart));
#else
  // 否则，返回从m_NoGndStart开始的剩余时间
  return GFI_TIMEOUT - (millis() - m_NoGndStart);
#endif // GFI
}
#endif // GFI || ADVPWR


#ifdef VOLTMETER
// 设置电压计的标度因子和偏移量
void J1772EVSEController::SetVoltmeter(uint16_t scale, uint32_t offset)
{
  m_VoltScaleFactor = scale;
  // 将电压标度因子写入EEPROM
  eeprom_write_word((uint16_t*)EOFS_VOLT_SCALE_FACTOR, scale);
  m_VoltOffset = offset;
  // 将电压偏移量写入EEPROM
  eeprom_write_dword((uint32_t*)EOFS_VOLT_OFFSET, offset);
}

// 读取电压计的电压值
uint32_t J1772EVSEController::ReadVoltmeter()
{
  unsigned int peak = 0;
  // 在一定时间内读取电压计的最大值
  for(uint32_t start_time = millis(); (millis() - start_time) < VOLTMETER_POLL_INTERVAL; ) {
    unsigned int val = adcVoltMeter.read();
    if (val > peak) peak = val; // 记录最大电压值
  }
  // 根据电压标度因子和偏移量计算实际电压
  m_Voltage = ((uint32_t)peak) * ((uint32_t)m_VoltScaleFactor) + m_VoltOffset;
  return m_Voltage; // 返回最终计算的电压值
}
#endif // VOLTMETER

#ifdef CHARGE_LIMIT
// 设置充电限制（以kWh为单位）
void J1772EVSEController::SetChargeLimitkWh(uint8_t kwh)
{
  if (kwh) {
    m_chargeLimitkWh = kwh;
    // 扩展会话时间，按kWh计算
    // m_chargeLimitTotWs = g_EnergyMeter.GetSessionWs() + (3600000ul * (uint32_t)kwh);
    // 设置会话kWh限制
    m_chargeLimitTotWs = (3600000ul * (uint32_t)kwh);
#ifdef DELAYTIMER
    g_DelayTimer.SetManualOverride(); // 设置手动覆盖延迟定时器
#endif // DELAYTIMER
    setVFlags(ECVF_CHARGE_LIMIT); // 设置充电限制标志
  }
  else {
    ClrChargeLimit(); // 清除充电限制
#ifdef DELAYTIMER
    g_DelayTimer.ClrManualOverride(); // 清除手动覆盖延迟定时器
#endif // DELAYTIMER
  }
}
#endif // CHARGE_LIMIT

#ifdef TIME_LIMIT
// 设置时间限制（以15分钟为单位）
void J1772EVSEController::SetTimeLimit15(uint8_t mind15)
{
  if (mind15) {
    m_timeLimit15 = mind15;
    // 扩展会话时间，按mind15的15分钟增量计算
    // m_timeLimitEnd = GetElapsedChargeTime() + (time_t)(15lu*60lu * (unsigned long)mind15);
    // 设置会话时间限制
    m_timeLimitEnd = (time_t)(15lu*60lu * (unsigned long)mind15);
#ifdef DELAYTIMER
    g_DelayTimer.SetManualOverride(); // 设置手动覆盖延迟定时器
#endif // DELAYTIMER
    setVFlags(ECVF_TIME_LIMIT); // 设置时间限制标志
  }
  else {
    ClrTimeLimit(); // 清除时间限制
#ifdef DELAYTIMER
    g_DelayTimer.ClrManualOverride(); // 清除手动覆盖延迟定时器
#endif // DELAYTIMER
  }
}
#endif // TIME_LIMIT

// 设置最大硬件电流容量（以安培为单位）
uint8_t J1772EVSEController::SetMaxHwCurrentCapacity(uint8_t amps)
{
  if ((amps >= MIN_CURRENT_CAPACITY_J1772) && (amps <= MAX_CURRENT_CAPACITY_L2)) {
    uint8_t eamps = eeprom_read_byte((uint8_t*)EOFS_MAX_HW_CURRENT_CAPACITY);
    if (!eamps || (eamps == (uint8_t)0xff)) {  // 如果从未写入过
      eeprom_write_byte((uint8_t*)EOFS_MAX_HW_CURRENT_CAPACITY, amps);
      m_MaxHwCurrentCapacity = amps;
      // 如果当前容量大于最大硬件电流容量，则将其调整到最大值
      if (m_CurrentCapacity > m_MaxHwCurrentCapacity) {
        SetCurrentCapacity(amps, 1, 1);
      }
      return 0; // 设置成功
    }
  }
  return 1; // 设置失败
}

//-- 结束 J1772EVSEController
