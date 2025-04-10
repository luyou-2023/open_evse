#include "open_evse.h"

#ifdef KWH_RECORDING

// 定义一个能量计量器实例
EnergyMeter g_EnergyMeter;

// 能量计量器构造函数
EnergyMeter::EnergyMeter()
{
  m_bFlags = 0;  // 初始化标志位
  m_wattSeconds = 0;  // 初始化瓦秒数

  // 检查 EEPROM 是否未初始化，如果未初始化则从 0kWh 开始
  if (eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED) == 0xffffffff) {
    // 如果 EEPROM 未初始化，设置四个字节为零，仅执行一次
    eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,0);
  }

  // 从 EEPROM 获取存储的 kWh 值
  m_wattHoursTot = eeprom_read_dword((uint32_t*)EOFS_KWH_ACCUMULATED);
}

// 更新能量计量器状态
void EnergyMeter::Update()
{
  // 1. 当 EV 连接时，充电会话开始，断开时会话结束
  // 2. 只有当继电器闭合时才会记录数据
  // 3. 只有当会话结束时，才会更新总 kWh
  uint8_t evconnected = g_EvseController.EvConnected();

  // 如果之前未连接现在连接，开始一个新会话
  if (!evConnected() && evconnected) {
    startSession();
  }
  // 如果之前连接现在断开，结束会话
  else if (!evconnected && evConnected()) {
    endSession();
  }

  if (inSession()) {  // 如果处于会话中
    uint8_t relayclosed = g_EvseController.RelayIsClosed();

    if (relayclosed) {
      if (!relayClosed()) {
        // 如果继电器刚闭合，不进行计算，只重置计时器
        m_lastUpdateMs = millis();
      }
      else {
        // 继电器闭合，进行能量使用量计算
        calcUsage();
      }
    }

    // 更新继电器状态
    if (relayclosed) setRelayClosed();
    else clrRelayClosed();
  }

  // 更新 EV 连接状态
  if (evconnected) setEvConnected();
  else clrEvConnected();
}

// 计算当前的能量使用量
void EnergyMeter::calcUsage()
{
  unsigned long curms = millis();
  unsigned long dms = curms - m_lastUpdateMs;
  if (dms > KWH_CALC_INTERVAL_MS) {  // 如果已过计算间隔
      uint32_t mv = g_EvseController.GetVoltage();  // 获取电压
      uint32_t ma = g_EvseController.GetChargingCurrent();  // 获取充电电流

      /*
       * 计算 'milliwatt-seconds' 的直观公式为：
       *     mws = (mv/1000) * (ma/1000) * dms;
       *
       * 然而，直接使用整数运算会有误差。为了避免这种情况，使用下面的复杂公式来保持精度。
       */
      uint32_t mws = (mv/16) * (ma/4) / 15625 * dms;  // 使用优化后的计算公式
#ifdef THREEPHASE
      // 对于三相电，结果需要乘以 3
      mws *= 3;
#endif // THREEPHASE
      // 将毫瓦秒转换为瓦秒并累加到计数器
      m_wattSeconds += mws / 1000;

      m_lastUpdateMs = curms;  // 更新最后一次计算时间
  }
}

// 开始充电会话
void EnergyMeter::startSession()
{
  endSession();  // 结束当前会话（如果有）
  m_wattSeconds = 0;  // 重置瓦秒数
  m_lastUpdateMs = millis();  // 记录当前时间
  setInSession();  // 设置会话状态
}

// 结束充电会话
void EnergyMeter::endSession()
{
  if (inSession()) {  // 如果当前处于会话状态
    clrInSession();  // 清除会话状态
    if (m_wattSeconds) {
      m_wattHoursTot += (m_wattSeconds / 3600UL);  // 将瓦秒转换为瓦时并累加
      SaveTotkWh();  // 保存总 kWh 到 EEPROM
    }
  }
}

// 保存总的 kWh 到 EEPROM
void EnergyMeter::SaveTotkWh()
{
  eeprom_write_dword((uint32_t*)EOFS_KWH_ACCUMULATED,m_wattHoursTot);  // 将总的 kWh 写入 EEPROM
}

#endif // KWH_RECORDING
