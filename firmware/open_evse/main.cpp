// -*- C++ -*-
/*
 * Open EVSE 固件
 *
 * 版权所有 (c) 2011-2023 Sam C. Lin
 * 版权所有 (c) 2011-2014 Chris Howell <chris1howell@msn.com>
 * 定时器代码 版权所有 (c) 2013 Kevin L <goldserve1@hotmail.com>
 * 部分代码 版权所有 (c) 2014-2015 Nick Sayer <nsayer@kfu.com>
 * 部分代码 版权所有 (c) 2015 Craig Kirkpatrick
 * 部分代码 版权所有 (c) 2015 William McBrine
 * 部分代码 版权所有 (c) 2019 Tim Kuechler BowElectric at gmail

 修订版 版本 作者  原因
 6/21/13  20b3  Scott Rubin 修复启用RTC时LCD显示的错误
 6/25/13  20b4  Scott Rubin 修复禁用RTC时的LCD显示错误，CLI修复
 6/30/13  20b5  Scott Rubin 添加LcdDetected()函数，防止LCD未安装时的挂起
 7/06/13  20b5  Scott Rubin 重写POST函数中的电源检测，支持1个或2个继电器
 7/11/13  20b5  Scott Rubin 如果EV已连接，则跳过POST；如果地线开放或继电器卡住，则无法充电
 8/12/13  20b5b Scott Rubin 修复GFI错误 - 更改gfi.Reset()以检查恒定的GFI信号
 8/26/13  20b6  Scott Rubin 添加卡住继电器状态延迟，修复卡住继电器状态退出（适用于Active E）
 9/20/13  20b7  Chris Howell 更新/调整/简化CLI消息
 10/25/14      Craig K 添加电流平滑处理
 3/1/15        Craig K 添加TEMPERATURE_MONITORING（温度监控）
 3/7/15        Craig K 添加KWH_RECORDING（千瓦时记录）
 10/28/2019    Tim Kuechler 添加HEARTBEAT_SUPERVISION（心跳监督）

 * 本文件是Open EVSE的一部分。

 * Open EVSE是自由软件；你可以在GNU通用公共许可证（版本3或（根据你的选择）任何更高版本）下重新发布和/或修改
 * 它。
 *
 * Open EVSE是根据希望它会有用的愿望发布的，
 * 但不提供任何担保；甚至不包括适销性或特定用途适用性的隐含担保。有关更多细节，请参见
 * GNU通用公共许可证。
 *
 * 你应该已经收到了GNU通用公共许可证的副本
 * 与Open EVSE一起；如果没有，请写信给
 * 美国自由软件基金会，地址：59 Temple Place - Suite 330，
 * 波士顿，MA 02111-1307，USA。
 */

#include <avr/io.h>       // AVR输入输出库
#include <avr/eeprom.h>   // AVR EEPROM操作库
#include <avr/wdt.h>      // AVR看门狗定时器库
#include <avr/pgmspace.h> // AVR程序存储器库
#include <pins_arduino.h> // Arduino引脚定义
#include "./Wire.h"       // I2C通信库
#include "./RTClib.h"     // RTC时钟库
#include "open_evse.h"    // Open EVSE相关定义头文件

// 如果使用I2CLCD_PCF8574，请取消注释下面的行，并注释掉LiquidTWI2.h
//#include "./LiquidCrystal_I2C.h"

// 温度监控部分
#ifdef TEMPERATURE_MONITORING
  #ifdef MCP9808_IS_ON_I2C
  #include "MCP9808.h"  // 添加环境温度传感器到I2C总线
  #endif
  #ifdef TMP007_IS_ON_I2C
  #include "./Adafruit_TMP007.h"   // 添加TMP007红外I2C传感器
  #endif
#endif // TEMPERATURE_MONITORING

// 按钮菜单设置
#ifdef BTN_MENU
SettingsMenu g_SettingsMenu; // 设置菜单
SetupMenu g_SetupMenu;       // 安装菜单
MaxCurrentMenu g_MaxCurrentMenu; // 最大电流菜单
#ifndef NOSETUP_MENU
SvcLevelMenu g_SvcLevelMenu; // 服务级别菜单
DiodeChkMenu g_DiodeChkMenu; // 二极管检查菜单
#ifdef RGBLCD
BklTypeMenu g_BklTypeMenu;   // 背光类型菜单
#endif // RGBLCD
#ifdef GFI_SELFTEST
GfiTestMenu g_GfiTestMenu;   // GFI自检菜单
#endif
#ifdef TEMPERATURE_MONITORING
TempOnOffMenu g_TempOnOffMenu; // 温度开关菜单
#endif // TEMPERATURE_MONITORING
VentReqMenu g_VentReqMenu;   // 通风请求菜单
#ifdef ADVPWR
GndChkMenu g_GndChkMenu;    // 地线检查菜单
RlyChkMenu g_RlyChkMenu;    // 继电器检查菜单
#endif // ADVPWR
#endif // NOSETUP_MENU
ResetMenu g_ResetMenu;      // 重置菜单
// 实例化额外的菜单 - GoldServe
#if defined(DELAYTIMER_MENU)
RTCMenu g_RTCMenu;           // RTC菜单
RTCMenuMonth g_RTCMenuMonth; // RTC月菜单
RTCMenuDay g_RTCMenuDay;     // RTC日菜单
RTCMenuYear g_RTCMenuYear;   // RTC年菜单
RTCMenuHour g_RTCMenuHour;   // RTC小时菜单
RTCMenuMinute g_RTCMenuMinute; // RTC分钟菜单
DelayMenu g_DelayMenu;       // 延时菜单
DelayMenuEnableDisable g_DelayMenuEnableDisable; // 延时启用/禁用菜单
DelayMenuStartHour g_DelayMenuStartHour; // 延时开始小时菜单
DelayMenuStopHour g_DelayMenuStopHour;   // 延时停止小时菜单
DelayMenuStartMin g_DelayMenuStartMin;   // 延时开始分钟菜单
DelayMenuStopMin g_DelayMenuStopMin;     // 延时停止分钟菜单
#endif // DELAYTIMER_MENU
#ifdef CHARGE_LIMIT
ChargeLimitMenu g_ChargeLimitMenu;  // 充电限制菜单
#endif // CHARGE_LIMIT
#ifdef TIME_LIMIT
TimeLimitMenu g_TimeLimitMenu;     // 时间限制菜单
#endif // TIME_LIMIT

// 菜单列表，存储所有设置菜单
Menu *g_SettingsMenuList[] = {
#ifdef TIME_LIMIT
  &g_TimeLimitMenu,  // 时间限制菜单
#endif // TIME_LIMIT
#ifdef CHARGE_LIMIT
  &g_ChargeLimitMenu, // 充电限制菜单
#endif // CHARGE_LIMIT
#ifdef DELAYTIMER_MENU
  &g_DelayMenu,      // 延时菜单
#endif // DELAYTIMER_MENU
  &g_SetupMenu,      // 设置菜单
  &g_ResetMenu,      // 重置菜单
  NULL
};

// 菜单列表，存储所有设置的子菜单
Menu *g_SetupMenuList[] = {
#ifdef NOSETUP_MENU
  &g_MaxCurrentMenu, // 最大电流菜单
  &g_RTCMenu,        // RTC菜单
#else // !NOSETUP_MENU
#ifdef DELAYTIMER_MENU
  &g_RTCMenu,        // RTC菜单
#endif // DELAYTIMER_MENU
#ifdef RGBLCD
  &g_BklTypeMenu,    // 背光类型菜单
#endif // RGBLCD
  &g_SvcLevelMenu,   // 服务级别菜单
  &g_DiodeChkMenu,   // 二极管检查菜单
  &g_VentReqMenu,    // 通风请求菜单
#ifdef ADVPWR
  &g_GndChkMenu,     // 地线检查菜单
  &g_RlyChkMenu,     // 继电器检查菜单
#endif // ADVPWR
#ifdef GFI_SELFTEST
  &g_GfiTestMenu,    // GFI自检菜单
#endif // GFI_SELFTEST
#ifdef TEMPERATURE_MONITORING
  &g_TempOnOffMenu,  // 温度开关菜单
#endif // TEMPERATURE_MONITORING
#endif // NOSETUP_MENU
  NULL
};

BtnHandler g_BtnHandler; // 按钮处理器
#endif // BTN_MENU

#define g_sHHMMfmt "%02d:%02d" // 时间格式定义，小时:分钟

//-- 全局变量定义开始

char g_sTmp[TMP_BUF_SIZE];  // 临时字符缓冲区

OnboardDisplay g_OBD;  // 屏幕显示管理

// 实例化RTC和延时定时器 - GoldServe
#ifdef RTC
RTC_DS1307 g_RTC;  // RTC时钟对象

#if defined(RAPI)
void SetRTC(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t mn, uint8_t s) {
  g_RTC.adjust(DateTime(y,m,d,h,mn,s));  // 设置RTC时间
}
void GetRTC(char *buf) {
  DateTime t = g_RTC.now();  // 获取当前时间
  sprintf(buf,"%d %d %d %d %d %d", t.year()-2000, t.month(), t.day(), t.hour(), t.minute(), t.second());
}
#endif // RAPI
#endif // RTC

#ifdef DELAYTIMER
DelayTimer g_DelayTimer;  // 延时定时器对象
#ifdef DELAYTIMER_MENU
// 支持RTC和延时定时器的开始变量 - GoldServe
uint16_t g_year;  // 年
uint8_t g_month;  // 月
uint8_t g_day;    // 日
uint8_t g_hour;   // 小时
uint8_t g_min;    // 分钟
#endif // DELAYTIMER_MENU
#endif // DELAYTIMER

#ifdef TEMPERATURE_MONITORING
TempMonitor g_TempMonitor;  // 温度监控对象
#endif // TEMPERATURE_MONITORING

#ifdef PP_AUTO_AMPACITY
AutoCurrentCapacityController g_ACCController;  // 自动电流容量控制器
#endif

//-- 全局变量定义结束

// 看门狗安全延迟 - 当延迟时间超过看门狗定时器时使用
// *不要* 在WDT_ENABLE()被调用之前使用这个函数
void wdt_delay(uint32_t ms)
{
  do {
    WDT_RESET();  // 重置看门狗定时器
    if (ms > WATCHDOG_TIMEOUT / 2) {
      delay(WATCHDOG_TIMEOUT / 2);  // 延迟一半的看门狗超时
      ms -= WATCHDOG_TIMEOUT / 2;
    }
    else {
      delay(ms);  // 延迟剩余时间
      ms = 0;
    }
  } while(ms);
}

// 简化I2C发送函数
static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);  // 使用Wire库发送数据
#else
  Wire.send(x);  // 旧版本Arduino的发送方式
#endif
}

// 简化I2C接收函数
static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();  // 使用Wire库接收数据
#else
  return Wire.receive();  // 旧版本Arduino的接收方式
#endif
}

// 如果数字大于0，填充零至指定的数字位
// 注意：此函数使用g_sTmp的*末尾*作为其缓冲区
char *u2a(unsigned long x, int8_t digits)
{
  char *s = g_sTmp + sizeof(g_sTmp);  // 设置指针指向缓冲区末尾

  *--s = 0;  // 结束符

  do {
    *--s = '0' + x % 10;  // 获取数字的最后一位
    x /= 10;  // 移除最后一位
    --digits;  // 减少数字位数
  } while (x || digits > 0);  // 如果还有数字或需要填充零

  return s;  // 返回填充后的字符串
}

// wdt_init 通过看门狗定时器重启后关闭看门狗定时器
void wdt_init(void) __attribute__((naked, used)) __attribute__((section(".init3")));
void wdt_init(void)
{
  MCUSR = 0;  // 清除MCU状态寄存器
  wdt_disable();  // 禁用看门狗定时器

  return;
}


#ifdef TEMPERATURE_MONITORING

#ifdef TEMPERATURE_MONITORING

void TempMonitor::Init()
{
  m_Flags = 0;
  m_MCP9808_temperature = TEMPERATURE_NOT_INSTALLED;  // 230代表23.0°C 使用整数来节省浮点库的使用
  m_DS3231_temperature = TEMPERATURE_NOT_INSTALLED;   // DS3231 RTC自带温度传感器
  m_TMP007_temperature = TEMPERATURE_NOT_INSTALLED;

#ifdef TEMPERATURE_MONITORING_NY
  LoadThresh();  // 如果有NY的条件，加载温度阈值
#endif

#ifdef MCP9808_IS_ON_I2C
  m_tempSensor.begin();  // 如果MCP9808传感器在I2C上，初始化温度传感器
#endif // MCP9808_IS_ON_I2C

#ifdef TMP007_IS_ON_I2C
  m_tmp007.begin();  // 如果TMP007传感器在I2C上，初始化TMP007传感器
#endif // TMP007_IS_ON_I2C
}

void TempMonitor::Read()
{
  unsigned long curms = millis();  // 获取当前的时间戳
  if ((curms - m_LastUpdate) >= TEMPMONITOR_UPDATE_INTERVAL) {  // 如果时间间隔足够长，更新温度值
#ifdef TMP007_IS_ON_I2C
    m_TMP007_temperature = m_tmp007.readObjTempC10();   // 使用TI TMP007红外传感器读取温度
#endif
#ifdef MCP9808_IS_ON_I2C
    m_MCP9808_temperature = m_tempSensor.readAmbient();  // 从MCP9808传感器读取温度
#endif

#ifdef RTC
#ifdef OPENEVSE_2
    m_DS3231_temperature = TEMPERATURE_NOT_INSTALLED;  // 如果是OpenEVSE II，DS3231传感器不使用
#else // !OPENEVSE_2
    // 下面的代码读取DS3231 RTC的内置温度传感器
    Wire.beginTransmission(DS1307_ADDRESS);
    wiresend(uint8_t(0x0e));  // 设置寄存器地址
    wiresend( 0x20 );          // 写入位5以启动温度转换
    Wire.endTransmission();

    Wire.beginTransmission(DS1307_ADDRESS);
    wiresend(uint8_t(0x11));  // 设置读取寄存器
    Wire.endTransmission();

    if(Wire.requestFrom(DS1307_ADDRESS, 2)) {  // 请求从DS3231读取温度数据
    m_DS3231_temperature = (((int16_t)wirerecv()) << 8) | (wirerecv()); // 读取高字节和低字节
    m_DS3231_temperature = m_DS3231_temperature >> 6;  // 去掉最低的6位
    if (m_DS3231_temperature & 0x0200) m_DS3231_temperature |= 0xFE00;  // 如果是负数，进行符号扩展
    m_DS3231_temperature = (m_DS3231_temperature * 10) / 4;  // 将温度转换为0.25°C分辨率
                                                                        // 注意：设备的符号位只影响上字节
                                                                        // 小的副作用是：-0.25°C、-0.5°C和-0.75°C的实际温度
                                                                        // 会被读取为正数，显示为+0.25°C…
                                                                        // 这是一种硬件限制，不用在软件中尝试修复
    }
    else
      m_DS3231_temperature = TEMPERATURE_NOT_INSTALLED;

#endif // OPENEVSE_2
#endif // RTC

    m_LastUpdate = curms;  // 更新最后更新时间
  }
}
#endif // TEMPERATURE_MONITORING


// OnboardDisplay 构造函数
OnboardDisplay::OnboardDisplay()
#if defined(I2CLCD) || defined(RGBLCD)
#ifdef I2CLCD_PCF8574
// 设置I2C LCD连接的引脚：
//                    地址，en,rw,rs,d4,d5,d6,d7,bl,blpol
  : m_Lcd(LCD_I2C_ADDR, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE)
#else
  : m_Lcd(LCD_I2C_ADDR,1)  // 如果没有使用PCF8574的I2C LCD，则使用默认的LCD地址
#endif // I2CLCD_PCF8574
#endif // defined(I2CLCD) || defined(RGBLCD)
{
  // 构造函数，初始化LCD显示对象
}


#if defined(DELAYTIMER)
const char CustomChar_6[8] PROGMEM = {0x0,0xe,0x15,0x17,0x11,0xe,0x0,0x0}; // 时钟图标
#endif
#ifdef DELAYTIMER
const char CustomChar_1[8] PROGMEM = {0x0,0x0,0xe,0xe,0xe,0x0,0x0,0x0}; // 停止图标（立方体）
const char CustomChar_2[8] PROGMEM = {0x0,0x8,0xc,0xe,0xc,0x8,0x0,0x0}; // 播放图标
#endif // DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
const char CustomChar_3[8] PROGMEM = {0x0,0xe,0xc,0x1f,0x3,0x6,0xc,0x8}; // 闪电图标
#endif
#ifdef AUTH_LOCK
const char CustomChar_4[8] PROGMEM = { // 锁图标
	0b00000,
	0b01110,
	0b01010,
	0b11111,
	0b11011,
	0b11011,
	0b01110,
	0b00000
};
#endif // AUTH_LOCK
#ifdef TIME_LIMIT
const char CustomChar_5[8] PROGMEM = { // 时间限制时钟图标
	0b00000,
	0b01110,
	0b10001,
	0b11101,
	0b10101,
	0b01110,
	0b00000,
	0b00000
};
#endif // TIME_LIMIT

#ifdef LCD16X2
// 创建自定义字符
void OnboardDisplay::MakeChar(uint8_t n, PGM_P bytes)
{
  memcpy_P(g_sTmp, bytes, 8);  // 从程序存储器复制自定义字符数据
  m_Lcd.createChar(n, (uint8_t*)g_sTmp);  // 创建LCD上的自定义字符
}
#endif // LCD16X2

// 初始化显示屏
void OnboardDisplay::Init()
{
  WDT_RESET();  // 重置看门狗定时器

#ifdef RGBLCD
  m_bFlags = 0;
#else
  m_bFlags = OBDF_MONO_BACKLIGHT;  // 设置为单色背光
#endif // RGBLCD

#ifdef GREEN_LED_REG
  pinGreenLed.init(GREEN_LED_REG,GREEN_LED_IDX,DigitalPin::OUT);  // 初始化绿色LED
  SetGreenLed(0);  // 设置绿色LED为关闭状态
#endif
#ifdef RED_LED_REG
  pinRedLed.init(RED_LED_REG,RED_LED_IDX,DigitalPin::OUT);  // 初始化红色LED
  SetRedLed(0);  // 设置红色LED为关闭状态
#endif

#ifdef LCD16X2
  LcdBegin(LCD_MAX_CHARS_PER_LINE, 2);  // 初始化LCD显示，设置每行字符数和行数
  LcdSetBacklightColor(WHITE);  // 设置背光颜色为白色

#if defined(DELAYTIMER)
  MakeChar(0,CustomChar_6);  // 创建时钟图标
#endif
#ifdef DELAYTIMER
  MakeChar(1,CustomChar_1);  // 创建停止图标
  MakeChar(2,CustomChar_2);  // 创建播放图标
#endif //#ifdef DELAYTIMER
#if defined(DELAYTIMER)||defined(CHARGE_LIMIT)
  MakeChar(3,CustomChar_3);  // 创建闪电图标
#endif
#ifdef AUTH_LOCK
  MakeChar(4,CustomChar_4);  // 创建锁图标
#endif
#ifdef TIME_LIMIT
  MakeChar(5,CustomChar_5);  // 创建时间限制时钟图标
#endif // TIME_LIMIT
  m_Lcd.clear();  // 清空LCD显示

#ifdef OPENEVSE_2
  LcdPrint_P(0,PSTR("Open EVSE II"));  // 显示Open EVSE II的标题
#else
  LcdPrint_P(0,PSTR("Open EVSE"));  // 显示Open EVSE的标题
#endif
  LcdPrint_P(0,1,PSTR("Ver. "));  // 显示版本号
  LcdPrint_P(VERSTR);  // 显示具体版本
  wdt_delay(1500);  // 延时1.5秒
  WDT_RESET();  // 重置看门狗定时器
#endif //#ifdef LCD16X2
}

// 在LCD屏幕上打印文本
#ifdef LCD16X2
void OnboardDisplay::LcdPrint(int x,int y,const char *s)
{
  m_Lcd.setCursor(x,y);  // 设置光标位置
  m_Lcd.print(s);  // 在LCD上打印文本
}
#endif // LCD16X2

// LcdPrint_P：将程序存储器中的字符串打印到LCD显示器
void OnboardDisplay::LcdPrint_P(PGM_P s)
{
  // 从程序存储器中复制字符串到缓冲区
  strncpy_P(m_strBuf, s, LCD_MAX_CHARS_PER_LINE);
  // 确保缓冲区末尾是字符串结束符
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  // 打印缓冲区内容到LCD显示器
  m_Lcd.print(m_strBuf);
}

// LcdPrint_P：在指定的y坐标上打印程序存储器中的字符串
void OnboardDisplay::LcdPrint_P(int y, PGM_P s)
{
  // 从程序存储器中复制字符串到缓冲区
  strncpy_P(m_strBuf, s, LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  // 调用LcdPrint方法打印到指定的y坐标位置
  LcdPrint(y, m_strBuf);
}

// LcdPrint_P：在指定的x、y坐标位置打印程序存储器中的字符串
void OnboardDisplay::LcdPrint_P(int x, int y, PGM_P s)
{
  // 从程序存储器中复制字符串到缓冲区
  strncpy_P(m_strBuf, s, LCD_MAX_CHARS_PER_LINE);
  m_strBuf[LCD_MAX_CHARS_PER_LINE] = 0;
  // 设置光标位置到指定的x、y坐标
  m_Lcd.setCursor(x, y);
  // 打印缓冲区内容到LCD显示器
  m_Lcd.print(m_strBuf);
}

// LcdMsg_P：在LCD上打印两行信息
void OnboardDisplay::LcdMsg_P(PGM_P l1, PGM_P l2)
{
  // 打印第一行信息
  LcdPrint_P(0, l1);
  // 打印第二行信息
  LcdPrint_P(1, l2);
}

// LcdPrint：在LCD上指定的y坐标打印字符串，并用空格填充剩余部分
void OnboardDisplay::LcdPrint(int y, const char *s)
{
  // 设置光标位置到第0列、第y行
  m_Lcd.setCursor(0, y);
  uint8_t i, len = strlen(s);
  // 如果字符串长度超过LCD每行最大字符数，截断长度
  if (len > LCD_MAX_CHARS_PER_LINE)
    len = LCD_MAX_CHARS_PER_LINE;
  // 打印字符串内容
  for (i = 0; i < len; i++) {
    m_Lcd.write(s[i]);
  }
  // 用空格填充剩余部分，确保整行都被填满
  for (i = len; i < LCD_MAX_CHARS_PER_LINE; i++) {
    m_Lcd.write(' ');
  }
}

// LcdMsg：打印两行信息到LCD
void OnboardDisplay::LcdMsg(const char *l1, const char *l2)
{
  // 打印第一行信息
  LcdPrint(0, l1);
  // 打印第二行信息
  LcdPrint(1, l2);
}
#endif // LCD16X2


// Update：根据更新模式更新显示内容
void OnboardDisplay::Update(int8_t updmode)
{
  // 如果更新被禁用并且控制器不在故障状态下，则不进行更新
  if (updateDisabled() && !g_EvseController.InFaultState()) return;

  uint8_t curstate = g_EvseController.GetState();
#ifdef LCD16X2
  uint8_t svclvl = g_EvseController.GetCurSvcLevel(); // 获取当前服务级别
  int currentcap = g_EvseController.GetCurrentCapacity(); // 获取当前电力容量
#endif
  unsigned long curms = millis(); // 获取当前毫秒数

  // 如果状态发生变化或者更新模式不是正常更新模式，则触发周期性更新
  if (g_EvseController.StateTransition() || (updmode != OBD_UPD_NORMAL)) {
    curms += 1000; // 增加1秒，触发周期性更新

    if (g_EvseController.InHardFault()) {
      // 如果发生硬故障，需要在硬故障外部调用时处理
      updmode = OBD_UPD_HARDFAULT; // 设置更新模式为硬故障更新
    }
  }
}

#ifdef LCD16X2
    sprintf(g_sTmp, g_sRdyLAstr, (int)svclvl, currentcap);  // 将服务等级和电流容量格式化为字符串
#endif
    switch(curstate) {
    case EVSE_STATE_A: // 未连接状态
      SetGreenLed(1);  // 点亮绿灯，表示系统就绪
      SetRedLed(0);    // 熄灭红灯，表示没有错误
#ifdef LCD16X2
      // 显示定时器和停止图标 - GoldServe
      LcdClear();      // 清屏
      LcdSetCursor(0, 0); // 设置光标位置
#ifdef AUTH_LOCK
      // 如果启用了授权锁定
      if (g_EvseController.AuthLockIsOn()) {
        LcdSetBacklightColor(TEAL);  // 设置背景灯为青色
        LcdWrite(4);  // 显示授权锁定图标
      }
      else {
        LcdSetBacklightColor(GREEN); // 否则设置为绿色
      }
#else
      LcdSetBacklightColor(GREEN);  // 默认设置背景灯为绿色
#endif // AUTH_LOCK
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();  // 显示延迟计时器图标
#endif // DELAYTIMER
      LcdPrint_P(g_psReady);  // 显示“准备好”信息
      LcdPrint(10, 0, g_sTmp);  // 显示服务等级和电流容量信息

#ifdef KWH_RECORDING
      sprintf(g_sTmp, STRF_WH, (g_EnergyMeter.GetSessionWs() / 3600));  // 显示本次会话的瓦时数
      LcdPrint(0, 1, g_sTmp);

      sprintf(g_sTmp, STRF_KWH, (g_EnergyMeter.GetTotkWh() / 1000));  // 显示累计的千瓦时数
      LcdPrint(7, 1, g_sTmp);
#endif // KWH_RECORDING

#endif //Adafruit RGB LCD
      // 注意：蓝色 LED 熄灭
      break;
    case EVSE_STATE_B: // 已连接但未充电状态
      SetGreenLed(1);  // 点亮绿灯
      SetRedLed(1);    // 点亮红灯
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdClear();      // 清屏
      LcdSetCursor(0, 0); // 设置光标位置
#ifdef AUTH_LOCK
      // 如果启用了授权锁定
      if (g_EvseController.AuthLockIsOn()) {
        LcdWrite(4);  // 显示授权锁定图标
        LcdSetBacklightColor(TEAL);  // 设置背景灯为青色
      }
      else {
        LcdSetBacklightColor(YELLOW); // 否则设置为黄色
      }
#else
      LcdSetBacklightColor(YELLOW);  // 默认设置背景灯为黄色
#endif // AUTH_LOCK
#ifdef CHARGE_LIMIT
      // 如果充电限制已设置
      if (g_EvseController.GetChargeLimitkWh()) {
        LcdWrite(3);  // 显示闪电图标，表示充电限制
      }
#endif
#ifdef TIME_LIMIT
      // 如果时间限制已设置
      if (g_EvseController.GetTimeLimit15()) {
        LcdWrite(5);  // 显示时钟图标，表示时间限制
      }
#endif
      // 显示定时器和停止图标 - GoldServe
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();  // 显示延迟计时器图标
#endif // DELAYTIMER
      LcdPrint_P(g_psEvConnected);  // 显示“已连接”信息
      LcdPrint(10, 0, g_sTmp);  // 显示服务等级和电流容量信息

#ifdef KWH_RECORDING
      sprintf(g_sTmp, STRF_WH, (g_EnergyMeter.GetSessionWs() / 3600));  // 显示本次会话的瓦时数
      LcdPrint(0, 1, g_sTmp);

      sprintf(g_sTmp, STRF_KWH, (g_EnergyMeter.GetTotkWh() / 1000));  // 显示累计的千瓦时数
      LcdPrint(7, 1, g_sTmp);
#endif // KWH_RECORDING

#endif //Adafruit RGB LCD
      // 注意：蓝色 LED 熄灭
      break;
    case EVSE_STATE_C: // 正在充电状态
      SetGreenLed(0);  // 熄灭绿灯，表示正在充电
      SetRedLed(0);    // 熄灭红灯，表示没有错误
#ifdef LCD16X2 //Adafruit RGB LCD
      LcdSetBacklightColor(TEAL);  // 设置背景灯为青色
      LcdClear();      // 清屏
      LcdSetCursor(0, 0); // 设置光标位置
      // 显示定时器和停止图标 - GoldServe
#ifdef CHARGE_LIMIT
      // 如果充电限制已设置
      if (g_EvseController.GetChargeLimitTotWs()) {
        LcdWrite(3);  // 显示闪电图标，表示充电限制
      }
#endif
#ifdef TIME_LIMIT
      // 如果时间限制已设置
      if (g_EvseController.GetTimeLimit15()) {
        LcdWrite(5);  // 显示时钟图标，表示时间限制
      }
#endif
#ifdef DELAYTIMER
      g_DelayTimer.PrintTimerIcon();  // 如果启用了延迟计时器，显示计时图标
#endif //#ifdef DELAYTIMER
      LcdPrint_P(g_psCharging);  // 显示“充电中”信息
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯亮
#ifdef AMMETER
      SetAmmeterDirty(1); // 强制更新电流表
#endif // AMMETER
      break;
    case EVSE_STATE_D: // 需要通风
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdMsg_P(g_psEvseError,g_psVentReq);  // 显示“需要通风”错误信息
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯关闭
      break;
    case EVSE_STATE_DIODE_CHK_FAILED:  // 二极管检查失败
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdMsg_P(g_psEvseError,g_psDiodeChkFailed);  // 显示“二极管检查失败”错误信息
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯关闭
      break;
    case EVSE_STATE_GFCI_FAULT:  // GFCI故障
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      if (updmode == OBD_UPD_HARDFAULT) {  // 如果是硬故障
        LcdMsg_P(g_psSvcReq,g_psGfciFault);  // 显示“服务请求 - GFCI故障”信息
      }
      else {
        // 如果是软故障，更新第二行并显示自动重试计数
        LcdPrint_P(0,g_psGfciFault);  // 显示“GFCI故障”信息
      }
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯关闭
      break;
#ifdef TEMPERATURE_MONITORING
    case EVSE_STATE_OVER_TEMPERATURE:    // 温度过高，显示红色信息
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdMsg_P(g_psSvcReq,g_psTemperatureFault);  // 显示“服务请求 - 温度故障”信息
#endif
      break;
#endif //TEMPERATURE_MONITORING
#ifdef OVERCURRENT_THRESHOLD
    case EVSE_STATE_OVER_CURRENT:  // 超过电流阈值
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdPrint_P(0,g_psSvcReq);  // 显示“服务请求”信息
      strcpy_P(g_sTmp,g_psOverCurrent);  // 设置电流过载提示信息
      sprintf(g_sTmp+strlen(g_sTmp)," %dA",(int)(g_EvseController.GetChargingCurrent()/1000-g_EvseController.GetCurrentCapacity()));
      LcdPrint(1,g_sTmp);  // 显示电流超载的信息
#endif
      break;
#endif // OVERCURRENT_THRESHOLD
    case EVSE_STATE_NO_GROUND:  // 无地线
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      if (updmode == OBD_UPD_HARDFAULT) {  // 如果是硬故障
        LcdMsg_P(g_EvseController.CGMIisEnabled() ? g_psSvcReq : g_psEvseError,g_psNoGround);  // 显示“无地线”信息
      }
      else {
        // 如果是软故障，更新第二行并显示自动重试计数
        LcdPrint_P(0,g_psNoGround);  // 显示“无地线”信息
      }
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯关闭
      break;
    case EVSE_STATE_STUCK_RELAY:  // 继电器卡住
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdMsg_P(updmode == OBD_UPD_HARDFAULT ? g_psSvcReq : g_psEvseError,g_psStuckRelay);  // 显示“继电器卡住”错误信息
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯关闭
      break;
    case EVSE_STATE_RELAY_CLOSURE_FAULT:  // 继电器闭合故障
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2 // 使用Adafruit RGB LCD屏
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdMsg_P(g_psSvcReq,g_psRelayClosureFault);  // 显示“继电器闭合故障”信息
#endif //Adafruit RGB LCD
      // 注：蓝色LED灯关闭
      break;
    case EVSE_STATE_DISABLED:  // 禁用状态
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2
      LcdSetBacklightColor(VIOLET);  // 设置背景灯为紫色
      LcdClear();  // 清屏
      LcdSetCursor(0,0);  // 设置光标位置
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {  // 如果启用了认证锁
        LcdWrite(4);  // 显示锁定图标
      }
#endif // AUTH_LOCK
      LcdPrint_P(g_psDisabled);  // 显示“禁用状态”信息
      LcdPrint(10,0,g_sTmp);  // 显示附加信息
#endif // LCD16X2
      break;
#ifdef GFI_SELFTEST
    case EVSE_STATE_GFI_TEST_FAILED:  // GFI自检失败
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
      LcdMsg_P(g_psTestFailed,g_psGfci);  // 显示“测试失败 - GFCI”信息
#endif
      break;
#endif // GFI_SELFTEST
    case EVSE_STATE_SLEEPING:  // 休眠状态
      SetGreenLed(1);  // 开启绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2
      LcdSetBacklightColor(g_EvseController.EvConnected() ? WHITE : VIOLET);  // 如果已连接，设置为白色，否则设置为紫色
      LcdClear();  // 清屏
      LcdSetCursor(0,0);  // 设置光标位置
#ifdef AUTH_LOCK
      if (g_EvseController.AuthLockIsOn()) {  // 如果启用了认证锁
        LcdWrite(4);  // 显示锁定图标
      }
#endif // AUTH_LOCK
      LcdPrint_P(g_psSleeping);  // 显示“休眠中”信息
      LcdPrint(10,0,g_sTmp);  // 显示附加信息
#endif // LCD16X2
      break;
    default:  // 默认状态
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
      // 注：蓝色LED灯关闭
    }
#ifdef TEMPERATURE_MONITORING
    if ((g_TempMonitor.OverTemperature() || g_TempMonitor.OverTemperatureLogged()) && !g_EvseController.InHardFault()) {  // 如果过温且没有硬故障
#ifdef LCD16X2
      LcdSetBacklightColor(RED);  // 设置背景灯为红色
#endif
      SetGreenLed(0);  // 关闭绿色LED灯
      SetRedLed(1);    // 开启红色LED灯
#ifdef LCD16X2
      LcdPrint_P(0,g_psHighTemp);  // 显示“温度过高”信息
#endif
    }
#endif // TEMPERATURE_MONITORING

// 将任何需要定期更新的内容放在这里
// 以下代码每秒只运行一次
//
if (((curms - m_LastUpdateMs) >= 1000) || (updmode == OBD_UPD_FORCE)) {
  m_LastUpdateMs = curms;  // 更新最后一次更新时间戳

#ifdef GFI
  // 如果没有硬故障，且当前状态是GFCI故障或没有接地
  if (!g_EvseController.InHardFault() &&
((curstate == EVSE_STATE_GFCI_FAULT) || (curstate == EVSE_STATE_NO_GROUND))) {
#ifdef LCD16X2
    strcpy(g_sTmp, g_sRetryIn);  // 设置显示文本
    int resetsec = (int)(g_EvseController.GetResetMs() / 1000ul);  // 获取重置时间（秒）
    if (resetsec >= 0) {
      // 格式化并显示重置倒计时
      sprintf(g_sTmp + sizeof(g_sTmp) - 6, g_sHHMMfmt, resetsec / 60, resetsec % 60);
      strcat(g_sTmp, g_sTmp + sizeof(g_sTmp) - 6);  // 拼接时间文本
      LcdPrint(1, g_sTmp);  // 在LCD上打印
    }
#endif // LCD16X2
    return;  // 如果发生GFCI故障或没有接地，退出当前函数
  }
#endif // GFI

#ifdef RTC
  DateTime currentTime = g_RTC.now();  // 获取当前RTC时间
#endif

#ifdef LCD16X2
#if defined(AMMETER)
  // 如果当前状态是充电状态，且电表显示需要更新
  if (((curstate == EVSE_STATE_C)
#ifdef ECVF_AMMETER_CAL
       || g_EvseController.AmmeterCalEnabled()
#endif
       ) && AmmeterIsDirty()) {
    SetAmmeterDirty(0);  // 重置电表脏标志

    uint32_t current = g_EvseController.GetChargingCurrent();  // 获取当前充电电流

#if defined(PP_AUTO_AMPACITY)
    int a = current / 1000;  // 获取安培数（千安培）
    int ma = (current % 1000) / 100;  // 获取毫安数
    if (ma >= 5) {
      a++;  // 如果毫安大于等于5，向上取整
    }
    sprintf(g_sTmp, "%d:%dA", a, g_EvseController.GetCurrentCapacity());  // 格式化电流容量字符串

    LcdPrint(9, 0, "       ");
    LcdPrint(LCD_MAX_CHARS_PER_LINE - strlen(g_sTmp), 0, g_sTmp);  // 在LCD上显示电流信息
#else //!PP_AUTO_AMPACITY
    if (current >= 1000) {  // 如果电流大于1000，显示电流值
      int a = current / 1000;
      int ma = (current % 1000) / 100;
      if (ma > 9) {
        ma = 0;  // 如果毫安大于9，清零并进位
        a++;
      }
      sprintf(g_sTmp, "%3d.%dA", a, ma);  // 格式化电流显示
    }
    else {
      strcpy_P(g_sTmp, PSTR("    0A"));  // 如果电流小于1000，显示"0A"
    }
    LcdPrint(10, 0, g_sTmp);  // 在LCD上打印电流
#endif // PP_AUTO_AMPACITY
  }
#endif // AMMETER

  if (curstate == EVSE_STATE_C) {
#ifndef KWH_RECORDING
    time_t elapsedTime = g_EvseController.GetElapsedChargeTime();  // 获取已充电时间
#endif

#ifdef KWH_RECORDING
    // 格式化并显示当前电能（Wh）
    sprintf(g_sTmp, STRF_WH, (g_EnergyMeter.GetSessionWs() / 3600));
    LcdPrint(0, 1, g_sTmp);

#ifdef VOLTMETER
    // 格式化并显示电压
    sprintf(g_sTmp, STRF_VOLT, (g_EvseController.GetVoltage() / 1000));
    LcdPrint(11, 1, g_sTmp);
#else
    // 格式化并显示累计的电能（kWh）
    sprintf(g_sTmp, STRF_KWH, (g_EnergyMeter.GetTotkWh() / 1000));
    LcdPrint(7, 1, g_sTmp);
#endif // VOLTMETER
#endif // KWH_RECORDING

#ifdef TEMPERATURE_MONITORING
    // 如果温度过高或需要总是显示温度
    if ((g_TempMonitor.OverTemperature()) || TEMPERATURE_DISPLAY_ALWAYS)  {
      g_OBD.LcdClearLine(1);  // 清空LCD第1行
      const char *tempfmt = "%2d.%1dC";  // 温度显示格式
#ifdef MCP9808_IS_ON_I2C
      if (g_TempMonitor.m_MCP9808_temperature != TEMPERATURE_NOT_INSTALLED) {
        // 如果MCP9808温度传感器安装，显示温度
        sprintf(g_sTmp, tempfmt, g_TempMonitor.m_MCP9808_temperature / 10, abs(g_TempMonitor.m_MCP9808_temperature % 10));
        LcdPrint(0, 1, g_sTmp);
      }
#endif

#ifdef RTC
      if (g_TempMonitor.m_DS3231_temperature != TEMPERATURE_NOT_INSTALLED) {
        // 如果DS3231传感器安装，显示温度
        sprintf(g_sTmp, tempfmt, g_TempMonitor.m_DS3231_temperature / 10, abs(g_TempMonitor.m_DS3231_temperature % 10));
        LcdPrint(5, 1, g_sTmp);
      }
#endif

#ifdef TMP007_IS_ON_I2C
      if (g_TempMonitor.m_TMP007_temperature != TEMPERATURE_NOT_INSTALLED) {
        // 如果TMP007传感器安装，显示温度
        sprintf(g_sTmp, tempfmt, g_TempMonitor.m_TMP007_temperature / 10, abs(g_TempMonitor.m_TMP007_temperature % 10));
        LcdPrint(11, 1, g_sTmp);
      }
#endif

      // 如果温度超过阈值并且启用了过热关机，开始闪烁红色LED
      if (g_TempMonitor.BlinkAlarm() && g_TempMonitor.OverTemperatureShutdown()) {
        g_TempMonitor.SetBlinkAlarm(0);  // 关闭闪烁标志
        SetRedLed(1);  // 启动红色LED
#ifdef LCD16X2
        LcdSetBacklightColor(RED);  // 设置LCD背景色为红色
#endif
      }
      else if (g_TempMonitor.BlinkAlarm() == 0) {  // 如果闪烁停止
        g_TempMonitor.SetBlinkAlarm(1);  // 重新启用闪烁
        SetRedLed(0);  // 关闭红色LED
#ifdef LCD16X2
        LcdSetBacklightColor(TEAL);  // 设置LCD背景色为青色
#endif
      }
    }
    else if (g_TempMonitor.BlinkAlarm() == 0) {
      // 如果闪烁停止，恢复正常显示
      g_TempMonitor.SetBlinkAlarm(1);
      SetRedLed(0);
#ifdef LCD16X2
      LcdSetBacklightColor(TEAL);  // 设置LCD背景色为青色
#endif
    }
#endif // TEMPERATURE_MONITORING
#ifndef KWH_RECORDING
    // 格式化并显示已充电时间（时:分:秒）
    int h = elapsedTime / 3600;
    int m = (elapsedTime % 3600) / 60;
    int s = elapsedTime % 60;
    sprintf(g_sTmp, "%02d:%02d:%02d", h, m, s);
#ifdef RTC
    g_sTmp[8] = ' ';
    g_sTmp[9] = ' ';
    g_sTmp[10] = ' ';
    sprintf(g_sTmp + 11, g_sHHMMfmt, currentTime.hour(), currentTime.minute());
#endif
    LcdPrint(1, g_sTmp);  // 在LCD上显示充电时间
#endif // KWH_RECORDING
#ifdef TEMPERATURE_MONITORING
    }
#endif // TEMPERATURE_MONITORING
  } // curstate == EVSE_STATE_C

  // 如果进入睡眠状态，显示延时计时器
#ifdef DELAYTIMER
  else if (curstate == EVSE_STATE_SLEEPING) {
    LcdSetCursor(0, 0);
    g_DelayTimer.PrintTimerIcon();  // 显示计时器图标
#ifdef AUTH_LOCK
    if (g_EvseController.AuthLockIsOn()) {
      LcdWrite(4);
    }
#endif // AUTH_LOCK
    LcdPrint_P(g_psSleeping);  // 显示"睡眠"信息
    sprintf(g_sTmp, "%02d:%02d:%02d", currentTime.hour(), currentTime.minute(), currentTime.second());
    LcdPrint(0, 1, g_sTmp);  // 显示当前时间
    if (g_DelayTimer.IsTimerEnabled()) {
      // 如果计时器启用，显示延时器的开始和结束时间
      LcdSetCursor(9, 0);
      LcdWrite(2);
      LcdWrite(6);
      sprintf(g_sTmp, g_sHHMMfmt, g_DelayTimer.GetStartTimerHour(), g_DelayTimer.GetStartTimerMin());
      LcdPrint(11, 0, g_sTmp);
      LcdSetCursor(9, 1);
      LcdWrite(1);
      LcdWrite(6);
      sprintf(g_sTmp, g_sHHMMfmt, g_DelayTimer.GetStopTimerHour(), g_DelayTimer.GetStopTimerMin());
      LcdPrint(11, 1, g_sTmp);
    } else {
      // 如果计时器未启用，显示准备信息
      sprintf(g_sTmp, g_sRdyLAstr, (int)svclvl, currentcap);
      LcdPrint(10, 0, g_sTmp);
    }
  }
#endif // DELAYTIMER
#endif // LCD16X2
}

#ifdef FT_ENDURANCE
LcdSetCursor(0, 0);
// 显示循环计数和ACPins状态
sprintf(g_sTmp, "%d %d", g_CycleCnt, (int)g_EvseController.ReadACPins());
LcdPrint(g_sTmp);
#endif // FT_ENDURANCE
}





#ifdef BTN_MENU
// 构造函数，初始化按钮状态和去抖动时间
Btn::Btn()
{
  buttonState = BTN_STATE_OFF; // 按钮初始状态为关闭
  lastDebounceTime = 0;        // 去抖动时间初始化为0
  vlongDebounceTime = 0;       // 超长去抖动时间初始化为0
}

// 初始化按钮
void Btn::init()
{
#ifdef BTN_REG
  // 设置按钮的引脚，使用内部上拉电阻
  pinBtn.init(BTN_REG,BTN_IDX,DigitalPin::INP_PU);
#endif
}

// 读取按钮状态
void Btn::read()
{
  uint8_t sample;              // 按钮样本数据
  unsigned long delta;         // 按钮按下的时间差

#ifdef ADAFRUIT_BTN
  // 如果按钮被按下，返回1；否则返回0
  sample = (g_OBD.readButtons() & BUTTON_SELECT) ? 1 : 0;
#else //!ADAFRUIT_BTN
  // 读取按钮引脚状态，按下为0，松开为1
  sample = pinBtn.read() ? 0 : 1;
#endif // ADAFRUIT_BTN

  // 如果按钮被按下且当前状态为长按状态，且去抖动时间未到，则切换按钮状态为关闭
  if (!sample && (buttonState == BTN_STATE_LONG) && !lastDebounceTime) {
    buttonState = BTN_STATE_OFF;
  }

  // 如果按钮状态是关闭或短按且去抖动时间未到
  if ((buttonState == BTN_STATE_OFF) ||
      ((buttonState == BTN_STATE_SHORT) && lastDebounceTime)) {
    if (sample) {  // 如果按钮被按下
      if (!lastDebounceTime && (buttonState == BTN_STATE_OFF)) {
        lastDebounceTime = millis();  // 记录按下时间
      }
      delta = millis() - lastDebounceTime; // 计算按下的时间差

      if (buttonState == BTN_STATE_OFF) { // 如果当前状态是关闭
        if (delta >= BTN_PRESS_SHORT) {   // 如果按下时间大于短按时间阈值
          buttonState = BTN_STATE_SHORT;  // 设置为短按状态
        }
      }
      else if (buttonState == BTN_STATE_SHORT) { // 如果当前状态是短按
        if (delta >= BTN_PRESS_LONG) {   // 如果按下时间大于长按时间阈值
          buttonState = BTN_STATE_LONG;  // 设置为长按状态
        }
      }
    }
    else { //!sample
      lastDebounceTime = 0;  // 如果按钮松开，重置去抖动时间
    }
  }

#ifdef RAPI_WF
  // 如果按钮长按且超长去抖动时间已到，执行WiFi模式设置
  else if (sample && vlongDebounceTime && (buttonState == BTN_STATE_LONG)) {
    if ((millis() - vlongDebounceTime) >= BTN_PRESS_VERYLONG) {
      vlongDebounceTime = 0;  // 重置超长去抖动时间
      RapiSetWifiMode(WIFI_MODE_AP_DEFAULT); // 设置WiFi模式为AP模式
    }
  }
#endif // RAPI_WF
}

// 检查是否为短按
uint8_t Btn::shortPress()
{
  if ((buttonState == BTN_STATE_SHORT) && !lastDebounceTime) { // 如果是短按且没有去抖动时间
    buttonState = BTN_STATE_OFF;  // 设置为关闭状态
    return 1; // 返回短按
  }
  else {
    return 0; // 否则返回0
  }
}

// 检查是否为长按
uint8_t Btn::longPress()
{
  if ((buttonState == BTN_STATE_LONG) && lastDebounceTime) {  // 如果是长按且去抖动时间已到
    vlongDebounceTime = lastDebounceTime;  // 保存当前的去抖动时间
    lastDebounceTime = 0;  // 重置去抖动时间
    return 1;  // 返回长按
  }
  else {
    return 0;  // 否则返回0
  }
}

// 菜单类构造函数
Menu::Menu()
{
}

// 初始化菜单
void Menu::init(const char *firstitem)
{
  m_CurIdx = 0;  // 初始化菜单索引
  g_OBD.LcdPrint_P(0, m_Title);  // 显示菜单标题
  g_OBD.LcdPrint(1, firstitem);  // 显示第一个菜单项
}

// 设置菜单构造函数
SettingsMenu::SettingsMenu()
{
  m_Title = g_psSettings;  // 设置菜单标题

  m_menuCnt = 0;
  while (g_SettingsMenuList[m_menuCnt]) { // 计算菜单项的数量
    m_menuCnt++;
  }
}

// 检查是否跳过限流设置
#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
void SettingsMenu::CheckSkipLimits()
{
  // 只有在汽车连接且没有错误时，才允许显示充电限制菜单项
  m_skipLimits = !g_EvseController.LimitsAllowed();
}
#endif // CHARGE_LIMIT

// 初始化设置菜单
void SettingsMenu::Init()
{
  m_CurIdx = 0;  // 初始化菜单索引

#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
  while (m_skipLimits && (
#if defined(CHARGE_LIMIT)
    (g_SettingsMenuList[m_CurIdx] == &g_ChargeLimitMenu) ||
#endif
#if defined(TIME_LIMIT)
    (g_SettingsMenuList[m_CurIdx] == &g_TimeLimitMenu)
#else
    0
#endif
  )) {
    m_CurIdx++;  // 跳过限流设置菜单项
  }
#endif // CHARGE_LIMIT || TIME_LIMIT

  g_OBD.LcdPrint_P(0, m_Title);  // 显示标题
  g_OBD.LcdPrint_P(1, g_SettingsMenuList[m_CurIdx]->m_Title);  // 显示当前菜单项标题
}

// 转到下一个菜单项
void SettingsMenu::Next()
{
  if (++m_CurIdx > m_menuCnt) {
    m_CurIdx = 0;  // 如果超出菜单项数量，则返回第一个菜单项
  }

#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
  while (m_skipLimits && (
#if defined(CHARGE_LIMIT)
    (g_SettingsMenuList[m_CurIdx] == &g_ChargeLimitMenu) ||
#endif
#if defined(TIME_LIMIT)
    (g_SettingsMenuList[m_CurIdx] == &g_TimeLimitMenu)
#else
    0
#endif
  )) {
    m_CurIdx++;  // 跳过限流设置菜单项
  }
#endif // CHARGE_LIMIT || TIME_LIMIT

  PGM_P title;
  if (m_CurIdx < m_menuCnt) {
    title = g_SettingsMenuList[m_CurIdx]->m_Title;  // 获取当前菜单项标题
  }
  else {
    title = g_psExit;  // 如果是最后一个菜单项，则显示退出
  }

  g_OBD.LcdPrint_P(1, title);  // 显示菜单标题
}

// 选择当前菜单项
Menu *SettingsMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {
    return g_SettingsMenuList[m_CurIdx];  // 返回当前菜单项
  }
  else {
    g_OBD.LcdClear();  // 清空屏幕
    return NULL;  // 返回空
  }
}

// SetupMenu 类构造函数，初始化菜单标题和菜单项计数
SetupMenu::SetupMenu()
{
  m_Title = g_psSetup;  // 设置菜单标题为“设置”

  m_menuCnt = 0;  // 初始化菜单项计数
  while (g_SetupMenuList[m_menuCnt]) {  // 统计菜单项数量，直到遇到空指针为止
    m_menuCnt++;
  }
}

// 初始化 SetupMenu 菜单，显示菜单标题和第一个菜单项
void SetupMenu::Init()
{
  m_CurIdx = 0;  // 当前索引从 0 开始
  g_OBD.LcdPrint_P(0,m_Title);  // 在 LCD 屏幕上显示菜单标题
  g_OBD.LcdPrint_P(1,g_SetupMenuList[0]->m_Title);  // 显示第一个菜单项
}

// 切换到下一个菜单项，如果超过了菜单项数目，则循环回到第一个菜单项
void SetupMenu::Next()
{
  if (++m_CurIdx > m_menuCnt) {  // 如果当前索引大于菜单项数目
    m_CurIdx = 0;  // 则回到第一个菜单项
  }

  PGM_P title;
  if (m_CurIdx < m_menuCnt) {  // 如果当前索引在有效范围内
    title = g_SetupMenuList[m_CurIdx]->m_Title;  // 获取当前菜单项的标题
  }
  else {  // 如果当前索引超出菜单项数目
    title = g_psExit;  // 设置标题为退出
  }

  g_OBD.LcdPrint_P(1,title);  // 在 LCD 上显示当前菜单项的标题
}

// 选择当前菜单项
Menu *SetupMenu::Select()
{
  if (m_CurIdx < m_menuCnt) {  // 如果当前索引在有效范围内
    return g_SetupMenuList[m_CurIdx];  // 返回当前菜单项
  }
  else {  // 如果当前索引超出菜单项数目
    m_CurIdx = 0;  // 重置索引为第一个菜单项
    g_EvseController.Reboot();  // 执行重启操作
    return &g_SetupMenu;  // 返回设置菜单
  }
}

#ifndef NOSETUP_MENU  // 如果未定义 NOSETUP_MENU，则编译以下代码

#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
#define SVC_LVL_MNU_ITEMCNT 3  // 如果定义了 ADVPWR 和 AUTOSVCLEVEL，则菜单项计数为 3
#else
#define SVC_LVL_MNU_ITEMCNT 2  // 否则菜单项计数为 2
#endif // ADVPWR && AUTOSVCLEVEL

// 服务等级菜单项定义
const char *g_SvcLevelMenuItems[] = {
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
  STR_AUTO,  // 如果定义了 ADVPWR 和 AUTOSVCLEVEL，添加自动服务等级项
#endif // ADVPWR && AUTOSVCLEVEL
  STR_LEVEL_1,  // 服务等级 1
  STR_LEVEL_2   // 服务等级 2
};

// RGBLCD 显示屏相关菜单项
#ifdef RGBLCD
// 背光类型菜单类
BklTypeMenu::BklTypeMenu()
{
  m_Title = g_psBklType;  // 设置菜单标题为“背光类型”
}

// 初始化背光类型菜单
void BklTypeMenu::Init()
{
  g_OBD.LcdPrint_P(0,m_Title);  // 在 LCD 上显示背光类型菜单的标题
  m_CurIdx = (g_BtnHandler.GetSavedLcdMode() == BKL_TYPE_RGB) ? 0 : 1;  // 根据保存的背光类型设置当前索引
  sprintf(g_sTmp,"+%s",g_BklMenuItems[m_CurIdx]);  // 格式化字符串显示背光类型项
  g_OBD.LcdPrint(1,g_sTmp);  // 在 LCD 上显示当前背光类型项
}

// 切换到下一个背光类型
void BklTypeMenu::Next()
{
  if (++m_CurIdx >= 2) {  // 如果当前索引超过 1，回到 0
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);  // 清除 LCD 第一行
  g_OBD.LcdSetCursor(0,1);  // 设置光标到第二行
  g_OBD.LcdPrint(g_sPlus);  // 打印加号符号
  g_OBD.LcdPrint(g_BklMenuItems[m_CurIdx]);  // 显示当前背光类型
}

// 选择当前背光类型，并设置 LCD 背光模式
Menu *BklTypeMenu::Select()
{
  g_OBD.LcdPrint(0,1,g_sPlus);  // 打印加号符号
  g_OBD.LcdPrint(g_BklMenuItems[m_CurIdx]);  // 显示当前背光类型

  // 设置背光类型
  g_EvseController.SetBacklightType((m_CurIdx == 0) ? BKL_TYPE_RGB : BKL_TYPE_MONO);
  g_BtnHandler.SetSavedLcdMode((m_CurIdx == 0) ? BKL_TYPE_RGB : BKL_TYPE_MONO);

  return &g_SetupMenu;  // 返回设置菜单
}
#endif // RGBLCD


// 服务级别菜单
SvcLevelMenu::SvcLevelMenu()
{
  // 设置菜单标题为服务级别
  m_Title = g_psSvcLevel;
}

void SvcLevelMenu::Init()
{
  // 在LCD上打印服务级别菜单标题
  g_OBD.LcdPrint_P(0, m_Title);

#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
  // 如果启用了自动服务级别，根据自动服务级别或当前服务级别设置菜单选项
  if (g_EvseController.AutoSvcLevelEnabled()) {
    m_CurIdx = 0;  // 自动服务级别启用时，默认选择第一个菜单项
  } else {
    m_CurIdx = g_EvseController.GetCurSvcLevel();  // 否则，根据当前服务级别选择菜单项
  }
#else
  // 否则根据当前的服务级别选择
  m_CurIdx = (g_EvseController.GetCurSvcLevel() == 1) ? 0 : 1;
#endif // ADVPWR && AUTOSVCLEVEL

  // 清空第二行LCD，显示当前选择的服务级别
  g_OBD.LcdClearLine(1);
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示选择标记
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);  // 显示当前服务级别菜单项
}

void SvcLevelMenu::Next()
{
  // 选择下一个服务级别菜单项，循环
  if (++m_CurIdx >= SVC_LVL_MNU_ITEMCNT) {
    m_CurIdx = 0;  // 如果超出范围，回到第一个菜单项
  }

  // 清空第二行LCD，重新显示当前选项
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0, 1);

#ifdef AUTOSVCLEVEL
  // 如果启用了自动服务级别，则显示“+”标记
  if ((g_EvseController.AutoSvcLevelEnabled() && !m_CurIdx) ||
      (!g_EvseController.AutoSvcLevelEnabled() && (g_EvseController.GetCurSvcLevel() == m_CurIdx))) {
    g_OBD.LcdPrint(g_sPlus);  // 显示选择标记
  }
#else
  // 否则，如果当前服务级别与菜单项匹配，显示“+”
  if (g_EvseController.GetCurSvcLevel() == (m_CurIdx + 1)) {
    g_OBD.LcdPrint(g_sPlus);  // 显示选择标记
  }
#endif //AUTOSVCLEVEL
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);  // 显示当前菜单项
}

Menu *SvcLevelMenu::Select()
{
#if defined(ADVPWR) && defined(AUTOSVCLEVEL)
  // 根据当前菜单选项设置服务级别或启用自动服务级别
  if (m_CurIdx == 0) {
    g_EvseController.EnableAutoSvcLevel(1);  // 启用自动服务级别
  }
  else {
    g_EvseController.SetSvcLevel(m_CurIdx);  // 设置为指定服务级别
    g_EvseController.EnableAutoSvcLevel(0);  // 禁用自动服务级别
  }
#else
  g_EvseController.SetSvcLevel(m_CurIdx + 1);  // 设置当前服务级别
#endif // ADVPWR && AUTOSVCLEVEL

  // 在LCD上显示选择的服务级别
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示选择标记
  g_OBD.LcdPrint(g_SvcLevelMenuItems[m_CurIdx]);  // 显示选中的服务级别项

  // 保存当前的EVSE设置
  g_EvseController.SaveEvseFlags();

  delay(500);  // 延时，确保设置生效

  return &g_SetupMenu;  // 返回到设置菜单
}

// 二极管检测菜单
DiodeChkMenu::DiodeChkMenu()
{
  // 设置菜单标题为二极管检测
  m_Title = g_psDiodeCheck;
}

void DiodeChkMenu::Init()
{
  // 在LCD上打印二极管检测菜单标题
  g_OBD.LcdPrint_P(0, m_Title);

  // 根据是否启用二极管检测，设置菜单项
  m_CurIdx = g_EvseController.DiodeCheckEnabled() ? 0 : 1;
  sprintf(g_sTmp, "+%s", g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1, g_sTmp);  // 显示当前选择的菜单项
}

void DiodeChkMenu::Next()
{
  // 循环切换二极管检测启用与禁用
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;  // 超过范围后回到第一个菜单项
  }

  // 清空第二行LCD，显示当前选项
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0, 1);

  uint8_t dce = g_EvseController.DiodeCheckEnabled();
  // 如果当前选择与实际状态一致，则显示选择标记
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);  // 显示选择标记
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前菜单项
}

Menu *DiodeChkMenu::Select()
{
  // 在LCD上显示选择标记
  g_OBD.LcdPrint(0, 1, g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  // 启用或禁用二极管检测
  g_EvseController.EnableDiodeCheck((m_CurIdx == 0) ? 1 : 0);

  delay(500);  // 延时，确保设置生效

  return &g_SetupMenu;  // 返回到设置菜单
}

// GFI自检菜单
#ifdef GFI_SELFTEST
GfiTestMenu::GfiTestMenu()
{
  // 设置菜单标题为GFI自检
  m_Title = g_psGfiTest;
}

void GfiTestMenu::Init()
{
  // 在LCD上打印GFI自检菜单标题
  g_OBD.LcdPrint_P(0, m_Title);

  // 根据是否启用GFI自检，设置菜单项
  m_CurIdx = g_EvseController.GfiSelfTestEnabled() ? 0 : 1;
  sprintf(g_sTmp, "+%s", g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1, g_sTmp);  // 显示当前选择的菜单项
}

void GfiTestMenu::Next()
{
  // 循环切换GFI自检启用与禁用
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;  // 超过范围后回到第一个菜单项
  }

  // 清空第二行LCD，显示当前选项
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0, 1);

  uint8_t dce = g_EvseController.GfiSelfTestEnabled();
  // 如果当前选择与实际状态一致，则显示选择标记
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);  // 显示选择标记
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前菜单项
}

Menu *GfiTestMenu::Select()
{
  // 在LCD上显示选择标记
  g_OBD.LcdPrint(0, 1, g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  // 启用或禁用GFI自检
  g_EvseController.EnableGfiSelfTest((m_CurIdx == 0) ? 1 : 0);

  delay(500);  // 延时，确保设置生效

  return &g_SetupMenu;  // 返回到设置菜单
}
#endif // GFI_SELFTEST

// 温度监测菜单
#ifdef TEMPERATURE_MONITORING
TempOnOffMenu::TempOnOffMenu()
{
  // 设置菜单标题为温度检测
  m_Title = g_psTempChk;
}

void TempOnOffMenu::Init()
{
  // 在LCD上打印温度检测菜单标题
  g_OBD.LcdPrint_P(0, m_Title);

  // 根据是否启用温度检测，设置菜单项
  m_CurIdx = g_EvseController.TempChkEnabled() ? 0 : 1;
  sprintf(g_sTmp, "+%s", g_YesNoMenuItems[m_CurIdx]);
  g_OBD.LcdPrint(1, g_sTmp);  // 显示当前选择的菜单项
}

void TempOnOffMenu::Next()
{
  // 循环切换温度检测启用与禁用
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;  // 超过范围后回到第一个菜单项
  }

  // 清空第二行LCD，显示当前选项
  g_OBD.LcdClearLine(1);
  g_OBD.LcdSetCursor(0, 1);

  uint8_t dce = g_EvseController.TempChkEnabled();
  // 如果当前选择与实际状态一致，则显示选择标记
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {
    g_OBD.LcdPrint(g_sPlus);  // 显示选择标记
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前菜单项
}

Menu *TempOnOffMenu::Select()
{
  // 在LCD上显示选择标记
  g_OBD.LcdPrint(0, 1, g_sPlus);
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);

  // 启用或禁用温度检测
  g_EvseController.EnableTempChk((m_CurIdx == 0) ? 1 : 0);

  delay(500);  // 延时，确保设置生效

  return &g_SetupMenu;  // 返回到设置菜单
}
#endif // TEMPERATURE_MONITORING

// VentReqMenu类的构造函数
VentReqMenu::VentReqMenu()
{
  m_Title = g_psVentReqChk;  // 初始化菜单标题为"VentReqChk"
}

// 初始化菜单，显示标题和当前设置
void VentReqMenu::Init()
{
  g_OBD.LcdPrint_P(0, m_Title);  // 在LCD显示屏上打印标题
  m_CurIdx = g_EvseController.VentReqEnabled() ? 0 : 1;  // 根据是否启用VentReq设置当前选项索引
  sprintf(g_sTmp, "+%s", g_YesNoMenuItems[m_CurIdx]);  // 格式化当前选项
  g_OBD.LcdPrint(1, g_sTmp);  // 在LCD第二行显示当前选项
}

// 切换到下一个选项
void VentReqMenu::Next()
{
  if (++m_CurIdx >= 2) {  // 如果当前选项索引超过1，重置为0
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);  // 清除LCD第二行
  g_OBD.LcdSetCursor(0, 1);  // 设置光标到LCD第二行的开始位置
  uint8_t dce = g_EvseController.VentReqEnabled();  // 获取当前VentReq的启用状态
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {  // 根据当前设置决定是否显示“+”标志
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项
}

// 选择当前选项并返回下一个菜单
Menu *VentReqMenu::Select()
{
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示“+”标志
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项

  g_EvseController.EnableVentReq((m_CurIdx == 0) ? 1 : 0);  // 根据选项启用或禁用VentReq

  delay(500);  // 延迟500ms

  return &g_SetupMenu;  // 返回设置菜单
}

#ifdef ADVPWR
// GndChkMenu类的构造函数
GndChkMenu::GndChkMenu()
{
  m_Title = g_psGndChk;  // 初始化菜单标题为"GndChk"
}

// 初始化菜单，显示标题和当前设置
void GndChkMenu::Init()
{
  g_OBD.LcdPrint_P(0, m_Title);  // 在LCD显示屏上打印标题
  m_CurIdx = g_EvseController.GndChkEnabled() ? 0 : 1;  // 根据是否启用Ground Check设置当前选项索引
  sprintf(g_sTmp, "+%s", g_YesNoMenuItems[m_CurIdx]);  // 格式化当前选项
  g_OBD.LcdPrint(1, g_sTmp);  // 在LCD第二行显示当前选项
}

// 切换到下一个选项
void GndChkMenu::Next()
{
  if (++m_CurIdx >= 2) {  // 如果当前选项索引超过1，重置为0
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);  // 清除LCD第二行
  g_OBD.LcdSetCursor(0, 1);  // 设置光标到LCD第二行的开始位置
  uint8_t dce = g_EvseController.GndChkEnabled();  // 获取当前Ground Check的启用状态
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {  // 根据当前设置决定是否显示“+”标志
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项
}

// 选择当前选项并返回下一个菜单
Menu *GndChkMenu::Select()
{
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示“+”标志
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项

  g_EvseController.EnableGndChk((m_CurIdx == 0) ? 1 : 0);  // 根据选项启用或禁用Ground Check

  delay(500);  // 延迟500ms

  return &g_SetupMenu;  // 返回设置菜单
}

// RlyChkMenu类的构造函数
RlyChkMenu::RlyChkMenu()
{
  m_Title = g_psRlyChk;  // 初始化菜单标题为"RlyChk"
}

// 初始化菜单，显示标题和当前设置
void RlyChkMenu::Init()
{
  g_OBD.LcdPrint_P(0, m_Title);  // 在LCD显示屏上打印标题
  m_CurIdx = g_EvseController.StuckRelayChkEnabled() ? 0 : 1;  // 根据是否启用Relay Check设置当前选项索引
  sprintf(g_sTmp, "+%s", g_YesNoMenuItems[m_CurIdx]);  // 格式化当前选项
  g_OBD.LcdPrint(1, g_sTmp);  // 在LCD第二行显示当前选项
}

// 切换到下一个选项
void RlyChkMenu::Next()
{
  if (++m_CurIdx >= 2) {  // 如果当前选项索引超过1，重置为0
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1);  // 清除LCD第二行
  g_OBD.LcdSetCursor(0, 1);  // 设置光标到LCD第二行的开始位置
  uint8_t dce = g_EvseController.StuckRelayChkEnabled();  // 获取当前Relay Check的启用状态
  if ((dce && !m_CurIdx) || (!dce && m_CurIdx)) {  // 根据当前设置决定是否显示“+”标志
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项
}

// 选择当前选项并返回下一个菜单
Menu *RlyChkMenu::Select()
{
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示“+”标志
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项

  g_EvseController.EnableStuckRelayChk((m_CurIdx == 0) ? 1 : 0);  // 根据选项启用或禁用Relay Check

  delay(500);  // 延迟500ms

  return &g_SetupMenu;  // 返回设置菜单
}
#endif // ADVPWR

// MaxCurrentMenu类的构造函数
MaxCurrentMenu::MaxCurrentMenu()
{
  m_Title = g_psMaxCurrent;  // 初始化菜单标题为"MaxCurrent"
}

// 初始化菜单，显示最大电流的标题和当前值
void MaxCurrentMenu::Init()
{
  uint8_t cursvclvl = g_EvseController.GetCurSvcLevel();  // 获取当前服务等级
  m_MinCurrent = MIN_CURRENT_CAPACITY_J1772;  // 设置最小电流为J1772标准
  if (cursvclvl == 1) {
    m_MaxCurrent = MAX_CURRENT_CAPACITY_L1;  // 如果服务等级为L1，最大电流为L1标准
  }
  else {
    m_MaxCurrent = g_EvseController.GetMaxHwCurrentCapacity();  // 否则获取硬件最大电流
  }

  sprintf(g_sTmp, g_sMaxCurrentFmt, (cursvclvl == 1) ? "L1" : "L2");  // 格式化最大电流标题
  g_OBD.LcdPrint(0, g_sTmp);  // 在LCD第一行显示最大电流标题
  m_CurIdx = g_EvseController.GetCurrentCapacity();  // 获取当前电流值
  if (m_CurIdx < m_MinCurrent) m_CurIdx = m_MinCurrent;  // 如果当前电流小于最小电流，设置为最小电流
  sprintf(g_sTmp, "+%dA", m_CurIdx);  // 格式化当前电流
  g_OBD.LcdPrint(1, g_sTmp);  // 在LCD第二行显示当前电流
}

// 切换到下一个电流值
void MaxCurrentMenu::Next()
{
  if (++m_CurIdx > m_MaxCurrent) {  // 如果当前电流超过最大电流，重置为最小电流
    m_CurIdx = m_MinCurrent;
  }
  g_OBD.LcdClearLine(1);  // 清除LCD第二行
  g_OBD.LcdSetCursor(0, 1);  // 设置光标到LCD第二行的开始位置
  if (g_EvseController.GetCurrentCapacity() == m_CurIdx) {  // 如果当前电流值已被设置为该值，显示“+”标志
    g_OBD.LcdPrint(g_sPlus);
  }
  g_OBD.LcdPrint(m_CurIdx);  // 显示当前电流值
  g_OBD.LcdPrint("A");  // 显示单位"A"
}

// 选择当前电流值并返回设置菜单
Menu *MaxCurrentMenu::Select()
{
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示“+”标志
  g_OBD.LcdPrint(m_CurIdx);  // 显示当前电流值
  g_OBD.LcdPrint("A");  // 显示单位"A"
  delay(500);  // 延迟500ms
  eeprom_write_byte((uint8_t*)((g_EvseController.GetCurSvcLevel() == 1) ? EOFS_CURRENT_CAPACITY_L1 : EOFS_CURRENT_CAPACITY_L2), m_CurIdx);  // 将当前电流值写入EEPROM
  g_EvseController.SetCurrentCapacity(m_CurIdx);  // 设置电流容量
  return &g_SetupMenu;  // 返回设置菜单
}

// ResetMenu类的构造函数
ResetMenu::ResetMenu()
{
  m_Title = g_psReset;  // 初始化菜单标题为"Reset"
}

// 初始化菜单，显示重置提示
void ResetMenu::Init()
{
  m_CurIdx = 0;  // 初始化当前选项索引为0
  g_OBD.LcdPrint_P(0, g_psResetNow);  // 在LCD显示屏上打印"Reset Now"
  g_OBD.LcdPrint(1, g_YesNoMenuItems[0]);  // 显示"Yes"选项
}

void ResetMenu::Next()
{
  // 如果当前索引（m_CurIdx）增加后超过或等于 2，则将其重置为 0
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  // 清空 LCD 屏幕的第二行
  g_OBD.LcdClearLine(1);
  // 在 LCD 屏幕上打印当前的选项（是或否）
  g_OBD.LcdPrint(0, 1, g_YesNoMenuItems[m_CurIdx]);
}

Menu *ResetMenu::Select()
{
  // 打印 "+" 符号
  g_OBD.LcdPrint(0, 1, g_sPlus);
  // 打印当前选项（是或否）
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);
  // 延迟 500 毫秒
  delay(500);
  // 如果选择的是第一个选项（是），重启 EVSE 控制器
  if (m_CurIdx == 0) {
    g_EvseController.Reboot();
  }
  // 返回空值，表示不切换到其他菜单
  return NULL;
}

#ifdef DELAYTIMER_MENU
// pluspos = -1 表示不显示 "+"
void DtsStrPrint1(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mn, int8_t pluspos)
{
  *g_sTmp = 0; // 清空临时字符串
  // 根据 pluspos 的不同值，决定是否在日期时间前面加上 "+" 符号
  if (pluspos == 0) strcat(g_sTmp, g_sPlus);
  strcat(g_sTmp, u2a(mo, 2)); // 将月份转换为两位字符串并连接
  strcat(g_sTmp, g_sSlash);    // 加入斜杠
  if (pluspos == 1) strcat(g_sTmp, g_sPlus);
  strcat(g_sTmp, u2a(d, 2));   // 将日期转换为两位字符串并连接
  strcat(g_sTmp, g_sSlash);    // 加入斜杠
  if (pluspos == 2) strcat(g_sTmp, g_sPlus);
  strcat(g_sTmp, u2a(y, 2));   // 将年份转换为两位字符串并连接
  strcat(g_sTmp, g_sSpace);    // 加入空格
  if (pluspos == 3) strcat(g_sTmp, g_sPlus);
  strcat(g_sTmp, u2a(h, 2));   // 将小时转换为两位字符串并连接
  strcat(g_sTmp, g_sColon);    // 加入冒号
  if (pluspos == 4) strcat(g_sTmp, g_sPlus);
  strcat(g_sTmp, u2a(mn, 2));  // 将分钟转换为两位字符串并连接

  // 将格式化后的日期时间字符串打印到 LCD 屏幕
  g_OBD.LcdPrint(1, g_sTmp);
}


RTCMenu::RTCMenu()
{
  m_Title = g_psRTC; // 设置菜单标题
}

void RTCMenu::Init()
{
  m_CurIdx = 0; // 初始选项索引为 0
  g_OBD.LcdPrint_P(0, g_psSetDateTime); // 显示设置日期时间的提示
  g_OBD.LcdPrint(1, g_YesNoMenuItems[0]); // 显示第一个选项（是）
}

void RTCMenu::Next()
{
  // 如果当前索引（m_CurIdx）增加后超过或等于 2，则将其重置为 0
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1); // 清空 LCD 屏幕的第二行
  g_OBD.LcdPrint(0, 1, g_YesNoMenuItems[m_CurIdx]); // 显示当前选项（是或否）
}

Menu *RTCMenu::Select()
{
  g_OBD.LcdPrint(0, 1, g_sPlus); // 打印 "+"
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]); // 打印当前选项（是或否）
  delay(500); // 延迟 500 毫秒
  if (m_CurIdx == 0) {
    // 如果选择了第一个选项（是），进入月份设置菜单
    return &g_RTCMenuMonth;
  } else {
    // 如果选择了第二个选项（否），返回到设置菜单
    return &g_SetupMenu;
  }
}

RTCMenuMonth::RTCMenuMonth()
{
}

void RTCMenuMonth::Init()
{
  g_OBD.LcdPrint_P(0, g_psRTC_Month); // 显示月份设置的提示
  DateTime t = g_RTC.now(); // 获取当前日期时间
  g_month = t.month();  // 获取当前月份
  g_day = t.day();      // 获取当前日期
  g_year = t.year() - 2000; // 获取当前年份，减去 2000
  g_hour = t.hour();    // 获取当前小时
  g_min = t.minute();   // 获取当前分钟
  m_CurIdx = g_month;   // 设置当前索引为当前月份

  DtsStrPrint1(g_year, g_month, g_day, g_hour, g_min, 0); // 打印当前日期时间
}

void RTCMenuMonth::Next()
{
  // 如果当前索引增加后超过 12（12 个月），则重置为 1
  if (++m_CurIdx >= 13) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1); // 清空 LCD 屏幕的第二行
  DtsStrPrint1(g_year, m_CurIdx, g_day, g_hour, g_min, 0); // 打印更新后的日期时间
}

Menu *RTCMenuMonth::Select()
{
  g_month = m_CurIdx;  // 设置当前月份
  DtsStrPrint1(g_year, m_CurIdx, g_day, g_hour, g_min, 0); // 打印更新后的日期时间
  delay(500); // 延迟 500 毫秒
  return &g_RTCMenuDay; // 进入日期设置菜单
}

RTCMenuDay::RTCMenuDay()
{
  m_Title = g_psRTC; // 设置菜单标题
}

void RTCMenuDay::Init()
{
  g_OBD.LcdPrint_P(0, g_psRTC_Day); // 显示日期设置的提示
  m_CurIdx = g_day; // 设置当前日期为当前系统日期
  DtsStrPrint1(g_year, g_month, m_CurIdx, g_hour, g_min, 1); // 打印当前日期时间
}

void RTCMenuDay::Next()
{
  // 如果当前日期（m_CurIdx）增加后超过 31，则重置为 1
  if (++m_CurIdx >= 32) {
    m_CurIdx = 1;
  }
  g_OBD.LcdClearLine(1); // 清空 LCD 屏幕的第二行
  DtsStrPrint1(g_year, g_month, m_CurIdx, g_hour, g_min, 1); // 打印更新后的日期时间
}

Menu *RTCMenuDay::Select()
{
  g_day = m_CurIdx;  // 设置当前日期
  DtsStrPrint1(g_year, g_month, m_CurIdx, g_hour, g_min, 1); // 打印更新后的日期时间
  delay(500); // 延迟 500 毫秒
  return &g_RTCMenuYear; // 进入年份设置菜单
}

RTCMenuYear::RTCMenuYear()
{
}

#define YEAR_MIN 23 // 最小年份值
#define YEAR_MAX 33 // 最大年份值

void RTCMenuYear::Init()
{
  g_OBD.LcdPrint_P(0, g_psRTC_Year); // 显示年份设置的提示
  m_CurIdx = g_year; // 设置当前年份
  // 如果当前年份不在合法范围内，设置为最小年份
  if (m_CurIdx < YEAR_MIN || m_CurIdx > YEAR_MAX) {
    m_CurIdx = YEAR_MIN;
    g_year = YEAR_MIN;
  }
  DtsStrPrint1(m_CurIdx, g_month, g_day, g_hour, g_min, 2); // 打印更新后的日期时间
}

void RTCMenuYear::Next()
{
  // 如果当前年份（m_CurIdx）增加后超过最大年份，则重置为最小年份
  if (++m_CurIdx > YEAR_MAX) {
    m_CurIdx = YEAR_MIN;
  }
  DtsStrPrint1(m_CurIdx, g_month, g_day, g_hour, g_min, 2); // 打印更新后的日期时间
}

Menu *RTCMenuYear::Select()
{
  g_year = m_CurIdx; // 设置当前年份
  DtsStrPrint1(m_CurIdx, g_month, g_day, g_hour, g_min, 2); // 打印更新后的日期时间
  delay(500); // 延迟 500 毫秒
  return &g_RTCMenuHour; // 进入小时设置菜单
}

RTCMenuHour::RTCMenuHour()
{
}

void RTCMenuHour::Init()
{
  g_OBD.LcdPrint_P(0, g_psRTC_Hour); // 显示小时设置的提示
  m_CurIdx = g_hour; // 设置当前小时
  DtsStrPrint1(g_year, g_month, g_day, m_CurIdx, g_min, 3); // 打印当前时间
}

void RTCMenuHour::Next()
{
  // 如果当前小时（m_CurIdx）增加后超过 23，则重置为 0
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1); // 清空 LCD 屏幕的第二行
  DtsStrPrint1(g_year, g_month, g_day, m_CurIdx, g_min, 3); // 打印更新后的时间
}

Menu *RTCMenuHour::Select()
{
  g_hour = m_CurIdx; // 设置当前小时
  DtsStrPrint1(g_year, g_month, g_day, m_CurIdx, g_min, 3); // 打印更新后的时间
  delay(500); // 延迟 500 毫秒
  return &g_RTCMenuMinute; // 进入分钟设置菜单
}

RTCMenuMinute::RTCMenuMinute()
{
}

void RTCMenuMinute::Init()
{
  g_OBD.LcdPrint_P(0, g_psRTC_Minute); // 显示分钟设置的提示
  m_CurIdx = g_min; // 设置当前分钟
  DtsStrPrint1(g_year, g_month, g_day, g_hour, m_CurIdx, 4); // 打印当前时间
}

void RTCMenuMinute::Next()
{
  // 如果当前分钟（m_CurIdx）增加后超过 59，则重置为 0
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;
  }
  g_OBD.LcdClearLine(1); // 清空 LCD 屏幕的第二行
  DtsStrPrint1(g_year, g_month, g_day, g_hour, m_CurIdx, 4); // 打印更新后的时间
}

Menu *RTCMenuMinute::Select()
{
  g_min = m_CurIdx; // 设置当前分钟
  DtsStrPrint1(g_year, g_month, g_day, g_hour, m_CurIdx, 4); // 打印更新后的时间
  g_RTC.adjust(DateTime(g_year, g_month, g_day, g_hour, g_min, 0)); // 更新 RTC 时间
  delay(500); // 延迟 500 毫秒
  return &g_SetupMenu; // 返回到设置菜单
}


// HsStrPrint1: 用于打印时间（小时和分钟）到 LCD 屏幕
void HsStrPrint1(uint8_t h, uint8_t m, int8_t pluspos = -1)
{
  *g_sTmp = 0;  // 清空临时字符串
  if (pluspos == 0) strcat(g_sTmp, g_sPlus);  // 如果 pluspos 为 0，在字符串前添加“+”
  strcat(g_sTmp, u2a(h, 2));  // 将小时（h）转换为 2 位数字并添加到字符串中
  strcat(g_sTmp, g_sColon);  // 添加冒号“:”
  if (pluspos == 1) strcat(g_sTmp, g_sPlus);  // 如果 pluspos 为 1，在字符串前添加“+”
  strcat(g_sTmp, u2a(m, 2));  // 将分钟（m）转换为 2 位数字并添加到字符串中
  g_OBD.LcdPrint(1, g_sTmp);  // 在 LCD 屏幕上打印结果
}

// DelayMenu：延时菜单类
DelayMenu::DelayMenu()
{
  m_Title = g_psDelayTimer;  // 设置菜单标题为延时计时器
}

// Init：初始化延时菜单
void DelayMenu::Init()
{
  m_CurIdx = 0;  // 当前菜单项索引初始化为 0
  g_OBD.LcdPrint_P(0, g_psDelayTimer);  // 在 LCD 屏幕上打印菜单标题
  g_OBD.LcdPrint(1, g_DelayMenuItems[0]);  // 显示第一个菜单项
}

// Next：切换到下一个菜单项
void DelayMenu::Next()
{
  if (++m_CurIdx >= 3) {
    m_CurIdx = 0;  // 如果当前索引大于等于 3，重置为 0
  }
  g_OBD.LcdClearLine(1);  // 清除当前行
  g_OBD.LcdPrint(0, 1, g_DelayMenuItems[m_CurIdx]);  // 显示当前菜单项
}

// Select：选择当前菜单项并返回相应的菜单
Menu *DelayMenu::Select()
{
  delay(500);  // 延迟 500 毫秒
  if (m_CurIdx == 0) {
    return &g_DelayMenuEnableDisable;  // 如果当前索引为 0，返回启用/禁用延时菜单
  } else if (m_CurIdx == 1) {
    return &g_DelayMenuStartHour;  // 如果当前索引为 1，返回设置延时开始小时菜单
  } else {
    return &g_DelayMenuStopHour;  // 如果当前索引为 2，返回设置延时停止小时菜单
  }
}

// DelayMenuEnableDisable：启用/禁用延时菜单
DelayMenuEnableDisable::DelayMenuEnableDisable()
{
  m_Title = g_psDelayTimer;  // 设置菜单标题为延时计时器
}

// Init：初始化启用/禁用菜单
void DelayMenuEnableDisable::Init()
{
  m_CurIdx = !g_DelayTimer.IsTimerEnabled();  // 根据当前计时器状态设置当前索引
  g_OBD.LcdPrint_P(0, g_psDelayTimer);  // 在 LCD 屏幕上打印菜单标题
  g_OBD.LcdClearLine(1);  // 清除当前行
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 显示“+”
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前启用/禁用选项
}

// Next：切换启用/禁用选项
void DelayMenuEnableDisable::Next()
{
  if (++m_CurIdx >= 2) {
    m_CurIdx = 0;  // 如果当前索引大于等于 2，重置为 0
  }
  g_OBD.LcdClearLine(1);  // 清除当前行
  if (m_CurIdx == !g_DelayTimer.IsTimerEnabled()) {
    g_OBD.LcdPrint(0, 1, g_sPlus);  // 如果当前选项与计时器状态相反，显示“+”
    g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前启用/禁用选项
  } else {
    g_OBD.LcdPrint(0, 1, g_YesNoMenuItems[m_CurIdx]);  // 显示当前启用/禁用选项
  }
}

// Select：选择启用/禁用选项并执行相应操作
Menu *DelayMenuEnableDisable::Select()
{
  g_OBD.LcdPrint(0, 1, g_sPlus);  // 在 LCD 上打印“+”
  g_OBD.LcdPrint(g_YesNoMenuItems[m_CurIdx]);  // 显示当前选项
  delay(500);  // 延迟 500 毫秒
  if (m_CurIdx == 0) {
    g_DelayTimer.Enable();  // 启用计时器
  } else {
    g_DelayTimer.Disable();  // 禁用计时器
  }
  return &g_SettingsMenu;  // 返回设置菜单
}

// DelayMenuStartHour：设置延时开始小时菜单
DelayMenuStartHour::DelayMenuStartHour()
{
}

// Init：初始化延时开始小时菜单
void DelayMenuStartHour::Init()
{
  g_OBD.LcdPrint_P(0, g_psDelayTimerStartHour);  // 在 LCD 屏幕上打印菜单标题
  g_hour = g_DelayTimer.GetStartTimerHour();  // 获取当前延时开始小时
  g_min = g_DelayTimer.GetStartTimerMin();  // 获取当前延时开始分钟
  m_CurIdx = g_hour;  // 设置当前索引为当前小时
  HsStrPrint1(m_CurIdx, g_min, 0);  // 打印当前开始时间
}

// Next：切换到下一个小时
void DelayMenuStartHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;  // 如果当前小时大于等于 24，重置为 0
  }
  HsStrPrint1(m_CurIdx, g_min, 0);  // 打印当前开始时间
}

// Select：选择当前开始小时并进入分钟设置菜单
Menu *DelayMenuStartHour::Select()
{
  g_hour = m_CurIdx;  // 设置开始小时
  HsStrPrint1(m_CurIdx, g_min, 0);  // 打印当前开始时间
  delay(500);  // 延迟 500 毫秒
  return &g_DelayMenuStartMin;  // 返回设置开始分钟菜单
}

// DelayMenuStopHour：设置延时停止小时菜单
DelayMenuStopHour::DelayMenuStopHour()
{
}

// Init：初始化延时停止小时菜单
void DelayMenuStopHour::Init()
{
  g_OBD.LcdPrint_P(0, g_psDelayTimerStopHour);  // 在 LCD 屏幕上打印菜单标题
  g_hour = g_DelayTimer.GetStopTimerHour();  // 获取当前延时停止小时
  g_min = g_DelayTimer.GetStopTimerMin();  // 获取当前延时停止分钟
  m_CurIdx = g_hour;  // 设置当前索引为当前小时
  HsStrPrint1(m_CurIdx, g_min, 0);  // 打印当前停止时间
}

// Next：切换到下一个小时
void DelayMenuStopHour::Next()
{
  if (++m_CurIdx >= 24) {
    m_CurIdx = 0;  // 如果当前小时大于等于 24，重置为 0
  }
  HsStrPrint1(m_CurIdx, g_min, 0);  // 打印当前停止时间
}

// Select：选择当前停止小时并进入分钟设置菜单
Menu *DelayMenuStopHour::Select()
{
  g_hour = m_CurIdx;  // 设置停止小时
  HsStrPrint1(m_CurIdx, g_min, 0);  // 打印当前停止时间
  delay(500);  // 延迟 500 毫秒
  return &g_DelayMenuStopMin;  // 返回设置停止分钟菜单
}

// DelayMenuStartMin：设置延时开始分钟菜单
DelayMenuStartMin::DelayMenuStartMin()
{
}

// Init：初始化延时开始分钟菜单
void DelayMenuStartMin::Init()
{
  g_OBD.LcdPrint_P(0, g_psDelayTimerStartMin);  // 在 LCD 屏幕上打印菜单标题
  m_CurIdx = g_min;  // 设置当前索引为当前分钟
  HsStrPrint1(g_hour, m_CurIdx, 1);  // 打印当前开始时间（小时和分钟）
}

// Next：切换到下一个分钟
void DelayMenuStartMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;  // 如果当前分钟大于等于 60，重置为 0
  }
  HsStrPrint1(g_hour, m_CurIdx, 1);  // 打印当前开始时间（小时和分钟）
}

// Select：选择当前开始分钟并设置开始计时器
Menu *DelayMenuStartMin::Select()
{
  g_min = m_CurIdx;  // 设置开始分钟
  HsStrPrint1(g_hour, m_CurIdx, 1);  // 打印当前开始时间（小时和分钟）
  g_DelayTimer.SetStartTimer(g_hour, g_min);  // 设置开始计时器
  delay(500);  // 延迟 500 毫秒
  return &g_SettingsMenu;  // 返回设置菜单
}

// DelayMenuStopMin：设置延时停止分钟菜单
DelayMenuStopMin::DelayMenuStopMin()
{
}

// Init：初始化延时停止分钟菜单
void DelayMenuStopMin::Init()
{
  g_OBD.LcdPrint_P(0, g_psDelayTimerStopMin);  // 在 LCD 屏幕上打印菜单标题
  m_CurIdx = g_min;  // 设置当前索引为当前分钟
  HsStrPrint1(g_hour, m_CurIdx, 1);  // 打印当前停止时间（小时和分钟）
}

// Next：切换到下一个分钟
void DelayMenuStopMin::Next()
{
  if (++m_CurIdx >= 60) {
    m_CurIdx = 0;  // 如果当前分钟大于等于 60，重置为 0
  }
  HsStrPrint1(g_hour, m_CurIdx, 1);  // 打印当前停止时间（小时和分钟）
}

// Select：选择当前停止分钟并设置停止计时器
Menu *DelayMenuStopMin::Select()
{
  g_min = m_CurIdx;  // 设置停止分钟
  HsStrPrint1(g_hour, m_CurIdx, 1);  // 打印当前停止时间（小时和分钟）
  g_DelayTimer.SetStopTimer(g_hour, g_min);  // 设置停止计时器
  delay(500);  // 延迟 500 毫秒
  return &g_SettingsMenu;  // 返回设置菜单
}

// ChargeLimitMenu：充电限制菜单类
ChargeLimitMenu::ChargeLimitMenu()
{
  m_Title = g_psChargeLimit;  // 设置菜单标题为充电限制
}

// showCurSel：显示当前选择的充电限制
void ChargeLimitMenu::showCurSel(uint8_t plus)
{
  *g_sTmp = 0;  // 清空临时字符串
  if (plus) strcpy(g_sTmp, g_sPlus);  // 如果 plus 为真，添加“+”
  if (m_CurIdx == 0) {
    strcat(g_sTmp, "off");  // 如果当前索引为 0，显示“off”
  } else {
    strcat(g_sTmp, u2a(m_CurIdx));  // 否则，显示充电限制数值
    strcat(g_sTmp, " kWh");  // 添加单位“kWh”
  }
  g_OBD.LcdPrint(1, g_sTmp);  // 在 LCD 屏幕上显示当前选择的充电限制
}


void ChargeLimitMenu::Init()
{
  // 获取当前充电限制 kWh
  m_CurIdx = g_EvseController.GetChargeLimitkWh();

  // 在 LCD 上显示充电限制文本
  g_OBD.LcdPrint_P(0,g_psChargeLimit);

  // 显示当前选择项
  showCurSel(1);
}

void ChargeLimitMenu::Next()
{
  // 如果当前索引小于 5，则自增 1，否则自增 5
  if (m_CurIdx < 5) m_CurIdx++;
  else m_CurIdx += 5;

  // 如果当前索引超过最大限制，则重置为 0
  if (m_CurIdx > MAX_CHARGE_LIMIT) {
    m_CurIdx = 0;
  }

  // 如果当前的充电限制与设置的充电限制一致，则显示已选择状态
  showCurSel((g_EvseController.GetChargeLimitkWh() == m_CurIdx) ? 1 : 0);
}

Menu *ChargeLimitMenu::Select()
{
  // 显示当前选择项
  showCurSel(1);

  // 设置新的充电限制
  g_EvseController.SetChargeLimitkWh(m_CurIdx);

  // 延时 500ms
  delay(500);

  // 如果当前选择项为 0，则返回设置菜单，否则返回空
  return m_CurIdx ? NULL : &g_SettingsMenu;
}

#endif // CHARGE_LIMIT


#ifdef TIME_LIMIT
// 最大时间限制 = MAX_TIME_LIMIT_D15 * 15 分钟
#define MAX_TIME_LIMIT_D15 32

TimeLimitMenu::TimeLimitMenu()
{
  m_Title = g_psTimeLimit;
}

void TimeLimitMenu::showCurSel(uint8_t plus)
{
  uint16_t limit = m_CurIdx * 15;
  *g_sTmp = 0;

  // 如果需要显示加号，则添加加号
  if (plus) strcpy(g_sTmp, g_sPlus);

  // 如果时间限制为 0，则显示“off”
  if (limit == 0) {
    strcat(g_sTmp,"off");
  }
  else {
    // 如果时间小于 60 分钟，直接显示分钟数
    if (limit < 60) {
      strcat(g_sTmp, u2a(limit));
      strcat(g_sTmp," min");
    }
    else {
      // 否则显示小时数，如果有半小时则显示 ".5"
      strcat(g_sTmp, u2a(limit / 60));
      if (limit % 60) { // 假设只有 30 分钟
        strcat(g_sTmp,".5");
      }
      strcat(g_sTmp," hr");
    }
  }

  // 在 LCD 上显示当前选择项
  g_OBD.LcdPrint(1, g_sTmp);
}

void TimeLimitMenu::Init()
{
  // 获取当前时间限制（以 15 分钟为单位）
  m_CurIdx = g_EvseController.GetTimeLimit15();

  // 在 LCD 上显示时间限制文本
  g_OBD.LcdPrint_P(0, g_psTimeLimit);

  // 显示当前选择项
  showCurSel(1);
}

void TimeLimitMenu::Next()
{
  // 根据当前索引调整时间限制，逐步增加
  if (m_CurIdx < 4) m_CurIdx++;
  else if (m_CurIdx < 12) m_CurIdx += 2;
  else m_CurIdx += 4;

  // 如果当前索引超过最大时间限制，则重置为 0
  if (m_CurIdx > MAX_TIME_LIMIT_D15) {
    m_CurIdx = 0;
  }

  // 显示当前选择项
  showCurSel((g_EvseController.GetTimeLimit15() == m_CurIdx) ? 1 : 0);
}

Menu *TimeLimitMenu::Select()
{
  // 显示当前选择项
  showCurSel(1);

  // 设置新的时间限制
  g_EvseController.SetTimeLimit15(m_CurIdx);

  // 延时 500ms
  delay(500);

  // 如果当前选择项为 0，则返回设置菜单，否则返回空
  return m_CurIdx ? NULL : &g_SettingsMenu;
}
#endif // TIME_LIMIT

BtnHandler::BtnHandler()
{
  m_CurMenu = NULL;
}

int8_t BtnHandler::DoShortPress(int8_t infaultstate)
{
#ifdef TEMPERATURE_MONITORING
    // 清除过热日志
    g_TempMonitor.ClrOverTemperatureLogged();
#endif
    // 如果当前菜单存在，调用菜单的 Next 方法
    if (m_CurMenu) {
      m_CurMenu->Next();
    }
    else {
      // 如果处于故障状态，则强制进入设置菜单
    if (infaultstate) return 1; // 触发长按动作
      else {
        // 如果 EVSE 被禁用或者处于睡眠状态，启用它；否则，进入睡眠模式
	if ((g_EvseController.GetState() == EVSE_STATE_DISABLED) ||
	    (g_EvseController.GetState() == EVSE_STATE_SLEEPING)) {
	  g_EvseController.Enable();
	}
	else {
	  g_EvseController.Sleep();
	}

#ifdef DELAYTIMER
	if (g_DelayTimer.IsTimerEnabled()) {
	  uint8_t intimeinterval = g_DelayTimer.IsInAwakeTimeInterval();
	  uint8_t sleeping = (g_EvseController.GetState() == EVSE_STATE_SLEEPING) ? 1 : 0;
	  // 如果当前时间在唤醒时间段内且设备处于睡眠状态，或者在其他情况下
	  if ((intimeinterval && sleeping) || (!intimeinterval && !sleeping)) {
	    g_DelayTimer.SetManualOverride();
          }
	  else {
	    g_DelayTimer.ClrManualOverride();
   	  }
	}
#endif // DELAYTIMER
      }
    }

  return 0;
}

void BtnHandler::ChkBtn()
{
  WDT_RESET();  // 重置看门狗定时器
  m_Btn.read();  // 读取按钮状态

  // 如果 EVSE 控制器按钮不可用，则处理按钮按下事件
  if (!g_EvseController.ButtonIsEnabled()) {
#ifdef RAPI_BTN
    // 如果按钮短按，发送按下事件
    if (m_Btn.shortPress()) {
      RapiSendButtonPress(0);
    } else if (m_Btn.longPress()) {
      RapiSendButtonPress(1);
    }
#endif // RAPI_BTN
    return;
  }

  // 获取故障状态
  int8_t infaultstate = g_EvseController.InFaultState();

  // 处理短按事件
  if (m_Btn.shortPress()) {
    if (DoShortPress(infaultstate)) {
      goto longpress;
    }
  }
  else if (m_Btn.longPress()) {
#ifdef TEMPERATURE_MONITORING
    // 清除过热日志
    g_TempMonitor.ClrOverTemperatureLogged();
#endif
  longpress:
    // 如果当前菜单存在，执行选择
    if (m_CurMenu) {
      m_CurMenu = m_CurMenu->Select();
      if (m_CurMenu) {
	uint8_t curidx = 0;
        // 如果当前菜单是设置菜单或配置菜单，保存当前索引
	if ((m_CurMenu == &g_SettingsMenu)||(m_CurMenu == &g_SetupMenu)) {
	  curidx = m_CurMenu->m_CurIdx;
	}
	m_CurMenu->Init();
	if ((m_CurMenu == &g_SettingsMenu)||(m_CurMenu == &g_SetupMenu)) {
	  // 恢复上一个菜单项
	  if (curidx > 0) {
	    m_CurMenu->m_CurIdx = curidx-1;
	    m_CurMenu->Next();
	  }
	}

      }
      else { // 退出菜单
	g_EvseController.Enable();
	g_OBD.DisableUpdate(0);
	g_OBD.LcdSetBacklightType(m_SavedLcdMode); // 退出菜单时恢复 LCD 模式
	g_OBD.Update(OBD_UPD_FORCE);
	g_EvseController.ClrInMenu();
      }
    }
    else {
      // 进入菜单
      g_EvseController.SetInMenu();
#if defined(CHARGE_LIMIT) || defined(TIME_LIMIT)
      g_SettingsMenu.CheckSkipLimits();
#endif // CHARGE_LIMIT
      // 保存当前 LCD 背光模式
      m_SavedLcdMode = g_OBD.IsLcdBacklightMono() ? BKL_TYPE_MONO : BKL_TYPE_RGB;
      g_OBD.LcdSetBacklightColor(WHITE);  // 设置背光为白色
      g_OBD.DisableUpdate(1);
      if (infaultstate) {
        // 如果处于故障状态，进入设置菜单
	m_CurMenu = &g_SetupMenu;
      }
      else {
        // 否则，进入设置菜单并使设备进入睡眠模式
	g_EvseController.Sleep();
	m_CurMenu = &g_SettingsMenu;
      }
      m_CurMenu->Init();
    }
  }
}
#endif // BTN_MENU

// 延时定时器功能 - GoldServe
#ifdef DELAYTIMER
void DelayTimer::Init() {
  // 读取 EEPROM 设置
  uint8_t rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_FLAGS);
  if (rtmp == 0xff) { // EEPROM 未初始化
    m_DelayTimerEnabled = 0x00;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  }
  else {
    m_DelayTimerEnabled = rtmp;
  }

  // 根据定时器是否启用设置 EVSE 控制器的延时标志
  if (m_DelayTimerEnabled) g_EvseController.SetDelayTimerOnFlag();
  else g_EvseController.ClrDelayTimerOnFlag();

  // 初始化开始时间（小时）
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_HOUR);
  if (rtmp == 0xff) { // EEPROM 未初始化
    m_StartTimerHour = DEFAULT_START_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_HOUR, m_StartTimerHour);
  }
  else {
    m_StartTimerHour = rtmp;
  }

  // 初始化开始时间（分钟）
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_START_MIN);
  if (rtmp == 0xff) { // EEPROM 未初始化
    m_StartTimerMin = DEFAULT_START_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_START_MIN, m_StartTimerMin);
  }
  else {
    m_StartTimerMin = rtmp;
  }

  // 初始化停止时间（小时）
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_HOUR);
  if (rtmp == 0xff) { // EEPROM 未初始化
    m_StopTimerHour = DEFAULT_STOP_HOUR;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_HOUR, m_StopTimerHour);
  }
  else {
    m_StopTimerHour = rtmp;
  }

  // 初始化停止时间（分钟）
  rtmp = eeprom_read_byte((uint8_t*)EOFS_TIMER_STOP_MIN);
  if (rtmp == 0xff) { // EEPROM 未初始化
    m_StopTimerMin = DEFAULT_STOP_MIN;
    eeprom_write_byte((uint8_t*)EOFS_TIMER_STOP_MIN, m_StopTimerMin);
  }
  else {
    m_StopTimerMin = rtmp;
  }

  // 清除手动覆盖设置
  ClrManualOverride();
}

uint8_t DelayTimer::IsInAwakeTimeInterval()
{
  uint8_t inTimeInterval = false;

  // 判断定时器是否启用并且时间设置有效
  if (IsTimerEnabled() && IsTimerValid()) {
    DateTime t = g_RTC.now();
    uint8_t currHour = t.hour();
    uint8_t currMin = t.minute();

    uint16_t startTimerMinutes = m_StartTimerHour * 60 + m_StartTimerMin;
    uint16_t stopTimerMinutes = m_StopTimerHour * 60 + m_StopTimerMin;
    uint16_t currTimeMinutes = currHour * 60 + currMin;

    // 判断停止时间是否跨越午夜
    if (stopTimerMinutes < startTimerMinutes) { // 结束时间是第二天的时间

      // 判断当前时间是否在有效区间内
      if ( ( (currTimeMinutes >= startTimerMinutes) && (currTimeMinutes > stopTimerMinutes) ) ||
           ( (currTimeMinutes <= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes) ) ){
        inTimeInterval = true;
      }
    }
    else { // 不跨越午夜
      // 判断当前时间是否在有效区间内
      if ((currTimeMinutes >= startTimerMinutes) && (currTimeMinutes < stopTimerMinutes)) {
        inTimeInterval = true;
      }
    }
  }

  return inTimeInterval;
}

void DelayTimer::CheckTime()
{
  // 判断 EVSE 控制器是否处于故障状态，或者是否处于禁用状态
  if (!g_EvseController.InFaultState() &&
      !(g_EvseController.GetState() == EVSE_STATE_DISABLED) &&
      IsTimerEnabled() &&
      IsTimerValid()) {
    unsigned long curms = millis();
    if ((curms - m_LastCheck) > 1000ul) {
      uint8_t inTimeInterval = IsInAwakeTimeInterval();
      uint8_t evseState = g_EvseController.GetState();

      if (inTimeInterval) { // 当前时间处于充电时段
        if (!ManualOverrideIsSet()) {
          // 如果没有手动覆盖并且 EVSE 处于睡眠状态，则启用 EVSE
          if (evseState == EVSE_STATE_SLEEPING) {
            g_EvseController.Enable();
          }
        }
        else {
          // 如果手动覆盖已设置并且 EVSE 不在睡眠状态，则清除手动覆盖
          if (evseState != EVSE_STATE_SLEEPING) {
            ClrManualOverride();
          }
        }
      }
      else { // 当前时间处于休眠时段
        if (!ManualOverrideIsSet()) {
          if ((evseState != EVSE_STATE_SLEEPING)) {
            g_EvseController.Sleep();
          }
        }
        else { // 手动覆盖已设置
          if (evseState == EVSE_STATE_SLEEPING) {
            ClrManualOverride();
          }
        }
      }

      m_LastCheck = curms;
    }
  }
}

void DelayTimer::Enable(){
  m_DelayTimerEnabled = 0x01;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  ClrManualOverride();
  g_EvseController.SetDelayTimerOnFlag();
  g_OBD.Update(OBD_UPD_FORCE);
}

void DelayTimer::Disable(){
  m_DelayTimerEnabled = 0x00;
  eeprom_write_byte((uint8_t*)EOFS_TIMER_FLAGS, m_DelayTimerEnabled);
  ClrManualOverride();
  g_EvseController.ClrDelayTimerOnFlag();
  g_OBD.Update(OBD_UPD_FORCE);
}

void DelayTimer::PrintTimerIcon(){
#ifdef LCD16X2
  if (!ManualOverrideIsSet() && IsTimerEnabled() && IsTimerValid()){
    g_OBD.LcdWrite(6);
  }
#endif // LCD16X2
}
// 延时定时器功能结束 - GoldServe
#endif //#ifdef DELAYTIMER

void ProcessInputs()
{
#ifdef RAPI
  RapiDoCmd(); // 执行 RAPI 命令
#endif
#ifdef BTN_MENU
  g_BtnHandler.ChkBtn(); // 检查按钮输入
#endif
#ifdef TEMPERATURE_MONITORING
  g_TempMonitor.Read();  // 每秒更新温度
#endif
}

void EvseReset()
{
  Wire.begin(); // 初始化 I2C 总线
  g_OBD.Init(); // 初始化 OBD

#ifdef RAPI
  RapiInit(); // 初始化 RAPI
#endif

  g_EvseController.Init(); // 初始化 EVSE 控制器

#ifdef DELAYTIMER
  g_DelayTimer.Init(); // 初始化延时定时器，必须在 EVSE 控制器初始化后调用
#endif  // DELAYTIMER

#ifdef PP_AUTO_AMPACITY
  g_ACCController.AutoSetCurrentCapacity(); // 自动设置电流容量
#endif
}

#ifdef PP_AUTO_AMPACITY
uint8_t StateTransitionReqFunc(uint8_t curPilotState,uint8_t newPilotState,uint8_t curEvseState,uint8_t newEvseState)
{
  uint8_t retEvseState = newEvseState;

  if ((newEvseState >= EVSE_STATE_B) && (newEvseState <= EVSE_STATE_C)) {
    // 注意：由于 J1772EvseController::Update() 在请求状态转换前已经完成了去抖动，因此这里不需要去抖动延迟
    if (g_ACCController.AutoSetCurrentCapacity()) {
      // 如果 PP 无效，则强制 EVSE 保持在状态 A
      retEvseState = EVSE_STATE_A;
    }
  }
  return retEvseState;
}
#endif //PP_AUTO_AMPACITY


void setup()
{
  wdt_disable();  // 禁用看门狗定时器，以防止在初始化时意外重启

  delay(400);  // 延迟400毫秒，给I2C设备时间初始化。否则在电源启动时可能会挂起

  Serial.begin(SERIAL_BAUD);  // 初始化串口通信，设置波特率

#ifdef BTN_MENU
  g_BtnHandler.init();  // 初始化按钮处理器
#endif // BTN_MENU

#ifdef PP_AUTO_AMPACITY
  g_EvseController.SetStateTransitionReqFunc(&StateTransitionReqFunc);  // 设置电动汽车充电站的状态转换请求函数
#endif //PP_AUTO_AMPACITY
  EvseReset();  // 重置电动汽车充电站

#ifdef TEMPERATURE_MONITORING
  g_TempMonitor.Init();  // 初始化温度监控
#endif

#ifdef BOOTLOCK
#ifdef LCD16X2
  g_OBD.LcdMsg_P(PSTR("Waiting for"),PSTR("Initialization.."));  // 在LCD屏上显示“等待初始化..”
#endif // LCD16X2
  while (g_EvseController.IsBootLocked()) {
    ProcessInputs();  // 在启动锁定时处理输入
  }
#endif // BOOTLOCK

  WDT_ENABLE();  // 启用看门狗定时器
}  // setup()


void loop()
{
  WDT_RESET();  // 重置看门狗定时器，防止重启

  g_EvseController.Update();  // 更新电动汽车充电站的状态

#ifdef KWH_RECORDING
  g_EnergyMeter.Update();  // 更新能量计（用于记录电能消耗）
#endif // KWH_RECORDING

#ifdef PERIODIC_LCD_REFRESH_MS
  // 定期刷新LCD（用于CE认证测试），如果LCD损坏则强制恢复
  {
    static unsigned long lastlcdreset = 0;
    if ((millis()-lastlcdreset) > PERIODIC_LCD_REFRESH_MS) {  // 如果经过的时间大于定时刷新间隔
      g_OBD.Update(OBD_UPD_FORCE);  // 强制更新LCD
      lastlcdreset = millis();  // 记录上次LCD刷新时间
    }
    else g_OBD.Update();  // 否则正常更新LCD
  }
#else // !PERIODIC_LCD_REFRESH_MS
  g_OBD.Update();  // 更新LCD
#endif // PERIODIC_LCD_REFRESH_MS

  ProcessInputs();  // 处理输入

  // 延迟定时器处理 - GoldServe
#ifdef DELAYTIMER
  g_DelayTimer.CheckTime();  // 检查延迟定时器的状态
#endif //#ifdef DELAYTIMER
}
