// ---------------------------------------------------------------------------
// 创建者: Francisco Malpartida 20/08/11
// 版权所有 2011 - 根据创作共用协议3.0：
//        姓名标示-相同方式共享 CC BY-SA
//
// 本软件按“原样”提供，未提供技术支持，并且没有任何
// 明示或暗示的担保，保证其适用于任何目的。
//
// 线程安全: 否
// 可扩展: 是
//
// @文件 LiquidCrystal_I2C.c
// 该文件实现了一个基本的液晶显示器库，作为 Arduino SDK 中的标准库之一，
// 但是使用了 I2C 输入输出扩展板。
//
// @简介
// 这是一个基于 Arduino SDK 中的 LiquidCrystal 库的基本实现。
// 原始库经过重构，使得该类实现了所有命令 LCD 的方法，
// 基于 Hitachi HD44780 及兼容芯片集，使用 I2C 扩展背包
// 例如 I2CLCDextraIO 和 PCF8574* I2C IO 扩展器 ASIC。
//
// 该类及其基类提供的功能与 Arduino LiquidCrystal 库的原始功能完全相同。
//
// @作者 F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------
#if (ARDUINO <  100)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#include <inttypes.h>
#include "./I2CIO.h"
#include "./LiquidCrystal_I2C.h"

// 常量定义
// ---------------------------------------------------------------------------

// 背光控制标志
/*!
 @定义
 @摘要   LCD_NOBACKLIGHT
 @讨论   无背光掩码
 */
#define LCD_NOBACKLIGHT 0x00

/*!
 @定义
 @摘要   LCD_BACKLIGHT
 @讨论   背光掩码，当背光打开时使用
 */
#define LCD_BACKLIGHT   0xFF


// 默认的库配置参数，仅使用 I2C 地址字段
// ---------------------------------------------------------------------------
/*!
 @定义
 @摘要   LCD 启用位
 @讨论   定义连接到 LCD 启用引脚的扩展器的 IO
 */
#define EN 6  // 启用位

/*!
 @定义
 @摘要   读取/写入位
 @讨论   定义连接到 LCD 读取/写入引脚的扩展器的 IO
 */
#define RW 5  // 读取/写入位

/*!
 @定义
 @摘要   寄存器选择位
 @讨论   定义连接到 LCD 寄存器选择引脚的扩展器的 IO
 */
#define RS 4  // 寄存器选择位

/*!
 @定义
 @摘要   LCD 数据线分配，本库仅支持 4 位 LCD 控制模式。
 @讨论   D4、D5、D6、D7 LCD 数据线与扩展模块的引脚映射
 */
#define D4 0
#define D5 1
#define D6 2
#define D7 3


// 构造函数
// ---------------------------------------------------------------------------
LiquidCrystal_I2C::LiquidCrystal_I2C( uint8_t lcd_Addr )
{
   config(lcd_Addr, EN, RW, RS, D4, D5, D6, D7);
}


LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t backlighPin,
                                     t_backlighPol pol = POSITIVE)
{
   config(lcd_Addr, EN, RW, RS, D4, D5, D6, D7);
   setBacklightPin(backlighPin, pol);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs)
{
   config(lcd_Addr, En, Rw, Rs, D4, D5, D6, D7);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t backlighPin,
                                     t_backlighPol pol = POSITIVE)
{
   config(lcd_Addr, En, Rw, Rs, D4, D5, D6, D7);
   setBacklightPin(backlighPin, pol);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t d4, uint8_t d5,
                                     uint8_t d6, uint8_t d7 )
{
   config(lcd_Addr, En, Rw, Rs, d4, d5, d6, d7);
}

LiquidCrystal_I2C::LiquidCrystal_I2C(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t d4, uint8_t d5,
                                     uint8_t d6, uint8_t d7, uint8_t backlighPin,
                                     t_backlighPol pol = POSITIVE )
{
   config(lcd_Addr, En, Rw, Rs, d4, d5, d6, d7);
   setBacklightPin(backlighPin, pol);
}

// 公共方法
// ---------------------------------------------------------------------------

//
// begin
void LiquidCrystal_I2C::begin(uint8_t cols, uint8_t lines, uint8_t dotsize)
{

   init();     // 初始化 I2C 扩展器接口
   LCD::begin ( cols, lines, dotsize );
}


// 用户命令 - 用户可以扩展此部分
//----------------------------------------------------------------------------
// 打开/关闭（可选）背光

//
// setBacklightPin
void LiquidCrystal_I2C::setBacklightPin ( uint8_t value, t_backlighPol pol = POSITIVE )
{
   _backlightPinMask = ( 1 << value );
   _polarity = pol;
   setBacklight(BACKLIGHT_OFF);
}

//
// setBacklight
void LiquidCrystal_I2C::setBacklight( uint8_t value )
{
   // 检查是否有背光
   // ----------------------------------------------------
   if ( _backlightPinMask != 0x0 )
   {
      // 根据极性设置掩码
      // ----------------------------------------------------------
      if  (((_polarity == POSITIVE) && (value > 0)) ||
           ((_polarity == NEGATIVE ) && ( value == 0 )))
      {
         _backlightStsMask = _backlightPinMask & LCD_BACKLIGHT;
      }
      else
      {
         _backlightStsMask = _backlightPinMask & LCD_NOBACKLIGHT;
      }
      _i2cio.write( _backlightStsMask );
   }
}


// 私有方法
// ---------------------------------------------------------------------------

//
// init
int LiquidCrystal_I2C::init()
{
   int status = 0;

   // 初始化背包 IO 扩展器
   // 和显示功能。
   // ------------------------------------------------------------------------
   if ( _i2cio.begin ( _Addr ) == 1 )
   {
      _i2cio.portMode ( OUTPUT );  // 设置整个 IO 扩展器为输出
      _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
      status = 1;
      _i2cio.write(0);  // 将整个端口设置为低
   }
   return ( status );
}

//
// config
void LiquidCrystal_I2C::config (uint8_t lcd_Addr, uint8_t En, uint8_t Rw, uint8_t Rs,
                                uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7 )
{
   _Addr = lcd_Addr;

   _backlightPinMask = 0;
   _backlightStsMask = LCD_NOBACKLIGHT;
   _polarity = POSITIVE;

   _En = ( 1 << En );
   _Rw = ( 1 << Rw );
   _Rs = ( 1 << Rs );

   // 初始化引脚映射
   _data_pins[0] = ( 1 << d4 );
   _data_pins[1] = ( 1 << d5 );
   _data_pins[2] = ( 1 << d6 );
   _data_pins[3] = ( 1 << d7 );
}



// 低级数据推送命令
//----------------------------------------------------------------------------

//
// send - 写入命令或数据
void LiquidCrystal_I2C::send(uint8_t value, uint8_t mode)
{
   // 不需要使用延迟例程，因为写入所需的时间已经足够
   // 用于切换使能引脚并执行命令。

   if ( mode == FOUR_BITS )
   {
      write4bits( (value & 0x0F), COMMAND );
   }
   else
   {
      write4bits( (value >> 4), mode );
      write4bits( (value & 0x0F), mode);
   }
}

//
// write4bits
void LiquidCrystal_I2C::write4bits ( uint8_t value, uint8_t mode )
{
   uint8_t pinMapValue = 0;

   // 将值映射到 LCD 引脚映射
   // --------------------------------
   for ( uint8_t i = 0; i < 4; i++ )
   {
      if ( ( value & 0x1 ) == 1 )
      {
         pinMapValue |= _data_pins[i];
      }
      value = ( value >> 1 );
   }

   // 是命令还是数据
   // -----------------------
   if ( mode == DATA )
   {
      mode = _Rs;
   }

   pinMapValue |= mode | _backlightStsMask;
   pulseEnable ( pinMapValue );
}

//
// pulseEnable
void LiquidCrystal_I2C::pulseEnable (uint8_t data)
{
   _i2cio.write (data | _En);   // En 高
   _i2cio.write (data & ~_En);  // En 低
}
