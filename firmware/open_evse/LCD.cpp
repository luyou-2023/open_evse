// ---------------------------------------------------------------------------
// 作者：Francisco Malpartida 创建于 20/08/11
// 版权所有 2011 - 根据创意共享许可协议 3.0：
//        署名-相同方式共享 CC BY-SA
//
// 本软件按“原样”提供，不提供技术支持，且不对其适用于任何目的提供任何明示或暗示的保证。
//
// 线程安全：否
// 可扩展：是
//
// @文件 LCD.cpp
// 该文件实现了一个基础的液晶显示库，作为标准库在 Arduino SDK 中提供。
//
// @简介
// 这是 Arduino SDK 中 HD44780 库的基础实现。该库是对 Arduino SDK 中提供的原始库的重构，简化了其扩展，
// 使其能够支持其他与 LCD 通信的机制，如 I2C、串口、移位寄存器等。
// 原始库经过重构，成为实现所有通用方法的基类，用于控制基于 Hitachi HD44780 和兼容芯片组的 LCD。
//
// 这个基类是一个纯抽象类，需进行扩展。作为参考，已扩展到支持 4 位和 8 位模式控制的 LCD 和 I2C 扩展背板，
// 如使用 PCF8574* I2C IO 扩展器的 I2CLCDextraIO。
//
// @版本 API 1.1.0
//
// 2012.03.29 bperrybap - 修改了比较，改为使用 LCD_5x8DOTS 而不是 0
// @作者 F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#if (ARDUINO <  100)
#include <WProgram.h>
#else
#include <Arduino.h>
#endif
#include "./LCD.h"

// 类构造函数
// ---------------------------------------------------------------------------
// 构造函数
LCD::LCD ()
{

}

// 公共方法
// ---------------------------------------------------------------------------
// 当显示器启动时，默认配置如下：
//
// 1. 清屏
// 2. 功能设置：
//    DL = 1; 8位接口数据
//    N = 0; 1行显示
//    F = 0; 5x8 点阵字符字体
// 3. 显示开关控制：
//    D = 0; 显示关闭
//    C = 0; 光标关闭
//    B = 0; 闪烁关闭
// 4. 输入模式设置：
//    I/D = 1; 增加 1
//    S = 0; 不移动
//
// 然而，重启 Arduino 不会重置 LCD，因此我们不能假设它在该状态下启动（调用 LiquidCrystal 构造函数时）。
// 调用 begin() 会重新初始化 LCD。
//
void LCD::begin(uint8_t cols, uint8_t lines, uint8_t dotsize)
{
   if (lines > 1)
   {
      _displayfunction |= LCD_2LINE; // 设置为 2 行显示
   }
   _numlines = lines;
   _cols = cols;

   // 对于某些 1 行显示，您可以选择 10 像素高的字体
   // ------------------------------------------------------------
   if ((dotsize != LCD_5x8DOTS) && (lines == 1))
   {
      _displayfunction |= LCD_5x10DOTS;
   }

   // 参考数据手册，电源电压超过 2.7V 后，至少需要 40ms 才能发送命令
   // Arduino 在 4.5V 以下就能启动，因此我们等待 50ms
   // ---------------------------------------------------------------------------
   delay (100); // 延迟 100ms

   // 将 LCD 设置为 4 位模式或 8 位模式
   // -------------------------------------
   if (! (_displayfunction & LCD_8BITMODE))
   {
      // 根据 Hitachi HD44780 数据手册，图 24，第 46 页

      // 初始状态为 8 位模式，尝试切换到 4 位模式
      send(0x03, FOUR_BITS);
      delayMicroseconds(4500); // 等待至少 4.1ms

      // 第二次尝试
      send ( 0x03, FOUR_BITS );
      delayMicroseconds(4500); // 等待至少 4.1ms

      // 第三次尝试
      send( 0x03, FOUR_BITS );
      delayMicroseconds(150);

      // 最终设置为 4 位接口
      send ( 0x02, FOUR_BITS );
   }
   else
   {
      // 根据 Hitachi HD44780 数据手册，图 23，第 45 页

      // 发送功能设置命令
      command(LCD_FUNCTIONSET | _displayfunction);
      delayMicroseconds(4500);  // 等待超过 4.1ms

      // 第二次尝试
      command(LCD_FUNCTIONSET | _displayfunction);
      delayMicroseconds(150);

      // 第三次尝试
      command(LCD_FUNCTIONSET | _displayfunction);
   }

   // 最后，设置行数、字体大小等
   command(LCD_FUNCTIONSET | _displayfunction);

   // 打开显示，光标和闪烁默认关闭
   _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
   display();

   // 清空 LCD
   clear();

   // 初始化为默认文本方向（适用于罗曼语系语言）
   _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
   // 设置输入模式
   command(LCD_ENTRYMODESET | _displaymode);

   backlight();

}

// 常用 LCD 命令
// ---------------------------------------------------------------------------
void LCD::clear()
{
   command(LCD_CLEARDISPLAY);             // 清屏，将光标位置设置为零
   delayMicroseconds(HOME_CLEAR_EXEC);    // 此命令需要时间
}

void LCD::home()
{
   command(LCD_RETURNHOME);             // 将光标位置设置为零
   delayMicroseconds(HOME_CLEAR_EXEC);  // 此命令需要时间
}

void LCD::setCursor(uint8_t col, uint8_t row)
{
   const byte row_offsetsDef[]   = { 0x00, 0x40, 0x14, 0x54 }; // 普通 LCD 的偏移量
   const byte row_offsetsLarge[] = { 0x00, 0x40, 0x10, 0x50 }; // 16x4 LCD 的偏移量

   if ( row >= _numlines )
   {
      row = _numlines-1;    // 行数从 0 开始
   }

   // 16x4 LCD 有特殊的内存映射布局
   // ----------------------------------------
   if ( _cols == 16 && _numlines == 4 )
   {
      command(LCD_SETDDRAMADDR | (col + row_offsetsLarge[row]));
   }
   else
   {
      command(LCD_SETDDRAMADDR | (col + row_offsetsDef[row]));
   }

}

// 开关显示
void LCD::noDisplay()
{
   _displaycontrol &= ~LCD_DISPLAYON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::display()
{
   _displaycontrol |= LCD_DISPLAYON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// 打开/关闭下划线光标
void LCD::noCursor()
{
   _displaycontrol &= ~LCD_CURSORON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LCD::cursor()
{
   _displaycontrol |= LCD_CURSORON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// 打开/关闭闪烁光标
void LCD::noBlink()
{
   _displaycontrol &= ~LCD_BLINKON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD::blink()
{
   _displaycontrol |= LCD_BLINKON;
   command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// 这些命令会滚动显示内容，而不改变 RAM
void LCD::scrollDisplayLeft(void)
{
   command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void LCD::scrollDisplayRight(void)
{
   command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// 文本从左到右流动
void LCD::leftToRight(void)
{
   _displaymode |= LCD_ENTRYLEFT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// 文本从右到左流动
void LCD::rightToLeft(void)
{
   _displaymode &= ~LCD_ENTRYLEFT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// 该方法将光标右移一个位置
void LCD::moveCursorRight(void)
{
   command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
}

// 该方法将光标左移一个位置
void LCD::moveCursorLeft(void)
{
   command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT);
}


// 此方法会将文本“右对齐”从光标开始
void LCD::autoscroll(void)
{
   _displaymode |= LCD_ENTRYSHIFTINCREMENT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// 此方法会将文本“左对齐”从光标开始
void LCD::noAutoscroll(void)
{
   _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
   command(LCD_ENTRYMODESET | _displaymode);
}

// 写入新字符到 CGRAM
void LCD::createChar(uint8_t location, uint8_t charmap[])
{
   location &= 0x7;            // 我们只有 8 个位置 0-7

   command(LCD_SETCGRAMADDR | (location << 3));
   delayMicroseconds(30);

   for (int i=0; i<8; i++)
   {
      write(charmap[i]);      // 调用虚拟写入方法
      delayMicroseconds(40);
   }
}

// 打开背光
void LCD::backlight ( void )
{
   setBacklight(255);
}

// 关闭背光
void LCD::noBacklight ( void )
{
   setBacklight(0);
}

// 完全打开 LCD（包括背光和 LCD）
void LCD::on ( void )
{
   display();
   backlight();
}

// 完全关闭 LCD（包括背光和 LCD）
void LCD::off ( void )
{
   noBacklight();
   noDisplay();
}

// 通用 LCD 命令 - 其余命令都使用的通用方法
// ---------------------------------------------------------------------------
void LCD::command(uint8_t value)
{
   send(value, COMMAND);
}

#if (ARDUINO <  100)
void LCD::write(uint8_t value)
{
   send(value, DATA);
}
#else
size_t LCD::write(uint8_t value)
{
   send(value, DATA);
   return 1;             // 假设成功
}
#endif
