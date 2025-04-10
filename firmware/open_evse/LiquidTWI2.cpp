#include "open_evse.h"

/*
  LiquidTWI2 高性能 I2C LCD 驱动器，支持 MCP23008 和 MCP23017
  由 Sam C. Lin / http://www.lincomatic.com 修改
  原始代码来自
   LiquidTWI by Matt Falcon (FalconFour) / http://falconfour.com
   逻辑借鉴了 Adafruit RGB LCD Shield 库
   Panelolu2 支持由 Tony Lock 提供 / http://blog.think3dprint3d.com
   Nick Sayer 的改进 / https://github.com/nsayer

  兼容 Adafruit I2C LCD 背包 (MCP23008) 和
  Adafruit RGB LCD Shield
*/
#if defined(MCP23017) || defined(MCP23008)
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#if defined (__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || (__AVR_ATtiny2313__)
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include "./Wire.h"
#endif
#if defined(ARDUINO) && (ARDUINO >= 100) // 对应 Arduino 环境
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


// MCP23017 - Adafruit RGB LCD Shield
// burstBits 函数的位模式如下：
//
//  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
//  RS RW EN D4 D5 D6 D7 LB LG LR BZ B4 B3 B2 B1 B0
//  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
#define M17_BIT_RS 0x8000
#define M17_BIT_RW 0x4000
#define M17_BIT_EN 0x2000
#define M17_BIT_D4 0x1000
#define M17_BIT_D5 0x0800
#define M17_BIT_D6 0x0400
#define M17_BIT_D7 0x0200
#define M17_BIT_LB 0x0100
#define M17_BIT_LG 0x0080
#define M17_BIT_LR 0x0040
#define M17_BIT_BZ 0x0020 // 在此引脚添加了蜂鸣器
#define M17_BIT_B4 0x0010
#define M17_BIT_B3 0x0008
#define M17_BIT_B2 0x0004
#define M17_BIT_B1 0x0002
#define M17_BIT_B0 0x0001

// 向 I2C 总线发送字节
static inline void wiresend(uint8_t x) {
#if ARDUINO >= 100
  Wire.write((uint8_t)x);
#else
  Wire.send(x);
#endif
}

// 从 I2C 总线接收字节
static inline uint8_t wirerecv(void) {
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}



// 当显示器上电时，它的配置如下：
//
// 1. 清除显示
// 2. 设置功能：
//    DL = 0; 4 位数据接口
//    N = 0; 1 行显示
//    F = 0; 5x8 点阵字符字体
// 3. 显示开/关控制：
//    D = 0; 显示关闭
//    C = 0; 光标关闭
//    B = 0; 闪烁关闭
// 4. 输入模式设置：
//    I/D = 1; 增量加 1
//    S = 0; 无位移
//
// 然而需要注意的是，复位 Arduino 时 LCD 不会被重置，所以我们
// 不能假设它在初始化时处于该状态（并且 LiquidTWI2 构造函数
// 被调用）。这就是为什么我们将初始化命令保存到当 sketch 调用
// begin() 时执行，除了配置扩展器，这是任何设置所需要的。

LiquidTWI2::LiquidTWI2(uint8_t i2cAddr,uint8_t detectDevice, uint8_t backlightInverted) {
  // 如果 detectDevice 不为 0，将 _deviceDetected 设置为 2，表示我们应该
  // 在 begin() 中扫描设备
#ifdef DETECT_DEVICE
  _deviceDetected = detectDevice ? 2 : 1;
#endif

  _backlightInverted = backlightInverted;

  // 将 i2cAddr 转换为我们的内部类状态
  _i2cAddr = i2cAddr;
  _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS; // 如果他们忘记调用 begin()，至少我们有一个默认值
#if defined(MCP23017)&&defined(MCP23008)
  _mcpType = DEFAULT_TYPE; // 默认类型
#endif
}

// 初始化 LCD
void LiquidTWI2::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  // 根据数据手册，显示器上电后需要等待至少 40ms
  // 然后再发送命令。Arduino 的开机时间远远早于 4.5V，因此我们等待 50 毫秒
  /* 这里不需要，因为 open_evse.ino 已经做了
  delay(50);

  Wire.begin();
  */

  uint8_t result;
#if defined(MCP23017)&&defined(MCP23008)
  if (_mcpType == LTI_TYPE_MCP23017) {
#endif
#ifdef MCP23017
    /* 不需要这部分代码
    // 设置默认值！
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_IODIRA);
    wiresend(0xFF);  // 设置 A 端口所有引脚为输入
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_IODIRB);
    wiresend(0xFF);  // 设置 B 端口所有引脚为输入
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif
    */

    // 现在设置输入/输出引脚
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_IODIRA);
    wiresend(0x1F); // 设置按钮输入，其它为输出
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif

    // 设置按钮的上拉电阻
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_GPPUA);
    wiresend(0x1F);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif

    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(MCP23017_IODIRB);
    wiresend(0x00); // 设置 B 端口所有引脚为输出
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif
#endif // MCP23017
#if defined(MCP23017)&&defined(MCP23008)
  }
  else { // MCP23008
#endif
#ifdef MCP23008
    // 初始化 GPIO 扩展器的头部
    Wire.beginTransmission(MCP23008_ADDRESS | _i2cAddr);
    wiresend(MCP23008_IODIR);
    wiresend(0xFF);  // 设置所有引脚为输入
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    wiresend(0x00);
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif

    // 设置 GPIO 扩展器的 I/O 方向为输出
    Wire.beginTransmission(MCP23008_ADDRESS | _i2cAddr);
    wiresend(MCP23008_IODIR);
    wiresend(0x00); // 所有引脚为输出
    result = Wire.endTransmission();
#ifdef DETECT_DEVICE
    if (result) {
        if (_deviceDetected == 2) {
          _deviceDetected = 0;
          return;
        }
    }
#endif
#endif // MCP23008
#if defined(MCP23017)&&defined(MCP23008)
  }
#endif


#ifdef DETECT_DEVICE
  // 如果到目前为止没有失败，那么我们通过检测
  if (_deviceDetected == 2) _deviceDetected = 1;
#endif

  if (lines > 1) {
    _displayfunction |= LCD_2LINE;  // 如果行数大于1，设置为2行显示
  }
  _numlines = lines;  // 设置行数
  _currline = 0;  // 当前行初始化为0

  // 对于一些单行显示器，可以选择10像素高的字体
  if ((dotsize != 0) && (lines == 1)) {
    _displayfunction |= LCD_5x10DOTS;  // 设置为5x10点阵字体
  }

  // 将LCD切换到4位模式
  // 使用一个非标准命令开始，确保我们正在以4位模式进行通信
  // 根据LCD数据手册，第一条命令是单个4位的突发，0011。
  //-----
  // 我们不能假设LCD面板与Arduino同时供电，因此我们必须执行一个软件重置，
  // 参见HD44780数据手册第45页。（kch）
  //-----
#if defined(MCP23017)&&defined(MCP23008)
  if (_mcpType == LTI_TYPE_MCP23017) {
#endif // defined(MCP23017)&&defined(MCP23008)
#ifdef MCP23017
    _backlightBits = 0; // 所有背光LED开启

    // MCP23017的突发位模式
    //
    //  B7 B6 B5 B4 B3 B2 B1 B0 A7 A6 A5 A4 A3 A2 A1 A0 - MCP23017
    //  15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
    //  RS RW EN D4 D5 D6 D7 B  G  R     B4 B3 B2 B1 B0
    for (uint8_t i=0;i < 3;i++) {
      burstBits8b((M17_BIT_EN|M17_BIT_D5|M17_BIT_D4) >> 8);
      burstBits8b((M17_BIT_D5|M17_BIT_D4) >> 8);
    }
    burstBits8b((M17_BIT_EN|M17_BIT_D5) >> 8);
    burstBits8b(M17_BIT_D5 >> 8);
#endif // MCP23017
#if defined(MCP23017)&&defined(MCP23008)
  }
  else {
#endif // defined(MCP23017)&&defined(MCP23008)
#ifdef MCP23008
    // MCP23008的突发位模式
    //
    //  7   6   5   4   3   2   1   0
    // LT  D7  D6  D5  D4  EN  RS  n/c
    //-----
    burstBits8(B10011100); // 发送LITE D4 D5高电平并使能
    burstBits8(B10011000); // 发送LITE D4 D5高电平并禁用
    burstBits8(B10011100); // 再次发送
    burstBits8(B10011000); // 再次发送
    burstBits8(B10011100); // 再次发送两次
    burstBits8(B10011000); // 再次发送
    burstBits8(B10010100); // 发送D4低电平并使LITE D5高电平并使能
    burstBits8(B10010000); // 发送D4低电平并使LITE D5高电平并禁用
#endif // MCP23008
#if defined(MCP23017)&&defined(MCP23008)
  }
#endif

  delay(5); // 这不应该是必须的，但有时候16MHz的速度太快

  command(LCD_FUNCTIONSET | _displayfunction); // 发送功能设置命令 0010NF00 (N=行数, F=字体)
  delay(5); // 为了安全起见...
  command(LCD_FUNCTIONSET | _displayfunction); // 再次发送一次
  delay(5); // 完成！

  // 使用我们的默认设置打开LCD显示。由于这些库似乎使用个人偏好设置，我喜欢启用光标。
  _displaycontrol = (LCD_DISPLAYON|LCD_BACKLIGHT);
  display();
  // 清屏
  clear();

  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  // 设置输入模式
  command(LCD_ENTRYMODESET | _displaymode);
}

/********** 高级命令，供用户使用！ */
void LiquidTWI2::clear()
{
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  command(LCD_CLEARDISPLAY);  // 清空显示，设置光标位置为零
  delayMicroseconds(2000);  // 此命令需要较长时间！
}

void LiquidTWI2::home()
{
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  command(LCD_RETURNHOME);  // 将光标位置设置为零
  delayMicroseconds(2000);  // 此命令需要较长时间！
}

void LiquidTWI2::setCursor(uint8_t col, uint8_t row)
{
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };  // 每行的偏移量
  if ( row > _numlines ) row = _numlines - 1;    // 行数从0开始
  command(LCD_SETDDRAMADDR | (col + row_offsets[row]));  // 设置DDRAM地址
}

// 快速打开/关闭显示
void LiquidTWI2::noDisplay() {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  _displaycontrol &= ~LCD_DISPLAYON;  // 关闭显示
  command(LCD_DISPLAYCONTROL | _displaycontrol);  // 发送控制命令
}
void LiquidTWI2::display() {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  _displaycontrol |= LCD_DISPLAYON;  // 打开显示
  command(LCD_DISPLAYCONTROL | _displaycontrol);  // 发送控制命令
}

// 打开/关闭下划线光标
void LiquidTWI2::noCursor() {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  _displaycontrol &= ~LCD_CURSORON;  // 关闭光标
  command(LCD_DISPLAYCONTROL | _displaycontrol);  // 发送控制命令
}
void LiquidTWI2::cursor() {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果设备未检测到，则返回
#endif
  _displaycontrol |= LCD_CURSORON;  // 打开光标
  command(LCD_DISPLAYCONTROL | _displaycontrol);  // 发送控制命令
}

// 开关闪烁光标
void LiquidTWI2::noBlink() {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  _displaycontrol &= ~LCD_BLINKON;  // 关闭闪烁
  command(LCD_DISPLAYCONTROL | _displaycontrol);  // 发送命令
}

void LiquidTWI2::blink() {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  _displaycontrol |= LCD_BLINKON;  // 开启闪烁
  command(LCD_DISPLAYCONTROL | _displaycontrol);  // 发送命令
}

// 这些命令可以滚动显示内容，不改变RAM中的内容
void LiquidTWI2::scrollDisplayLeft(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);  // 向左滚动
}

void LiquidTWI2::scrollDisplayRight(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);  // 向右滚动
}

// 设置文本从左到右流动
void LiquidTWI2::leftToRight(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  _displaymode |= LCD_ENTRYLEFT;  // 设置为从左到右显示
  command(LCD_ENTRYMODESET | _displaymode);  // 发送命令
}

// 设置文本从右到左流动
void LiquidTWI2::rightToLeft(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  _displaymode &= ~LCD_ENTRYLEFT;  // 设置为从右到左显示
  command(LCD_ENTRYMODESET | _displaymode);  // 发送命令
}

// 从光标位置开始，文本自动滚动
void LiquidTWI2::autoscroll(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;  // 启用自动滚动
  command(LCD_ENTRYMODESET | _displaymode);  // 发送命令
}

// 关闭自动滚动
void LiquidTWI2::noAutoscroll(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;  // 禁用自动滚动
  command(LCD_ENTRYMODESET | _displaymode);  // 发送命令
}

// 允许我们填充前8个CGRAM位置，使用自定义字符
void LiquidTWI2::createChar(uint8_t location, uint8_t charmap[]) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  location &= 0x7;  // 只使用位置0-7
  command(LCD_SETCGRAMADDR | (location << 3));  // 设置CGRAM地址
  for (int i = 0; i < 8; i++) {
    write(charmap[i]);  // 写入自定义字符数据
  }
}

/*********** 中级命令，发送数据/命令 ***********/
inline void LiquidTWI2::command(uint8_t value) {
  send(value, LOW);  // 发送命令
}
#if defined(ARDUINO) && (ARDUINO >= 100) //scl
inline size_t LiquidTWI2::write(uint8_t value) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return 1;  // 如果没有检测到设备，则返回1
#endif
  send(value, HIGH);  // 发送数据
  return 1;
}
#else
inline void LiquidTWI2::write(uint8_t value) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  send(value, HIGH);  // 发送数据
}
#endif

/************ 低级数据推送命令 **********/
#ifdef MCP23017
// 读取按钮状态
uint8_t LiquidTWI2::readButtons(void) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return 0;  // 如果没有检测到设备，则返回0
#endif
  Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
  wiresend(MCP23017_GPIOA);
  Wire.endTransmission();

  Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
  return ~wirerecv() & ALL_BUTTON_BITS;  // 返回按钮状态
}
#endif // MCP23017

// 设置背光，如果使用了LCD背板
void LiquidTWI2::setBacklight(uint8_t status) {
#ifdef DETECT_DEVICE
  if (!_deviceDetected) return;  // 如果没有检测到设备，则返回
#endif
  if (_backlightInverted) status ^= 0x7;  // 反转背光状态
#if defined(MCP23017) && defined(MCP23008)
  if (_mcpType == LTI_TYPE_MCP23017) {
#endif
#ifdef MCP23017
  // LED亮起时，位被清除
  _backlightBits = M17_BIT_LB | M17_BIT_LG | M17_BIT_LR;  // 所有背光关闭
  if (status & RED) _backlightBits &= ~M17_BIT_LR;  // 红色打开
  if (status & GREEN) _backlightBits &= ~M17_BIT_LG;  // 绿色打开
  if (status & BLUE) _backlightBits &= ~M17_BIT_LB;  // 蓝色打开

  burstBits16(_backlightBits);  // 发送背光状态
#endif // MCP23017
#if defined(MCP23017) && defined(MCP23008)
  }
  else {
#endif
#ifdef MCP23008
  bitWrite(_displaycontrol, 3, status);  // 设置背光标志
  burstBits8((_displaycontrol & LCD_BACKLIGHT) ? 0x80 : 0x00);  // 发送背光状态
#endif // MCP23008
#if defined(MCP23017) && defined(MCP23008)
  }
#endif
}

// 写入命令或数据，通过I2C将数据传送给扩展器
void LiquidTWI2::send(uint8_t value, uint8_t mode) {
#if defined(MCP23017) && defined(MCP23008)
  if (_mcpType == LTI_TYPE_MCP23017) {
#endif
#ifdef MCP23017
  // 高速传输
  uint8_t buf = _backlightBits >> 8;
  // 发送高4位
  if (value & 0x10) buf |= M17_BIT_D4 >> 8;
  if (value & 0x20) buf |= M17_BIT_D5 >> 8;
  if (value & 0x40) buf |= M17_BIT_D6 >> 8;
  if (value & 0x80) buf |= M17_BIT_D7 >> 8;

  if (mode) buf |= (M17_BIT_RS | M17_BIT_EN) >> 8;  // 设置RS和EN
  else buf |= M17_BIT_EN >> 8;  // 只设置EN

  burstBits8b(buf);  // 发送数据

  buf &= ~(M17_BIT_EN >> 8);  // 关闭EN
  burstBits8b(buf);  // 重新发送

  // 发送低4位
  buf = _backlightBits >> 8;
  if (value & 0x01) buf |= M17_BIT_D4 >> 8;
  if (value & 0x02) buf |= M17_BIT_D5 >> 8;
  if (value & 0x04) buf |= M17_BIT_D6 >> 8;
  if (value & 0x08) buf |= M17_BIT_D7 >> 8;

  if (mode) buf |= (M17_BIT_RS | M17_BIT_EN) >> 8;  // 设置RS和EN
  else buf |= M17_BIT_EN >> 8;  // 只设置EN

  burstBits8b(buf);  // 发送数据

  buf &= ~(M17_BIT_EN >> 8);  // 关闭EN
  burstBits8b(buf);  // 重新发送
#endif // MCP23017
#if defined(MCP23017) && defined(MCP23008)
  }
  else {
#endif
#ifdef MCP23008
  byte buf;
  // 发送高4位
  buf = (value & B11110000) >> 1;  // 提取高4位并左移到数据引脚
  if (mode) buf |= 3 << 1;  // 启用RS和EN
  else buf |= 2 << 1;  // 仅启用EN

  buf |= (_displaycontrol & LCD_BACKLIGHT) ? 0x80 : 0x00;  // 使用DISPLAYCONTROL命令来掩码背光位
  burstBits8(buf);  // 发送数据

  buf &= ~(1 << 2);  // 关闭EN
  burstBits8(buf);  // 重新发送

  // 发送低4位
  buf = (value & B1111) << 3;  // 提取低4位并左移到数据引脚
  if (mode) buf |= 3 << 1;  // 启用RS和EN
  else buf |= 2 << 1;  // 仅启用EN

  buf |= (_displaycontrol & LCD_BACKLIGHT) ? 0x80 : 0x00;  // 使用DISPLAYCONTROL命令来掩码背光位
  burstBits8(buf);  // 发送数据
  buf &= ~(1 << 2);  // 关闭EN
  burstBits8(buf);  // 重新发送
#endif // MCP23008
#if defined(MCP23017) && defined(MCP23008)
  }
#endif
}

#ifdef MCP23017
// 值字节顺序为 BA
void LiquidTWI2::burstBits16(uint16_t value) {
  // 我们在需要时使用此方法将位突发发送到 GPIO 芯片。避免重复的代码。
  Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
  wiresend(MCP23017_GPIOA);
  wiresend(value & 0xFF); // 发送 A 位
  wiresend(value >> 8);   // 发送 B 位
  while(Wire.endTransmission());
}

/*
void LiquidTWI2::burstBits8a(uint8_t value) {
  // 我们在需要时使用此方法将位突发发送到 GPIO 芯片。避免重复的代码。
  Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
  wiresend(MCP23017_GPIOA);
  wiresend(value); // 最后的位已经压缩，我们完成了。
  while(Wire.endTransmission());
}
*/
void LiquidTWI2::burstBits8b(uint8_t value) {
  // 我们在需要时使用此方法将位突发发送到 GPIO 芯片。避免重复的代码。
  Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
  wiresend(MCP23017_GPIOB);
  wiresend(value); // 最后的位已经压缩，我们完成了。
  while(Wire.endTransmission());
}
#endif // MCP23017
#ifdef MCP23008
void LiquidTWI2::burstBits8(uint8_t value) {
  // 我们在需要时使用此方法将位突发发送到 GPIO 芯片。避免重复的代码。
  Wire.beginTransmission(MCP23008_ADDRESS | _i2cAddr);
  wiresend(MCP23008_GPIO);
  wiresend(value); // 最后的位已经压缩，我们完成了。
  while(Wire.endTransmission());
}
#endif // MCP23008

#if defined(MCP23017)
// 直接访问寄存器进行中断设置和读取，同时也用于使用蜂鸣器引脚的音调功能
uint8_t LiquidTWI2::readRegister(uint8_t reg) {
  // 读取一个寄存器
  Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
  wiresend(reg);
  Wire.endTransmission();

  Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
  return wirerecv();
}

// 设置寄存器
void LiquidTWI2::setRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
    wiresend(reg);
    wiresend(value);
    Wire.endTransmission();
}

// 按指定频率（Hz）和持续时间（毫秒）周期性地控制蜂鸣器引脚
// 注意：一个 16MHz Arduino 上的 100kHz TWI/I2C 总线最大只能达到约 1500Hz 的频率
void LiquidTWI2::buzz(long duration, uint16_t freq) {
  int currentRegister = 0;

  // 读取 gpio 寄存器
  Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
  wiresend(MCP23017_GPIOA);
  Wire.endTransmission();
  Wire.requestFrom(MCP23017_ADDRESS | _i2cAddr, 1);
  currentRegister = wirerecv();

  duration *=1000; // 将毫秒转换为微秒
  unsigned long cycletime = 1000000UL / freq; // 周期时间（微秒）
  unsigned long cycles = (unsigned long)duration / cycletime;
  unsigned long ontime;
  while (cycles-- > 0)
  {
    ontime = micros();
        Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
        wiresend(MCP23017_GPIOA);
        wiresend(currentRegister |= M17_BIT_BZ);
        while(Wire.endTransmission());
    while((long)(ontime + (cycletime/2) - micros()) > 0);
        Wire.beginTransmission(MCP23017_ADDRESS | _i2cAddr);
        wiresend(MCP23017_GPIOA);
        wiresend(currentRegister &= ~M17_BIT_BZ);
        while(Wire.endTransmission());
    while((long)(ontime + cycletime - micros()) > 0);
   }
}
#endif //MCP23017

#endif // defined(MCP23017) || defined(MCP23008)
