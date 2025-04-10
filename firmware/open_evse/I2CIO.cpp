// ---------------------------------------------------------------------------
// Created by Francisco Malpartida on 20/08/11.
// Copyright 2011 - Under creative commons license 3.0:
//        Attribution-ShareAlike CC BY-SA
//
// 本软件按“原样”提供，不提供技术支持，也没有任何形式的保证，
// 无论是明示还是暗示，均不对其在任何目的下的有效性负责。
//
// 线程安全：否
// 可扩展：是
//
// @file I2CIO.h
// 该文件实现了一个基础的IO库，使用PCF8574 I2C IO扩展芯片。
//
// @brief
// 实现一个基础的IO库，驱动PCF8574* I2C IO扩展ASIC。
// 该库实现了基本的IO方法，用于配置IO引脚方向、读取和写入uint8_t操作，
// 以及基本的引脚电平操作，设置或读取特定的IO端口。
//
//
// @version API 1.0.0
//
// @author F. Malpartida - fmalpartida@gmail.com
// ---------------------------------------------------------------------------

#if (ARDUINO <  100)
#include <WProgram.h>  // 对于100版本以下的Arduino，包含WProgram.h
#else
#include <Arduino.h>   // 对于100及以上版本的Arduino，包含Arduino.h
#endif

#include <inttypes.h>  // 包含uint8_t和其他整数类型

#include "./Wire.h"     // 引入Wire库用于I2C通信
#include "./I2CIO.h"    // 引入I2CIO头文件，定义IO库

// 类的成员变量
// ---------------------------------------------------------------------------

// 构造函数，用于初始化I2CIO对象
// ---------------------------------------------------------------------------
I2CIO::I2CIO ( )
{
   _i2cAddr     = 0x0;     // 初始化I2C地址为0x0
   _dirMask     = 0xFF;    // 初始化所有引脚为输入
   _shadow      = 0x0;     // 初始化没有值设置
   _initialised = false;   // 初始化状态为未初始化
}

// 公共方法
// ---------------------------------------------------------------------------

//
// begin
// 初始化I2CIO对象，设置I2C地址
int I2CIO::begin (  uint8_t i2cAddr )
{
   _i2cAddr = i2cAddr;   // 设置I2C地址

   // 不需要 - open_evse.ino 中已经做了初始化 Wire.begin();

   _initialised = Wire.requestFrom ( _i2cAddr, (uint8_t)1 ); // 请求从设备读取1个字节

#if (ARDUINO <  100)
   _shadow = Wire.receive ();  // 获取接收到的数据字节
#else
   _shadow = Wire.read (); // 获取接收到的数据字节，移除不需要的读取
#endif

   return ( _initialised );  // 返回初始化状态
}

//
// pinMode
// 设置指定引脚的输入输出模式
void I2CIO::pinMode ( uint8_t pin, uint8_t dir )
{
   if ( _initialised )   // 检查是否已初始化
   {
      if ( OUTPUT == dir )  // 如果方向为输出
      {
         _dirMask &= ~( 1 << pin );  // 设置为输出
      }
      else
      {
         _dirMask |= ( 1 << pin );  // 设置为输入
      }
   }
}

//
// portMode
// 设置整个端口的输入输出模式
void I2CIO::portMode ( uint8_t dir )
{
   if ( _initialised )  // 检查是否已初始化
   {
      if ( dir == INPUT )  // 如果是输入模式
      {
         _dirMask = 0xFF;  // 设置所有引脚为输入
      }
      else
      {
         _dirMask = 0x00;  // 设置所有引脚为输出
      }
   }
}

//
// read
// 读取I2C设备的值
uint8_t I2CIO::read ( void )
{
   uint8_t retVal = 0;

   if ( _initialised )   // 检查是否已初始化
   {
      Wire.requestFrom ( _i2cAddr, (uint8_t)1 );  // 请求读取1个字节
#if (ARDUINO <  100)
      retVal = ( _dirMask & Wire.receive ( ) );  // 读取数据并进行掩码操作
#else
      retVal = ( _dirMask & Wire.read ( ) );     // 读取数据并进行掩码操作
#endif
   }
   return ( retVal );   // 返回读取结果
}

//
// write
// 向I2C设备写入数据
int I2CIO::write ( uint8_t value )
{
   int status = 0;

   if ( _initialised )  // 检查是否已初始化
   {
      // 只写入已经初始化为输出的端口，并更新设备的输出阴影值
      _shadow = ( value & ~(_dirMask) );  // 只保留输出端口的值

      Wire.beginTransmission ( _i2cAddr );  // 开始I2C通信
#if (ARDUINO <  100)
      Wire.send ( _shadow );  // 发送数据字节
#else
      Wire.write ( _shadow );  // 发送数据字节
#endif
      status = Wire.endTransmission ();  // 结束I2C通信
   }
   return ( (status == 0) );  // 返回通信是否成功
}

//
// digitalRead
// 读取指定引脚的数字状态
uint8_t I2CIO::digitalRead ( uint8_t pin )
{
   uint8_t pinVal = 0;

   // 检查是否已初始化，并确保引脚在设备的范围内
   if ( ( _initialised ) && ( pin <= 7 ) )
   {
      // 清除非输入端口的值并获取引脚值
      pinVal = this->read() & _dirMask;  // 读取数据并应用掩码
      pinVal = ( pinVal >> pin ) & 0x01; // 获取指定引脚的值
   }
   return (pinVal);  // 返回引脚的值
}

//
// digitalWrite
// 设置指定引脚的数字电平
int I2CIO::digitalWrite ( uint8_t pin, uint8_t level )
{
   uint8_t writeVal;
   int status = 0;

   // 检查是否已初始化，并确保引脚在设备的范围内
   if ( ( _initialised ) && ( pin <= 7 ) )
   {
      // 只有在端口被配置为输出时，才会写入HIGH状态
      writeVal = ( 1 << pin ) & ~_dirMask;  // 创建写入值
      if ( level == HIGH )  // 如果设置为高电平
      {
         _shadow |= writeVal;  // 更新阴影值
      }
      else
      {
         _shadow &= ~writeVal;  // 更新阴影值
      }
      status = this->write ( _shadow );  // 写入更新后的值
   }
   return ( status );  // 返回写入状态
}

// 私有方法
// ---------------------------------------------------------------------------
