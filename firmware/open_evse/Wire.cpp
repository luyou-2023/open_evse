/*
  TwoWire.cpp - TWI/I2C 库，适用于 Wiring 和 Arduino
  Copyright (c) 2006 Nicholas Zambetti.  版权所有。

  本库是自由软件；您可以根据 GNU 宽通用公共许可证的条款重新分发和/或
  修改它；该许可证由自由软件基金会发布；无论是版本 2.1，还是（由您选择）任何更新版本。

  本库按“原样”分发，希望它能有用，
  但不提供任何担保；甚至没有对适销性或特定用途的适用性担保。有关更多详情，
  请参阅 GNU 宽通用公共许可证。

  您应该已经与本库一起收到了一份 GNU 宽通用公共许可证；如果没有，请写信给自由软件基金会，
  地址是美国马萨诸塞州波士顿市富兰克林街 51 号，第五层，邮政编码：02110-1301。

  由 Todd Krein（todd@krein.org）修改于 2012 年，实现了重复启动功能。
*/

extern "C" {
  #include <stdlib.h>       // 标准库：包括动态内存分配、字符串操作等
  #include <string.h>       // 字符串操作相关函数
  #include <inttypes.h>     // 整数类型定义
  #include "./twi.h"       // TWI/I2C 接口头文件
}

#include "./Wire.h"         // 引入 Wire 库头文件，定义了 TwoWire 类

// 初始化类变量 ////////////////////////////////////////////////////////////

// 接收缓冲区，用于存储从从设备接收到的数据
uint8_t TwoWire::rxBuffer[BUFFER_LENGTH];
// 接收缓冲区的索引，指示当前处理的数据位置
uint8_t TwoWire::rxBufferIndex = 0;
// 接收缓冲区的长度，表示当前接收到的数据量
uint8_t TwoWire::rxBufferLength = 0;

// 发送地址，指向当前传输数据的目标设备地址
uint8_t TwoWire::txAddress = 0;
// 发送缓冲区，用于存储待发送的数据
uint8_t TwoWire::txBuffer[BUFFER_LENGTH];
// 发送缓冲区的索引，指示当前发送的数据位置
uint8_t TwoWire::txBufferIndex = 0;
// 发送缓冲区的长度，表示当前待发送的数据量
uint8_t TwoWire::txBufferLength = 0;

// 当前是否正在传输数据，1 表示传输中，0 表示空闲
uint8_t TwoWire::transmitting = 0;
// 用户定义的请求数据回调函数
void (*TwoWire::user_onRequest)(void);
// 用户定义的接收数据回调函数
void (*TwoWire::user_onReceive)(int);

// 构造函数 /////////////////////////////////////////////////////////////////

TwoWire::TwoWire()
{
  // 默认构造函数，无需执行任何操作
}

// 公共方法 /////////////////////////////////////////////////////////////////

void TwoWire::begin(void)
{
  // 初始化接收和发送缓冲区的索引和值
  rxBufferIndex = 0;
  rxBufferLength = 0;

  txBufferIndex = 0;
  txBufferLength = 0;

  // 初始化 TWI（I2C）接口
  twi_init();
}

void TwoWire::begin(uint8_t address)
{
  // 设置从设备地址
  twi_setAddress(address);
  // 附加从设备传输请求事件处理函数
  twi_attachSlaveTxEvent(onRequestService);
  // 附加从设备接收数据事件处理函数
  twi_attachSlaveRxEvent(onReceiveService);
  // 调用默认的 begin() 方法初始化
  begin();
}

void TwoWire::begin(int address)
{
  // 将整数类型的地址转换为 uint8_t 类型，并调用对应的 begin 方法
  begin((uint8_t)address);
}

void TwoWire::setClock(uint32_t frequency)
{
  // 设置 I2C 时钟频率，根据给定频率计算 TWBR 寄存器的值
  TWBR = ((F_CPU / frequency) - 16) / 2;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  // 限制请求数据的数量不能超过缓冲区长度
  if(quantity > BUFFER_LENGTH){
    quantity = BUFFER_LENGTH;
  }
  // 从指定地址读取数据并存入缓冲区
  uint8_t read = twi_readFrom(address, rxBuffer, quantity, sendStop);
  // 设置接收缓冲区的索引和值
  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity)
{
  // 默认调用 requestFrom 方法，并设置发送停止信号为 true
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity)
{
  // 将整数类型的地址和数量转换为 uint8_t 类型，并调用对应的 requestFrom 方法
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t TwoWire::requestFrom(int address, int quantity, int sendStop)
{
  // 将整数类型的地址、数量和发送停止信号转换为 uint8_t 类型，并调用对应的 requestFrom 方法
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

void TwoWire::beginTransmission(uint8_t address)
{
  // 设置当前传输状态为正在传输
  transmitting = 1;
  // 设置目标从设备地址
  txAddress = address;
  // 重置发送缓冲区的索引和值
  txBufferIndex = 0;
  txBufferLength = 0;
}

void TwoWire::beginTransmission(int address)
{
  // 将整数类型的地址转换为 uint8_t 类型，并调用对应的 beginTransmission 方法
  beginTransmission((uint8_t)address);
}

//
//	原本，'endTransmission' 是一个 f(void) 函数。
//	它已被修改为接受一个参数，用来指示是否在总线上执行 STOP 操作。
//	调用 endTransmission(false) 允许草图执行重复开始操作。
//
//	警告：在库中没有任何内容跟踪总线操作是否已正确以 STOP 结束。
//	如果没有调用 endTransmission(true)，则可能会将总线留在挂起状态。
//	某些 I2C 设备如果没有看到 STOP，会表现出异常行为。
//
uint8_t TwoWire::endTransmission(uint8_t sendStop)
{
  // 传输缓冲区（阻塞模式）
  int8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, sendStop);
  // 重置传输缓冲区的索引
  txBufferIndex = 0;
  txBufferLength = 0;
  // 表示我们已经完成了传输
  transmitting = 0;
  return ret;
}

// 提供与原始定义的兼容性，并保持 endTransmission 的预期行为
//
uint8_t TwoWire::endTransmission(void)
{
  return endTransmission(true); // 默认调用 endTransmission(true)
}

// 必须在以下情况下调用：
// - 从设备的传输回调中
// - 或者在调用 beginTransmission(address) 后
size_t TwoWire::write(uint8_t data)
{
  if(transmitting){
    // 在主传输模式下
    // 如果缓冲区已满，则无需继续写入
    if(txBufferLength >= BUFFER_LENGTH){
      setWriteError(); // 设置写入错误
      return 0;
    }
    // 将字节放入传输缓冲区
    txBuffer[txBufferIndex] = data;
    ++txBufferIndex;
    // 更新缓冲区中的字节数
    txBufferLength = txBufferIndex;
  }else{
    // 在从设备发送模式下
    // 回复主设备
    twi_transmit(&data, 1);
  }
  return 1;
}

// 必须在以下情况下调用：
// - 从设备的传输回调中
// - 或者在调用 beginTransmission(address) 后
size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
  if(transmitting){
    // 在主传输模式下
    for(size_t i = 0; i < quantity; ++i){
      write(data[i]);
    }
  }else{
    // 在从设备发送模式下
    // 回复主设备
    twi_transmit(data, quantity);
  }
  return quantity;
}

// 必须在以下情况下调用：
// - 从设备的接收回调中
// - 或者在调用 requestFrom(address, numBytes) 后
int TwoWire::available(void)
{
  // 返回当前接收缓冲区中剩余的字节数
  return rxBufferLength - rxBufferIndex;
}

// 必须在以下情况下调用：
// - 从设备的接收回调中
// - 或者在调用 requestFrom(address, numBytes) 后
int TwoWire::read(void)
{
  int value = -1;

  // 获取每次调用时的下一个字节
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}

// 必须在以下情况下调用：
// - 从设备的接收事件回调中
// - 或者在调用 requestFrom(address, numBytes) 后
int TwoWire::peek(void)
{
  int value = -1;

  // 如果接收缓冲区中还有数据，获取下一个字节但不改变缓冲区索引
  if(rxBufferIndex < rxBufferLength){
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void TwoWire::flush(void)
{
  // XXX: 这个功能尚未实现。
}

// 内部函数，在数据接收到时被调用
void TwoWire::onReceiveService(uint8_t* inBytes, int numBytes)
{
  // 如果用户没有注册回调函数，则不做任何处理
  if(!user_onReceive){
    return;
  }
  // 如果接收缓冲区正被主设备的 requestFrom() 操作使用，且还没有读取完数据，则不处理
  // 这样可能会丢失数据，但允许稍微的“愚蠢”操作
  // 意味着用户可能还没有完全读取主设备 requestFrom() 请求的数据
  if(rxBufferIndex < rxBufferLength){
    return;
  }
  // 将 twi 接收缓冲区中的数据复制到本地读取缓冲区
  // 这样可以实现并行的读取操作
  for(uint8_t i = 0; i < numBytes; ++i){
    rxBuffer[i] = inBytes[i];
  }
  // 设置接收缓冲区迭代器变量
  rxBufferIndex = 0;
  rxBufferLength = numBytes;
  // 提醒用户程序数据已接收
  user_onReceive(numBytes);
}

// 内部函数，在请求数据时被调用
void TwoWire::onRequestService(void)
{
  // 如果用户没有注册回调函数，则不做任何处理
  if(!user_onRequest){
    return;
  }
  // 重置传输缓冲区的迭代器变量
  // !!! 这将会中断任何挂起的主设备 sendTo() 操作
  txBufferIndex = 0;
  txBufferLength = 0;
  // 提醒用户程序可以进行数据发送
  user_onRequest();
}

// 设置在从设备写入时调用的函数
void TwoWire::onReceive( void (*function)(int) )
{
  user_onReceive = function;
}

// 设置在从设备读取时调用的函数
void TwoWire::onRequest( void (*function)(void) )
{
  user_onRequest = function;
}

// 预实例化对象 //////////////////////////////////////////////////////
TwoWire Wire = TwoWire();  // 创建一个 TwoWire 实例对象，Wire

