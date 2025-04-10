/*
  twi.c - TWI/I2C 库，用于 Wiring 和 Arduino
  Copyright (c) 2006 Nicholas Zambetti.  保留所有权利。

  本库是自由软件；你可以根据 GNU 较宽松公共许可证的条款重新发布和/或
  修改它；许可证的版本为 2.1，或（根据你的选择）任何更新的版本。

  本库的分发是为了有用，但不提供任何保证；甚至不包括适销性或特定用途的适用性暗示。详细信息请参见 GNU 较宽松公共许可证。

  你应该已经收到了 GNU 较宽松公共许可证的副本；如果没有，写信给 Free Software Foundation，Inc.，地址是 51 Franklin St, Fifth Floor, Boston, MA 02110-1301  美国。

  2012 年由 Todd Krein 修改（todd@krein.org），实现了重复启动功能。
*/

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  // 清除特定位
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   // 设置特定位
#endif

#include "./twi.h"

#define false 0
#define true (!false)

static volatile uint8_t twi_state;                  // TWI 状态
static volatile uint8_t twi_slarw;                  // 读取或写入操作标记
static volatile uint8_t twi_sendStop;               // 是否在事务结束时发送停止信号
static volatile uint8_t twi_inRepStart;            // 是否处于重复启动中

static void (*twi_onSlaveTransmit)(void);           // 从设备发送数据时调用的回调函数
static void (*twi_onSlaveReceive)(uint8_t*, int);   // 从设备接收数据时调用的回调函数

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];  // 主设备数据缓冲区
static volatile uint8_t twi_masterBufferIndex;      // 主设备数据缓冲区索引
static volatile uint8_t twi_masterBufferLength;     // 主设备数据缓冲区长度

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];     // 传输数据缓冲区
static volatile uint8_t twi_txBufferIndex;          // 传输数据缓冲区索引
static volatile uint8_t twi_txBufferLength;         // 传输数据缓冲区长度

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];     // 接收数据缓冲区
static volatile uint8_t twi_rxBufferIndex;          // 接收数据缓冲区索引

static volatile uint8_t twi_error;                  // 错误状态

/*
 * 函数 twi_init
 * 描述     初始化 TWI 引脚并设置 TWI 位速率
 * 输入     无
 * 输出     无
 */
void twi_init(void)
{
  // 初始化 TWI 状态
  twi_state = TWI_READY;
  twi_sendStop = true;		// 默认值为 true
  twi_inRepStart = false;

  // 激活 TWI 的内部上拉电阻
  PORTC |= (1 << PC4) | (1 << PC5);  // 配置 SDA 和 SCL 引脚为输入并启用上拉电阻

  // 初始化 TWI 预分频器和比特率
  cbi(TWSR, TWPS0);  // 清除 TWI 预分频器设置
  cbi(TWSR, TWPS1);
  TWBR = ((F_CPU / TWI_FREQ) - 16) / 2;  // 设置 TWI 比特率寄存器 TWBR

  /* 根据 Atmega128 手册公式计算 TWI 比特率：
  SCL 频率 = CPU 时钟频率 / (16 + (2 * TWBR))
  注意：在主机模式下，TWBR 应该为 10 或更高
  对于 16 MHz 的 Wiring 板，设置 100 kHz 的 TWI */

  // 启用 TWI 模块、应答和 TWI 中断
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

/*
 * 函数 twi_setAddress
 * 描述     设置从设备地址并启用中断
 * 输入     地址：7 位 I2C 设备地址
 * 输出     无
 */
void twi_setAddress(uint8_t address)
{
  // 设置 TWI 从设备地址（跳过 TWGCE 位）
  TWAR = address << 1;  // 设置从设备地址
}

/*
 * 函数 twi_readFrom
 * 描述     尝试成为 TWI 总线主设备并从总线上读取一系列字节
 * 输入     address: 7 位 I2C 设备地址
 *          data: 数据字节数组指针
 *          length: 要读取的字节数
 *          sendStop: 一个布尔值，表示是否在结束时发送停止信号
 * 输出     返回读取的字节数
 */
uint8_t twi_readFrom(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop)
{
  uint8_t i;

  // 确保数据能适配缓冲区
  if(TWI_BUFFER_LENGTH < length){
    return 0;  // 如果数据长度超过缓冲区容量，返回 0
  }

  // 等待 TWI 准备好，成为主设备接收器
  while(TWI_READY != twi_state){
    continue;
  }
  twi_state = TWI_MRX;  // 设置 TWI 状态为接收器模式
  twi_sendStop = sendStop;
  twi_error = 0xFF;  // 重置错误状态

  // 初始化缓冲区索引变量
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length - 1;  // 接收时需要在倒数第二个字节时设置 NACK

  // 构造 sla+w，表示从设备地址和写操作标志
  twi_slarw = TW_READ;
  twi_slarw |= address << 1;

  if (true == twi_inRepStart) {
    // 如果处于重复启动状态，说明已经发送过启动信号
    twi_inRepStart = false;  // 清除重复启动状态
    TWDR = twi_slarw;
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // 启用中断，但不启用启动信号
  }
  else
    // 发送启动信号
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);  // 发送启动信号

  // 等待读取操作完成
  while(TWI_MRX == twi_state){
    continue;
  }

  if (twi_masterBufferIndex < length)
    length = twi_masterBufferIndex;  // 返回实际读取的字节数

  // 将缓冲区中的数据复制到目标数组
  for(i = 0; i < length; ++i){
    data[i] = twi_masterBuffer[i];
  }

  return length;
}

/*
 * 函数 twi_writeTo
 * 描述     尝试成为 TWI 总线主设备并向总线上的设备写入一系列字节
 * 输入     address: 7 位 I2C 设备地址
 *          data: 数据字节数组指针
 *          length: 数组中的字节数
 *          wait: 布尔值，指示是否等待写入完成
 *          sendStop: 布尔值，指示是否在结束时发送停止信号
 * 输出     返回状态：
 *          0 .. 成功
 *          1 .. 数据长度超出缓冲区
 *          2 .. 地址发送，收到 NACK
 *          3 .. 数据发送，收到 NACK
 *          4 .. 其他 TWI 错误（如总线仲裁丢失，总线错误等）
 */
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
  uint8_t i;

  // 确保数据能适配缓冲区
  if(TWI_BUFFER_LENGTH < length){
    return 1;  // 如果数据长度超出缓冲区，返回 1
  }

  // 等待 TWI 准备好，成为主设备发送器
  while(TWI_READY != twi_state){
    continue;
  }
  twi_state = TWI_MTX;  // 设置 TWI 状态为发送器模式
  twi_sendStop = sendStop;
  twi_error = 0xFF;  // 重置错误状态

  // 初始化缓冲区索引变量
  twi_masterBufferIndex = 0;
  twi_masterBufferLength = length;

  // 将数据复制到 TWI 缓冲区
  for(i = 0; i < length; ++i){
    twi_masterBuffer[i] = data[i];
  }

  // 构造 sla+w，表示从设备地址和写操作标志
  twi_slarw = TW_WRITE;
  twi_slarw |= address << 1;

  if (true == twi_inRepStart) {
    // 如果处于重复启动状态，说明已经发送过启动信号
    twi_inRepStart = false;  // 清除重复启动状态
    TWDR = twi_slarw;
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // 启用中断，但不启用启动信号
  }
  else
    // 发送启动信号
    TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);  // 发送启动信号

  // 等待写入操作完成
  while(wait && (TWI_MTX == twi_state)){
    continue;
  }

  // 检查是否有错误
  if (twi_error == 0xFF)
    return 0;  // 成功
  else if (twi_error == TW_MT_SLA_NACK)
    return 2;  // 地址发送，NACK 收到
  else if (twi_error == TW_MT_DATA_NACK)
    return 3;  // 数据发送，NACK 收到
  else
    return 4;  // 其他错误
}

/* 
 * 函数 twi_transmit
 * 描述    将数据填充到从设备的发送缓冲区中
 *         必须在从设备发送事件回调中调用
 * 输入    data: 指向字节数组的指针
 *         length: 数组中字节的数量
 * 输出    1: 缓冲区长度不足
 *         2: 当前不是从设备发送模式
 *         0: 成功
 */
uint8_t twi_transmit(const uint8_t* data, uint8_t length)
{
  uint8_t i;

  // 确保数据能够适应缓冲区
  if(TWI_BUFFER_LENGTH < length){
    return 1;  // 数据长度超过缓冲区大小
  }

  // 确保当前设备是从设备发送模式
  if(TWI_STX != twi_state){
    return 2;  // 当前不是从设备发送模式
  }

  // 设置发送数据长度并将数据拷贝到发送缓冲区
  twi_txBufferLength = length;
  for(i = 0; i < length; ++i){
    twi_txBuffer[i] = data[i];  // 将数据逐字节拷贝到发送缓冲区
  }

  return 0;  // 返回0表示成功
}

/*
 * 函数 twi_attachSlaveRxEvent
 * 描述    设置在从设备读取操作之前调用的回调函数
 * 输入    function: 回调函数的指针
 * 输出    无
 */
void twi_attachSlaveRxEvent( void (*function)(uint8_t*, int) )
{
  twi_onSlaveReceive = function;  // 设置从设备接收事件的回调函数
}

/*
 * 函数 twi_attachSlaveTxEvent
 * 描述    设置在从设备写入操作之前调用的回调函数
 * 输入    function: 回调函数的指针
 * 输出    无
 */
void twi_attachSlaveTxEvent( void (*function)(void) )
{
  twi_onSlaveTransmit = function;  // 设置从设备发送事件的回调函数
}

/*
 * 函数 twi_reply
 * 描述    发送字节或准备接收线
 * 输入    ack: 指示是否应回应ACK或NACK的字节
 * 输出    无
 */
void twi_reply(uint8_t ack)
{
  // 发送主设备读取准备信号，带或不带ACK
  if(ack){
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);  // 发送ACK
  }else{
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);  // 发送NACK
  }
}

/*
 * 函数 twi_stop
 * 描述    释放总线主控制权
 * 输入    无
 * 输出    无
 */
void twi_stop(void)
{
  // 发送停止条件
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // 等待停止条件在总线上被执行
  // TWINT 在停止条件后不会再被设置
  while(TWCR & _BV(TWSTO)){
    continue;  // 等待直到停止条件执行完成
  }

  // 更新TWI状态
  twi_state = TWI_READY;  // 状态设置为准备就绪
}

/*
 * 函数 twi_releaseBus
 * 描述    释放总线控制权
 * 输入    无
 * 输出    无
 */
void twi_releaseBus(void)
{
  // 释放总线控制权
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);

  // 更新TWI状态
  twi_state = TWI_READY;  // 状态设置为准备就绪
}

// 中断服务例程（ISR），处理TWI（I2C）总线的不同状态
ISR(TWI_vect)
{
  switch(TW_STATUS){
    // 所有主机状态
    case TW_START:     // 发送开始条件
    case TW_REP_START: // 发送重复开始条件
      // 将设备地址和读写位写入输出寄存器并发送ACK
      TWDR = twi_slarw;
      twi_reply(1);  // 发送ACK
      break;

    // 主机发送器状态
    case TW_MT_SLA_ACK:  // 从设备接收了地址的ACK
    case TW_MT_DATA_ACK: // 从设备接收了数据的ACK
      // 如果有数据需要发送，发送数据；否则停止
      if(twi_masterBufferIndex < twi_masterBufferLength){
        // 将数据写入输出寄存器并发送ACK
        TWDR = twi_masterBuffer[twi_masterBufferIndex++];
        twi_reply(1);
      }else{
        // 如果设置了发送停止条件，发送停止条件；否则准备发送重复的START条件
        if (twi_sendStop)
          twi_stop();
        else {
          twi_inRepStart = true;  // 设置标志，准备发送START
          // 发送START条件，不立即启用中断，等到下一个事务再处理
          TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
          twi_state = TWI_READY;
        }
      }
      break;
    case TW_MT_SLA_NACK:  // 地址发送完毕，接收到NACK
      twi_error = TW_MT_SLA_NACK;  // 设置错误状态
      twi_stop();  // 发送停止条件
      break;
    case TW_MT_DATA_NACK: // 数据发送完毕，接收到NACK
      twi_error = TW_MT_DATA_NACK;  // 设置错误状态
      twi_stop();  // 发送停止条件
      break;
    case TW_MT_ARB_LOST: // 丢失总线仲裁
      twi_error = TW_MT_ARB_LOST;  // 设置错误状态
      twi_releaseBus();  // 释放总线
      break;

    // 主机接收器状态
    case TW_MR_DATA_ACK: // 数据接收，发送ACK
      // 将接收到的数据放入缓冲区
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
    case TW_MR_SLA_ACK:  // 地址发送完毕，接收到ACK
      // 如果还有数据需要接收，发送ACK；否则发送NACK
      if(twi_masterBufferIndex < twi_masterBufferLength){
        twi_reply(1);  // 发送ACK
      }else{
        twi_reply(0);  // 发送NACK
      }
      break;
    case TW_MR_DATA_NACK: // 数据接收，发送NACK
      // 将最后一个字节放入缓冲区
      twi_masterBuffer[twi_masterBufferIndex++] = TWDR;
      if (twi_sendStop)
        twi_stop();
      else {
        twi_inRepStart = true;  // 设置标志，准备发送START
        // 发送START条件
        TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
        twi_state = TWI_READY;
      }
      break;
    case TW_MR_SLA_NACK: // 地址发送完毕，接收到NACK
      twi_stop();  // 发送停止条件
      break;
    // TW_MR_ARB_LOST 在 TW_MT_ARB_LOST 状态处理中

    // 从设备接收器状态
    case TW_SR_SLA_ACK:   // 被寻址，返回ACK
    case TW_SR_GCALL_ACK: // 被通用寻址，返回ACK
    case TW_SR_ARB_LOST_SLA_ACK:   // 丢失仲裁，返回ACK
    case TW_SR_ARB_LOST_GCALL_ACK: // 丢失仲裁，返回ACK
      // 进入从设备接收模式
      twi_state = TWI_SRX;
      // 指示接收缓冲区可以被覆盖并发送ACK
      twi_rxBufferIndex = 0;
      twi_reply(1);  // 发送ACK
      break;
    case TW_SR_DATA_ACK:       // 数据接收，发送ACK
    case TW_SR_GCALL_DATA_ACK: // 通用数据接收，发送ACK
      // 如果接收缓冲区还有空间
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        // 将字节放入缓冲区并发送ACK
        twi_rxBuffer[twi_rxBufferIndex++] = TWDR;
        twi_reply(1);  // 发送ACK
      }else{
        // 否则发送NACK
        twi_reply(0);  // 发送NACK
      }
      break;
    case TW_SR_STOP: // 接收到停止或重复开始条件
      // 如果接收缓冲区有空间，则在数据后添加空字符
      if(twi_rxBufferIndex < TWI_BUFFER_LENGTH){
        twi_rxBuffer[twi_rxBufferIndex] = '\0';
      }
      // 发送ACK并停止接口以进行时钟拉伸
      twi_stop();
      // 调用用户定义的回调函数
      twi_onSlaveReceive(twi_rxBuffer, twi_rxBufferIndex);
      // 重置接收缓冲区
      twi_rxBufferIndex = 0;
      // 确认未来的响应并离开从设备接收器状态
      twi_releaseBus();
      break;
    case TW_SR_DATA_NACK:       // 数据接收，返回NACK
    case TW_SR_GCALL_DATA_NACK: // 通用数据接收，返回NACK
      // 对主设备发送NACK
      twi_reply(0);  // 发送NACK
      break;

    // 从设备发送器状态
    case TW_ST_SLA_ACK:          // 被寻址，返回ACK
    case TW_ST_ARB_LOST_SLA_ACK: // 丢失仲裁，返回ACK
      // 进入从设备发送模式
      twi_state = TWI_STX;
      // 为发送缓冲区索引做准备
      twi_txBufferIndex = 0;
      // 将发送缓冲区长度设置为零，以验证用户是否更改了它
      twi_txBufferLength = 0;
      // 请求填充发送缓冲区并设置长度
      // 注意：用户必须调用 twi_transmit(bytes, length) 来执行此操作
      twi_onSlaveTransmit();
      // 如果用户没有更改缓冲区和长度，则初始化它
      if(0 == twi_txBufferLength){
        twi_txBufferLength = 1;
        twi_txBuffer[0] = 0x00;
      }
      // 发送缓冲区中的第一个字节，并继续
    case TW_ST_DATA_ACK: // 字节发送，接收到ACK
      // 将数据写入输出寄存器
      TWDR = twi_txBuffer[twi_txBufferIndex++];
      // 如果还有更多数据需要发送，发送ACK，否则发送NACK
      if(twi_txBufferIndex < twi_txBufferLength){
        twi_reply(1);  // 发送ACK
      }else{
        twi_reply(0);  // 发送NACK
      }
      break;
    case TW_ST_DATA_NACK: // 接收到NACK，表示结束
    case TW_ST_LAST_DATA: // 接收到ACK，但已经完成发送
      // 对未来的响应发送ACK
      twi_reply(1);  // 发送ACK
      // 离开从设备发送器状态
      twi_state = TWI_READY;
      break;

    // 所有状态
    case TW_NO_INFO:   // 无状态信息
      break;
    case TW_BUS_ERROR: // 总线错误，非法的停止/开始条件
      twi_error = TW_BUS_ERROR;  // 设置错误状态
      twi_stop();  // 发送停止条件
      break;
  }
}
