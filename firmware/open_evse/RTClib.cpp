// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

#include "./Wire.h"
#include <avr/pgmspace.h> // 引入用于访问存储在程序存储器（Flash）中的数据的库
#include "RTClib.h" // 引入RTC（实时时钟）库
#include "i2caddr.h" // 引入I2C地址相关的头文件

#define SECONDS_PER_DAY 86400L // 每天的秒数，86400秒

#define SECONDS_FROM_1970_TO_2000 946684800 // 1970年1月1日到2000年1月1日的秒数，用于时间戳转换

#if (ARDUINO >= 100)
 #include <Arduino.h> // 如果使用Arduino 1.x及以上版本，包含Arduino头文件
#else
 #include <WProgram.h> // 否则，使用旧版的Arduino头文件
#endif

// 工具函数，部分内容可以暴露在DateTime API中，以便需要时使用

static const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 }; // 每个月的天数，存储在程序存储器中。必须是const类型，否则编译器会报错

// 计算从2000年1月1日到指定日期（年、月、日）之间的天数，适用于2001到2099年
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
    if (y >= 2000) // 如果年份大于等于2000
        y -= 2000; // 转换为从2000年开始的年份（例如2001年变为1）
    uint16_t days = d; // 从日期开始
    for (uint8_t i = 1; i < m; ++i) // 累加到指定月份的天数
        days += pgm_read_byte(daysInMonth + i - 1); // 读取存储在程序存储器中的每个月的天数
    if (m > 2 && y % 4 == 0) // 如果是闰年并且月份大于2
        ++days; // 2月有29天
    return days + 365 * y + (y + 3) / 4 - 1; // 计算从2000年到目标日期的总天数
}

// 将日期转换为秒数
static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
    return ((days * 24L + h) * 60 + m) * 60 + s; // 返回自1970年1月1日以来的秒数
}

////////////////////////////////////////////////////////////////////////////////
// DateTime实现 - 忽略时区和夏令时变化
// 注意：也忽略了闰秒，参考 http://en.wikipedia.org/wiki/Leap_second

// 使用给定时间戳（自1970年以来的秒数）初始化DateTime对象
DateTime::DateTime (uint32_t t) {
  t -= SECONDS_FROM_1970_TO_2000;    // 将时间戳从1970年转换为2000年

    ss = t % 60; // 获取秒数
    t /= 60; // 转换为分钟
    mm = t % 60; // 获取分钟数
    t /= 60; // 转换为小时
    hh = t % 24; // 获取小时数
    uint16_t days = t / 24; // 获取天数
    uint8_t leap; // 闰年标志
    for (yOff = 0; ; ++yOff) { // 循环计算年份
        leap = yOff % 4 == 0; // 判断是否是闰年
        if (days < 365u + leap) // 如果剩余的天数少于当前年天数
            break;
        days -= 365 + leap; // 减去这一年的天数
    }
    for (m = 1; ; ++m) { // 循环计算月份
        uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1); // 获取当前月份的天数
        if (leap && m == 2) // 如果是闰年且月份是2月
            ++daysPerMonth; // 2月有29天
        if (days < daysPerMonth) // 如果剩余的天数小于当前月的天数
            break;
        days -= daysPerMonth; // 减去该月的天数
    }
    d = days + 1; // 设置日期
}

// 使用指定的年月日时分秒初始化DateTime对象
DateTime::DateTime (uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
    if (year >= 2000) // 如果年份大于等于2000
        year -= 2000; // 转换为从2000年开始的年份
    yOff = year; // 设置年份
    m = month; // 设置月份
    d = day; // 设置日期
    hh = hour; // 设置小时
    mm = min; // 设置分钟
    ss = sec; // 设置秒数
}

// 将日期字符串转换为两位数的数字（例如："09" 转换为 9）
static uint8_t conv2d(const char* p) {
    uint8_t v = 0; // 初始化为0
    if ('0' <= *p && *p <= '9') // 如果字符是数字
        v = *p - '0'; // 将字符转换为数字
    return 10 * v + *++p - '0'; // 处理第二位数字并返回
}

// 使用编译器的时间进行初始化，传入编译时的日期和时间
//   DateTime now (__DATE__, __TIME__);
DateTime::DateTime (const char* date, const char* time) {
    // 示例输入：date = "Dec 26 2009", time = "12:34:56"
    yOff = conv2d(date + 9); // 获取年份（从日期字符串的第9个字符开始）
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0]) { // 根据月份的首字母确定月份
        case 'J': m = (date[1] == 'a') ? 1 : ((date[2] == 'n') ? 6 : 7); break;
        case 'F': m = 2; break;
        case 'A': m = date[2] == 'r' ? 4 : 8; break;
        case 'M': m = date[2] == 'r' ? 3 : 5; break;
        case 'S': m = 9; break;
        case 'O': m = 10; break;
        case 'N': m = 11; break;
        case 'D': m = 12; break;
    }
    d = conv2d(date + 4); // 获取日期（从日期字符串的第4个字符开始）
    hh = conv2d(time); // 获取小时（从时间字符串的开始）
    mm = conv2d(time + 3); // 获取分钟（从时间字符串的第3个字符开始）
    ss = conv2d(time + 6); // 获取秒数（从时间字符串的第6个字符开始）
}

// 返回当前日期对应的星期几（0 = 周日，1 = 周一，... 6 = 周六）
uint8_t DateTime::dayOfWeek() const {
    uint16_t day = date2days(yOff, m, d);  // 将当前日期转换为自 2000 年 1 月 1 日以来的天数
    return (day + 6) % 7; // 2000年1月1日是星期六，返回 6 表示星期六
}

// 返回从1970年1月1日到当前时间的 Unix 时间戳（秒数）
uint32_t DateTime::unixtime(void) const {
  uint32_t t;
  uint16_t days = date2days(yOff, m, d);  // 将日期转换为天数
  t = time2long(days, hh, mm, ss);        // 将天数和时间转为秒数
  t += SECONDS_FROM_1970_TO_2000;  // 从 1970 年到 2000 年的秒数

  return t;
}

////////////////////////////////////////////////////////////////////////////////
// RTC_DS1307 实现

// 将 BCD（十进制编码的二进制）转换为二进制
static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
// 将二进制转换为 BCD（十进制编码的二进制）
static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }

#if (ARDUINO >= 100)

// 检查 RTC_DS1307 是否正在运行
uint8_t RTC_DS1307::isrunning(void) {
  Wire.beginTransmission(DS1307_ADDRESS);  // 开始与 RTC 的通信
  Wire.write(0);  // 向 RTC 发送命令（读取控制寄存器）
  Wire.endTransmission();  // 结束通信

  Wire.requestFrom(DS1307_ADDRESS, 1);  // 请求读取一个字节的数据
  uint8_t ss = Wire.read();  // 读取秒数
  return !(ss >> 7);  // 检查秒数的最高位（0 表示 RTC 正在运行）
}

// 调整 RTC_DS1307 的时间
void RTC_DS1307::adjust(const DateTime& dt) {
    Wire.beginTransmission(DS1307_ADDRESS);  // 开始与 RTC 的通信
    Wire.write(0);  // 设置时间的起始位置
    Wire.write(bin2bcd(dt.second()));  // 设置秒
    Wire.write(bin2bcd(dt.minute()));  // 设置分钟
    Wire.write(bin2bcd(dt.hour()));  // 设置小时
    Wire.write(bin2bcd(0));  // 设置星期（未使用，默认为 0）
    Wire.write(bin2bcd(dt.day()));  // 设置日期
    Wire.write(bin2bcd(dt.month()));  // 设置月份
    Wire.write(bin2bcd(dt.year() - 2000));  // 设置年份（2000 年减去 2000）
    Wire.write(0);  // 设置控制寄存器的其他值（未使用）
    Wire.endTransmission();  // 结束通信
}

// 从 RTC_DS1307 获取当前时间
DateTime RTC_DS1307::now() {
  Wire.beginTransmission(DS1307_ADDRESS);  // 开始与 RTC 的通信
  Wire.write(0);  // 发送命令（读取时间数据）
  Wire.endTransmission();  // 结束通信

  Wire.requestFrom(DS1307_ADDRESS, 7);  // 请求读取 7 个字节的数据（秒、分钟、小时、星期、日期、月份、年份）
  uint8_t ss = bcd2bin(Wire.read() & 0x7F);  // 读取并转换秒数，去掉最高位的标志位
  uint8_t mm = bcd2bin(Wire.read());  // 读取并转换分钟数
  uint8_t hh = bcd2bin(Wire.read());  // 读取并转换小时数
  Wire.read();  // 读取星期（未使用）
  uint8_t d = bcd2bin(Wire.read());  // 读取并转换日期
  uint8_t m = bcd2bin(Wire.read());  // 读取并转换月份
  uint16_t y = bcd2bin(Wire.read()) + 2000;  // 读取并转换年份（加上 2000）

  return DateTime(y, m, d, hh, mm, ss);  // 返回一个 DateTime 对象
}

#else

// 对于低版本的 Arduino，使用 send 和 receive 替代 write 和 read
uint8_t RTC_DS1307::isrunning(void) {
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.send(0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 1);
  uint8_t ss = Wire.receive();
  return !(ss >> 7);
}

// 调整 RTC_DS1307 的时间
void RTC_DS1307::adjust(const DateTime& dt) {
    // 开始与 DS1307 RTC 通信
    Wire.beginTransmission(DS1307_ADDRESS);

    // 发送控制寄存器的起始地址
    Wire.send(0);

    // 发送秒数，并将其从二进制转换为 BCD 格式
    Wire.send(bin2bcd(dt.second()));

    // 发送分钟数，将其从二进制转换为 BCD 格式
    Wire.send(bin2bcd(dt.minute()));

    // 发送小时数，将其从二进制转换为 BCD 格式
    Wire.send(bin2bcd(dt.hour()));

    // 发送星期数（未使用），默认设置为 0
    Wire.send(bin2bcd(0));

    // 发送日期，将其从二进制转换为 BCD 格式
    Wire.send(bin2bcd(dt.day()));

    // 发送月份，将其从二进制转换为 BCD 格式
    Wire.send(bin2bcd(dt.month()));

    // 发送年份，将其从二进制转换为 BCD 格式，2000 年减去 2000
    Wire.send(bin2bcd(dt.year() - 2000));

    // 发送控制寄存器的结束地址（未使用）
    Wire.send(0);

    // 结束与 RTC 的通信
    Wire.endTransmission();
}

// 从 RTC_DS1307 获取当前时间
DateTime RTC_DS1307::now() {
    // 开始与 DS1307 RTC 通信
    Wire.beginTransmission(DS1307_ADDRESS);

    // 发送控制寄存器的起始地址
    Wire.send(0);

    // 结束当前的传输
    Wire.endTransmission();

    // 请求从 DS1307 读取 7 个字节的数据（秒、分钟、小时、星期、日期、月份、年份）
    Wire.requestFrom(DS1307_ADDRESS, 7);

    // 读取并转换秒数，去掉最高位（标志位）
    uint8_t ss = bcd2bin(Wire.receive() & 0x7F);

    // 读取并转换分钟数
    uint8_t mm = bcd2bin(Wire.receive());

    // 读取并转换小时数
    uint8_t hh = bcd2bin(Wire.receive());

    // 读取星期数（未使用）
    Wire.receive();

    // 读取并转换日期
    uint8_t d = bcd2bin(Wire.receive());

    // 读取并转换月份
    uint8_t m = bcd2bin(Wire.receive());

    // 读取并转换年份（加上 2000 年）
    uint16_t y = bcd2bin(Wire.receive()) + 2000;

    // 返回一个 DateTime 对象，包含获取的日期和时间信息
    return DateTime(y, m, d, hh, mm, ss);
}
