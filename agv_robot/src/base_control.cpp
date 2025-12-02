#include "base_control.h"

BaseControl::BaseControl(ros::NodeHandle nh, int baudrate, std::string port)
    : nh_(nh) {
  while (!openSerialPort(baudrate, port)) {
    sleep(5);
  }
  clearError();
  enableAGV();
  binitdone = true;
  cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 10, &BaseControl::cmd_vel_callback, this);
  joy_vel_sub = nh_.subscribe<geometry_msgs::Twist>(
      "joy_vel", 10, &BaseControl::joy_vel_callback, this);
}
void BaseControl::run() {
  if (binitdone) {
    for (int i = 1; i < 5; i++) {
      // 0x6f设置速度值
      sendAgvCmd(i, 0x6f, ac, abcd[i - 1]);
      memset(&ac, 0, sizeof(ac));
      usleep(25000);
    }
  }

  if (serial_.available()) {
    std_msgs::String result;
    result.data = serial_.read(serial_.available());
    // printf("read: %X %X %X %X %X %X %X %X\n", result.data[0], result.data[1],
    //        result.data[2], result.data[3], result.data[4], result.data[5],
    //        result.data[6], result.data[7]);
  } else
    std::cout << "no message" << std::endl;
}
bool BaseControl::openSerialPort(int baudrate, std::string port) {
  try  // set serial port
  {
    serial_.setPort(port);
    serial_.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_.setTimeout(to);
    serial_.setParity(serial::parity_none);
    serial_.setBytesize(serial::eightbits);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return false;
  }

  if (serial_.isOpen())  // test if serial port is open
  {
    ROS_INFO_STREAM("Serial Port initialized");
    return true;
  } else {
    return false;
  }
}
void BaseControl::clearError() {
  for (int i = 1; i < 5; i++) {
    memset(&ac, 0, sizeof(ac));
    //转入未使能状态
    sendAgvCmd(i, 0x16, ac);
    usleep(50000);
  }

  for (int i = 1; i < 5; i++) {
    memset(&ac, 0, sizeof(ac));
    //编码器清零
    sendAgvCmd(i, 0x50, ac);
    usleep(50000);
  }

  for (int i = 1; i < 5; i++) {
    memset(&ac, 0, sizeof(ac));
    //清除错误
    sendAgvCmd(i, 0x17, ac);
    usleep(50000);
  }
}
void BaseControl::enableAGV() {
  for (int i = 1; i < 5; i++) {
    memset(&ac, 0, sizeof(ac));
    //使能
    sendAgvCmd(i, 0x15, ac);
    usleep(100000);
  }
}
bool BaseControl::writeSp(AgvCmd buf, int size) {
  unsigned char buff[8] = {buf.id,      buf.cmd,     buf.data[0], buf.data[1],
                           buf.data[2], buf.data[3], buf.crc[0],  buf.crc[1]};
  if (!serial_.write(buff, 8)) return false;
  return true;
}
bool BaseControl::sendAgvCmd(int id, unsigned char cmd, AgvCmd &buf,
                             float value) {
  unsigned char uc_spd[4];
  buf.id = (unsigned char)id;
  buf.cmd = cmd;

  // 0x23 校准  0x6f设置速度值
  if (cmd == 0x23 || cmd == 0x6f) {
    // 这里需要转成IEE754再下发

    ConvertDexToIEE754(value, uc_spd);
    for (int i = 0; i < 4; i++) {
      buf.data[i] = uc_spd[i];
    }
  }
  //使用这个CRC
  unsigned short scrc = getCRC16((unsigned char *)&buf, 6);
  // unsigned short scrc1 = makeCRC((unsigned char*)&buf, 6);
  // unsigned short scrc2 = calcCRC16((unsigned char*)&buf, 6);
  if (scrc == 0) {
    ROS_ERROR_STREAM("AfvCrcError");
    return false;
  }
  buf.crc[0] = (unsigned char)(scrc >> 8);
  buf.crc[1] = (unsigned char)scrc;
  //写入数据
  if (!writeSp(buf, sizeof(buf))) {
    ROS_ERROR_STREAM("agvcommunicationerror");
    return false;
  }
  return true;
}
void BaseControl::ConvertDexToIEE754(float fpointer, unsigned char *a) {
  int Flag = 0;
  double integer, decimal;               //整数，小数  
  unsigned long bininteger, bindecimal;  //二进制整数，二进制小数
  int _power, i;

  if (fpointer < 0) {
    fpointer = fpointer * (-1);
    Flag = 1;
  }

  decimal =
      modf(fpointer, &integer);  //将整数位到存到 integer ，小数位返回到decimal
  if (decimal || integer)  //判断 fpointer是否为0
  {
    bindecimal =
        (unsigned long)(decimal *
                        0x800000);  // 0x800000=2^23
                                    // 。得到小数位二进制表现形式         
    while ((bindecimal & 0xff800000) >
           0)  //计算有没有超过23位二进制数           
      bindecimal >>= 1;
    if (integer > 0) {
      bininteger = (unsigned long)integer;
      for (
          i = 0; i < 32;
          i++)  //计算整数部分的2的幂指数，整数位转化为二进制后的位数，计算小数点移动的位数
      {
        if (bininteger & 0x1) _power = i;
        bininteger >>= 0x1;
      }
      bininteger = (unsigned long)integer;
      bininteger &= ~(0x1 << _power);  //去掉最高位的1
      if (_power >= 23)  //如果幂指数>23 则舍弃小数位部分
      {
        bininteger >>= (_power - 23);
        bindecimal = 127 + _power;
        bininteger |= bindecimal << 23;
      } else {
        bininteger <<= (23 - _power);  //将去掉最高位的整数二进制，移至第22为
        bindecimal >>= _power;           //将小数部分左移1，
        bininteger |= bindecimal;        //二者向或得到1.x的x，
        bindecimal = 127 + _power;       //指数，右移的小数点数+127
        bininteger |= bindecimal << 23;  // 指数为右移23为变为或上x。
      }
    } else if (integer == 0) {
      bindecimal <<= 9;  //将小数部分的二进制移至最高位31位
      _power = 0;
      bininteger = bindecimal;
      while (bininteger == ((bindecimal << 1) >>
                            1))  //判断小数位最高位是否为1.  最高位为0 ：真
      {
        _power++;
        bindecimal <<= 0x1;
        bininteger = bindecimal;  //直到最高位为1,退出循环
      }
      _power++;
      bindecimal <<= 0x1;       //将1.x的1去掉 求x的值，
      bindecimal >>= 9;         //将小数位回到0-22位   
      bininteger = bindecimal;  //暂存到二进制整数中，
      bindecimal = 127 - _power;
      bininteger |= bindecimal << 23;  //将指数为右移值23为向或得到其值，
    }
    if (Flag == 1) bininteger |= 0x80000000;

    /*i = 0;
                a[i++] = (unsigned char)((bininteger >> 24) & 0xff);
                a[i++] = (unsigned char)((bininteger >> 16) & 0xff);
                a[i++] = (unsigned char)((bininteger >> 8) & 0xff);
                a[i++] = (unsigned char)(bininteger & 0xff);*/

    i = 4;
    a[--i] = (unsigned char)((bininteger >> 24) & 0xff);
    a[--i] = (unsigned char)((bininteger >> 16) & 0xff);
    a[--i] = (unsigned char)((bininteger >> 8) & 0xff);
    a[--i] = (unsigned char)(bininteger & 0xff);
  }
}
unsigned short BaseControl::getCRC16(const unsigned char *ptr,
                                     unsigned char len) {
  unsigned char i;
  unsigned short crc = 0xFFFF;
  if (len == 0) {
    len = 1;
  }
  while (len--) {
    crc ^= *ptr;
    for (i = 0; i < 8; i++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
    ptr++;
  }
  return (crc);
}

void BaseControl::joy_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
  if (cmd_vel_received && ros::Time::now() - cmd_vel_watch > ros::Duration(0.5))
    cmd_vel_received = false;
  memset(abcd, 0, sizeof(abcd));
  joy_vel_received =
      msg->linear.x != 0 || msg->linear.y != 0 || msg->angular.z != 0;
  if (!cmd_vel_received || joy_vel_received) {
    // abcd[0] = (msg->linear.x - msg->linear.y + msg->angular.z * 2 * 0.5) * 7;
    // abcd[1] = (msg->linear.x + msg->linear.y + msg->angular.z * 2 * 0.5) * 7;
    // abcd[2] = (msg->linear.x - msg->linear.y - msg->angular.z * 2 * 0.5) *
    // -7; abcd[3] = (msg->linear.x + msg->linear.y - msg->angular.z * 2 * 0.5)
    // * -7;
    abcd[0] =
        (msg->linear.x - msg->linear.y + msg->angular.z * 0.51) * 1.87 * 15;
    abcd[1] =
        (msg->linear.x + msg->linear.y + msg->angular.z * 0.51) * 1.87 * 15;
    abcd[2] =
        (msg->linear.x - msg->linear.y - msg->angular.z * 0.51) * -1.87 * 15;
    abcd[3] =
        (msg->linear.x + msg->linear.y - msg->angular.z * 0.51) * -1.87 * 15;
  }
}
void BaseControl::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
  cmd_vel_watch = ros::Time::now();
  if (joy_vel_received) return;
  memset(abcd, 0, sizeof(abcd));
  abcd[0] = (msg->linear.x - msg->linear.y + msg->angular.z * 0.51) * 1.87 * 15;
  abcd[1] = (msg->linear.x + msg->linear.y + msg->angular.z * 0.51) * 1.87 * 15;
  abcd[2] =
      (msg->linear.x - msg->linear.y - msg->angular.z * 0.51) * -1.87 * 15;
  abcd[3] =
      (msg->linear.x + msg->linear.y - msg->angular.z * 0.51) * -1.87 * 15;
  cmd_vel_received =
      msg->linear.x != 0 || msg->linear.y != 0 || msg->angular.z != 0;
}