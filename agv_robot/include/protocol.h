#ifndef AGV_ROBOT_PROTOCOL_H
#define AGV_ROBOT_PROTOCOL_H
class NaviStatus {
 public:
  unsigned char uchHead = 0xf1;
  /// NaviStatus[0]
  /// bit0bit1:未初始化=0，能遥控不能导航=01，初始化已完成=10；
  /// bit2bit3:未建图=0，已完成建图=10；
  /// bit4bit5:未导航=0，正在执导航=01，暂停导航（可以继续）=10；
  /// bit6:备用；
  /// bit7:未定位=0，正在定位=1
  /// NaviStatus[1]:无错误=0，错误代码=其他。
  // # 01 正常状态，开始导航
  // # 02 正常状态，到达目标点，成功
  // # 03 异常状态，导航至目标点失败，放弃
  // # 04 状态，导航至目标点过程中暂停导航，放弃
  // # 05 正常状态，空闲状态下暂停导航
  // # 06 正常状态，04情况下继续，接近目标点，成功
  // # 07 正常状态，04情况下继续，未接近目标点，开始导航
  // # 08 正常状态，空闲状态下继续导航
  // # 09 异常状态，暂停状态下发送新目标点，放弃
  unsigned char NaviStatus[2];
  /// 小车应执行的速度 vx, vy(m/s), vth(rad/s)
  unsigned char uchAgvCmd[12];
  /// 小车状态x, y, yaw
  unsigned char uchAGVStatus[12];
  unsigned char uchEnd = 0xf2;
  /// 从uchHead到uchEnd计算校验和
  unsigned char uchCheckSum;
};
// f3 03 00 00 00 00 00 00 00 00
class NaviCmd {
 public:
  unsigned char uchHead = 0xf3;
  // bit0bit1: 默认 00，新目标点 11，暂停导航 10，继续导航 01；
  unsigned char uchNaviCmd = 0;
  /// 单一目标点数据，x,y,theta(rad)
  unsigned char uchNaviTarget[12];
  unsigned char uchEnd = 0xf4;
  /// 从uchHead到uchEnd计算校验和
  unsigned char uchCheckSum;
};
#endif