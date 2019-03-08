#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <can_msgs/Frame.h>
#include<socket_can/SpeedMilSteer.h>

#define command "/sbin/ip link set can0 type can bitrate 1000000"
#define up "ifconfig can0 up"
#define down "ifconfig can0 down"
namespace Can_bridge
{
class Socketcan
{
public:
  Socketcan();
  ~Socketcan(){ close(s); };

private:
  struct sockaddr_can addr;
  struct ifreq ifr;
public:
  int s;
};
}//
#endif
