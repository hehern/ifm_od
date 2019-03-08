#include "socketcan.h"

namespace Can_bridge
{
Socketcan::Socketcan()
{
    s = socket(PF_CAN,SOCK_RAW,CAN_RAW);
    strcpy(ifr.ifr_name,"can0");
    ioctl(s,SIOCGIFINDEX,&ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(s,(struct sockaddr *)&addr,sizeof(addr));
}
}//