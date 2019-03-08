#ifndef IFM_TYPES_H_
#define IFM_TYPES_H_
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#define UDP_PACKET_BUF_LEN (1460)

typedef unsigned char ifm_o3m_uint8_t;
typedef signed char ifm_o3m_sint8_t;

typedef unsigned short ifm_o3m_uint16_t;
typedef signed short ifm_o3m_sint16_t;

typedef unsigned int ifm_o3m_uint32_t;
typedef signed int ifm_o3m_sint32_t;

typedef unsigned long long ifm_o3m_uint64_t;
typedef signed long long ifm_o3m_sint64_t;

typedef float ifm_o3m_float32_t;
typedef double ifm_o3m_float64_t;

struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))
typedef PointXYZIR VPoint; 
typedef pcl::PointCloud<VPoint> VPointCloud; 

// This pragma sets the alignment to 1 byte in Microsofts Visual C++. 
// This is necessary so that there are no gaps between the elements of the struct. 
// The size of the elements are chosen in a way so that it shouldn't happen on a 32bit system, 
// but we have to make sure.
// 
// With this gapless structure we are able to access in the packet header without further copying on 
// a little endian CPU (like x86). On a big endian machine this won't work.
#pragma pack(push, 1)

// This struct represents the first part of each UDP packet.
typedef struct PacketHeader
{
    ifm_o3m_uint16_t Version;
    ifm_o3m_uint16_t Device;
    ifm_o3m_uint32_t PacketCounter;
    ifm_o3m_uint32_t CycleCounter;
    ifm_o3m_uint16_t NumberOfPacketsInCycle;
    ifm_o3m_uint16_t IndexOfPacketInCycle;
    ifm_o3m_uint16_t NumberOfPacketsInChannel;
    ifm_o3m_uint16_t IndexOfPacketInChannel;
    ifm_o3m_uint32_t ChannelID;
    ifm_o3m_uint32_t TotalLengthOfChannel;
    ifm_o3m_uint32_t LengthPayload;
} PacketHeader;

typedef struct ChannelHeader
{
    ifm_o3m_uint32_t StartDelimiter;
    ifm_o3m_uint8_t reserved[24];
} ChannelHeader;

typedef struct ChannelEnd
{
    ifm_o3m_uint32_t EndDelimiter;
} ChannelEnd;

// This pragma restores the alignment to the previous value in Microsofts Visual C++
#pragma pack(pop)
#endif