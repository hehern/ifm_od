#include "UDPGet.h"

namespace Camera
{
void UDPGet::startUDPConnection(int port_number, std::string devip_str)
{
    port = port_number;
    devip_str_ = devip_str;
    int size;
    int ret;
    // create socket 
    if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sockfd < 0)
    {
        printf("Coudln't create the UDP socket\n");
    }

    // 设置地址信息、IP信息
    size = sizeof(struct sockaddr_in);
	bzero(&saddr,size);
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(port);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //绑定地址信息、IP信息
    ret = bind(sockfd,(struct sockaddr*)&saddr,sizeof(struct sockaddr));
    if(ret < 0)
    {
        printf("Bind didn't work\n");
    }
}
void UDPGet::stopUDPConnection()
{
    close(sockfd);
}
// Extracts the data in the payload of the udp packet und puts it into the channel buffer
int UDPGet::processPacket(ifm_o3m_sint8_t* currentPacketData,  // payload of the udp packet (without ethernet/IP/UDP header)
    ifm_o3m_uint32_t currentPacketSize, // size of the udp packet payload
    ifm_o3m_sint8_t* channelBuffer,      // buffer for a complete channel
    ifm_o3m_uint32_t channelBufferSize, // size of the buffer for the complete channel
    ifm_o3m_uint32_t* pos)              // the current pos in the channel buffer
{

    // There is always a PacketHeader structure at the beginning
    PacketHeader* ph = (PacketHeader*)currentPacketData;
    int Start = sizeof(PacketHeader);
    int Length = currentPacketSize - sizeof(PacketHeader);
    /*std::cout << "get a package" << std::endl;
    std::cout << "ph->Version = " << ph->Version << std::endl;
    std::cout << "ph->Device = " << ph->Device << std::endl;
    std::cout << "ph->PacketCounter = " << ph->PacketCounter << std::endl;
    std::cout << "ph->CycleCounter = " << ph->CycleCounter << std::endl;
    std::cout << "ph->NumberOfPacketsInCycle = " << ph->NumberOfPacketsInCycle << std::endl;
    std::cout << "ph->IndexOfPacketInCycle = " << ph->IndexOfPacketInCycle << std::endl;
    std::cout << "ph->NumberOfPacketsInChannel = " << ph->NumberOfPacketsInChannel << std::endl;
    std::cout << "ph->IndexOfPacketInChannel = " << ph->IndexOfPacketInChannel << std::endl;
    std::cout << "ph->ChannelID = " << ph->ChannelID << std::endl;
    std::cout << "ph->TotalLengthOfChannel = " << ph->TotalLengthOfChannel << std::endl;
    std::cout << "ph->LengthPayload = " << ph->LengthPayload << std::endl;*/

    // Only the first packet of a channel contains a ChannelHeader
    if(ph->IndexOfPacketInChannel == 0)
    {
        Start += sizeof(ChannelHeader);
        Length -= sizeof(ChannelHeader);
    }

    // Only the last packet of a channel contains an EndDelimiter (at the end, after the data)
    if(ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel - 1)
    {
        Length -= sizeof(ChannelEnd);
    }

    // Is the buffer big enough?
    if((*pos) + Length > channelBufferSize)
    {
        // Too small means either an error in the program logic or a corrupt packet
        printf("Channel buffer is too small.\n");
        return RESULT_ERROR;
    }
    else
    {
        memcpy(channelBuffer + (*pos), currentPacketData + Start, Length);
    }

    (*pos) += Length;

    return RESULT_OK;

}
int UDPGet::Receiver(int channelOfInterest)
{
    // Implemented channels: 8 (pixel data + objects) and 20 (objects only)
    // Only channel 8 or 20 are transmitted. The setting can be done in VisionAssistant
    // (Device setup -> Ethernet -> Output pixel data via Ethernet)
    const ifm_o3m_uint32_t customerDataChannel = channelOfInterest;

    // holds the sender information of the received packet
    //struct sockaddr remoteAddr;
    struct sockaddr_in remoteAddr;   //19.2.19hrn
    socklen_t remoteAddrLen;

    // buffer for a single UDP packet
    ifm_o3m_sint8_t udpPacketBuf[UDP_PACKET_BUF_LEN];

    // As the alignment was forced to 1 we can work with the struct on the buffer.
    // This assumes the byte order is little endian which it is on a PC.
    PacketHeader* ph = (PacketHeader*)udpPacketBuf;

    // the size of the channel may change so the size will be taken from the packet
    ifm_o3m_uint32_t channel_buf_size = 0;
    ifm_o3m_sint8_t* channelBuf = NULL;

    // As there is no offset in the packet header we have to remember where the next part should go
    ifm_o3m_uint32_t pos_in_channel = 0;

    // remember the counter of the previous packet so we know when we are losing packets
    ifm_o3m_uint32_t previous_packet_counter = 0;
    ifm_o3m_sint32_t previous_packet_counter_valid = 0;

    // the receiption of the data may start at any time. So we wait til we find the beginning of our channel
    ifm_o3m_sint32_t startOfChannelFound = 0;

    remoteAddrLen = sizeof(struct sockaddr);

    // run for all eternity as long as no error occurs
    while(1)
    {
        // receive the data. rc contains the number of received bytes and also the error code
        // IMPORTANT: This is a blocking call. If it doesn't receive anything it will wait forever
        ifm_o3m_uint32_t rc = recvfrom(sockfd, (char*)udpPacketBuf, UDP_PACKET_BUF_LEN, 0, (struct sockaddr*)&remoteAddr, &remoteAddrLen);
        //std::cout << "devip_str_ = " << devip_str_ << std::endl;
        //std::cout << "remoteAddr.sin_addr.s_addr = " << remoteAddr.sin_addr.s_addr << std::endl;
        //std::cout << "devip_.s_addr = " << devip_.s_addr << std::endl;
        if(rc == -1)
        {
            printf("Error in recvfrom\n");
            return RESULT_ERROR;
        }
        else if(devip_str_ != "" && remoteAddr.sin_addr.s_addr != devip_.s_addr)
        {
            //std::cout << "haha" << std::endl;
            continue;
        }
        else
        {
            // Check the packet counter for missing packets
            if(previous_packet_counter_valid)
            {
                // if the type of the variables is ui32, it will also work when the wrap around happens.
                if((ph->PacketCounter - previous_packet_counter) != 1)
                {
                    printf("Packet Counter jumped from %ul to %ul (missing packets; try to redirect output)\n", previous_packet_counter, ph->PacketCounter);

                    // With this it will ignore the already received parts and resynchronize at 
                    // the beginning of the next cycle.
                    startOfChannelFound = 0;
                }
            }

            previous_packet_counter = ph->PacketCounter;
            previous_packet_counter_valid = 1;

            // is this the channel with our data?
            if(ph->ChannelID == customerDataChannel)
            {                
                // are we at the beginning of the channel?
                if(ph->IndexOfPacketInChannel == 0)
                {
                    startOfChannelFound = 1;

                    // If we haven't allocated memory for the channel do it now.
                    if(channel_buf_size == 0)
                    {
                        channel_buf_size = ph->TotalLengthOfChannel;
                        channelBuf = (ifm_o3m_sint8_t*)malloc(channel_buf_size);
                    }

                    // as we reuse the buffer we clear it at the beginning of a transmission
                    memset(channelBuf, 0, channel_buf_size);
                    pos_in_channel = 0;

                }

                // if we have found the start of the channel at least once, we are ready to process the packet
                if(startOfChannelFound)
                {
                    processPacket(udpPacketBuf, rc, channelBuf, channel_buf_size, &pos_in_channel);

                    // Have we found the last packet in this channel? Then we are able to process it
                    // The index is zero based so a channel with n parts will have indices from 0 to n-1
                    if(ph->IndexOfPacketInChannel == ph->NumberOfPacketsInChannel - 1)
                    {
                        // pos_in_channel is the position where the (not existing) next packet would be 
                        // placed. This is also the size of the data.
                        processChannel(customerDataChannel, channelBuf, pos_in_channel);
                        return RESULT_OK;
                    }
                }
            }
        }
    }

    return RESULT_ERROR;
}
// this function shows how to interprete channel data depending on the channel number
int UDPGet::processChannel(ifm_o3m_uint32_t channel_no, void* buf, ifm_o3m_uint32_t size)
{
    switch(channel_no)
    {
    case 8:
        return processChannel8(buf, size);
    case 14:
        return processChannel14(buf, size);
    case 20:
        return processChannel20(buf, size);
    case 256:
        return processChannel256(buf, size);
    default:
        break;
    }
    return RESULT_OK;
}
int UDPGet::processChannel8(void* buf, ifm_o3m_uint32_t size)
{
    Channel8* p_ch8 = NULL;

    // Is this DI structure version 1.4?
    p_ch8 = ifm_o3m_ConvertBufferToLittleEndian_ODA1_1_4(buf, size);
    if(p_ch8)
    {
        // yes it is, so copy the data from this structure
        copyChannel8(p_ch8);
        return RESULT_OK;
    }

    // This is no known version of the data.
    printf("*** Unknown version, check if you use the appropriate FW-version *** \n\n");
    return RESULT_ERROR;
}
int UDPGet::copyChannel8(Channel8* p)
{
    int i;

    if (p == NULL)
        return RESULT_ERROR;
    //copy distance data
    for (i = 0; i<NUM_SENSOR_PIXELS; i++)
    {
        g_distanceData.distanceData[i] = p->distanceImageResult.distanceData[i];
        g_distanceData.confidence[i] = p->distanceImageResult.confidence[i];
        g_distanceData.X[i] = p->distanceImageResult.X[i];
        g_distanceData.Y[i] = p->distanceImageResult.Y[i];
        g_distanceData.Z[i] = p->distanceImageResult.Z[i];
        g_distanceData.ampl_norm[i] = p->distanceImageResult.amplitude[i] * p->distanceImageResult.amplitude_normalization[(p->distanceImageResult.confidence[i] >> 1) & 3];
    }
    //copy object detection data
    for(int j=0; j<20; j++)
    {
        g_objectData_ch8.objectList[j].id = p->objectDetectionResult.objectList[j].id;
        g_objectData_ch8.objectList[j].history = p->objectDetectionResult.objectList[j].history;
        g_objectData_ch8.objectList[j].measured = p->objectDetectionResult.objectList[j].measured;
        g_objectData_ch8.objectList[j].type = p->objectDetectionResult.objectList[j].type;
        g_objectData_ch8.objectList[j].age = p->objectDetectionResult.objectList[j].age;
        g_objectData_ch8.objectList[j].x1 = p->objectDetectionResult.objectList[j].x1;
        g_objectData_ch8.objectList[j].y1 = p->objectDetectionResult.objectList[j].y1;
        g_objectData_ch8.objectList[j].x2 = p->objectDetectionResult.objectList[j].x2;
        g_objectData_ch8.objectList[j].y2 = p->objectDetectionResult.objectList[j].y2;
        g_objectData_ch8.objectList[j].zMin = p->objectDetectionResult.objectList[j].zMin;
        g_objectData_ch8.objectList[j].zMax = p->objectDetectionResult.objectList[j].zMax;
        g_objectData_ch8.objectList[j].vX = p->objectDetectionResult.objectList[j].vX;
        g_objectData_ch8.objectList[j].vY = p->objectDetectionResult.objectList[j].vY;
        g_objectData_ch8.objectList[j].vZ = p->objectDetectionResult.objectList[j].vZ;
        g_objectData_ch8.objectList[j].aX = p->objectDetectionResult.objectList[j].aX;
        g_objectData_ch8.objectList[j].aY = p->objectDetectionResult.objectList[j].aY;
        g_objectData_ch8.objectList[j].aZ = p->objectDetectionResult.objectList[j].aZ;
        g_objectData_ch8.objectList[j].existenceProbability = p->objectDetectionResult.objectList[j].existenceProbability;
        g_objectData_ch8.objectList[j].vXQuality = p->objectDetectionResult.objectList[j].vXQuality;
        g_objectData_ch8.objectList[j].vYQuality = p->objectDetectionResult.objectList[j].vYQuality;
        g_objectData_ch8.objectList[j].distanceToEgo = p->objectDetectionResult.objectList[j].distanceToEgo;

    }
    g_objectData_ch8.crashPredictorResult.crashPredicted = p->objectDetectionResult.crashPredictorResult.crashPredicted;
    g_objectData_ch8.crashPredictorResult.crashPredictTime = p->objectDetectionResult.crashPredictorResult.crashPredictTime;
    g_objectData_ch8.crashPredictorResult.relImpactVelocity = p->objectDetectionResult.crashPredictorResult.relImpactVelocity;
    g_objectData_ch8.crashPredictorResult.crashObjectID = p->objectDetectionResult.crashPredictorResult.crashObjectID;
    g_objectData_ch8.crashPredictorResult.criticality = p->objectDetectionResult.crashPredictorResult.criticality;
    //copy calibration result
    g_calibrationResult.commonCalibrationResult.calibValid = p->calibrationResult.commonCalibrationResult.calibValid;
    g_calibrationResult.commonCalibrationResult.calibrationStableCounter = p->calibrationResult.commonCalibrationResult.calibrationStableCounter;
    g_calibrationResult.commonCalibrationResult.calibResult.transX = p->calibrationResult.commonCalibrationResult.calibResult.transX;
    g_calibrationResult.commonCalibrationResult.calibResult.transY = p->calibrationResult.commonCalibrationResult.calibResult.transY;
    g_calibrationResult.commonCalibrationResult.calibResult.transZ = p->calibrationResult.commonCalibrationResult.calibResult.transZ;
    g_calibrationResult.commonCalibrationResult.calibResult.rotX = p->calibrationResult.commonCalibrationResult.calibResult.rotX;
    g_calibrationResult.commonCalibrationResult.calibResult.rotY = p->calibrationResult.commonCalibrationResult.calibResult.rotY;
    g_calibrationResult.commonCalibrationResult.calibResult.rotZ = p->calibrationResult.commonCalibrationResult.calibResult.rotZ;
    //copy logic output
    for(int k=0; k<100; k++)
    {
        g_logicOutput.digitalOutput[k] = p->logicOutput.digitalOutput[k];
    }
    for(int l=0; l<20; l++)
    {
        g_logicOutput.analogOutput[l] = p->logicOutput.analogOutput[l];
    }
    
    return RESULT_OK;
}
int UDPGet::processChannel14(void* buf, ifm_o3m_uint32_t size)
{
    Channel14* p_ch = NULL;

    // Is this DI structure version 1.5?
    p_ch = ifm_o3m_ConvertBufferToLittleEndian_ODD1_1_5(buf, size);
    if(p_ch)
    {
        // yes it is, so copy the data from this structure
        copyChannel14(p_ch);
        return RESULT_OK;
    }

    // This is no known version of the data.
    printf("*** Unknown version, check if you use the appropriate FW-version *** \n\n");
    return RESULT_ERROR;
}
int UDPGet::copyChannel14(Channel14* p)
{
    if(p == NULL)
        return RESULT_ERROR;

    g_parameterData.ObjectListCust_objectDetectionVariant = p->DspCustomerParameter.ObjectListCust_objectDetectionVariant;
    g_parameterData.Modulation_Frequency_Mode = p->DspCustomerParameter.Modulation_Frequency_Mode;

    return RESULT_OK;
}
int UDPGet::processChannel20(void* buf, ifm_o3m_uint32_t size)
{
    Channel20* p_ch20 = NULL;

    // Is this DI structure version 1.4?
    p_ch20 = ifm_o3m_ConvertBufferToLittleEndian_ODA2_1_4(buf, size);
    if(p_ch20)
    {
        // yes it is, so copy the data from this structure
        copyChannel20(p_ch20);
        return RESULT_OK;
    }

    // This is no known version of the data.
    printf("*** Unknown version, check if you use the appropriate FW-version *** \n\n");
    return RESULT_ERROR;
}
int UDPGet::copyChannel20(Channel20* p)
{
    int i;

    if(p == NULL)
        return RESULT_ERROR;

    for(i = 0; i<20; i++)
    {
        g_objectData.objectList[i].id = p->objectDetectionResult.objectList[i].id;
        g_objectData.objectList[i].x1 = p->objectDetectionResult.objectList[i].x1;
        g_objectData.objectList[i].x2 = p->objectDetectionResult.objectList[i].x2;
    }

    return RESULT_OK;
}
int UDPGet::processChannel256(void* buf, ifm_o3m_uint32_t size)
{
    Channel256* p_ch256 = ifm_o3m_ConvertBufferToLittleEndian_ODK1_1_6(buf, size);
    if(p_ch256)
    {
        copyChannel256(p_ch256);
        return RESULT_OK;
    }

    printf("*** Not supported version, check if you use the appropriate LG FW-version    *** \n\n");
    printf("*** FW-version must match with the FW-packet of this sample code             *** \n");
    printf("*** Connect with Vision Assistant and look into Device setup -> Device       *** \n\n");
    return RESULT_ERROR;
}
int UDPGet::copyChannel256(Channel256* p)
{
    if(p == NULL)
        return RESULT_ERROR;

    g_statusData.OpMode = p->OpMode;
    g_statusData.Variant[0] = p->KpCustomerParameter.Variant[0];
    g_statusData.Variant[1] = p->KpCustomerParameter.Variant[1];
    g_statusData.Variant[2] = p->KpCustomerParameter.Variant[2];
    g_statusData.mainTemperature = p->CameraMainTemperature;

    return RESULT_OK;
}
}//namespace Camera

