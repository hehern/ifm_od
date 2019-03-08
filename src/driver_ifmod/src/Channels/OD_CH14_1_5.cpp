#include "OD_CH14_1_5.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5* ifm_o3m_ConvertBufferToLittleEndian_ODD1_1_5(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5* res = (ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5*)buffer;
    if( (!buffer) || (bufferSize != 5296) || (sizeof(ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5) != 5296) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'D') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 5  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5* ifm_o3m_ConvertBufferToBigEndian_ODD1_1_5(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5* res = (ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5*)buffer;
    if( (!buffer) || (bufferSize != 5296) || (sizeof(ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5) != 5296) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'D') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 5  ) ) )
    {
        return 0;
    }

    /* DspCustomerParameter.destinationUDPPort */
    IFM_SWAP16(&buf[52]);
    /* DspCustomerParameter.VehicleDim_xMin */
    IFM_SWAP32(&buf[60]);
    /* DspCustomerParameter.VehicleDim_xMax */
    IFM_SWAP32(&buf[64]);
    /* DspCustomerParameter.VehicleDim_yMin */
    IFM_SWAP32(&buf[68]);
    /* DspCustomerParameter.VehicleDim_yMax */
    IFM_SWAP32(&buf[72]);
    /* DspCustomerParameter.VehicleDim_zMax */
    IFM_SWAP32(&buf[76]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_transX */
    IFM_SWAP32(&buf[80]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_transY */
    IFM_SWAP32(&buf[84]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_transZ */
    IFM_SWAP32(&buf[88]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_rotX */
    IFM_SWAP32(&buf[92]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_rotY */
    IFM_SWAP32(&buf[96]);
    /* DspCustomerParameter.PMDExtrCalib_camCal_rotZ */
    IFM_SWAP32(&buf[100]);
    /* DspCustomerParameter.PMDExtrCalib_IlluCal_transX */
    IFM_SWAP32(&buf[104]);
    /* DspCustomerParameter.PMDExtrCalib_IlluCal_transY */
    IFM_SWAP32(&buf[108]);
    /* DspCustomerParameter.PMDExtrCalib_IlluCal_transZ */
    IFM_SWAP32(&buf[112]);
    /* DspCustomerParameter.ObjectListCust_spatialFilterXMin */
    IFM_SWAP32(&buf[120]);
    /* DspCustomerParameter.ObjectListCust_spatialFilterXMax */
    IFM_SWAP32(&buf[124]);
    /* DspCustomerParameter.ObjectListCust_spatialFilterYMin */
    IFM_SWAP32(&buf[128]);
    /* DspCustomerParameter.ObjectListCust_spatialFilterYMax */
    IFM_SWAP32(&buf[132]);
    /* DspCustomerParameter.ObjectListCust_spatialFilterZMin */
    IFM_SWAP32(&buf[136]);
    /* DspCustomerParameter.ObjectListCust_spatialFilterZMax */
    IFM_SWAP32(&buf[140]);
    /* DspCustomerParameter.ObjectListCust_reflectorThresholdValue */
    IFM_SWAP32(&buf[144]);
    /* DspCustomerParameter.ObjectListCust_ObjectDetectionZMin */
    IFM_SWAP32(&buf[152]);
    /* DspCustomerParameter.ObjectListCust_ObjectDetectionZMax */
    IFM_SWAP32(&buf[156]);
    /* DspCustomerParameter.ObjectListCust_accBrake */
    IFM_SWAP32(&buf[164]);
    /* DspCustomerParameter.ObjectListCust_delayBrake */
    IFM_SWAP32(&buf[168]);
    /* DspCustomerParameter.ObjectListCust_egoVMin */
    IFM_SWAP32(&buf[172]);
    /* DspCustomerParameter.ObjectListCust_egoVMax */
    IFM_SWAP32(&buf[176]);
    /* DspCustomerParameter.ObjectListCust_cpDeactivateTimeAfterTrigger */
    IFM_SWAP32(&buf[184]);
    /* DspCustomerParameter.ObjectListCust_cpMaxCrashObjectDistance */
    IFM_SWAP32(&buf[188]);
    /* DspCustomerParameter.ObjectListCust_cpMinDistAllowed */
    IFM_SWAP32(&buf[192]);
    /* DspCustomerParameter.AutoCalibParam_xPattern */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+200]);
    }
    /* DspCustomerParameter.AutoCalibParam_yPattern */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+232]);
    }
    /* DspCustomerParameter.AutoCalibParam_zPattern */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+264]);
    }
    /* DspCustomerParameter.num_Frames_Averaging */
    IFM_SWAP32(&buf[308]);
    /* DspCustomerParameter.pixelPlausiReflectivityThreshold */
    IFM_SWAP32(&buf[312]);
    /* DspCustomerParameter.amplThresholdFactor */
    IFM_SWAP32(&buf[320]);
    /* DspCustomerParameter.Logic.logicGraph.operation */
    for(i = 0; i < 500; i++)
    {
        IFM_SWAP16(&buf[(i*8)+324]);
    }
    /* DspCustomerParameter.Logic.logicGraph.inputs */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+326]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+334]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+342]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+350]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+358]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+366]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+374]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+382]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+390]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+398]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+406]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+414]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+422]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+430]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+438]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+446]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+454]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+462]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+470]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+478]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+486]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+494]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+502]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+510]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+518]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+526]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+534]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+542]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+550]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+558]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+566]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+574]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+582]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+590]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+598]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+606]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+614]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+622]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+630]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+638]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+646]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+654]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+662]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+670]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+678]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+686]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+694]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+702]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+710]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+718]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+726]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+734]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+742]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+750]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+758]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+766]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+774]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+782]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+790]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+798]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+806]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+814]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+822]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+830]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+838]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+846]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+854]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+862]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+870]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+878]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+886]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+894]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+902]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+910]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+918]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+926]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+934]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+942]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+950]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+958]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+966]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+974]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+982]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+990]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+998]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1006]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1014]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1022]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1030]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1038]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1046]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1054]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1062]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1070]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1078]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1086]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1094]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1102]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1110]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1118]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1126]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1134]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1142]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1150]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1158]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1166]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1174]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1182]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1190]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1198]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1206]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1214]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1222]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1230]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1238]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1246]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1254]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1262]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1270]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1278]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1286]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1294]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1302]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1310]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1318]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1326]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1334]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1342]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1350]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1358]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1366]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1374]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1382]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1390]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1398]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1406]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1414]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1422]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1430]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1438]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1446]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1454]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1462]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1470]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1478]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1486]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1494]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1502]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1510]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1518]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1526]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1534]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1542]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1550]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1558]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1566]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1574]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1582]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1590]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1598]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1606]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1614]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1622]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1630]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1638]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1646]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1654]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1662]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1670]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1678]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1686]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1694]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1702]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1710]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1718]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1726]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1734]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1742]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1750]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1758]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1766]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1774]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1782]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1790]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1798]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1806]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1814]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1822]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1830]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1838]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1846]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1854]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1862]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1870]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1878]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1886]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1894]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1902]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1910]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1918]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1926]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1934]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1942]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1950]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1958]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1966]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1974]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1982]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1990]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+1998]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2006]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2014]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2022]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2030]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2038]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2046]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2054]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2062]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2070]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2078]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2086]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2094]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2102]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2110]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2118]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2126]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2134]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2142]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2150]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2158]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2166]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2174]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2182]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2190]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2198]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2206]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2214]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2222]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2230]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2238]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2246]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2254]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2262]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2270]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2278]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2286]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2294]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2302]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2310]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2318]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2326]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2334]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2342]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2350]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2358]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2366]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2374]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2382]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2390]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2398]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2406]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2414]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2422]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2430]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2438]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2446]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2454]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2462]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2470]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2478]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2486]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2494]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2502]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2510]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2518]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2526]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2534]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2542]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2550]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2558]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2566]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2574]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2582]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2590]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2598]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2606]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2614]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2622]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2630]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2638]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2646]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2654]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2662]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2670]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2678]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2686]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2694]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2702]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2710]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2718]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2726]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2734]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2742]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2750]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2758]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2766]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2774]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2782]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2790]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2798]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2806]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2814]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2822]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2830]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2838]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2846]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2854]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2862]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2870]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2878]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2886]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2894]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2902]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2910]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2918]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2926]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2934]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2942]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2950]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2958]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2966]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2974]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2982]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2990]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+2998]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3006]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3014]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3022]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3030]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3038]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3046]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3054]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3062]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3070]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3078]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3086]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3094]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3102]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3110]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3118]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3126]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3134]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3142]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3150]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3158]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3166]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3174]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3182]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3190]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3198]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3206]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3214]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3222]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3230]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3238]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3246]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3254]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3262]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3270]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3278]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3286]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3294]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3302]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3310]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3318]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3326]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3334]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3342]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3350]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3358]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3366]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3374]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3382]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3390]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3398]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3406]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3414]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3422]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3430]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3438]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3446]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3454]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3462]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3470]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3478]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3486]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3494]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3502]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3510]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3518]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3526]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3534]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3542]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3550]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3558]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3566]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3574]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3582]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3590]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3598]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3606]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3614]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3622]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3630]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3638]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3646]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3654]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3662]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3670]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3678]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3686]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3694]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3702]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3710]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3718]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3726]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3734]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3742]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3750]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3758]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3766]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3774]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3782]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3790]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3798]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3806]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3814]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3822]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3830]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3838]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3846]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3854]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3862]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3870]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3878]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3886]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3894]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3902]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3910]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3918]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3926]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3934]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3942]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3950]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3958]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3966]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3974]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3982]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3990]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+3998]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4006]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4014]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4022]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4030]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4038]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4046]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4054]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4062]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4070]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4078]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4086]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4094]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4102]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4110]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4118]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4126]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4134]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4142]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4150]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4158]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4166]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4174]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4182]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4190]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4198]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4206]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4214]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4222]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4230]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4238]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4246]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4254]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4262]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4270]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4278]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4286]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4294]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4302]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4310]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP16(&buf[(i*2)+4318]);
    }
    /* DspCustomerParameter.Logic.imeasIntrospection.channelID */
    for(i = 0; i < 25; i++)
    {
        IFM_SWAP16(&buf[(i*34)+4324]);
    }
    /* DspCustomerParameter.ObjectListCust_delayBrake2 */
    IFM_SWAP32(&buf[5176]);
    /* DspCustomerParameter.ObjectListCust_delayBrake3 */
    IFM_SWAP32(&buf[5180]);
    /* DspCustomerParameter.zoneBasedWarning_Xmin */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5184]);
    }
    /* DspCustomerParameter.zoneBasedWarning_Xmax */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5196]);
    }
    /* DspCustomerParameter.zoneBasedWarning_Ymin */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5208]);
    }
    /* DspCustomerParameter.zoneBasedWarning_Ymax */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5220]);
    }
    /* DspCustomerParameter.RoD_Xmin */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5236]);
    }
    /* DspCustomerParameter.RoD_Xmax */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5244]);
    }
    /* DspCustomerParameter.RoD_Ymin */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5252]);
    }
    /* DspCustomerParameter.RoD_Ymax */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5260]);
    }
    /* DspCustomerParameter.RoD_Zmin */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5268]);
    }
    /* DspCustomerParameter.RoD_Zmax */
    for(i = 0; i < 2; i++)
    {
        IFM_SWAP32(&buf[(i*4)+5276]);
    }

    return res;
}
