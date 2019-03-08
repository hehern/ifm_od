#include "OD_CH20_1_4.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4* ifm_o3m_ConvertBufferToLittleEndian_ODA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4* res = (ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4*)buffer;
    if( (!buffer) || (bufferSize != 2500) || (sizeof(ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4) != 2500) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'A') && (res->struct_id[3] == '2') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4* ifm_o3m_ConvertBufferToBigEndian_ODA2_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4* res = (ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4*)buffer;
    if( (!buffer) || (bufferSize != 2500) || (sizeof(ifm_o3m_AlgoIFOutputNoDI_ODA2_1_4) != 2500) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'A') && (res->struct_id[3] == '2') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    /* masterclockTimestamp */
    IFM_SWAP32(&buf[12]);
    /* frameCounter */
    IFM_SWAP32(&buf[16]);
    /* available */
    IFM_SWAP32(&buf[20]);
    /* cameraCalibration.transX */
    IFM_SWAP32(&buf[24]);
    /* cameraCalibration.transY */
    IFM_SWAP32(&buf[28]);
    /* cameraCalibration.transZ */
    IFM_SWAP32(&buf[32]);
    /* cameraCalibration.rotX */
    IFM_SWAP32(&buf[36]);
    /* cameraCalibration.rotY */
    IFM_SWAP32(&buf[40]);
    /* cameraCalibration.rotZ */
    IFM_SWAP32(&buf[44]);
    /* fieldOfView.upperLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+48]);
    }
    /* fieldOfView.upperRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+60]);
    }
    /* fieldOfView.lowerLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+72]);
    }
    /* fieldOfView.lowerRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+84]);
    }
    /* intrExtrCalib_2d.intrCalib_2D_fx */
    IFM_SWAP32(&buf[96]);
    /* intrExtrCalib_2d.intrCalib_2D_fy */
    IFM_SWAP32(&buf[100]);
    /* intrExtrCalib_2d.intrCalib_2D_mx */
    IFM_SWAP32(&buf[104]);
    /* intrExtrCalib_2d.intrCalib_2D_my */
    IFM_SWAP32(&buf[108]);
    /* intrExtrCalib_2d.intrCalib_alpha */
    IFM_SWAP32(&buf[112]);
    /* intrExtrCalib_2d.intrCalib_k1 */
    IFM_SWAP32(&buf[116]);
    /* intrExtrCalib_2d.intrCalib_k2 */
    IFM_SWAP32(&buf[120]);
    /* intrExtrCalib_2d.intrCalib_k5 */
    IFM_SWAP32(&buf[124]);
    /* intrExtrCalib_2d.intrCalib_k3 */
    IFM_SWAP32(&buf[128]);
    /* intrExtrCalib_2d.intrCalib_k4 */
    IFM_SWAP32(&buf[132]);
    /* intrExtrCalib_2d.extrCalib_center_tx */
    IFM_SWAP32(&buf[136]);
    /* intrExtrCalib_2d.extrCalib_center_ty */
    IFM_SWAP32(&buf[140]);
    /* intrExtrCalib_2d.extrCalib_center_tz */
    IFM_SWAP32(&buf[144]);
    /* intrExtrCalib_2d.extrCalib_delta_tx */
    IFM_SWAP32(&buf[148]);
    /* intrExtrCalib_2d.extrCalib_delta_ty */
    IFM_SWAP32(&buf[152]);
    /* intrExtrCalib_2d.extrCalib_delta_tz */
    IFM_SWAP32(&buf[156]);
    /* intrExtrCalib_2d.extrCalib_rot_x */
    IFM_SWAP32(&buf[160]);
    /* intrExtrCalib_2d.extrCalib_rot_y */
    IFM_SWAP32(&buf[164]);
    /* intrExtrCalib_2d.extrCalib_rot_z */
    IFM_SWAP32(&buf[168]);
    /* illuPosition */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+172]);
    }
    /* blockageRatio */
    IFM_SWAP32(&buf[184]);
    /* objectDetectionResult.objectList.id */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+192]);
    }
    /* objectDetectionResult.objectList.history */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+196]);
    }
    /* objectDetectionResult.objectList.measured */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+200]);
    }
    /* objectDetectionResult.objectList.type */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+204]);
    }
    /* objectDetectionResult.objectList.age */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+208]);
    }
    /* objectDetectionResult.objectList.x1 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+212]);
    }
    /* objectDetectionResult.objectList.y1 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+216]);
    }
    /* objectDetectionResult.objectList.x2 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+220]);
    }
    /* objectDetectionResult.objectList.y2 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+224]);
    }
    /* objectDetectionResult.objectList.zMin */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+228]);
    }
    /* objectDetectionResult.objectList.zMax */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+232]);
    }
    /* objectDetectionResult.objectList.vX */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+236]);
    }
    /* objectDetectionResult.objectList.vY */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+240]);
    }
    /* objectDetectionResult.objectList.vZ */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+244]);
    }
    /* objectDetectionResult.objectList.aX */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+248]);
    }
    /* objectDetectionResult.objectList.aY */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+252]);
    }
    /* objectDetectionResult.objectList.aZ */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+256]);
    }
    /* objectDetectionResult.objectList.existenceProbability */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+260]);
    }
    /* objectDetectionResult.objectList.vXQuality */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+264]);
    }
    /* objectDetectionResult.objectList.vYQuality */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+268]);
    }
    /* objectDetectionResult.objectList.distanceToEgo */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+272]);
    }
    /* objectDetectionResult.crashPredictorResult.crashPredicted */
    IFM_SWAP32(&buf[1872]);
    /* objectDetectionResult.crashPredictorResult.crashPredictTime */
    IFM_SWAP32(&buf[1876]);
    /* objectDetectionResult.crashPredictorResult.relImpactVelocity */
    IFM_SWAP32(&buf[1880]);
    /* objectDetectionResult.crashPredictorResult.crashObjectID */
    IFM_SWAP32(&buf[1884]);
    /* objectDetectionResult.crashPredictorResult.criticality */
    IFM_SWAP32(&buf[1888]);
    /* objectDetectionResult.crashPredictorResult.minimumZoneTriggered */
    IFM_SWAP32(&buf[1892]);
    /* objectDetectionResult.crashPredictorResult.zoneResults.zoneState */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*8)+1896]);
    }
    /* objectDetectionResult.crashPredictorResult.zoneResults.objectID */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*8)+1900]);
    }
    /* calibrationResult.commonCalibrationResult.calibValid */
    IFM_SWAP32(&buf[1920]);
    /* calibrationResult.commonCalibrationResult.calibrationStableCounter */
    IFM_SWAP32(&buf[1924]);
    /* calibrationResult.commonCalibrationResult.calibResult.transX */
    IFM_SWAP32(&buf[1928]);
    /* calibrationResult.commonCalibrationResult.calibResult.transY */
    IFM_SWAP32(&buf[1932]);
    /* calibrationResult.commonCalibrationResult.calibResult.transZ */
    IFM_SWAP32(&buf[1936]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotX */
    IFM_SWAP32(&buf[1940]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotY */
    IFM_SWAP32(&buf[1944]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotZ */
    IFM_SWAP32(&buf[1948]);
    /* calibrationResult.pacCalibrationResult.triangleDetections.score */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*40)+1956]);
    }
    /* calibrationResult.pacCalibrationResult.triangleDetections.pos3D */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1960]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2000]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2040]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2080]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2120]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2160]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2200]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2240]);
    }
    /* calibrationResult.pacCalibrationResult.triangleDetections.corners */
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+1972]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2012]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2052]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2092]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2132]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2172]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2212]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2252]);
    }
    /* calibrationResult.pacCalibrationResult.frameValid */
    IFM_SWAP32(&buf[2276]);
    /* calibrationResult.pacCalibrationResult.frameReprojectError */
    IFM_SWAP32(&buf[2280]);
    /* calibrationResult.streetCalibrationResult.planeValid */
    IFM_SWAP32(&buf[2284]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.pitchAngle */
    IFM_SWAP32(&buf[2288]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.rollAngle */
    IFM_SWAP32(&buf[2292]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.camHeight */
    IFM_SWAP32(&buf[2296]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normalx */
    IFM_SWAP32(&buf[2300]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normaly */
    IFM_SWAP32(&buf[2304]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normalz */
    IFM_SWAP32(&buf[2308]);
    /* calibrationResult.streetCalibrationResult.plausibility */
    IFM_SWAP32(&buf[2312]);
    /* calibrationResult.streetCalibrationResult.distanceDeviation */
    IFM_SWAP32(&buf[2316]);
    /* logicOutput.analogOutput */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2420]);
    }

    return res;
}
