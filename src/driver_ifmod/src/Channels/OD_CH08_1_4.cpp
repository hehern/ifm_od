#include "OD_CH08_1_4.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_AlgoIFOutput_ODA1_1_4 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_ODA1_1_4* ifm_o3m_ConvertBufferToLittleEndian_ODA1_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_AlgoIFOutput_ODA1_1_4* res = (ifm_o3m_AlgoIFOutput_ODA1_1_4*)buffer;
    if( (!buffer) || (bufferSize != 20952) || (sizeof(ifm_o3m_AlgoIFOutput_ODA1_1_4) != 20952) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'A') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_AlgoIFOutput_ODA1_1_4. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_AlgoIFOutput_ODA1_1_4* ifm_o3m_ConvertBufferToBigEndian_ODA1_1_4(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_AlgoIFOutput_ODA1_1_4* res = (ifm_o3m_AlgoIFOutput_ODA1_1_4*)buffer;
    if( (!buffer) || (bufferSize != 20952) || (sizeof(ifm_o3m_AlgoIFOutput_ODA1_1_4) != 20952) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'A') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 4  ) ) )
    {
        return 0;
    }

    /* distanceImageResult.sensorWidth */
    IFM_SWAP16(&buf[12]);
    /* distanceImageResult.sensorHeight */
    IFM_SWAP16(&buf[14]);
    /* distanceImageResult.distanceData */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP16(&buf[(i*2)+16]);
    }
    /* distanceImageResult.X */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP32(&buf[(i*4)+2064]);
    }
    /* distanceImageResult.Y */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP32(&buf[(i*4)+6160]);
    }
    /* distanceImageResult.Z */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP32(&buf[(i*4)+10256]);
    }
    /* distanceImageResult.confidence */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP16(&buf[(i*2)+14352]);
    }
    /* distanceImageResult.amplitude */
    for(i = 0; i < 1024; i++)
    {
        IFM_SWAP16(&buf[(i*2)+16400]);
    }
    /* distanceImageResult.amplitude_normalization */
    for(i = 0; i < 4; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18448]);
    }
    /* distanceImageResult.masterclockTimestamp */
    IFM_SWAP32(&buf[18464]);
    /* distanceImageResult.frameCounter */
    IFM_SWAP32(&buf[18468]);
    /* distanceImageResult.available */
    IFM_SWAP32(&buf[18472]);
    /* distanceImageResult.cameraCalibration.transX */
    IFM_SWAP32(&buf[18476]);
    /* distanceImageResult.cameraCalibration.transY */
    IFM_SWAP32(&buf[18480]);
    /* distanceImageResult.cameraCalibration.transZ */
    IFM_SWAP32(&buf[18484]);
    /* distanceImageResult.cameraCalibration.rotX */
    IFM_SWAP32(&buf[18488]);
    /* distanceImageResult.cameraCalibration.rotY */
    IFM_SWAP32(&buf[18492]);
    /* distanceImageResult.cameraCalibration.rotZ */
    IFM_SWAP32(&buf[18496]);
    /* distanceImageResult.fieldOfView.upperLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18500]);
    }
    /* distanceImageResult.fieldOfView.upperRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18512]);
    }
    /* distanceImageResult.fieldOfView.lowerLeft */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18524]);
    }
    /* distanceImageResult.fieldOfView.lowerRight */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18536]);
    }
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_2D_fx */
    IFM_SWAP32(&buf[18548]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_2D_fy */
    IFM_SWAP32(&buf[18552]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_2D_mx */
    IFM_SWAP32(&buf[18556]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_2D_my */
    IFM_SWAP32(&buf[18560]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_alpha */
    IFM_SWAP32(&buf[18564]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_k1 */
    IFM_SWAP32(&buf[18568]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_k2 */
    IFM_SWAP32(&buf[18572]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_k5 */
    IFM_SWAP32(&buf[18576]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_k3 */
    IFM_SWAP32(&buf[18580]);
    /* distanceImageResult.intrExtrCalib_2d.intrCalib_k4 */
    IFM_SWAP32(&buf[18584]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_center_tx */
    IFM_SWAP32(&buf[18588]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_center_ty */
    IFM_SWAP32(&buf[18592]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_center_tz */
    IFM_SWAP32(&buf[18596]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_delta_tx */
    IFM_SWAP32(&buf[18600]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_delta_ty */
    IFM_SWAP32(&buf[18604]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_delta_tz */
    IFM_SWAP32(&buf[18608]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_rot_x */
    IFM_SWAP32(&buf[18612]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_rot_y */
    IFM_SWAP32(&buf[18616]);
    /* distanceImageResult.intrExtrCalib_2d.extrCalib_rot_z */
    IFM_SWAP32(&buf[18620]);
    /* distanceImageResult.illuPosition */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+18624]);
    }
    /* distanceImageResult.blockageRatio */
    IFM_SWAP32(&buf[18636]);
    /* objectDetectionResult.objectList.id */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18644]);
    }
    /* objectDetectionResult.objectList.history */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18648]);
    }
    /* objectDetectionResult.objectList.measured */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18652]);
    }
    /* objectDetectionResult.objectList.type */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18656]);
    }
    /* objectDetectionResult.objectList.age */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18660]);
    }
    /* objectDetectionResult.objectList.x1 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18664]);
    }
    /* objectDetectionResult.objectList.y1 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18668]);
    }
    /* objectDetectionResult.objectList.x2 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18672]);
    }
    /* objectDetectionResult.objectList.y2 */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18676]);
    }
    /* objectDetectionResult.objectList.zMin */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18680]);
    }
    /* objectDetectionResult.objectList.zMax */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18684]);
    }
    /* objectDetectionResult.objectList.vX */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18688]);
    }
    /* objectDetectionResult.objectList.vY */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18692]);
    }
    /* objectDetectionResult.objectList.vZ */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18696]);
    }
    /* objectDetectionResult.objectList.aX */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18700]);
    }
    /* objectDetectionResult.objectList.aY */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18704]);
    }
    /* objectDetectionResult.objectList.aZ */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18708]);
    }
    /* objectDetectionResult.objectList.existenceProbability */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18712]);
    }
    /* objectDetectionResult.objectList.vXQuality */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18716]);
    }
    /* objectDetectionResult.objectList.vYQuality */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18720]);
    }
    /* objectDetectionResult.objectList.distanceToEgo */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*84)+18724]);
    }
    /* objectDetectionResult.crashPredictorResult.crashPredicted */
    IFM_SWAP32(&buf[20324]);
    /* objectDetectionResult.crashPredictorResult.crashPredictTime */
    IFM_SWAP32(&buf[20328]);
    /* objectDetectionResult.crashPredictorResult.relImpactVelocity */
    IFM_SWAP32(&buf[20332]);
    /* objectDetectionResult.crashPredictorResult.crashObjectID */
    IFM_SWAP32(&buf[20336]);
    /* objectDetectionResult.crashPredictorResult.criticality */
    IFM_SWAP32(&buf[20340]);
    /* objectDetectionResult.crashPredictorResult.minimumZoneTriggered */
    IFM_SWAP32(&buf[20344]);
    /* objectDetectionResult.crashPredictorResult.zoneResults.zoneState */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*8)+20348]);
    }
    /* objectDetectionResult.crashPredictorResult.zoneResults.objectID */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*8)+20352]);
    }
    /* calibrationResult.commonCalibrationResult.calibValid */
    IFM_SWAP32(&buf[20372]);
    /* calibrationResult.commonCalibrationResult.calibrationStableCounter */
    IFM_SWAP32(&buf[20376]);
    /* calibrationResult.commonCalibrationResult.calibResult.transX */
    IFM_SWAP32(&buf[20380]);
    /* calibrationResult.commonCalibrationResult.calibResult.transY */
    IFM_SWAP32(&buf[20384]);
    /* calibrationResult.commonCalibrationResult.calibResult.transZ */
    IFM_SWAP32(&buf[20388]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotX */
    IFM_SWAP32(&buf[20392]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotY */
    IFM_SWAP32(&buf[20396]);
    /* calibrationResult.commonCalibrationResult.calibResult.rotZ */
    IFM_SWAP32(&buf[20400]);
    /* calibrationResult.pacCalibrationResult.triangleDetections.score */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*40)+20408]);
    }
    /* calibrationResult.pacCalibrationResult.triangleDetections.pos3D */
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20412]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20452]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20492]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20532]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20572]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20612]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20652]);
    }
    for(i = 0; i < 3; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20692]);
    }
    /* calibrationResult.pacCalibrationResult.triangleDetections.corners */
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20424]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20464]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20504]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20544]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20584]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20624]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20664]);
    }
    for(i = 0; i < 6; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20704]);
    }
    /* calibrationResult.pacCalibrationResult.frameValid */
    IFM_SWAP32(&buf[20728]);
    /* calibrationResult.pacCalibrationResult.frameReprojectError */
    IFM_SWAP32(&buf[20732]);
    /* calibrationResult.streetCalibrationResult.planeValid */
    IFM_SWAP32(&buf[20736]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.pitchAngle */
    IFM_SWAP32(&buf[20740]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.rollAngle */
    IFM_SWAP32(&buf[20744]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.camHeight */
    IFM_SWAP32(&buf[20748]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normalx */
    IFM_SWAP32(&buf[20752]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normaly */
    IFM_SWAP32(&buf[20756]);
    /* calibrationResult.streetCalibrationResult.planeEstimation.normalz */
    IFM_SWAP32(&buf[20760]);
    /* calibrationResult.streetCalibrationResult.plausibility */
    IFM_SWAP32(&buf[20764]);
    /* calibrationResult.streetCalibrationResult.distanceDeviation */
    IFM_SWAP32(&buf[20768]);
    /* logicOutput.analogOutput */
    for(i = 0; i < 20; i++)
    {
        IFM_SWAP32(&buf[(i*4)+20872]);
    }

    return res;
}
