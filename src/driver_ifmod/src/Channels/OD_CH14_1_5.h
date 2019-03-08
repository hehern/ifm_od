/*needspatch@xmldoc*/ /* This file provides the definition of the struct SDspFrameCustomerImeas_t in the interface version ODD1_1.5.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_SDSPFRAMECUSTOMERIMEAS_T_ODD1_1_5_CONVERTER_H_INCLUDED
#define IFM_O3M_SDSPFRAMECUSTOMERIMEAS_T_ODD1_1_5_CONVERTER_H_INCLUDED

#include "../../include/ifm_types.h"

/* The struct has explicit padding, so that it should be usable on any target 
   without special compiler flags or pragmas related to padding.
*/
typedef struct
{
    /*@xmldoc(magic_no)*/
    ifm_o3m_sint8_t magic_no[4];
    /*@xmldoc(struct_id)*/
    ifm_o3m_sint8_t struct_id[4];
    /*@xmldoc(version)*/
    ifm_o3m_uint8_t version[2];
    ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
    struct
    {
        /*@xmldoc(DspCustomerParameter.BlockIdName)*/
        ifm_o3m_sint8_t BlockIdName[21];
        /*@xmldoc(DspCustomerParameter.BlockVersion)*/
        ifm_o3m_uint8_t BlockVersion[3];
        /*@xmldoc(DspCustomerParameter.Variant)*/
        ifm_o3m_sint8_t Variant[3];
        /*@xmldoc(DspCustomerParameter.Ipv4AddressCamera)*/
        ifm_o3m_uint8_t Ipv4AddressCamera[4];
        /*@xmldoc(DspCustomerParameter.SubnetMask)*/
        ifm_o3m_uint8_t SubnetMask[4];
        /*@xmldoc(DspCustomerParameter.Ipv4AddressDestination)*/
        ifm_o3m_uint8_t Ipv4AddressDestination[4];
        ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.destinationUDPPort)*/
        ifm_o3m_uint16_t destinationUDPPort;
        /*@xmldoc(DspCustomerParameter.EthernetOutputConfiguration)*/
        ifm_o3m_uint8_t EthernetOutputConfiguration;
        /*@xmldoc(DspCustomerParameter.EthernetLoadConfiguration)*/
        ifm_o3m_uint8_t EthernetLoadConfiguration;
        /*@xmldoc(DspCustomerParameter.DistanceImageOnSwitch)*/
        ifm_o3m_uint8_t DistanceImageOnSwitch;
        ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_004; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.VehicleDim_xMin)*/
        ifm_o3m_float32_t VehicleDim_xMin;
        /*@xmldoc(DspCustomerParameter.VehicleDim_xMax)*/
        ifm_o3m_float32_t VehicleDim_xMax;
        /*@xmldoc(DspCustomerParameter.VehicleDim_yMin)*/
        ifm_o3m_float32_t VehicleDim_yMin;
        /*@xmldoc(DspCustomerParameter.VehicleDim_yMax)*/
        ifm_o3m_float32_t VehicleDim_yMax;
        /*@xmldoc(DspCustomerParameter.VehicleDim_zMax)*/
        ifm_o3m_float32_t VehicleDim_zMax;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_transX)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_transX;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_transY)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_transY;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_transZ)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_transZ;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_rotX)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_rotX;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_rotY)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_rotY;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_camCal_rotZ)*/
        ifm_o3m_float32_t PMDExtrCalib_camCal_rotZ;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCal_transX)*/
        ifm_o3m_float32_t PMDExtrCalib_IlluCal_transX;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCal_transY)*/
        ifm_o3m_float32_t PMDExtrCalib_IlluCal_transY;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCal_transZ)*/
        ifm_o3m_float32_t PMDExtrCalib_IlluCal_transZ;
        /*@xmldoc(DspCustomerParameter.PMDExtrCalib_IlluCalibIsRelative)*/
        ifm_o3m_uint8_t PMDExtrCalib_IlluCalibIsRelative;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_sprayRemovalSensitivity)*/
        ifm_o3m_uint8_t ObjectListCust_sprayRemovalSensitivity;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_pixelPlausibilizationThresholds)*/
        ifm_o3m_uint8_t ObjectListCust_pixelPlausibilizationThresholds;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_blockageSensitivity)*/
        ifm_o3m_uint8_t ObjectListCust_blockageSensitivity;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_spatialFilterXMin)*/
        ifm_o3m_float32_t ObjectListCust_spatialFilterXMin;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_spatialFilterXMax)*/
        ifm_o3m_float32_t ObjectListCust_spatialFilterXMax;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_spatialFilterYMin)*/
        ifm_o3m_float32_t ObjectListCust_spatialFilterYMin;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_spatialFilterYMax)*/
        ifm_o3m_float32_t ObjectListCust_spatialFilterYMax;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_spatialFilterZMin)*/
        ifm_o3m_float32_t ObjectListCust_spatialFilterZMin;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_spatialFilterZMax)*/
        ifm_o3m_float32_t ObjectListCust_spatialFilterZMax;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_reflectorThresholdValue)*/
        ifm_o3m_float32_t ObjectListCust_reflectorThresholdValue;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_autocalibrationMode)*/
        ifm_o3m_uint8_t ObjectListCust_autocalibrationMode;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_objectDetectionVariant)*/
        ifm_o3m_uint8_t ObjectListCust_objectDetectionVariant;
        ifm_o3m_uint8_t pad_005; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_006; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.ObjectListCust_ObjectDetectionZMin)*/
        ifm_o3m_float32_t ObjectListCust_ObjectDetectionZMin;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_ObjectDetectionZMax)*/
        ifm_o3m_float32_t ObjectListCust_ObjectDetectionZMax;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_CrashPredictorSensitivity)*/
        ifm_o3m_uint8_t ObjectListCust_CrashPredictorSensitivity;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_EgoMotionDynamics)*/
        ifm_o3m_uint8_t ObjectListCust_EgoMotionDynamics;
        ifm_o3m_uint8_t pad_007; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_008; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.ObjectListCust_accBrake)*/
        ifm_o3m_float32_t ObjectListCust_accBrake;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_delayBrake)*/
        ifm_o3m_float32_t ObjectListCust_delayBrake;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_egoVMin)*/
        ifm_o3m_float32_t ObjectListCust_egoVMin;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_egoVMax)*/
        ifm_o3m_float32_t ObjectListCust_egoVMax;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_EgoDataMode)*/
        ifm_o3m_uint8_t ObjectListCust_EgoDataMode;
        ifm_o3m_uint8_t pad_009; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_010; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_011; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.ObjectListCust_cpDeactivateTimeAfterTrigger)*/
        ifm_o3m_float32_t ObjectListCust_cpDeactivateTimeAfterTrigger;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_cpMaxCrashObjectDistance)*/
        ifm_o3m_float32_t ObjectListCust_cpMaxCrashObjectDistance;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_cpMinDistAllowed)*/
        ifm_o3m_float32_t ObjectListCust_cpMinDistAllowed;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_cpActivate)*/
        ifm_o3m_uint8_t ObjectListCust_cpActivate;
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_numberOfPatterns)*/
        ifm_o3m_uint8_t AutoCalibParam_numberOfPatterns;
        ifm_o3m_uint8_t pad_012; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_013; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_xPattern)*/
        ifm_o3m_float32_t AutoCalibParam_xPattern[8];
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_yPattern)*/
        ifm_o3m_float32_t AutoCalibParam_yPattern[8];
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_zPattern)*/
        ifm_o3m_float32_t AutoCalibParam_zPattern[8];
        /*@xmldoc(DspCustomerParameter.AutoCalibParam_patternType)*/
        ifm_o3m_uint8_t AutoCalibParam_patternType[8];
        /*@xmldoc(DspCustomerParameter.twoD_mirrorX)*/
        ifm_o3m_uint8_t twoD_mirrorX;
        /*@xmldoc(DspCustomerParameter.twoD_mirrorY)*/
        ifm_o3m_uint8_t twoD_mirrorY;
        /*@xmldoc(DspCustomerParameter.triggeredStreetCalibration)*/
        ifm_o3m_uint8_t triggeredStreetCalibration;
        ifm_o3m_uint8_t pad_014; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.num_Frames_Averaging)*/
        ifm_o3m_sint32_t num_Frames_Averaging;
        /*@xmldoc(DspCustomerParameter.pixelPlausiReflectivityThreshold)*/
        ifm_o3m_float32_t pixelPlausiReflectivityThreshold;
        /*@xmldoc(DspCustomerParameter.Modulation_Frequency_Mode)*/
        ifm_o3m_uint8_t Modulation_Frequency_Mode;
        ifm_o3m_uint8_t pad_015; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_016; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_017; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.amplThresholdFactor)*/
        ifm_o3m_float32_t amplThresholdFactor;
        struct
        {
            struct
            {
                /*@xmldoc(DspCustomerParameter.Logic.logicGraph.operation)*/
                ifm_o3m_uint16_t operation;
                /*@xmldoc(DspCustomerParameter.Logic.logicGraph.inputs)*/
                ifm_o3m_uint16_t inputs[3];
            } logicGraph[500];
            struct
            {
                /*@xmldoc(DspCustomerParameter.Logic.imeasIntrospection.channelID)*/
                ifm_o3m_uint16_t channelID;
                /*@xmldoc(DspCustomerParameter.Logic.imeasIntrospection.expression)*/
                ifm_o3m_sint8_t expression[32];
            } imeasIntrospection[25];
        } Logic;
        ifm_o3m_uint8_t pad_018; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_019; /* explicit padding, do not access this member */
        /*@xmldoc(DspCustomerParameter.ObjectListCust_delayBrake2)*/
        ifm_o3m_float32_t ObjectListCust_delayBrake2;
        /*@xmldoc(DspCustomerParameter.ObjectListCust_delayBrake3)*/
        ifm_o3m_float32_t ObjectListCust_delayBrake3;
        /*@xmldoc(DspCustomerParameter.zoneBasedWarning_Xmin)*/
        ifm_o3m_float32_t zoneBasedWarning_Xmin[3];
        /*@xmldoc(DspCustomerParameter.zoneBasedWarning_Xmax)*/
        ifm_o3m_float32_t zoneBasedWarning_Xmax[3];
        /*@xmldoc(DspCustomerParameter.zoneBasedWarning_Ymin)*/
        ifm_o3m_float32_t zoneBasedWarning_Ymin[3];
        /*@xmldoc(DspCustomerParameter.zoneBasedWarning_Ymax)*/
        ifm_o3m_float32_t zoneBasedWarning_Ymax[3];
        /*@xmldoc(DspCustomerParameter.RotateImage90)*/
        ifm_o3m_uint8_t RotateImage90;
        /*@xmldoc(DspCustomerParameter.Reserved)*/
        ifm_o3m_uint8_t Reserved[3];
        /*@xmldoc(DspCustomerParameter.RoD_Xmin)*/
        ifm_o3m_float32_t RoD_Xmin[2];
        /*@xmldoc(DspCustomerParameter.RoD_Xmax)*/
        ifm_o3m_float32_t RoD_Xmax[2];
        /*@xmldoc(DspCustomerParameter.RoD_Ymin)*/
        ifm_o3m_float32_t RoD_Ymin[2];
        /*@xmldoc(DspCustomerParameter.RoD_Ymax)*/
        ifm_o3m_float32_t RoD_Ymax[2];
        /*@xmldoc(DspCustomerParameter.RoD_Zmin)*/
        ifm_o3m_float32_t RoD_Zmin[2];
        /*@xmldoc(DspCustomerParameter.RoD_Zmax)*/
        ifm_o3m_float32_t RoD_Zmax[2];
        /*@xmldoc(DspCustomerParameter.RoD2D_Xmin)*/
        ifm_o3m_uint8_t RoD2D_Xmin[3];
        /*@xmldoc(DspCustomerParameter.RoD2D_Xmax)*/
        ifm_o3m_uint8_t RoD2D_Xmax[3];
        /*@xmldoc(DspCustomerParameter.RoD2D_Ymin)*/
        ifm_o3m_uint8_t RoD2D_Ymin[3];
        /*@xmldoc(DspCustomerParameter.RoD2D_Ymax)*/
        ifm_o3m_uint8_t RoD2D_Ymax[3];
    } DspCustomerParameter;
} ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5;

        
/* Casts the buffer to ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5 (if possible) and returns a pointer to it.
   Use this function on little Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5* ifm_o3m_ConvertBufferToLittleEndian_ODD1_1_5(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5. 
   Note: The original buffer is modified in place. 
   Use this function on big Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_SDspFrameCustomerImeas_t_ODD1_1_5* ifm_o3m_ConvertBufferToBigEndian_ODD1_1_5(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
