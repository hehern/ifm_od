/*needspatch@xmldoc*/ /* This file provides the definition of the struct Meas_Customer_Data_Type in the interface version ODK1_1.6.
   
   It also provides endianess-aware functions to safely convert a binary buffer to an 
   instance of this struct (if possible).
*/

#ifndef IFM_O3M_MEAS_CUSTOMER_DATA_TYPE_ODK1_1_6_CONVERTER_H_INCLUDED
#define IFM_O3M_MEAS_CUSTOMER_DATA_TYPE_ODK1_1_6_CONVERTER_H_INCLUDED

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
    /*@xmldoc(CameraMainTemperature)*/
    ifm_o3m_float32_t CameraMainTemperature;
    /*@xmldoc(CameraPartNumber)*/
    ifm_o3m_sint8_t CameraPartNumber[6];
    /*@xmldoc(CameraSerialNumber)*/
    ifm_o3m_sint8_t CameraSerialNumber[20];
    /*@xmldoc(CameraVariant)*/
    ifm_o3m_sint8_t CameraVariant[3];
    ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_004; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_005; /* explicit padding, do not access this member */
    /*@xmldoc(CANBaudrate)*/
    ifm_o3m_uint32_t CANBaudrate;
    /*@xmldoc(CANSourceAddress)*/
    ifm_o3m_uint8_t CANSourceAddress;
    ifm_o3m_uint8_t pad_006; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_007; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_008; /* explicit padding, do not access this member */
    /*@xmldoc(DTCList)*/
    ifm_o3m_uint32_t DTCList[16];
    /*@xmldoc(IlluPartNumber)*/
    ifm_o3m_sint8_t IlluPartNumber[6];
    /*@xmldoc(IlluSerialNumber)*/
    ifm_o3m_sint8_t IlluSerialNumber[20];
    ifm_o3m_uint8_t pad_009; /* explicit padding, do not access this member */
    ifm_o3m_uint8_t pad_010; /* explicit padding, do not access this member */
    /*@xmldoc(IlluTemperature)*/
    ifm_o3m_float32_t IlluTemperature;
    /*@xmldoc(OpMode)*/
    ifm_o3m_uint8_t OpMode;
    /*@xmldoc(SoftwareVersion)*/
    ifm_o3m_uint8_t SoftwareVersion[3];
    struct
    {
        /*@xmldoc(KpCustomerParameter.BlockIdName)*/
        ifm_o3m_sint8_t BlockIdName[21];
        /*@xmldoc(KpCustomerParameter.BlockVersion)*/
        ifm_o3m_uint8_t BlockVersion[3];
        /*@xmldoc(KpCustomerParameter.Variant)*/
        ifm_o3m_sint8_t Variant[3];
        /*@xmldoc(KpCustomerParameter.CycleTime)*/
        ifm_o3m_uint8_t CycleTime;
        /*@xmldoc(KpCustomerParameter.BeginHeatingTemperature)*/
        ifm_o3m_sint8_t BeginHeatingTemperature;
        /*@xmldoc(KpCustomerParameter.StopHeatingTemperature)*/
        ifm_o3m_sint8_t StopHeatingTemperature;
        ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_002; /* explicit padding, do not access this member */
        /*@xmldoc(KpCustomerParameter.CANBaudrate)*/
        ifm_o3m_uint32_t CANBaudrate;
        /*@xmldoc(KpCustomerParameter.MasterSlaveConfiguration)*/
        ifm_o3m_uint8_t MasterSlaveConfiguration;
        /*@xmldoc(KpCustomerParameter.CANProtocol)*/
        ifm_o3m_uint8_t CANProtocol;
        /*@xmldoc(KpCustomerParameter.CANopenNodeAddress)*/
        ifm_o3m_uint8_t CANopenNodeAddress;
        /*@xmldoc(KpCustomerParameter.CANOutputCycleModulo)*/
        ifm_o3m_uint8_t CANOutputCycleModulo;
        /*@xmldoc(KpCustomerParameter.J1939SourceAddress)*/
        ifm_o3m_uint8_t J1939SourceAddress;
        /*@xmldoc(KpCustomerParameter.CANMaxNumberOfObjects)*/
        ifm_o3m_uint8_t CANMaxNumberOfObjects;
        /*@xmldoc(KpCustomerParameter.DefaultParameterActive)*/
        ifm_o3m_uint8_t DefaultParameterActive;
        /*@xmldoc(KpCustomerParameter.UserTextFieldRW)*/
        ifm_o3m_uint8_t UserTextFieldRW[64];
        /*@xmldoc(KpCustomerParameter.UserPassword)*/
        ifm_o3m_uint8_t UserPassword[30];
        /*@xmldoc(KpCustomerParameter.VSCanOutput1OnOff)*/
        ifm_o3m_uint8_t VSCanOutput1OnOff;
        /*@xmldoc(KpCustomerParameter.VSCanOutput2OnOff)*/
        ifm_o3m_uint8_t VSCanOutput2OnOff;
        /*@xmldoc(KpCustomerParameter.VSCanOutput3OnOff)*/
        ifm_o3m_uint8_t VSCanOutput3OnOff;
        /*@xmldoc(KpCustomerParameter.OnlineParamOnOff)*/
        ifm_o3m_uint8_t OnlineParamOnOff;
        ifm_o3m_uint8_t pad_003; /* explicit padding, do not access this member */
        /*@xmldoc(KpCustomerParameter.VSInput0Timeout)*/
        ifm_o3m_uint16_t VSInput0Timeout;
        /*@xmldoc(KpCustomerParameter.VSInput1Timeout)*/
        ifm_o3m_uint16_t VSInput1Timeout;
        ifm_o3m_uint8_t pad_004; /* explicit padding, do not access this member */
        ifm_o3m_uint8_t pad_005; /* explicit padding, do not access this member */
    } KpCustomerParameter;
    struct
    {
        struct
        {
            /*@xmldoc(KP_Info2D.s_Algo2DPrimStatistics.b_AlgoPrimsUserLimitReached)*/
            ifm_o3m_uint8_t b_AlgoPrimsUserLimitReached;
            /*@xmldoc(KP_Info2D.s_Algo2DPrimStatistics.b_AlgoPrimsIPCLimitReached)*/
            ifm_o3m_uint8_t b_AlgoPrimsIPCLimitReached;
            /*@xmldoc(KP_Info2D.s_Algo2DPrimStatistics.ui16_TotalShortAlgoPrims)*/
            ifm_o3m_uint16_t ui16_TotalShortAlgoPrims;
            /*@xmldoc(KP_Info2D.s_Algo2DPrimStatistics.ui16_TotalLongAlgoPrims)*/
            ifm_o3m_uint16_t ui16_TotalLongAlgoPrims;
            /*@xmldoc(KP_Info2D.s_Algo2DPrimStatistics.ui8_NumItemsOutOfBounds)*/
            ifm_o3m_uint8_t ui8_NumItemsOutOfBounds;
            ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
            /*@xmldoc(KP_Info2D.s_Algo2DPrimStatistics.ui32_ItemsOutOfBounds)*/
            ifm_o3m_uint32_t ui32_ItemsOutOfBounds[8];
        } s_Algo2DPrimStatistics;
        struct
        {
            /*@xmldoc(KP_Info2D.s_Fixed2DPrimStatistics.b_FixedPrimsUserLimitReached)*/
            ifm_o3m_uint8_t b_FixedPrimsUserLimitReached;
            /*@xmldoc(KP_Info2D.s_Fixed2DPrimStatistics.b_FixedPrimsIPCLimitReached)*/
            ifm_o3m_uint8_t b_FixedPrimsIPCLimitReached;
            /*@xmldoc(KP_Info2D.s_Fixed2DPrimStatistics.ui16_TotalShortFixedPrims)*/
            ifm_o3m_uint16_t ui16_TotalShortFixedPrims;
            /*@xmldoc(KP_Info2D.s_Fixed2DPrimStatistics.ui16_TotalLongFixedPrims)*/
            ifm_o3m_uint16_t ui16_TotalLongFixedPrims;
        } s_Fixed2DPrimStatistics;
        struct
        {
            /*@xmldoc(KP_Info2D.s_IPC2DPrimStatistics.b_IPC2DPrimsLimitReached)*/
            ifm_o3m_uint8_t b_IPC2DPrimsLimitReached;
            ifm_o3m_uint8_t pad_001; /* explicit padding, do not access this member */
            /*@xmldoc(KP_Info2D.s_IPC2DPrimStatistics.ui16_TotalShortIPC2DPrims)*/
            ifm_o3m_uint16_t ui16_TotalShortIPC2DPrims;
            /*@xmldoc(KP_Info2D.s_IPC2DPrimStatistics.ui16_TotalLongIPC2DPrims)*/
            ifm_o3m_uint16_t ui16_TotalLongIPC2DPrims;
        } s_IPC2DPrimStatistics;
        /*@xmldoc(KP_Info2D.f32_AverageAlgoPrimsGenTime)*/
        ifm_o3m_float32_t f32_AverageAlgoPrimsGenTime;
        /*@xmldoc(KP_Info2D.f32_AverageAlgoPrimsGenfps)*/
        ifm_o3m_float32_t f32_AverageAlgoPrimsGenfps;
    } KP_Info2D;
    /*@xmldoc(PasswordLocked)*/
    ifm_o3m_uint32_t PasswordLocked;
    struct
    {
        /*@xmldoc(AnalogMonValues.TempSens)*/
        ifm_o3m_float32_t TempSens;
        /*@xmldoc(AnalogMonValues.Vdd5V9)*/
        ifm_o3m_float32_t Vdd5V9;
        /*@xmldoc(AnalogMonValues.Vdd5V0)*/
        ifm_o3m_float32_t Vdd5V0;
        /*@xmldoc(AnalogMonValues.Vdd3V3)*/
        ifm_o3m_float32_t Vdd3V3;
        /*@xmldoc(AnalogMonValues.Vdd1V25FeR)*/
        ifm_o3m_float32_t Vdd1V25FeR;
        /*@xmldoc(AnalogMonValues.Vdd1V25)*/
        ifm_o3m_float32_t Vdd1V25;
        /*@xmldoc(AnalogMonValues.VddFe5V0FeR)*/
        ifm_o3m_float32_t VddFe5V0FeR;
        /*@xmldoc(AnalogMonValues.VddFe5V0)*/
        ifm_o3m_float32_t VddFe5V0;
        /*@xmldoc(AnalogMonValues.UBattMs)*/
        ifm_o3m_float32_t UBattMs;
        /*@xmldoc(AnalogMonValues.Vdd1V8)*/
        ifm_o3m_float32_t Vdd1V8;
        /*@xmldoc(AnalogMonValues.Vdd1V8FeR)*/
        ifm_o3m_float32_t Vdd1V8FeR;
        /*@xmldoc(AnalogMonValues.Vdd3V3FeR)*/
        ifm_o3m_float32_t Vdd3V3FeR;
        /*@xmldoc(AnalogMonValues.V2D3V3Rflt)*/
        ifm_o3m_float32_t V2D3V3Rflt;
        /*@xmldoc(AnalogMonValues.V2D1V3Rflt)*/
        ifm_o3m_float32_t V2D1V3Rflt;
    } AnalogMonValues;
    struct
    {
        /*@xmldoc(AnalogMonValues_2D.f32_ImagerTemperature)*/
        ifm_o3m_float32_t f32_ImagerTemperature;
        /*@xmldoc(AnalogMonValues_2D.f32_VoltageValue_1V8)*/
        ifm_o3m_float32_t f32_VoltageValue_1V8;
        /*@xmldoc(AnalogMonValues_2D.f32_VoltageValue_1V8APT)*/
        ifm_o3m_float32_t f32_VoltageValue_1V8APT;
        /*@xmldoc(AnalogMonValues_2D.f32_VoltageValue_2V8)*/
        ifm_o3m_float32_t f32_VoltageValue_2V8;
        /*@xmldoc(AnalogMonValues_2D.f32_VoltageValue_2V8A)*/
        ifm_o3m_float32_t f32_VoltageValue_2V8A;
        /*@xmldoc(AnalogMonValues_2D.f32_Vdd2V5REF)*/
        ifm_o3m_float32_t f32_Vdd2V5REF;
        /*@xmldoc(AnalogMonValues_2D.f32_AnVidSt)*/
        ifm_o3m_float32_t f32_AnVidSt;
        /*@xmldoc(AnalogMonValues_2D.f32_AnVidDac)*/
        ifm_o3m_float32_t f32_AnVidDac;
    } AnalogMonValues_2D;
    struct
    {
        /*@xmldoc(AnalogMonValues_Illu.f32_dkVoltagemin)*/
        ifm_o3m_float32_t f32_dkVoltagemin;
        /*@xmldoc(AnalogMonValues_Illu.f32_dkVoltagenom)*/
        ifm_o3m_float32_t f32_dkVoltagenom;
        /*@xmldoc(AnalogMonValues_Illu.f32_IlluTemp)*/
        ifm_o3m_float32_t f32_IlluTemp;
    } AnalogMonValues_Illu;
} ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6;

        
/* Casts the buffer to ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6 (if possible) and returns a pointer to it.
   Use this function on big Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6* ifm_o3m_ConvertBufferToBigEndian_ODK1_1_6(void *buffer, ifm_o3m_uint32_t bufferSize);

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6. 
   Note: The original buffer is modified in place. 
   Use this function on little Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6* ifm_o3m_ConvertBufferToLittleEndian_ODK1_1_6(void *buffer, ifm_o3m_uint32_t bufferSize);

#endif
