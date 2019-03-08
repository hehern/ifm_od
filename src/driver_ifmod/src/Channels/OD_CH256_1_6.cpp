#include "OD_CH256_1_6.h"

/* Macros are used for performing endianess corrections; these might be replaced with compiler- or machine-dependent equivalents */
#define IFM_SWAP16(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[1]; (b)[1] = tmp; }
#define IFM_SWAP32(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[3]; (b)[3] = tmp; tmp = (b)[1]; (b)[1] = (b)[2]; (b)[2] = tmp; }
#define IFM_SWAP64(b) {ifm_o3m_uint8_t tmp = (b)[0]; (b)[0] = (b)[7]; (b)[7] = tmp; tmp = (b)[1]; (b)[1] = (b)[6]; (b)[6] = tmp; tmp = (b)[2]; (b)[2] = (b)[5]; (b)[5] = tmp; tmp = (b)[3]; (b)[3] = (b)[4]; (b)[4] = tmp; }

/* Casts the buffer to ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6 (if possible) and returns a pointer to it.
   Use this function on big Endian systems.

   Returns NULL in case of errors. */
ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6* ifm_o3m_ConvertBufferToBigEndian_ODK1_1_6(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6* res = (ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6*)buffer;
    if( (!buffer) || (bufferSize != 468) || (sizeof(ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6) != 468) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'K') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 6  ) ) )
    {
        return 0;
    }

    return res;
}

/* Converts the endianess of the buffer to native form and returns a pointer to ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6. 
   Note: The original buffer is modified in place. 
   Use this function on little Endian systems.
   
   Returns NULL in case of errors. */
ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6* ifm_o3m_ConvertBufferToLittleEndian_ODK1_1_6(void *buffer, ifm_o3m_uint32_t bufferSize)
{
    ifm_o3m_uint32_t i;
    ifm_o3m_uint8_t *buf = (ifm_o3m_uint8_t *)buffer;
    ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6* res = (ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6*)buffer;
    if( (!buffer) || (bufferSize != 468) || (sizeof(ifm_o3m_Meas_Customer_Data_Type_ODK1_1_6) != 468) )
    {
        return 0;
    }
    if ( ! ( (res->magic_no[0] == 'O' ) && (res->magic_no[1] == '3' ) && (res->magic_no[2] == 'M' ) && (res->magic_no[3] == '!') &&
             (res->struct_id[0] == 'O') && (res->struct_id[1] == 'D') && (res->struct_id[2] == 'K') && (res->struct_id[3] == '1') &&
             (res->version[0] == 1) && (res->version[1] == 6  ) ) )
    {
        return 0;
    }

    /* CameraMainTemperature */
    IFM_SWAP32(&buf[12]);
    /* CANBaudrate */
    IFM_SWAP32(&buf[48]);
    /* DTCList */
    for(i = 0; i < 16; i++)
    {
        IFM_SWAP32(&buf[(i*4)+56]);
    }
    /* IlluTemperature */
    IFM_SWAP32(&buf[148]);
    /* KpCustomerParameter.CANBaudrate */
    IFM_SWAP32(&buf[188]);
    /* KpCustomerParameter.VSInput0Timeout */
    IFM_SWAP16(&buf[298]);
    /* KpCustomerParameter.VSInput1Timeout */
    IFM_SWAP16(&buf[300]);
    /* KP_Info2D.s_Algo2DPrimStatistics.ui16_TotalShortAlgoPrims */
    IFM_SWAP16(&buf[306]);
    /* KP_Info2D.s_Algo2DPrimStatistics.ui16_TotalLongAlgoPrims */
    IFM_SWAP16(&buf[308]);
    /* KP_Info2D.s_Algo2DPrimStatistics.ui32_ItemsOutOfBounds */
    for(i = 0; i < 8; i++)
    {
        IFM_SWAP32(&buf[(i*4)+312]);
    }
    /* KP_Info2D.s_Fixed2DPrimStatistics.ui16_TotalShortFixedPrims */
    IFM_SWAP16(&buf[346]);
    /* KP_Info2D.s_Fixed2DPrimStatistics.ui16_TotalLongFixedPrims */
    IFM_SWAP16(&buf[348]);
    /* KP_Info2D.s_IPC2DPrimStatistics.ui16_TotalShortIPC2DPrims */
    IFM_SWAP16(&buf[352]);
    /* KP_Info2D.s_IPC2DPrimStatistics.ui16_TotalLongIPC2DPrims */
    IFM_SWAP16(&buf[354]);
    /* KP_Info2D.f32_AverageAlgoPrimsGenTime */
    IFM_SWAP32(&buf[356]);
    /* KP_Info2D.f32_AverageAlgoPrimsGenfps */
    IFM_SWAP32(&buf[360]);
    /* PasswordLocked */
    IFM_SWAP32(&buf[364]);
    /* AnalogMonValues.TempSens */
    IFM_SWAP32(&buf[368]);
    /* AnalogMonValues.Vdd5V9 */
    IFM_SWAP32(&buf[372]);
    /* AnalogMonValues.Vdd5V0 */
    IFM_SWAP32(&buf[376]);
    /* AnalogMonValues.Vdd3V3 */
    IFM_SWAP32(&buf[380]);
    /* AnalogMonValues.Vdd1V25FeR */
    IFM_SWAP32(&buf[384]);
    /* AnalogMonValues.Vdd1V25 */
    IFM_SWAP32(&buf[388]);
    /* AnalogMonValues.VddFe5V0FeR */
    IFM_SWAP32(&buf[392]);
    /* AnalogMonValues.VddFe5V0 */
    IFM_SWAP32(&buf[396]);
    /* AnalogMonValues.UBattMs */
    IFM_SWAP32(&buf[400]);
    /* AnalogMonValues.Vdd1V8 */
    IFM_SWAP32(&buf[404]);
    /* AnalogMonValues.Vdd1V8FeR */
    IFM_SWAP32(&buf[408]);
    /* AnalogMonValues.Vdd3V3FeR */
    IFM_SWAP32(&buf[412]);
    /* AnalogMonValues.V2D3V3Rflt */
    IFM_SWAP32(&buf[416]);
    /* AnalogMonValues.V2D1V3Rflt */
    IFM_SWAP32(&buf[420]);
    /* AnalogMonValues_2D.f32_ImagerTemperature */
    IFM_SWAP32(&buf[424]);
    /* AnalogMonValues_2D.f32_VoltageValue_1V8 */
    IFM_SWAP32(&buf[428]);
    /* AnalogMonValues_2D.f32_VoltageValue_1V8APT */
    IFM_SWAP32(&buf[432]);
    /* AnalogMonValues_2D.f32_VoltageValue_2V8 */
    IFM_SWAP32(&buf[436]);
    /* AnalogMonValues_2D.f32_VoltageValue_2V8A */
    IFM_SWAP32(&buf[440]);
    /* AnalogMonValues_2D.f32_Vdd2V5REF */
    IFM_SWAP32(&buf[444]);
    /* AnalogMonValues_2D.f32_AnVidSt */
    IFM_SWAP32(&buf[448]);
    /* AnalogMonValues_2D.f32_AnVidDac */
    IFM_SWAP32(&buf[452]);
    /* AnalogMonValues_Illu.f32_dkVoltagemin */
    IFM_SWAP32(&buf[456]);
    /* AnalogMonValues_Illu.f32_dkVoltagenom */
    IFM_SWAP32(&buf[460]);
    /* AnalogMonValues_Illu.f32_IlluTemp */
    IFM_SWAP32(&buf[464]);

    return res;
}
