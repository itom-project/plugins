/*
 * 1394-Based Digital Camera Control Library
 *
 * Allied Vision Technologies (AVT) specific extensions
 *
 * Written by Pierre MOOS <pierre.moos@gmail.com>
 *
 * Copyright (C) 2005 Inria Sophia-Antipolis
 *
 * Additions by Allied Vision Technologies GmbH.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "vendor/avt.h"

#include "avt_csr_structs_adv.h"

#ifndef MIN
  #define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
  #define MAX(a,b) ((a) < (b) ? (b) : (a))
#endif

/********************************************************/
/* Configuration Register Offsets for Advances features */
/********************************************************/

#define REG_CAMERA_AVT_VERSION_INFO1                    0x010U
#define REG_CAMERA_AVT_VERSION_INFO3                    0x018U
#define REG_CAMERA_AVT_ADV_INQ_1                        0x040U
#define REG_CAMERA_AVT_ADV_INQ_2                        0x044U
#define REG_CAMERA_AVT_ADV_INQ_3                        0x048U
#define REG_CAMERA_AVT_ADV_INQ_4                        0x04CU
#define REG_CAMERA_AVT_MAX_RESOLUTION                   0x200U
#define REG_CAMERA_AVT_TIMEBASE                         0x208U
#define REG_CAMERA_AVT_EXTD_SHUTTER                     0x20CU
#define REG_CAMERA_AVT_TEST_IMAGE                       0x210U
#define REG_CAMERA_AVT_SEQUENCE_CTRL                    0x220U
#define REG_CAMERA_AVT_SEQUENCE_PARAM                   0x224U
#define REG_CAMERA_AVT_LUT_CTRL                         0x240U
#define REG_CAMERA_AVT_LUT_MEM_CTRL                     0x244U
#define REG_CAMERA_AVT_LUT_INFO                         0x248U
#define REG_CAMERA_AVT_SHDG_CTRL                        0x250U
#define REG_CAMERA_AVT_SHDG_MEM_CTRL                    0x254U
#define REG_CAMERA_AVT_SHDG_INFO                        0x258U
#define REG_CAMERA_AVT_DEFERRED_TRANS                   0x260U
#define REG_CAMERA_AVT_FRAMEINFO                        0x270U
#define REG_CAMERA_AVT_FRAMECOUNTER                     0x274U
#define REG_CAMERA_AVT_HDR_CONTROL                      0x280U
#define REG_CAMERA_AVT_KNEEPOINT_1                      0x284U
#define REG_CAMERA_AVT_KNEEPOINT_2                      0x288U
#define REG_CAMERA_AVT_KNEEPOINT_3                      0x28CU
#define REG_CAMERA_AVT_DSNU_CONTROL                     0x290U
#define REG_CAMERA_AVT_BLEMISH_CONTROL                  0x294U
#define REG_CAMERA_AVT_DPC_CONTROL                      0x298U
#define REG_CAMERA_AVT_HDR_PIKE_CONTROL                 0x2B0U
#define REG_CAMERA_AVT_IO_INP_CTRL1                     0x300U
#define REG_CAMERA_AVT_IO_INP_CTRL2                     0x304U
#define REG_CAMERA_AVT_IO_INP_CTRL3                     0x308U
#define REG_CAMERA_AVT_IO_INP_CTRL4                     0x30CU
#define REG_CAMERA_AVT_IO_INP_CTRL5                     0x310U
#define REG_CAMERA_AVT_IO_INP_CTRL6                     0x314U
#define REG_CAMERA_AVT_IO_INP_CTRL7                     0x318U
#define REG_CAMERA_AVT_IO_INP_CTRL8                     0x31CU
#define REG_CAMERA_AVT_IO_OUTP_CTRL1                    0x320U
#define REG_CAMERA_AVT_IO_OUTP_CTRL2                    0x324U
#define REG_CAMERA_AVT_IO_OUTP_CTRL3                    0x328U
#define REG_CAMERA_AVT_IO_OUTP_CTRL4                    0x32CU
#define REG_CAMERA_AVT_IO_OUTP_CTRL5                    0x330U
#define REG_CAMERA_AVT_IO_OUTP_CTRL6                    0x334U
#define REG_CAMERA_AVT_IO_OUTP_CTRL7                    0x338U
#define REG_CAMERA_AVT_IO_OUTP_CTRL8                    0x33CU
#define REG_CAMERA_AVT_IO_INTENA_DELAY                  0x340U
#define REG_CAMERA_AVT_AUTOSHUTTER_CTRL                 0x360U
#define REG_CAMERA_AVT_AUTOSHUTTER_LO                   0x364U
#define REG_CAMERA_AVT_AUTOSHUTTER_HI                   0x368U
#define REG_CAMERA_AVT_AUTOGAIN_CTRL                    0x370U
#define REG_CAMERA_AVT_AUTOFNC_AOI                      0x390U
#define REG_CAMERA_AVT_AF_AREA_POSITION                 0x394U
#define REG_CAMERA_AVT_AF_AREA_SIZE                     0x398U
#define REG_CAMERA_AVT_COLOR_CORR                       0x3A0U
#define REG_CAMERA_AVT_COLOR_CORR_CRR                   0x3A4U
#define REG_CAMERA_AVT_COLOR_CORR_CGR                   0x3A8U
#define REG_CAMERA_AVT_COLOR_CORR_CBR                   0x3ACU
#define REG_CAMERA_AVT_COLOR_CORR_CRG                   0x3B0U
#define REG_CAMERA_AVT_COLOR_CORR_CGG                   0x3B4U
#define REG_CAMERA_AVT_COLOR_CORR_CBG                   0x3B8U
#define REG_CAMERA_AVT_COLOR_CORR_CRB                   0x3BCU
#define REG_CAMERA_AVT_COLOR_CORR_CGB                   0x3C0U
#define REG_CAMERA_AVT_COLOR_CORR_CBB                   0x3C4U
#define REG_CAMERA_AVT_TRIGGER_DELAY                    0x400U
#define REG_CAMERA_AVT_MIRROR_IMAGE                     0x410U
#define REG_CAMERA_AVT_CHANNEL_ADJUST_CTRL              0x420U
#define REG_CAMERA_AVT_CHANNEL_ADJUST_VALUE             0x424U
#define REG_CAMERA_AVT_CHANNEL_ADJUST_OFFSET_CTRL       0x430U
#define REG_CAMERA_AVT_CHANNEL_ADJUST_OFFSET_VALUE      0x434U
#define REG_CAMERA_AVT_LOW_SMEAR                        0x440U
#define REG_CAMERA_AVT_SOFT_RESET                       0x510U
#define REG_CAMERA_AVT_HSNRR                            0x520U
#define REG_CAMERA_AVT_TIMESTAMP                        0x540U
#define REG_CAMERA_AVT_USER_PROFILE                     0x550U
#define REG_CAMERA_AVT_MAX_ISO_SIZE                     0x560U
#define REG_CAMERA_AVT_PARAMUPD_MODE                    0x570U
#define REG_CAMERA_AVT_LOW_NOISE_BINNING                0x5B0U
#define REG_CAMERA_AVT_GLOBAL_RESET_RELEASE_SHUTTER     0x5C0U
#define REG_CAMERA_AVT_TIMESTAMP_NEW                    0x600U
#define REG_CAMERA_AVT_FRAMEINFO_NEW                    0x610U
#define REG_CAMERA_AVT_TRGCOUNTER_NEW                   0x620U
#define REG_CAMERA_AVT_SIS                              0x630U
#define REG_CAMERA_AVT_SWFEATURE                        0x640U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL1                0x800U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL2                0x804U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL3                0x808U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL4                0x80CU
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL5                0x810U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL6                0x814U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL7                0x818U
#define REG_CAMERA_AVT_IO_OUTP_PWM_CTRL8                0x81CU
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL1            0x840U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL2            0x850U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL3            0x860U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL4            0x870U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL5            0x880U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL6            0x890U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL7            0x8A0U
#define REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL8            0x8B0U

#define REG_CAMERA_AVT_GPDATA_INFO                      0xFFCU
#define REG_CAMERA_AVT_GPDATA_BUFFER                    0x1000U


#define SIZE_AVT_SMART_FEATURE_STRUCT_V1    584


/************************************************************************/
/* Get Version          (Read Only)                                     */
/************************************************************************/
dc1394error_t
dc1394_avt_get_version(dc1394camera_t *camera,
                       uint32_t *UCType, uint32_t *Version,
                       uint32_t *Camera_ID, uint32_t *FPGA_Version)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve uC */
    *UCType =dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_VERSION_INFO1,&value);

    /* uC Version : Bits 16..31 */
    *Version =(uint32_t)(value & 0xFFFFUL );

    /*  Retrieve Camera ID and FPGA_Version */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_VERSION_INFO3, &value);
    DC1394_ERR_RTN(err,"Could not get AVT version info 3");

    /* Camera_ID : bit 0-15 */
    *Camera_ID =(uint32_t)(value >>16 );

    /* FPGA_Version : bit 16-31 */
    *FPGA_Version=(uint32_t)(value & 0xFFFFUL );

    return DC1394_SUCCESS;

}


/************************************************************************/
/* Adjust Frames                                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_adjust_frames(uint32_t Camera_ID, dc1394video_frame_t *frame)
{
    /* do nothing when frame is captured by a non-AVT camera */
	if( NULL!=frame->camera && frame->camera->vendor_id!=DC1394_AVT_VENDOR_ID )
		return DC1394_SUCCESS;

	switch( frame->color_coding )
    	{
        case DC1394_COLOR_CODING_MONO8:
        case DC1394_COLOR_CODING_MONO16:
		break;
	default:
		return DC1394_SUCCESS;
	}

    switch( Camera_ID )
    {
        case DC1394_AVT_CAMERA_ID_DF145C:
        case DC1394_AVT_CAMERA_ID_DF201C:
        case DC1394_AVT_CAMERA_ID_DF145C_1:
        case DC1394_AVT_CAMERA_ID_DF201C_1:
        case DC1394_AVT_CAMERA_ID_MF033C:
        case DC1394_AVT_CAMERA_ID_MF046C:
        case DC1394_AVT_CAMERA_ID_MF080C:
        case DC1394_AVT_CAMERA_ID_MF145C:
        case DC1394_AVT_CAMERA_ID_MF145C3:
        case DC1394_AVT_CAMERA_ID_M2F033C:
        case DC1394_AVT_CAMERA_ID_M2F046C:
        case DC1394_AVT_CAMERA_ID_M2F080C:
        case DC1394_AVT_CAMERA_ID_M2F145C:
        case DC1394_AVT_CAMERA_ID_M2F145C3:
        case DC1394_AVT_CAMERA_ID_M2F080C_30FPS:
        case DC1394_AVT_CAMERA_ID_M2F145C4:
        case DC1394_AVT_CAMERA_ID_M2F201C:
        case DC1394_AVT_CAMERA_ID_M2F146C:
        case DC1394_AVT_CAMERA_ID_GF033C:
        case DC1394_AVT_CAMERA_ID_GF046C:
        case DC1394_AVT_CAMERA_ID_GF080C:
        case DC1394_AVT_CAMERA_ID_GF033C_BL:
        case DC1394_AVT_CAMERA_ID_GF025C:
        case DC1394_AVT_CAMERA_ID_GF029C:
        case DC1394_AVT_CAMERA_ID_GF038C:
        case DC1394_AVT_CAMERA_ID_GF038C_NIR:
        case DC1394_AVT_CAMERA_ID_GF044C_NIR:
        case DC1394_AVT_CAMERA_ID_GF080C_BL:
        case DC1394_AVT_CAMERA_ID_GF044C:
        case DC1394_AVT_CAMERA_ID_GF146C:
        case DC1394_AVT_CAMERA_ID_GF503C:
            frame->color_filter = DC1394_COLOR_FILTER_RGGB;
            break;
        case DC1394_AVT_CAMERA_ID_MF131C:
            frame->color_filter = DC1394_COLOR_FILTER_GBRG;
            break;
        case DC1394_AVT_CAMERA_ID_GF036C:
            frame->color_filter = DC1394_COLOR_FILTER_GRBG;
            break;

        default:
            return DC1394_SUCCESS;
    }

    if( DC1394_COLOR_CODING_MONO8 == frame->color_coding )
    {
        frame->color_coding = DC1394_COLOR_CODING_RAW8;
    }
    if( DC1394_COLOR_CODING_MONO16 == frame->color_coding )
    {
        frame->color_coding = DC1394_COLOR_CODING_RAW16;
    }

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Advanced feature inquiry  - deprecated                           */
/************************************************************************/
dc1394error_t
dc1394_avt_get_advanced_feature_inquiry(dc1394camera_t *camera,
                                        dc1394_avt_adv_feature_info_t *adv_feature)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve first group of features presence */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_ADV_INQ_1, &value);
    DC1394_ERR_RTN(err,"Could not get AVT advanced features INQ 1");

    adv_feature->MaxResolution=                (value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TimeBase=                     (value & 0x40000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->ExtdShutter=                  (value & 0x20000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TestImage=                    (value & 0x10000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->FrameInfo=                    (value & 0x08000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Sequences=                    (value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->VersionInfo=                  (value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_1 7
    adv_feature->Lookup_Tables=                (value & 0x00800000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Shading=                      (value & 0x00400000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->DeferredTrans=                (value & 0x00200000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->HDR_Mode=                     (value & 0x00100000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->DSNU=                         (value & 0x00080000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->BlemishCorrection=            (value & 0x00040000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TriggerDelay=                 (value & 0x00020000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->MirrorImage=                  (value & 0x00010000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->SoftReset=                    (value & 0x00008000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->HSNR=                         (value & 0x00004000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->ColorCorrection=              (value & 0x00002000UL) ? DC1394_TRUE : DC1394_FALSE;

    adv_feature->UserProfiles=                 (value & 0x00001000UL) ? DC1394_TRUE : DC1394_FALSE; /* whats this? */
    //ADV_INQ_1 20
    adv_feature->UserSets=                     (value & 0x00000400UL) ? DC1394_TRUE : DC1394_FALSE; /* 'user profiles' */
    adv_feature->TimeStamp=                    (value & 0x00000200UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->FrmCntStamp=                  (value & 0x00000100UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TrgCntStamp=                  (value & 0x00000080UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_1 25-30
    adv_feature->GP_Buffer=                    (value & 0x00000001UL) ? DC1394_TRUE : DC1394_FALSE;

    /* Remember this request have been done */
    adv_feature->features_requested = DC1394_TRUE;

    /* Retrieve second group of features presence */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_ADV_INQ_2, &value);
    DC1394_ERR_RTN(err,"Could not get AVT advanced features INQ 2");

    adv_feature->Input_1 =                        (value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Input_2 =                        (value & 0x40000000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_2 2-7
    adv_feature->Output_1=                        (value & 0x00800000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Output_2=                        (value & 0x00400000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Output_3=                        (value & 0x00200000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Output_4=                        (value & 0x00100000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_2 12-15
    adv_feature->IntEnaDelay=                     (value & 0x00008000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->IncDecoder=                      (value & 0x00004000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_2 18-31

    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_ADV_INQ_3, &value);
    DC1394_ERR_RTN(err,"Could not get AVT advanced features INQ 3");

    adv_feature->CameraStatus=                    (value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_3 1-3
    adv_feature->AutoShutter=                     (value & 0x08000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->AutoGain=                        (value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->AutoFunctionAOI=                 (value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_3 7-31

    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_ADV_INQ_4, &value);
    DC1394_ERR_RTN(err,"Could not get AVT advanced features INQ 4");

    adv_feature->HDRPike=                         (value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    //ADV_INQ_4 1-31

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Advanced ('smart') feature inquiry                               */
/************************************************************************/
dc1394error_t dc1394_avt_get_smart_feature_inquiry(dc1394camera_t *camera,
                                                   dc1394_avt_smart_feature_info_t *smart_feature,
                                                   int size )
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394error_t err;
    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) smart_feature;

    memset ( smart_feature_full, 0, size );
    smart_feature_full->Size = size;

    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_ADV_INQ_1,
                                             (uint32_t*) &smart_feature_full->Internal.Inq1,
                                             4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced features INQ");

    smart_feature_full->MaxResolution      = (smart_feature_full->Internal.Inq1.m_bMaxResolution != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->TimeBase           = (smart_feature_full->Internal.Inq1.m_bTimeBase != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ExtdShutter        = (smart_feature_full->Internal.Inq1.m_bExtdShutter != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->TestImage          = (smart_feature_full->Internal.Inq1.m_bTestImage != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->FrameCounter       = (smart_feature_full->Internal.Inq1.m_bFrmCounter2 != 0 ||
                                              smart_feature_full->Internal.Inq1.m_bFrameInfo != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Sequences          = (smart_feature_full->Internal.Inq1.m_bSequences != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->VersionInfo        = (smart_feature_full->Internal.Inq1.m_bVersionInfo != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Lookup_Tables      = (smart_feature_full->Internal.Inq1.m_bLut != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Shading            = (smart_feature_full->Internal.Inq1.m_bShading != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->DeferredTransport  = (smart_feature_full->Internal.Inq1.m_bDeferredTrans != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->HDR_Mode           = (smart_feature_full->Internal.Inq1.m_bCmosHdrMode != 0 ||
                                              smart_feature_full->Internal.Inq4.m_bPikeHdrMode != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->DSNU               = (smart_feature_full->Internal.Inq1.m_bFpnCorrection != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->BlemishCorrection  = (smart_feature_full->Internal.Inq1.m_bBlemishCorr != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->TriggerDelay       = (smart_feature_full->Internal.Inq1.m_bTriggerDelay != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->MirrorImage        = (smart_feature_full->Internal.Inq1.m_bImageMirror != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->SoftReset          = (smart_feature_full->Internal.Inq1.m_bSoftReset != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->HSNR               = (smart_feature_full->Internal.Inq1.m_bHighSNR != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ColorCorrection    = (smart_feature_full->Internal.Inq1.m_bColorCorr != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ColorAvg           = (smart_feature_full->Internal.Inq1.m_bColorAvg != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->SIS                = ( /*smart_feature_full->Internal.Inq1.m_bTimestamp != 0 ||*/
                                              smart_feature_full->Internal.Inq1.m_bTimestamp2 != 0 ||
                                              smart_feature_full->Internal.Inq3.m_bSIS != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->UserProfiles       = (smart_feature_full->Internal.Inq1.m_bUserProfiles != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->TriggerCounter     = (smart_feature_full->Internal.Inq1.m_bTrgCounter != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ParamListBuffer    = (smart_feature_full->Internal.Inq1.m_bParamListBuffer != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->GP_Buffer          = (smart_feature_full->Internal.Inq1.m_bGPBuffer != 0) ? DC1394_TRUE : DC1394_FALSE;

    smart_feature_full->Input_1            = (smart_feature_full->Internal.Inq2.m_bInp_1 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_2            = (smart_feature_full->Internal.Inq2.m_bInp_2 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_3            = (smart_feature_full->Internal.Inq2.m_bInp_3 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_4            = (smart_feature_full->Internal.Inq2.m_bInp_4 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_5            = (smart_feature_full->Internal.Inq2.m_bInp_5 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_6            = (smart_feature_full->Internal.Inq2.m_bInp_6 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_7            = (smart_feature_full->Internal.Inq2.m_bInp_7 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Input_8            = (smart_feature_full->Internal.Inq2.m_bInp_8 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_1           = (smart_feature_full->Internal.Inq2.m_bOutp_1 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_2           = (smart_feature_full->Internal.Inq2.m_bOutp_2 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_3           = (smart_feature_full->Internal.Inq2.m_bOutp_3 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_4           = (smart_feature_full->Internal.Inq2.m_bOutp_4 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_5           = (smart_feature_full->Internal.Inq2.m_bOutp_5 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_6           = (smart_feature_full->Internal.Inq2.m_bOutp_6 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_7           = (smart_feature_full->Internal.Inq2.m_bOutp_7 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_8           = (smart_feature_full->Internal.Inq2.m_bOutp_8 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->IntEnaDelay        = (smart_feature_full->Internal.Inq2.m_bIntEnaDelay != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->IncDecoder         = (smart_feature_full->Internal.Inq2.m_bIncDecoder != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_1_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_1_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_2_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_2_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_3_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_3_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_4_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_4_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_5_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_5_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_6_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_6_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_7_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_7_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->Output_8_PWM       = (smart_feature_full->Internal.Inq2.m_bOutp_8_PWM != 0) ? DC1394_TRUE : DC1394_FALSE;

    smart_feature_full->CameraStatus       = (smart_feature_full->Internal.Inq3.m_bCameraStatus != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ParamUpdTiming     = (smart_feature_full->Internal.Inq3.m_bParamUpdTiming != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->F7ModeMapping      = (smart_feature_full->Internal.Inq3.m_bF7ModeMapping != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->AutoShutter        = (smart_feature_full->Internal.Inq3.m_bAutoShutter != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->AutoGain           = (smart_feature_full->Internal.Inq3.m_bAutoGain != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->AutoFunctionAOI    = (smart_feature_full->Internal.Inq3.m_bAutoFncAOI != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->SequenceStep       = (smart_feature_full->Internal.Inq3.m_bSequenceStep != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->LowNoiseBinning    = (smart_feature_full->Internal.Inq3.m_bLowNoiseBinning != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->GlobalResetReleaseShutter = (smart_feature_full->Internal.Inq3.m_bGlobResRelShutter != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->DefectPixelCorrection     = (smart_feature_full->Internal.Inq3.m_bDpc != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->SWFeatureControl   = (smart_feature_full->Internal.Inq3.m_bSWFeatureCtrl != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_1    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_1 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_2    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_2 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_3    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_3 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_4    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_4 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_5    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_5 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_6    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_6 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_7    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_7 != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->InputDebounce_8    = (smart_feature_full->Internal.Inq3.m_bInpDebounce_8 != 0) ? DC1394_TRUE : DC1394_FALSE;

    smart_feature_full->HDRPike            = (smart_feature_full->Internal.Inq4.m_bPikeHdrMode != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ChannelAdjustGain  = (smart_feature_full->Internal.Inq4.m_bPikeChannelComp != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->LowSmear           = (smart_feature_full->Internal.Inq4.m_bLowSmear != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->AdvWhiteBal        = (smart_feature_full->Internal.Inq4.m_bAdvWhiteBal != 0) ? DC1394_TRUE : DC1394_FALSE;
    smart_feature_full->ChannelAdjustOffset=(smart_feature_full->Internal.Inq4.m_bPikeChannelOffset != 0) ? DC1394_TRUE : DC1394_FALSE;

    /* older AVT cameras have the Autofunc AOI bit in Inq3 unset but do support AutofuncAOI -
       also check the actual AutofuncAOI register */
    if( DC1394_FALSE == smart_feature_full->AutoFunctionAOI )
    {
        dc1394error_t err;
        dc1394_avt_csradv_autofnc_aoi_t sAFAOI;

        /* get register */
        err=dc1394_get_adv_control_registers(camera,REG_CAMERA_AVT_AUTOFNC_AOI, (uint32_t*) &sAFAOI, 1);
        DC1394_ERR_RTN(err,"Could not get AVT autofunction AOI Register");

        if( DC1394_SUCCESS == err && 0 != sAFAOI.m_Ctrl.m_bPresence )
        {
            smart_feature_full->AutoFunctionAOI = DC1394_TRUE;
        }
    }

    if(smart_feature_full->SWFeatureControl==DC1394_TRUE)
    {
        dc1394_avt_csradv_swfeature_t sSWFeature;
        err = dc1394_get_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_SWFEATURE,
                                                 (uint32_t*) &sSWFeature,
                                                 sizeof(dc1394_avt_csradv_swfeature_t)/4 );
        smart_feature_full->LedBlanking = ( err==DC1394_SUCCESS && sSWFeature.m.m_bBlankLED_Inq!=0 )? DC1394_TRUE : DC1394_FALSE;
    }
    else
    {
        smart_feature_full->LedBlanking = DC1394_FALSE;
    }

    if(smart_feature_full->Internal.Inq3.m_bMaxIsoSize != 0)
    {
        dc1394_avt_csradv_max_isosize_t sMaxIso;

        err = dc1394_get_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_MAX_ISO_SIZE,
                                                 (uint32_t*) &sMaxIso,
                                                 sizeof(dc1394_avt_csradv_max_isosize_t)/4 );
        smart_feature_full->MaxIsoSize_S400 = ( err==DC1394_SUCCESS && sMaxIso.m_S400.m.m_bPresence!=0 )? DC1394_TRUE : DC1394_FALSE;
        smart_feature_full->MaxIsoSize_S800 = ( err==DC1394_SUCCESS && sMaxIso.m_S800.m.m_bPresence!=0 )? DC1394_TRUE : DC1394_FALSE;
    }
    else
    {
        smart_feature_full->MaxIsoSize_S400 = DC1394_FALSE;
        smart_feature_full->MaxIsoSize_S800 = DC1394_FALSE;
    }
    return DC1394_SUCCESS;
}


/************************************************************************/
/* Print Advanced features - deprecated                                 */
/************************************************************************/
dc1394error_t
dc1394_avt_print_advanced_feature(dc1394_avt_adv_feature_info_t *adv_feature)
{
    puts ("ADVANCED FEATURES SUPPORTED:");
    if(adv_feature->MaxResolution == DC1394_TRUE) puts (" MaxResolution ");
    if(adv_feature->TimeBase == DC1394_TRUE)         puts (" TimeBase ");
    if(adv_feature->ExtdShutter == DC1394_TRUE)         puts (" ExtdShutter ");
    if(adv_feature->TestImage == DC1394_TRUE)         puts (" TestImage ");
    if(adv_feature->FrameInfo == DC1394_TRUE)         puts (" FrameInfo ");
    if(adv_feature->Sequences == DC1394_TRUE)         puts (" Sequences ");
    if(adv_feature->VersionInfo == DC1394_TRUE)         puts (" VersionInfo ");
    //ADV_INQ_1 7
    if(adv_feature->Lookup_Tables == DC1394_TRUE)        puts (" Lookup_Tables ");
    if(adv_feature->Shading == DC1394_TRUE)         puts (" Shading ");
    if(adv_feature->DeferredTrans == DC1394_TRUE) puts (" DeferredTrans ");
    if(adv_feature->HDR_Mode == DC1394_TRUE)         puts (" HDR_Mode ");
    if(adv_feature->DSNU == DC1394_TRUE)                 puts (" DSNU ");
    if(adv_feature->BlemishCorrection == DC1394_TRUE)                 puts (" BlemishCorrection ");
    if(adv_feature->TriggerDelay == DC1394_TRUE)         puts (" TriggerDelay ");
    if(adv_feature->MirrorImage == DC1394_TRUE)         puts (" MirrorImage ");
    if(adv_feature->SoftReset == DC1394_TRUE)         puts (" SoftReset ");
    if(adv_feature->HSNR == DC1394_TRUE)         puts (" HSNR ");
    if(adv_feature->ColorCorrection == DC1394_TRUE)         puts (" ColorCorrection ");
    if(adv_feature->UserProfiles == DC1394_TRUE)         puts (" UserProfiles ");
    //ADV_INQ_1 20
    if(adv_feature->UserSets == DC1394_TRUE)         puts (" UserSets ");
    if(adv_feature->TimeStamp == DC1394_TRUE)         puts (" TimeStamp ");
    if(adv_feature->FrmCntStamp == DC1394_TRUE)         puts (" FrmCntStamp ");
    if(adv_feature->TrgCntStamp == DC1394_TRUE)         puts (" TrgCntStamp ");
    //ADV_INQ_1 25-30
    if(adv_feature->GP_Buffer == DC1394_TRUE)         puts (" GP_Buffer ");

    if(adv_feature->Input_1 == DC1394_TRUE)        puts (" Input_1 ");
    if(adv_feature->Input_2 == DC1394_TRUE)         puts (" Input_2 ");
    //ADV_INQ_2 2-7
    if(adv_feature->Output_1 == DC1394_TRUE)         puts (" Output_1 ");
    if(adv_feature->Output_2 == DC1394_TRUE)         puts (" Output_2 ");
    if(adv_feature->Output_3 == DC1394_TRUE)         puts (" Output_3 ");
    if(adv_feature->Output_4 == DC1394_TRUE)         puts (" Output_4 ");
    //ADV_INQ_2 12-15
    if(adv_feature->IntEnaDelay == DC1394_TRUE)         puts (" IntEnaDelay ");
    if(adv_feature->IncDecoder == DC1394_TRUE)         puts (" IncDecoder ");
    //ADV_INQ_2 18-31

    if(adv_feature->CameraStatus == DC1394_TRUE)         puts (" CameraStatus ");
    //ADV_INQ_3 1-3
    if(adv_feature->AutoShutter == DC1394_TRUE)         puts (" AutoShutter ");
    if(adv_feature->AutoGain == DC1394_TRUE)         puts (" AutoGain ");
    if(adv_feature->AutoFunctionAOI == DC1394_TRUE)         puts (" AutoFunctionAOI ");
    //ADV_INQ_3 7-31

    if(adv_feature->HDRPike == DC1394_TRUE)         puts (" HDRPike ");
    //ADV_INQ_4 1-31

    return DC1394_SUCCESS;

}


/************************************************************************/
/* Print Smart features                                                 */
/************************************************************************/
dc1394error_t
dc1394_avt_print_smart_features(dc1394_avt_smart_feature_info_t *adv_feature)
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>adv_feature->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    puts ("ADVANCED FEATURES SUPPORTED:");
    if(adv_feature->MaxResolution == DC1394_TRUE)       puts (" MaxResolution ");
    if(adv_feature->TimeBase == DC1394_TRUE)            puts (" TimeBase ");
    if(adv_feature->ExtdShutter == DC1394_TRUE)         puts (" ExtdShutter ");
    if(adv_feature->TestImage == DC1394_TRUE)           puts (" TestImage ");
    if(adv_feature->FrameCounter == DC1394_TRUE)        puts (" FrameCounter ");
    if(adv_feature->Sequences == DC1394_TRUE)           puts (" Sequences ");
    if(adv_feature->VersionInfo == DC1394_TRUE)         puts (" VersionInfo ");
    if(adv_feature->Lookup_Tables == DC1394_TRUE)       puts (" Lookup_Tables ");
    if(adv_feature->Shading == DC1394_TRUE)             puts (" Shading ");
    if(adv_feature->DeferredTransport == DC1394_TRUE)   puts (" DeferredTransport ");
    if(adv_feature->HDR_Mode == DC1394_TRUE)            puts (" HDR_Mode ");
    if(adv_feature->DSNU == DC1394_TRUE)                puts (" DSNU ");
    if(adv_feature->BlemishCorrection == DC1394_TRUE)   puts (" BlemishCorrection ");
    if(adv_feature->TriggerDelay == DC1394_TRUE)        puts (" TriggerDelay ");
    if(adv_feature->MirrorImage == DC1394_TRUE)         puts (" MirrorImage ");
    if(adv_feature->SoftReset == DC1394_TRUE)           puts (" SoftReset ");
    if(adv_feature->HSNR == DC1394_TRUE)                puts (" HSNR ");
    if(adv_feature->ColorCorrection == DC1394_TRUE)     puts (" ColorCorrection ");
    if(adv_feature->ColorAvg == DC1394_TRUE)            puts (" ColorAvg ");
    if(adv_feature->SIS == DC1394_TRUE)                 puts (" SIS ");
    if(adv_feature->UserProfiles == DC1394_TRUE)        puts (" UserProfiles ");
    if(adv_feature->TriggerCounter == DC1394_TRUE)      puts (" TriggerCounter ");
    if(adv_feature->ParamListBuffer == DC1394_TRUE)     puts (" ParamListBuffer ");
    if(adv_feature->GP_Buffer == DC1394_TRUE)           puts (" GP_Buffer ");

    if(adv_feature->Input_1 == DC1394_TRUE)             puts (" Input_1 ");
    if(adv_feature->Input_2 == DC1394_TRUE)             puts (" Input_2 ");
    if(adv_feature->Input_3 == DC1394_TRUE)             puts (" Input_3 ");
    if(adv_feature->Input_4 == DC1394_TRUE)             puts (" Input_4 ");
    if(adv_feature->Input_5 == DC1394_TRUE)             puts (" Input_5 ");
    if(adv_feature->Input_6 == DC1394_TRUE)             puts (" Input_6 ");
    if(adv_feature->Input_7 == DC1394_TRUE)             puts (" Input_7 ");
    if(adv_feature->Input_8 == DC1394_TRUE)             puts (" Input_8 ");
    if(adv_feature->Output_1 == DC1394_TRUE)            puts (" Output_1 ");
    if(adv_feature->Output_2 == DC1394_TRUE)            puts (" Output_2 ");
    if(adv_feature->Output_3 == DC1394_TRUE)            puts (" Output_3 ");
    if(adv_feature->Output_4 == DC1394_TRUE)            puts (" Output_4 ");
    if(adv_feature->Output_5 == DC1394_TRUE)            puts (" Output_5 ");
    if(adv_feature->Output_6 == DC1394_TRUE)            puts (" Output_6 ");
    if(adv_feature->Output_7 == DC1394_TRUE)            puts (" Output_7 ");
    if(adv_feature->Output_8 == DC1394_TRUE)            puts (" Output_8 ");
    if(adv_feature->IntEnaDelay == DC1394_TRUE)         puts (" IntEnaDelay ");
    if(adv_feature->IncDecoder == DC1394_TRUE)          puts (" IncDecoder ");
    if(adv_feature->Output_1_PWM == DC1394_TRUE)        puts (" Output_1_PWM ");
    if(adv_feature->Output_2_PWM == DC1394_TRUE)        puts (" Output_2_PWM ");
    if(adv_feature->Output_3_PWM == DC1394_TRUE)        puts (" Output_3_PWM ");
    if(adv_feature->Output_4_PWM == DC1394_TRUE)        puts (" Output_4_PWM ");
    if(adv_feature->Output_5_PWM == DC1394_TRUE)        puts (" Output_5_PWM ");
    if(adv_feature->Output_6_PWM == DC1394_TRUE)        puts (" Output_6_PWM ");
    if(adv_feature->Output_7_PWM == DC1394_TRUE)        puts (" Output_7_PWM ");
    if(adv_feature->Output_8_PWM == DC1394_TRUE)        puts (" Output_8_PWM ");

    if(adv_feature->CameraStatus == DC1394_TRUE)        puts (" CameraStatus ");
    if(adv_feature->MaxIsoSize_S400 == DC1394_TRUE)     puts (" MaxIsoSize_S400 ");
    if(adv_feature->MaxIsoSize_S800 == DC1394_TRUE)     puts (" MaxIsoSize_S800 ");
    if(adv_feature->ParamUpdTiming == DC1394_TRUE)      puts (" ParamUpdTiming ");
    if(adv_feature->F7ModeMapping == DC1394_TRUE)       puts (" F7ModeMapping ");
    if(adv_feature->AutoShutter == DC1394_TRUE)         puts (" AutoShutter ");
    if(adv_feature->AutoGain == DC1394_TRUE)            puts (" AutoGain ");
    if(adv_feature->AutoFunctionAOI == DC1394_TRUE)     puts (" AutoFunctionAOI ");
    if(adv_feature->SequenceStep == DC1394_TRUE)        puts (" SequenceStep ");
    if(adv_feature->LowNoiseBinning == DC1394_TRUE)     puts (" LowNoiseBinning ");

    if(adv_feature->GlobalResetReleaseShutter == DC1394_TRUE) puts (" GlobalResetReleaseShutter ");
    if(adv_feature->DefectPixelCorrection == DC1394_TRUE)     puts (" DefectPixelCorrection ");
    if(adv_feature->SWFeatureControl == DC1394_TRUE)    puts (" SWFeatureControl ");
    if(adv_feature->LedBlanking == DC1394_TRUE)         puts (" LedBlanking ");
    if(adv_feature->InputDebounce_1 == DC1394_TRUE)     puts (" InputDebounce_1 ");
    if(adv_feature->InputDebounce_2 == DC1394_TRUE)     puts (" InputDebounce_2 ");
    if(adv_feature->InputDebounce_3 == DC1394_TRUE)     puts (" InputDebounce_3 ");
    if(adv_feature->InputDebounce_4 == DC1394_TRUE)     puts (" InputDebounce_4 ");
    if(adv_feature->InputDebounce_5 == DC1394_TRUE)     puts (" InputDebounce_5 ");
    if(adv_feature->InputDebounce_6 == DC1394_TRUE)     puts (" InputDebounce_6 ");
    if(adv_feature->InputDebounce_7 == DC1394_TRUE)     puts (" InputDebounce_7 ");
    if(adv_feature->InputDebounce_8 == DC1394_TRUE)     puts (" InputDebounce_8 ");

    if(adv_feature->HDRPike == DC1394_TRUE)             puts (" HDRPike ");
    if(adv_feature->ChannelAdjustGain == DC1394_TRUE)   puts (" ChannelAdjustGain ");
    if(adv_feature->LowSmear == DC1394_TRUE)            puts (" LowSmear ");
    if(adv_feature->AdvWhiteBal == DC1394_TRUE)         puts (" AdvWhiteBal ");
    if(adv_feature->ChannelAdjustOffset == DC1394_TRUE) puts (" ChannelAdjustOffset ");
    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get shading correction feature - deprecated                          */
/************************************************************************/
dc1394error_t
dc1394_avt_get_shading(dc1394camera_t *camera,
                       dc1394bool_t *on_off, dc1394bool_t *compute,
                       dc1394bool_t *show, uint32_t *frame_nb)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve shading properties */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_SHDG_CTRL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT shading control reg");

    /* Shading ON / OFF : Bit 6 */
    if (on_off)
        *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* Compute : Bit 5 */
    if (compute)
        *compute = (uint32_t)((value & 0x4000000UL) >> 26);

    /* Show image : Bit 4 */
    if (show)
        *show = (uint32_t)((value & 0x8000000UL) >> 27);

    /* Number of images for auto computing of the shading reference: Bits 24..31 */
    if (frame_nb)
        *frame_nb =(uint32_t)((value & 0xFFUL));

    return DC1394_SUCCESS;

}


/************************************************************************/
/* Get shading correction feature                                       */
/************************************************************************/
dc1394error_t dc1394_avt_get_shading_correction(dc1394camera_t *camera,
                                                dc1394switch_t *on_off,
                                                dc1394bool_t *build_err,
                                                dc1394switch_t *show,
                                                uint32_t *frame_nb,
                                                uint32_t *mem_channel,
                                                uint32_t *mem_channel_err)
{
    dc1394error_t err;
    dc1394_avt_csradv_shading_t sShading;

    /* Retrieve shading registers */
    err=dc1394_get_adv_control_registers( camera,
                                          REG_CAMERA_AVT_SHDG_CTRL,
                                          (uint32_t*) &sShading,
                                          sizeof(dc1394_avt_csradv_shading_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT shading control reg");

    *on_off = (sShading.m_Ctrl.m_bOnOff!=0) ? DC1394_ON : DC1394_OFF;
    *build_err = (sShading.m_Ctrl.m_bBuildError!=0) ? DC1394_TRUE : DC1394_FALSE;
    *show = (sShading.m_Ctrl.m_bShowImg!=0) ? DC1394_ON : DC1394_OFF;
    *frame_nb = sShading.m_Ctrl.m_nGrabCount;
    *mem_channel = sShading.m_Ctrl.m_nMemChn;
    *mem_channel_err = sShading.m_Ctrl.m_nMemChnError;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set shading correction feature - deprecated                          */
/************************************************************************/
dc1394error_t
dc1394_avt_set_shading(dc1394camera_t *camera,
                       dc1394bool_t on_off, dc1394bool_t compute,
                       dc1394bool_t show, uint32_t frame_nb)
{
    dc1394error_t err;
    dc1394_avt_csradv_shading_t sShading;

    /* Retrieve current shading properties */
    err=dc1394_get_adv_control_registers( camera,
                                          REG_CAMERA_AVT_SHDG_CTRL,
                                          (uint32_t*) &sShading,
                                          1 );
    DC1394_ERR_RTN(err,"Could not get AVT shading control reg");

    /* Shading ON / OFF : Bit 6 */
    sShading.m_Ctrl.m_bOnOff = (on_off==DC1394_TRUE)? 1:0;

    /* Compute : Bit 5 */
    sShading.m_Ctrl.m_bBuildTable = (compute==DC1394_TRUE)? 1:0;

    /* Show Image : Bit 4 */
    sShading.m_Ctrl.m_bShowImg = (show==DC1394_TRUE)? 1:0;

    /* Number of images : Bits 24..31 */
    sShading.m_Ctrl.m_nGrabCount = frame_nb;

    /* Set new parameters */
    err=dc1394_set_adv_control_registers(camera,REG_CAMERA_AVT_SHDG_CTRL,(uint32_t*) &sShading,1);
    DC1394_ERR_RTN(err,"Could not set AVT shading control reg");

    /* Poll register until busy is cleared */
    do
    {
        usleep(500000);
        err=dc1394_get_adv_control_registers( camera,
                                              REG_CAMERA_AVT_SHDG_CTRL,
                                              (uint32_t*) &sShading,
                                              1 );
        DC1394_ERR_RTN(err,"Could not get AVT shading control reg");
    } while( sShading.m_Ctrl.m_bBusy != 0 );

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set shading correction feature                                       */
/************************************************************************/
dc1394error_t dc1394_avt_set_shading_correction(dc1394camera_t *camera,
                                                dc1394switch_t on_off,
                                                dc1394switch_t compute,
                                                dc1394switch_t show,
                                                uint32_t frame_nb,
                                                uint32_t mem_channel,
                                                dc1394switch_t mem_clear,
                                                dc1394switch_t mem_load,
                                                dc1394switch_t mem_save)
{
    dc1394error_t err;
    dc1394_avt_csradv_shading_t sShading;

    /* Retrieve shading registers */
    err=dc1394_get_adv_control_registers( camera,
                                          REG_CAMERA_AVT_SHDG_CTRL,
                                          (uint32_t*) &sShading,
                                          sizeof(dc1394_avt_csradv_shading_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT shading control reg");

    sShading.m_Ctrl.m_bOnOff      = (DC1394_ON==on_off)? 1 : 0;
    sShading.m_Ctrl.m_bBuildTable = (DC1394_ON==compute)? 1 : 0;
    sShading.m_Ctrl.m_bShowImg    = (DC1394_ON==show)? 1 : 0;
    sShading.m_Ctrl.m_nGrabCount  = frame_nb;
    sShading.m_Ctrl.m_nMemChn     = mem_channel;
    sShading.m_Ctrl.m_bMemClear   = (DC1394_ON==mem_clear)? 1 : 0;
    sShading.m_Ctrl.m_bMemLoad    = (DC1394_ON==mem_load)? 1 : 0;
    sShading.m_Ctrl.m_bMemSave    = (DC1394_ON==mem_save)? 1 : 0;

    /* write back register */
    err=dc1394_set_adv_control_registers( camera,
                                          REG_CAMERA_AVT_SHDG_CTRL,
                                          (uint32_t*) &sShading,
                                          1 );
    DC1394_ERR_RTN(err,"Could not get AVT shading control reg");

    /* poll camera until busy bit is cleared */
    do
    {
        usleep(500000);
        err=dc1394_get_adv_control_registers( camera,
                                              REG_CAMERA_AVT_SHDG_CTRL,
                                              (uint32_t*) &sShading,
                                              sizeof(dc1394_avt_csradv_shading_t)/4 );
        DC1394_ERR_RTN(err,"Could not get AVT shading control reg");
    } while (sShading.m_Ctrl.m_bBusy!=0);

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get shading  mem ctrl                                                */
/************************************************************************/
dc1394error_t
dc1394_avt_get_shading_mem_ctrl(dc1394camera_t *camera, dc1394bool_t *en_write,
                                dc1394bool_t *en_read, uint32_t *addroffset)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current memory shading properties */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_SHDG_MEM_CTRL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT shading memory control");

    /* Enable write access : Bit 5 */
    if (en_write)
        *en_write = (uint32_t)((value & 0x4000000UL) >> 26);

    /* Enable read access : Bit 6 */
    if (en_read)
        *en_read = (uint32_t)((value & 0x2000000UL) >> 25);

    /* addroffset in byte : Bits 8..31 */
    if (addroffset)
        *addroffset =(uint32_t)((value & 0xFFFFFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set shading mem ctrl                                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_set_shading_mem_ctrl(dc1394camera_t *camera,
                                dc1394bool_t en_write, dc1394bool_t en_read, uint32_t addroffset)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current shading properties */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_SHDG_MEM_CTRL, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT shading memory control");

    /* read access enable : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((en_read ) << 25);

    /* write access enable : Bit 5 */
    curval = (curval & 0xFBFFFFFFUL) | ((en_write ) << 26);

    /* Number of images : Bits 8..31 */
    curval = (curval & 0xFF000000UL) | ((addroffset & 0xFFFFFFUL ));

    /* Set new parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_SHDG_MEM_CTRL, curval);
    DC1394_ERR_RTN(err,"Could not get AVT LUT memory control");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get shading info - deprecated                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_get_shading_info(dc1394camera_t *camera, uint32_t *MaxImageSize)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve shading info */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_SHDG_INFO, &value);
    DC1394_ERR_RTN(err,"Could not get AVT shading info");

    /* Max Shading Image size(byte) : Bits 8..31 */
    *MaxImageSize =(uint32_t)((value & 0xFFFFFFUL));

    return DC1394_SUCCESS;

}


/************************************************************************/
/* Get shading correction info                                          */
/************************************************************************/
dc1394error_t dc1394_avt_get_shading_correction_info(dc1394camera_t *camera,
                                                     uint32_t *MaxImageSize,
                                                     uint32_t *MemChannelCount)
{
    dc1394error_t err;
    dc1394_avt_csradv_shading_t sShading;

    /* Retrieve shading registers */
    err=dc1394_get_adv_control_registers( camera,
                                          REG_CAMERA_AVT_SHDG_CTRL,
                                          (uint32_t*) &sShading,
                                          sizeof(dc1394_avt_csradv_shading_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT shading control reg");

    *MaxImageSize = sShading.m_Info.m_nMaxSize;
    *MemChannelCount = sShading.m_Info.m_nMemChnCount;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Multiple slope parameters (HDR)                                  */
/************************************************************************/
dc1394error_t
dc1394_avt_get_multiple_slope(dc1394camera_t *camera,
                              dc1394bool_t *on_off, uint32_t *points_nb,uint32_t *kneepoint1,
                              uint32_t *kneepoint2, uint32_t *kneepoint3)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current hdr parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_HDR_CONTROL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT HDR control register");

    /* Multiple slope ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* Number of actives points : Bits 28..31 */
    *points_nb =(uint32_t)((value & 0xFUL));

    /* kneepoints */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_KNEEPOINT_1, kneepoint1);
    DC1394_ERR_RTN(err,"Could not get AVT kneepoint 1");
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_KNEEPOINT_2, kneepoint2);
    DC1394_ERR_RTN(err,"Could not get AVT kneepoint 2");
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_KNEEPOINT_3, kneepoint3);
    DC1394_ERR_RTN(err,"Could not get AVT kneepoint 3");

    return DC1394_SUCCESS;

}


/************************************************************************/
/* Set Multiple slope parameters                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_set_multiple_slope(dc1394camera_t *camera,
                              dc1394bool_t on_off, uint32_t points_nb, uint32_t kneepoint1,
                              uint32_t kneepoint2, uint32_t kneepoint3)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current hdr parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_HDR_CONTROL, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT HDR control reg");

    /* Shading ON / OFF : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25);

    /* Number of points : Bits 28..31 */
    curval = (curval & 0xFFFFFFF0UL) | ((points_nb & 0xFUL ));

    /* Set new hdr parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_HDR_CONTROL, curval);
    DC1394_ERR_RTN(err,"Could not set AVT HDR control reg");

    /* kneepoints */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_KNEEPOINT_1, kneepoint1);
    DC1394_ERR_RTN(err,"Could not set AVT kneepoint 1");
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_KNEEPOINT_2, kneepoint2);
    DC1394_ERR_RTN(err,"Could not set AVT kneepoint 2");
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_KNEEPOINT_3, kneepoint3);
    DC1394_ERR_RTN(err,"Could not set AVT kneepoint 3");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Shutter Timebase                                                 */
/************************************************************************/
dc1394error_t
dc1394_avt_get_timebase(dc1394camera_t *camera, uint32_t *timebase_id)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current timebase */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TIMEBASE, &value);
    DC1394_ERR_RTN(err,"Could not get AVT timebase");

    /* Time base ID : Bits 29..31 */
    *timebase_id =(uint32_t)((value & 0xFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Shutter Timebase (acquisition must be stopped)                   */
/************************************************************************/
dc1394error_t
dc1394_avt_set_timebase(dc1394camera_t *camera, uint32_t timebase_id)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current timebase */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TIMEBASE, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT timebase");

    curval = (curval & 0xFFFFFFF0UL) | ((timebase_id & 0xFUL ));

    /* Set new timebase */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_TIMEBASE, curval);
    DC1394_ERR_RTN(err,"Could not set AVT timebase");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Extented Shutter                                                 */
/************************************************************************/
dc1394error_t
dc1394_avt_get_extented_shutter(dc1394camera_t *camera, uint32_t *timebase_id)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current extented shutter value */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_EXTD_SHUTTER, &value);
    DC1394_ERR_RTN(err,"Could not get AVT extended shutter reg");

    /* Exposure Time in us: Bits 6..31 */
    *timebase_id =(uint32_t)((value & 0xFFFFFFFUL));

    return DC1394_SUCCESS;

}


/************************************************************************/
/* Set Extented shutter                                                 */
/************************************************************************/
dc1394error_t
dc1394_avt_set_extented_shutter(dc1394camera_t *camera, uint32_t timebase_id)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current extented shutter value */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_EXTD_SHUTTER, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT extended shutter reg");

    /* Time base ID : Bits 6..31 */
    curval = (curval & 0xF0000000UL) | ((timebase_id & 0x0FFFFFFFUL ));

    /* Set new extented shutter value */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_EXTD_SHUTTER, curval);
    DC1394_ERR_RTN(err,"Could not set AVT extended shutter reg");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get MaxResolution          (Read Only)                               */
/************************************************************************/
dc1394error_t
dc1394_avt_get_MaxResolution(dc1394camera_t *camera, uint32_t *MaxHeight, uint32_t *MaxWidth)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve the maximum resolution */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_MAX_RESOLUTION, &value);
    DC1394_ERR_RTN(err,"Could not get AVT max resolution");

    /* MaxHeight : Bits 0..15 */
    *MaxHeight =(uint32_t)(value >> 16);
    /* MaxWidth : Bits 16..31 */
    *MaxWidth =(uint32_t)(value & 0xFFFFUL );

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Auto Shutter                                                     */
/************************************************************************/
dc1394error_t
dc1394_avt_get_auto_shutter(dc1394camera_t *camera, uint32_t *MinValue, uint32_t *MaxValue)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current min auto shutter value */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_AUTOSHUTTER_LO, &value);
    DC1394_ERR_RTN(err,"Could not get AVT autoshutter LSB");

    *MinValue =(uint32_t)value;

    /* Retrieve current max auto shutter value */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_AUTOSHUTTER_HI, &value);
    DC1394_ERR_RTN(err,"Could not get AVT autoshutter MSB");

    *MaxValue =(uint32_t)value;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Auto shutter                                                     */
/************************************************************************/
dc1394error_t
dc1394_avt_set_auto_shutter(dc1394camera_t *camera, uint32_t MinValue, uint32_t MaxValue)
{
    dc1394error_t err;
    /* Set min auto shutter value */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_AUTOSHUTTER_LO, MinValue);
    DC1394_ERR_RTN(err,"Could not set AVT autoshutter LSB");

    /* Set max auto shutter value */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_AUTOSHUTTER_HI, MaxValue);
    DC1394_ERR_RTN(err,"Could not set AVT autoshutter MSB");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Auto Gain                                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_get_auto_gain(dc1394camera_t *camera, uint32_t *MinValue, uint32_t *MaxValue)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve auto gain values */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_AUTOGAIN_CTRL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT autogain");

    /* Min : bits 20..31 */
    *MinValue =(uint32_t)(value & 0xFFFUL);
    /* Max : bits 4..15 */
    *MaxValue =(uint32_t)((value >> 16) & 0xFFFUL);

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Auto gain                                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_set_auto_gain(dc1394camera_t *camera, uint32_t MinValue, uint32_t MaxValue)
{
    dc1394error_t err;
    uint32_t value;

    /* Max : bits 4..15, Min : bits 20..31  */
    value = ( MaxValue <<16 ) | ( MinValue );

    /* Set new parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_AUTOGAIN_CTRL,value);
    DC1394_ERR_RTN(err,"Could not set AVT autogain");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Trigger delay                                                    */
/************************************************************************/
dc1394error_t
dc1394_avt_get_trigger_delay(dc1394camera_t *camera, dc1394bool_t *on_off, uint32_t *DelayTime)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve trigger delay */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TRIGGER_DELAY, &value);
    DC1394_ERR_RTN(err,"Could not get AVT trigger delay");

    /* trigger_delay ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* Delai time in us : Bits 11..31 */
    *DelayTime =(uint32_t)((value & 0xFFFFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Trigger delay                                                    */
/************************************************************************/
dc1394error_t
dc1394_avt_set_trigger_delay(dc1394camera_t *camera, dc1394bool_t on_off, uint32_t DelayTime)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve trigger delay */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TRIGGER_DELAY, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT trigger delay");

    /* trigger_delay ON / OFF : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25);

    /* Delay time in us : Bits 11..31 */
    curval = (curval & 0xFFF00000UL) | DelayTime;

    /* Set new parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_TRIGGER_DELAY, curval);
    DC1394_ERR_RTN(err,"Could not set AVT trigger delay");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Mirror                                                           */
/************************************************************************/
dc1394error_t
dc1394_avt_get_mirror(dc1394camera_t *camera, dc1394bool_t *on_off)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve Mirror mode */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_MIRROR_IMAGE, &value);
    DC1394_ERR_RTN(err,"Could not get AVT mirror image");

    /* mirror ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Mirror                                                           */
/************************************************************************/
dc1394error_t
dc1394_avt_set_mirror(dc1394camera_t *camera, dc1394bool_t on_off)
{
    dc1394error_t err;
    uint32_t curval;

    /* ON / OFF : Bit 6 */
    curval = on_off << 25;

    /* Set mirror mode */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_MIRROR_IMAGE, curval);
    DC1394_ERR_RTN(err,"Could not set AVT mirror image");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get DSNU - deprecated                                                */
/************************************************************************/
dc1394error_t
dc1394_avt_get_dsnu(dc1394camera_t *camera, dc1394bool_t *on_off,uint32_t *frame_nb)
{
    dc1394error_t err;
    uint32_t value;
    /* Retrieve dsnu parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_DSNU_CONTROL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT DSNU control");

    /* ON / OFF : Bit 6 */
    *on_off = !(uint32_t)((value & 0x2000000UL) >> 25);

    /* Number of images : Bits 24..31 */
    *frame_nb =(uint32_t)((value & 0xFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get DSNU correction                                                  */
/************************************************************************/
dc1394error_t dc1394_avt_get_dsnu_correction(dc1394camera_t *camera,
                                             dc1394switch_t *on_off,
                                             dc1394bool_t   *build_error,
                                             uint32_t       *frame_nb,
                                             dc1394switch_t *show_image)
{
    dc1394error_t err;
    dc1394_avt_csradv_dsnucorrection_t sDSNU;

    /* Retrieve dsnu config */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DSNU_CONTROL,
                                             (uint32_t*) &sDSNU,
                                             sizeof(dc1394_avt_csradv_dsnucorrection_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT DSNU control");

    *on_off = (sDSNU.m.m_bOnOff != 0) ? DC1394_ON : DC1394_OFF;
    *build_error = (sDSNU.m.m_bBuildError != 0) ? DC1394_TRUE : DC1394_FALSE;
    *frame_nb = sDSNU.m.m_nGrabCount;
    *show_image = (sDSNU.m.m_bShowImg != 0) ? DC1394_ON : DC1394_OFF;

    return err;
}


/************************************************************************/
/* Set DSNU - deprecated                                                */
/************************************************************************/
dc1394error_t
dc1394_avt_set_dsnu(dc1394camera_t *camera,
                    dc1394bool_t on_off, dc1394bool_t compute, uint32_t frame_nb)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current dsnu parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_DSNU_CONTROL, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT DSNU control");

    /* Compute : Bit 5 */
    curval = (curval & 0xFBFFFFFFUL) | ((compute ) << 26);

    /* ON / OFF : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((!on_off ) << 25);

    /* Number of images : Bits 24..31 */
    curval = (curval & 0xFFFFFF00UL) | ((frame_nb & 0xFFUL ));

    /* Set new dsnu parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_DSNU_CONTROL, curval);
    DC1394_ERR_RTN(err,"Could not set AVT DSNU control");

    int cont=1;
    while (cont) {
        usleep(50000);
        err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_DSNU_CONTROL, &curval);
        DC1394_ERR_RTN(err,"Could not get AVT DSNU control");
        if ((curval & 0x01000000UL)==0)
            cont=0;
    }
    return DC1394_SUCCESS;
}

/************************************************************************/
/* Set DSNU correction                                                  */
/************************************************************************/
dc1394error_t dc1394_avt_set_dsnu_correction(dc1394camera_t *camera,
                                             dc1394switch_t on_off,
                                             dc1394switch_t compute_image,
                                             uint32_t       frame_nb,
                                             dc1394switch_t show_image,
                                             dc1394switch_t load_image,
                                             dc1394switch_t save_image)
{
    dc1394error_t err;
    dc1394_avt_csradv_dsnucorrection_t sDSNU;

    /* Retrieve dsnu register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DSNU_CONTROL,
                                             (uint32_t*) &sDSNU,
                                             sizeof(dc1394_avt_csradv_dsnucorrection_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT DSNU control");

    sDSNU.m.m_bOnOff = (on_off==DC1394_ON) ? 1 : 0;
    sDSNU.m.m_bBuildTable = (compute_image==DC1394_ON) ? 1 : 0;
    sDSNU.m.m_nGrabCount = frame_nb;
    sDSNU.m.m_bShowImg = (show_image==DC1394_ON) ? 1 : 0;
    sDSNU.m.m_bMemLoad = (load_image==DC1394_ON) ? 1 : 0;
    sDSNU.m.m_bMemSave = (save_image==DC1394_ON) ? 1 : 0;

    /* Write back dsnu register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DSNU_CONTROL,
                                             (uint32_t*) &sDSNU,
                                             sizeof(dc1394_avt_csradv_dsnucorrection_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT DSNU control");

    /* wait until busy flag is cleared */
    int cont=1;
    while (cont) {
        usleep(50000);
        err = dc1394_get_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_DSNU_CONTROL,
                                                 (uint32_t*) &sDSNU,
                                                 sizeof(dc1394_avt_csradv_dsnucorrection_t)/4 );
        DC1394_ERR_RTN(err,"Could not get AVT DSNU control");
        if (sDSNU.m.m_bBusy==0)
            cont=0;
    }
    return err;
}

/************************************************************************/
/* Get BLEMISH - deprecated                                             */
/************************************************************************/
dc1394error_t
dc1394_avt_get_blemish(dc1394camera_t *camera, dc1394bool_t *on_off, uint32_t *frame_nb)
{
    dc1394error_t err;
    uint32_t value;

    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_BLEMISH_CONTROL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT blemish control");

    /* ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* Number of images : Bits 24..31 */
    *frame_nb =(uint32_t)((value & 0xFFUL));

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Get blemish pixel correction                                         */
/************************************************************************/
dc1394error_t dc1394_avt_get_blemish_correction(dc1394camera_t *camera,
                                                dc1394switch_t *on_off,
                                                dc1394bool_t   *build_error,
                                                uint32_t       *frame_nb,
                                                dc1394switch_t *show_image)
{
    dc1394error_t err;
    dc1394_avt_csradv_blemishcorrection_t sBlemish;

    /* Retrieve dsnu config */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_BLEMISH_CONTROL,
                                             (uint32_t*) &sBlemish,
                                             sizeof(dc1394_avt_csradv_blemishcorrection_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT blemish control");

    *on_off = (sBlemish.m.m_bOnOff != 0) ? DC1394_ON : DC1394_OFF;
    *build_error = (sBlemish.m.m_bBuildError != 0) ? DC1394_TRUE : DC1394_FALSE;
    *frame_nb = sBlemish.m.m_nGrabCount;
    *show_image = (sBlemish.m.m_bShowImg != 0) ? DC1394_ON : DC1394_OFF;

    return err;
}

/************************************************************************/
/* Set BLEMISH - deprecated                                             */
/************************************************************************/
dc1394error_t
dc1394_avt_set_blemish(dc1394camera_t *camera,
                       dc1394bool_t on_off, dc1394bool_t compute, uint32_t frame_nb)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current blemish parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_BLEMISH_CONTROL, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT blemish control");

    /* Compute : Bit 5 */
    curval = (curval & 0xFBFFFFFFUL) | ((compute ) << 26);

    /* ON / OFF : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25);

    /* Number of images : Bits 24..31 */
    curval = (curval & 0xFFFFFF00UL) | ((frame_nb & 0xFFUL ));

    /* Set new blemish parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_BLEMISH_CONTROL, curval);
    DC1394_ERR_RTN(err,"Could not set AVT blemish control");

    int cont=1;
    while (cont) {
        usleep(50000);
        err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_BLEMISH_CONTROL, &curval);
        DC1394_ERR_RTN(err,"Could not get AVT DSNU control");
        if ((curval & 0x01000000UL)==0)
            cont=0;
    }

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Set blemish pixel correction                                         */
/************************************************************************/
dc1394error_t dc1394_avt_set_blemish_correction(dc1394camera_t *camera,
                                                dc1394switch_t on_off,
                                                dc1394switch_t compute_image,
                                                uint32_t       frame_nb,
                                                dc1394switch_t show_image,
                                                dc1394switch_t load_image,
                                                dc1394switch_t save_image)
{
    dc1394error_t err;
    dc1394_avt_csradv_blemishcorrection_t sBlemish;

    /* Retrieve dsnu register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_BLEMISH_CONTROL,
                                             (uint32_t*) &sBlemish,
                                             sizeof(dc1394_avt_csradv_blemishcorrection_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT blemish control");

    sBlemish.m.m_bOnOff = (on_off==DC1394_ON) ? 1 : 0;
    sBlemish.m.m_bBuildTable = (compute_image==DC1394_ON) ? 1 : 0;
    sBlemish.m.m_nGrabCount = frame_nb;
    sBlemish.m.m_bShowImg = (show_image==DC1394_ON) ? 1 : 0;
    sBlemish.m.m_bMemLoad = (load_image==DC1394_ON) ? 1 : 0;
    sBlemish.m.m_bMemSave = (save_image==DC1394_ON) ? 1 : 0;

    /* Write back dsnu register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_BLEMISH_CONTROL,
                                             (uint32_t*) &sBlemish,
                                             sizeof(dc1394_avt_csradv_blemishcorrection_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT blemish control");

    /* wait until busy flag is cleared */
    int cont=1;
    while (cont) {
        usleep(50000);
        err = dc1394_get_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_BLEMISH_CONTROL,
                                                 (uint32_t*) &sBlemish,
                                                 sizeof(dc1394_avt_csradv_blemishcorrection_t)/4 );
        DC1394_ERR_RTN(err,"Could not get AVT blemish control");
        if (sBlemish.m.m_bBusy==0)
            cont=0;
    }
    return err;
}

/************************************************************************/
/* Get IO                                                               */
/************************************************************************/
dc1394error_t
dc1394_avt_get_io(dc1394camera_t *camera, uint32_t IO,
                  dc1394bool_t *polarity, uint32_t *mode, dc1394bool_t *pinstate)
{
    dc1394error_t err;
    uint32_t value;

    /* check that param IO is valid */
    switch(IO)
    {
        case REG_CAMERA_AVT_IO_INP_CTRL1:
        case REG_CAMERA_AVT_IO_INP_CTRL2:
        case REG_CAMERA_AVT_IO_INP_CTRL3:
        case REG_CAMERA_AVT_IO_INP_CTRL4:
        case REG_CAMERA_AVT_IO_INP_CTRL5:
        case REG_CAMERA_AVT_IO_INP_CTRL6:
        case REG_CAMERA_AVT_IO_INP_CTRL7:
        case REG_CAMERA_AVT_IO_INP_CTRL8:
        case REG_CAMERA_AVT_IO_OUTP_CTRL1:
        case REG_CAMERA_AVT_IO_OUTP_CTRL2:
        case REG_CAMERA_AVT_IO_OUTP_CTRL3:
        case REG_CAMERA_AVT_IO_OUTP_CTRL4:
        case REG_CAMERA_AVT_IO_OUTP_CTRL5:
        case REG_CAMERA_AVT_IO_OUTP_CTRL6:
        case REG_CAMERA_AVT_IO_OUTP_CTRL7:
        case REG_CAMERA_AVT_IO_OUTP_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* Retrieve IO parameters */
    err=dc1394_get_adv_control_register(camera,IO, &value);
    DC1394_ERR_RTN(err,"Could not get AVT IO register");

    /* polarity : Bit 7 */
    *polarity = (uint32_t)((value & 0x1000000UL) >> 24);

    /* pinstate : Bit 31 */
    *pinstate = (uint32_t)((value & 0x1UL));

    /* mode : Bits 11..15 */
    *mode =(uint32_t)((value >> 16 ) & 0x1FUL);

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Set IO                                                               */
/************************************************************************/
dc1394error_t
dc1394_avt_set_io(dc1394camera_t *camera,uint32_t IO,
                  dc1394bool_t polarity, uint32_t mode, dc1394bool_t pinstate)
{
    dc1394error_t err;
    uint32_t curval;

    /* check that param IO is valid */
    switch(IO)
    {
        case REG_CAMERA_AVT_IO_INP_CTRL1:
        case REG_CAMERA_AVT_IO_INP_CTRL2:
        case REG_CAMERA_AVT_IO_INP_CTRL3:
        case REG_CAMERA_AVT_IO_INP_CTRL4:
        case REG_CAMERA_AVT_IO_INP_CTRL5:
        case REG_CAMERA_AVT_IO_INP_CTRL6:
        case REG_CAMERA_AVT_IO_INP_CTRL7:
        case REG_CAMERA_AVT_IO_INP_CTRL8:
        case REG_CAMERA_AVT_IO_OUTP_CTRL1:
        case REG_CAMERA_AVT_IO_OUTP_CTRL2:
        case REG_CAMERA_AVT_IO_OUTP_CTRL3:
        case REG_CAMERA_AVT_IO_OUTP_CTRL4:
        case REG_CAMERA_AVT_IO_OUTP_CTRL5:
        case REG_CAMERA_AVT_IO_OUTP_CTRL6:
        case REG_CAMERA_AVT_IO_OUTP_CTRL7:
        case REG_CAMERA_AVT_IO_OUTP_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* Retrieve current IO parameters */
    err=dc1394_get_adv_control_register(camera,IO, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT IO register");

    /* polarity : Bit 7 */
    curval = (curval & 0xFEFFFFFFUL) | ((polarity ) << 24);

    /* mode : Bits 11..15 */
    curval = (curval & 0xFFE0FFFFUL) | ((mode << 16) & 0x1F0000UL );

    /* Pin state: bit 31 */
    if (mode==1)
        curval = (curval & 0xFFFFFFFEUL) | pinstate;

    /* Set  new IO parameters */
    err=dc1394_set_adv_control_register(camera,IO, curval);
    DC1394_ERR_RTN(err,"Could not set AVT IO register");

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Get PWM IO Info                                                      */
/************************************************************************/
dc1394error_t dc1394_avt_get_io_pwmout_info(dc1394camera_t *camera,
                                            uint32_t       pwm_output_pin,
                                            uint32_t       *min_period)
{
    dc1394error_t err;
    dc1394_avt_csradv_io_outp_pwmx_t sPWM;

    /* check that param pwm_output_pin is valid */
    switch(pwm_output_pin)
    {
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL1:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL2:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL3:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL4:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL5:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL6:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL7:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* get register */
    err=dc1394_get_adv_control_registers(camera, pwm_output_pin, (uint32_t*) &sPWM, 1);
    DC1394_ERR_RTN(err,"Could not get AVT PWM Output register");

    /* copy inquiry data */
    *min_period = sPWM.m_Ctrl.m_nMinPeriod;

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Get PWM IO                                                           */
/************************************************************************/
dc1394error_t dc1394_avt_get_io_pwmout(dc1394camera_t *camera,
                                       uint32_t       pwm_output_pin,
                                       uint32_t       *period,
                                       uint32_t       *pulse_width)
{
    dc1394error_t err;
    dc1394_avt_csradv_io_outp_pwmx_t sPWM;

    /* check that param pwm_output_pin is valid */
    switch(pwm_output_pin)
    {
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL1:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL2:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL3:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL4:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL5:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL6:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL7:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* get register */
    err=dc1394_get_adv_control_registers(camera,
                                         pwm_output_pin,
                                         (uint32_t*) &sPWM,
                                         sizeof(dc1394_avt_csradv_io_outp_pwmx_t)/4);
    DC1394_ERR_RTN(err,"Could not get AVT PWM Output register");

    /* read feature state */
    *period      = sPWM.m_Pwm.m_nPeriod;
    *pulse_width = sPWM.m_Pwm.m_nPulseWidth;

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Set PWM IO                                                           */
/************************************************************************/
dc1394error_t dc1394_avt_set_io_pwmout(dc1394camera_t *camera,
                                       uint32_t       pwm_output_pin,
                                       uint32_t       period,
                                       uint32_t       pulse_width)
{
    dc1394error_t err;
    dc1394_avt_csradv_io_outp_pwmx_t sPWM;

    /* check that param pwm_output_pin is valid */
    switch(pwm_output_pin)
    {
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL1:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL2:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL3:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL4:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL5:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL6:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL7:
        case REG_CAMERA_AVT_IO_OUTP_PWM_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* get register */
    err=dc1394_get_adv_control_registers(camera,
                                         pwm_output_pin,
                                         (uint32_t*) &sPWM,
                                         sizeof(dc1394_avt_csradv_io_outp_pwmx_t)/4);
    DC1394_ERR_RTN(err,"Could not get AVT PWM Output register");

    /* set feature state */
    sPWM.m_Pwm.m_nPeriod = period;
    sPWM.m_Pwm.m_nPulseWidth = pulse_width;

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             pwm_output_pin,
                                             (uint32_t*) &sPWM,
                                             sizeof(dc1394_avt_csradv_io_outp_pwmx_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT PWM Output register");

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Get Input Debounce Info                                              */
/************************************************************************/
dc1394error_t dc1394_avt_get_io_inp_debounce_info(dc1394camera_t *camera,
                                                  uint32_t       debounce_inp_pin,
                                                  uint32_t       *min_debounce_time,
                                                  uint32_t       *max_debounce_time)
{
    dc1394error_t err;
    dc1394_avt_csradv_io_inp_debounce_x sDebounce;

    /* check that param debounce_inp_pin is valid */
    switch(debounce_inp_pin)
    {
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL1:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL2:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL3:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL4:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL5:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL6:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL7:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* get registers */
    err=dc1394_get_adv_control_registers(camera,
                                         debounce_inp_pin,
                                         (uint32_t*) &sDebounce,
                                         sizeof(dc1394_avt_csradv_io_inp_debounce_x)/4);
    DC1394_ERR_RTN(err,"Could not get AVT Input Debounce register");

    /* read feature info */
    *min_debounce_time = sDebounce.m_nMinDebTime;
    *max_debounce_time = sDebounce.m_nMaxDebTime;

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Get Input Debounce                                                   */
/************************************************************************/
dc1394error_t dc1394_avt_get_io_inp_debounce(dc1394camera_t *camera,
                                             uint32_t       debounce_inp_pin,
                                             uint32_t       *debounce_time)
{
    dc1394error_t err;
    dc1394_avt_csradv_io_inp_debounce_x sDebounce;

    /* check that param debounce_inp_pin is valid */
    switch(debounce_inp_pin)
    {
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL1:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL2:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL3:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL4:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL5:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL6:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL7:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* get registers */
    err=dc1394_get_adv_control_registers(camera,
                                         debounce_inp_pin,
                                         (uint32_t*) &sDebounce,
                                         sizeof(dc1394_avt_csradv_io_inp_debounce_x)/4);
    DC1394_ERR_RTN(err,"Could not get AVT Input Debounce register");

    /* read feature config */
    *debounce_time = sDebounce.m_Ctrl.m.m_nDebTime;

    return DC1394_SUCCESS;
}

/************************************************************************/
/* Set Input Debounce                                                   */
/************************************************************************/
dc1394error_t dc1394_avt_set_io_inp_debounce(dc1394camera_t *camera,
                                             uint32_t       debounce_inp_pin,
                                             uint32_t       debounce_time)
{
    dc1394error_t err;
    dc1394_avt_csradv_io_inp_debounce_x sDebounce;

    /* check that param debounce_inp_pin is valid */
    switch(debounce_inp_pin)
    {
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL1:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL2:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL3:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL4:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL5:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL6:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL7:
        case REG_CAMERA_AVT_IO_INP_DEBOUNCE_CTRL8:
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* get registers */
    err=dc1394_get_adv_control_registers(camera,
                                         debounce_inp_pin,
                                         (uint32_t*) &sDebounce,
                                         sizeof(dc1394_avt_csradv_io_inp_debounce_x)/4);
    DC1394_ERR_RTN(err,"Could not get AVT Input Debounce register");

    /* read feature config */
    sDebounce.m_Ctrl.m.m_nDebTime = debounce_time;

    /* write back registers */
    err=dc1394_set_adv_control_registers(camera,
                                         debounce_inp_pin,
                                         (uint32_t*) &sDebounce,
                                         sizeof(dc1394_avt_csradv_io_inp_debounce_x)/4);
    DC1394_ERR_RTN(err,"Could not set AVT Input Debounce register");

    return DC1394_SUCCESS;
}

/************************************************************************/
/* BusReset IEEE1394                                                    */
/************************************************************************/
dc1394error_t
dc1394_avt_reset(dc1394camera_t *camera)
{
    dc1394error_t err;
    uint32_t value;
    /* ON / OFF : Bit 6 */
    value= (1<<25) + 200; /*2sec*/
    /* Reset */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_SOFT_RESET,value);
    DC1394_ERR_RTN(err,"Could not set AVT soft reset");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Lookup Tables (LUT)                                              */
/************************************************************************/
dc1394error_t
dc1394_avt_get_lut(dc1394camera_t *camera, dc1394bool_t *on_off, uint32_t *lutnb)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current luts parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_LUT_CTRL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT LUT control");

    /* Shading ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* Number of lut : Bits 26..31 */
    *lutnb =(uint32_t)((value & 0x3FUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Lookup Tables (LUT)                                              */
/************************************************************************/
dc1394error_t
dc1394_avt_set_lut(dc1394camera_t *camera, dc1394bool_t on_off, uint32_t lutnb)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current luts parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_LUT_CTRL, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT LUT control");

    /* Shading ON / OFF : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25);

    /* Number of lut : Bits 26..31 */
    curval = (curval & 0xFFFFFFB0UL) | ((lutnb & 0x3FUL ));

    /* Set new luts parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_LUT_CTRL, curval);
    DC1394_ERR_RTN(err,"Could not set AVT LUT control");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get LUT ctrl                                                         */
/************************************************************************/
dc1394error_t
dc1394_avt_get_lut_mem_ctrl(dc1394camera_t *camera, dc1394bool_t *en_write,
                            uint32_t * AccessLutNo,uint32_t *addroffset)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current memory luts parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_LUT_MEM_CTRL, &value);
    DC1394_ERR_RTN(err,"Could not get AVT LUT memory control");

    /* Enable write access : Bit 5 */
    *en_write = (uint32_t)((value & 0x4000000UL) >> 26);

    /* AccessLutNo : Bits 8..15 */
    *AccessLutNo=(uint32_t)((value >> 16) & 0xFFUL);

    /* addroffset in byte : Bits 16..31 */
    *addroffset =(uint32_t)((value & 0xFFFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set LUT ctrl                                                                 */
/************************************************************************/
dc1394error_t
dc1394_avt_set_lut_mem_ctrl(dc1394camera_t *camera,
                            dc1394bool_t en_write, uint32_t AccessLutNo, uint32_t addroffset)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current memory luts parameters */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_LUT_MEM_CTRL, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT LUT memory control");

    /* write access enable : Bit 5 */
    curval = (curval & 0xFBFFFFFFUL) | ((en_write ) << 26);

    /* AccessLutNo : Bits 8..15 */
    curval = (curval & 0xFF00FFFFUL) | ((AccessLutNo << 16) & 0xFF0000UL );

    /* Number of images : Bits 16..31 */
    curval = (curval & 0xFFFF0000UL) | ((addroffset & 0xFFFFUL ));

    /* Set new parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_LUT_MEM_CTRL, curval);
    DC1394_ERR_RTN(err,"Could not set AVT LUT memory control");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get LUT  info - deprecated                                           */
/************************************************************************/
dc1394error_t
dc1394_avt_get_lut_info(dc1394camera_t *camera, uint32_t *NumOfLuts, uint32_t *MaxLutSize)
{
    dc1394error_t err;
    uint32_t value;
    /* Retrieve luts info */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_LUT_INFO, &value);
    DC1394_ERR_RTN(err,"Could not get AVT LUT info");

    /* NumOfLuts : Bits 8..15 */
    *NumOfLuts=(uint32_t)((value >> 16) & 0xFFUL);

    /* MaxLutSize : Bits 16..31 */
    *MaxLutSize =(uint32_t)((value & 0xFFFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Extd. LUT info                                                   */
/************************************************************************/
dc1394error_t dc1394_avt_get_lut_extd_info(dc1394camera_t *camera,
                                           uint32_t       *NumOfLuts,
                                           uint32_t       *MaxValue,
                                           uint32_t       *NumOfValues,
                                           uint32_t       *MaxLutSize)
{
    dc1394error_t err;

    /* get register */
    dc1394_avt_csradv_lut_ctrl_t sLut;
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LUT_CTRL,
                                             (uint32_t*) &sLut,
                                             sizeof(dc1394_avt_csradv_lut_ctrl_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature LUT");

    /* find out bits per value */
    dc1394_avt_family_t eFamily = (dc1394_avt_family_t) ( ( camera->guid >> 24 ) & 0xff );
    uint32_t bitsPerVal;
    switch ( eFamily )
    {
        case DC1394_AVT_FAMILY_DOLPHIN_OLD:
        case DC1394_AVT_FAMILY_DOLPHIN:
            /* Dolphin cameras do not support BitsPerValue inquiry, they have always 10 bpv */
            bitsPerVal = 10;
            break;
        default:
            bitsPerVal = sLut.m_Info.m_nBitsPerValue;
            break;
    }

    /* return info */
    *NumOfLuts = sLut.m_Info.m_nNumOfLuts;
    *MaxValue = (1<<bitsPerVal)-1;
    *NumOfValues = (bitsPerVal<=8)? sLut.m_Info.m_nMaxSize : sLut.m_Info.m_nMaxSize/2;
    *MaxLutSize = sLut.m_Info.m_nMaxSize;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Autofunction AOI Info                                            */
/************************************************************************/
dc1394error_t dc1394_avt_get_autofunc_aoi_info(dc1394camera_t *camera,
                                               uint32_t       *unit_x,
                                               uint32_t       *unit_y)
{
    dc1394error_t err;
    dc1394_avt_csradv_autofnc_aoi_t sAFAOI;

    /* get register */
    err=dc1394_get_adv_control_registers(camera,REG_CAMERA_AVT_AUTOFNC_AOI, (uint32_t*) &sAFAOI, 1);
    DC1394_ERR_RTN(err,"Could not get AVT autofunction AOI Register");

    /* read units - 128 is the default unit size when unit inquiries are set to 0 */
    *unit_x = (sAFAOI.m_Ctrl.m_nXUnits==0)? 128 : sAFAOI.m_Ctrl.m_nXUnits;
    *unit_y = (sAFAOI.m_Ctrl.m_nYUnits==0)? 128 : sAFAOI.m_Ctrl.m_nYUnits;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Autofunction AOI - deprecated                                    */
/************************************************************************/
dc1394error_t
dc1394_avt_get_aoi(dc1394camera_t *camera,
                   dc1394bool_t *on_off, int *left, int *top, int *width, int *height)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current mode*/
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_AUTOFNC_AOI, &value);
    DC1394_ERR_RTN(err,"Could not get AVT autofocus AOI");

    /*  ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* Retrieve current size of area*/
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_AF_AREA_SIZE, &value);
    DC1394_ERR_RTN(err,"Could not get AVT AF area size");

    /* width : Bits 0..15 */
    *width =(uint32_t)(value >> 16);
    /* height : Bits 16..31 */
    *height =(uint32_t)(value & 0xFFFFUL );

    /* Retrieve current position of area*/
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_AF_AREA_POSITION, &value);
    DC1394_ERR_RTN(err,"Could not get AVT AF area position");

    /* left : Bits 0..15 */
    *left =(uint32_t)(value >> 16);
    /* top : Bits 16..31 */
    *top =(uint32_t)(value & 0xFFFFUL );

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Autofunction AOI                                                 */
/************************************************************************/
dc1394error_t dc1394_avt_get_autofunc_aoi(dc1394camera_t *camera,
                                          dc1394switch_t *on_off,
                                          dc1394switch_t *show_area,
                                          uint32_t       *left,
                                          uint32_t       *top,
                                          uint32_t       *width,
                                          uint32_t       *height)
{
    dc1394error_t err;
    dc1394_avt_csradv_autofnc_aoi_t sAFAOI;

    /* get register */
    err=dc1394_get_adv_control_registers(camera,
                                         REG_CAMERA_AVT_AUTOFNC_AOI,
                                         (uint32_t*) &sAFAOI,
                                         sizeof(dc1394_avt_csradv_autofnc_aoi_t)/4);
    DC1394_ERR_RTN(err,"Could not get AVT autofunction AOI Registers");

    /* read config */
    *on_off = (sAFAOI.m_Ctrl.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;
    *show_area = (sAFAOI.m_Ctrl.m_bShowWorkArea!=0)? DC1394_ON : DC1394_OFF;
    *left = sAFAOI.m_ImagePos.m_nLeft;
    *top = sAFAOI.m_ImagePos.m_nTop;
    *width = sAFAOI.m_ImageSize.m_nWidth;
    *height = sAFAOI.m_ImageSize.m_nHeight;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Autofunction AOI - deprecated                                    */
/************************************************************************/
dc1394error_t
dc1394_avt_set_aoi(dc1394camera_t *camera,
                   dc1394bool_t on_off,int left, int top, int width, int height)
{
    dc1394error_t err;
    uint32_t curval;

    /* ON / OFF : Bit 6 */
    curval = on_off << 25;

    /* Set feature on off */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_AUTOFNC_AOI, curval);
    DC1394_ERR_RTN(err,"Could not set AVT autofocus AOI");

    /* Set size */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_AF_AREA_SIZE, (width << 16) | height);
    DC1394_ERR_RTN(err,"Could not set AVT AF area size");

    /* Set position */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_AF_AREA_POSITION,(left << 16) | top );
    DC1394_ERR_RTN(err,"Could not set AVT AF area position");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Autofunction AOI                                                 */
/************************************************************************/
dc1394error_t dc1394_avt_set_autofunc_aoi(dc1394camera_t *camera,
                                          dc1394switch_t on_off,
                                          dc1394switch_t show_area,
                                          uint32_t       left,
                                          uint32_t       top,
                                          uint32_t       width,
                                          uint32_t       height)
{
    dc1394error_t err;
    dc1394_avt_csradv_autofnc_aoi_t sAFAOI;

    /* get registers */
    err=dc1394_get_adv_control_registers(camera,
                                         REG_CAMERA_AVT_AUTOFNC_AOI,
                                         (uint32_t*) &sAFAOI,
                                         sizeof(dc1394_avt_csradv_autofnc_aoi_t)/4);
    DC1394_ERR_RTN(err,"Could not get AVT Autofunction AOI Registers");

    /* set config */
    sAFAOI.m_Ctrl.m_bOnOff = (on_off==DC1394_ON)? 1:0;
    sAFAOI.m_Ctrl.m_bShowWorkArea = (show_area==DC1394_ON)? 1:0;
    sAFAOI.m_ImagePos.m_nLeft = left;
    sAFAOI.m_ImagePos.m_nTop = top;
    sAFAOI.m_ImageSize.m_nWidth = width;
    sAFAOI.m_ImageSize.m_nHeight = height;

    /* write back registers */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_AUTOFNC_AOI,
                                             (uint32_t*) &sAFAOI,
                                             sizeof(dc1394_avt_csradv_autofnc_aoi_t)/4);
    DC1394_ERR_RTN(err,"Could not set AVT Autofunction AOI Registers");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get supported test_images                                            */
/************************************************************************/
dc1394error_t
dc1394_avt_get_test_images_info(dc1394camera_t *camera,
                                dc1394bool_t   *TestImage1,
                                dc1394bool_t   *TestImage2,
                                dc1394bool_t   *TestImage3,
                                dc1394bool_t   *TestImage4,
                                dc1394bool_t   *TestImage5,
                                dc1394bool_t   *TestImage6,
                                dc1394bool_t   *TestImage7)
{
    dc1394error_t err;
    dc1394_avt_csradv_testpix_t sTestPix;

    /* Retrieve test image number */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TEST_IMAGE, (uint32_t*)  &sTestPix);
    DC1394_ERR_RTN(err,"Could not get AVT test image");

    /* read inquiries */
    *TestImage1 = (sTestPix.m.m_bImg1Inq!=0)? DC1394_TRUE : DC1394_FALSE;
    *TestImage2 = (sTestPix.m.m_bImg2Inq!=0)? DC1394_TRUE : DC1394_FALSE;
    *TestImage3 = (sTestPix.m.m_bImg3Inq!=0)? DC1394_TRUE : DC1394_FALSE;
    *TestImage4 = (sTestPix.m.m_bImg4Inq!=0)? DC1394_TRUE : DC1394_FALSE;
    *TestImage5 = (sTestPix.m.m_bImg5Inq!=0)? DC1394_TRUE : DC1394_FALSE;
    *TestImage6 = (sTestPix.m.m_bImg6Inq!=0)? DC1394_TRUE : DC1394_FALSE;
    *TestImage7 = (sTestPix.m.m_bImg7Inq!=0)? DC1394_TRUE : DC1394_FALSE;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get test_images                                                      */
/************************************************************************/
dc1394error_t
dc1394_avt_get_test_images(dc1394camera_t *camera, uint32_t *image_no)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve test image number */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TEST_IMAGE, &value);
    DC1394_ERR_RTN(err,"Could not get AVT test image");

    /* Numero Image : Bits 28..31 */
    *image_no =(uint32_t)((value & 0xFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set test_images                                                      */
/************************************************************************/
dc1394error_t
dc1394_avt_set_test_images(dc1394camera_t *camera, uint32_t image_no)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current test image */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_TEST_IMAGE, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT test image");

    /* Numero Image : Bits 28..31 */
    curval = (curval & 0xFFFFFFF0UL) | ((image_no & 0xFUL ));

    /* Set new test image */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_TEST_IMAGE,curval);
    DC1394_ERR_RTN(err,"Could not set AVT test image");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get frame info - deprecated                                          */
/************************************************************************/
dc1394error_t
dc1394_avt_get_frame_info(dc1394camera_t *camera, uint32_t *framecounter)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve frame info */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_FRAMECOUNTER, &value);
    DC1394_ERR_RTN(err,"Could not get AVT framecounter");

    /* framecounter : Bits 0..31 */
    *framecounter =(uint32_t)(value);

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get frame counter                                                    */
/************************************************************************/
dc1394error_t
dc1394_avt_get_frame_counter(dc1394camera_t *camera, const dc1394_avt_smart_feature_info_t *feature_info, uint32_t *framecounter)
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>feature_info->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394error_t err;
    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) feature_info;

    /* get register offset */
    uint64_t offset;
    if( smart_feature_full->Internal.Inq1.m_bFrameInfo != 0 )
    {
        offset = REG_CAMERA_AVT_FRAMEINFO;
    }
    else if( smart_feature_full->Internal.Inq1.m_bFrmCounter2 != 0 )
    {
        offset = REG_CAMERA_AVT_FRAMEINFO_NEW;
    }
    else return DC1394_FUNCTION_NOT_SUPPORTED;

    /* get register */
    dc1394_avt_csradv_frameinfo_t sFrameInfo;
    err = dc1394_get_adv_control_registers ( camera,
                                             offset,
                                             (uint32_t*) &sFrameInfo,
                                             sizeof(dc1394_avt_csradv_frameinfo_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature FrameInfo");

    /* get frame counter */
    *framecounter = sFrameInfo.m_nFrameCounter;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Reset frame info - deprecated                                        */
/************************************************************************/
dc1394error_t
dc1394_avt_reset_frame_info(dc1394camera_t *camera)
{
    dc1394error_t err;
    /* Reset counter */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_FRAMEINFO,1 << 30);
    DC1394_ERR_RTN(err,"Could not get AVT frame info");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Reset frame counter                                                  */
/************************************************************************/
dc1394error_t
dc1394_avt_reset_frame_counter(dc1394camera_t                           *camera,
                               const dc1394_avt_smart_feature_info_t    *feature_info)
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>feature_info->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394error_t err;
    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) feature_info;

    /* get register offset */
    uint64_t offset;
    if( smart_feature_full->Internal.Inq1.m_bFrameInfo != 0 )
    {
        offset = REG_CAMERA_AVT_FRAMEINFO;
    }
    else if( smart_feature_full->Internal.Inq1.m_bFrmCounter2 != 0 )
    {
        offset = REG_CAMERA_AVT_FRAMEINFO_NEW;
    }
    else return DC1394_FUNCTION_NOT_SUPPORTED;

    /* get register */
    dc1394_avt_csradv_frameinfo_t sFrameInfo;
    err = dc1394_get_adv_control_registers ( camera,
                                             offset,
                                             (uint32_t*) &sFrameInfo,
                                             1 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature FrameInfo");

    /* set reset flag */
    sFrameInfo.m_Cmd.m_bClearFrameCounter = 1;

    /* write register */
    err = dc1394_set_adv_control_registers ( camera,
                                             offset,
                                             (uint32_t*) &sFrameInfo,
                                             1 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature FrameInfo");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get trigger counter                                                  */
/************************************************************************/
dc1394error_t
dc1394_avt_get_trigger_counter(dc1394camera_t *camera, uint32_t *triggercounter)
{
    dc1394error_t err;

    /* get register */
    dc1394_avt_csradv_frameinfo_t sTriggerInfo;
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_TRGCOUNTER_NEW,
                                             (uint32_t*) &sTriggerInfo,
                                             sizeof(dc1394_avt_csradv_frameinfo_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature TriggerCounter");

    /* get frame counter */
    *triggercounter = sTriggerInfo.m_nFrameCounter;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Reset trigger counter                                                */
/************************************************************************/
dc1394error_t
dc1394_avt_reset_trigger_counter(dc1394camera_t *camera)
{
    dc1394error_t err;

    /* get register */
    dc1394_avt_csradv_frameinfo_t sTriggerInfo;
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_TRGCOUNTER_NEW,
                                             (uint32_t*) &sTriggerInfo,
                                             1 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature TriggerCounter");

    /* set reset flag */
    sTriggerInfo.m_Cmd.m_bClearFrameCounter = 1;

    /* write register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_TRGCOUNTER_NEW,
                                             (uint32_t*) &sTriggerInfo,
                                             1 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature TriggerCounter");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Deferred image transport                                         */
/************************************************************************/
dc1394error_t
dc1394_avt_get_deferred_trans(dc1394camera_t *camera,
                              dc1394bool_t *HoldImage, dc1394bool_t * FastCapture,
                              uint32_t *FifoSize, uint32_t *NumOfImages )
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve Deferred image transport mode */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_DEFERRED_TRANS, &value);
    DC1394_ERR_RTN(err,"Could not get AVT deferred transfer info");

    /* enable/disable deferred transport mode : Bit 6 */
    *HoldImage = (uint32_t)((value & 0x2000000UL) >> 25);

    /* enable/disable fast capture mode (format 7 only) : Bit 7 */
    *FastCapture = (uint32_t)((value & 0x1000000UL) >> 24);

    /* Size of fifo in number of image : Bits 16..23 */
    *FifoSize =(uint32_t)((value >> 8 & 0xFFUL));

    /* Number of images in buffer: Bits 24..31 */
    *NumOfImages =(uint32_t)((value & 0xFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Deferred image transport                                         */
/************************************************************************/
dc1394error_t
dc1394_avt_set_deferred_trans(dc1394camera_t *camera,
                              dc1394bool_t HoldImage, dc1394bool_t FastCapture,
                              uint32_t FifoSize, uint32_t NumOfImages,
                              dc1394bool_t SendImage)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve current image transport mode */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_DEFERRED_TRANS, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT deferred transfer info");

    /* Send NumOfImages now : Bit 5 */
    curval = (curval & 0xFBFFFFFFUL) | ((SendImage ) << 26);

    /* enable/disable deferred transport mode : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((HoldImage ) << 25);

    /* enable/disable fast capture mode (format 7 only) : Bit 7 */
    curval = (curval & 0xFEFFFFFFUL) | ((FastCapture ) << 24);

    /* Size of fifo in number of image : Bits 16..23 */
    curval = (curval & 0xFFFF00FFUL) | (((FifoSize << 8) & 0xFF00UL ));

    /* Number of images : Bits 24..31 */
    curval = (curval & 0xFFFFFF00UL) | ((NumOfImages & 0xFFUL ));

    /* Set new parameters */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_DEFERRED_TRANS, curval);
    DC1394_ERR_RTN(err,"Could not set AVT deferred transfer info");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get GPData info                                                      */
/************************************************************************/
dc1394error_t
dc1394_avt_get_gpdata_info(dc1394camera_t *camera, uint32_t *BufferSize)
{
    dc1394error_t err;
    uint32_t value;
    /* Retrieve info on the general purpose buffer */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_GPDATA_INFO, &value);
    DC1394_ERR_RTN(err,"Could not get AVT GP data info");

    /* BufferSize : Bits 16..31 */
    *BufferSize =(uint32_t)((value & 0xFFFFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Read size number of bytes from GPData buffer                         */
/************************************************************************/
dc1394error_t
dc1394_avt_read_gpdata(dc1394camera_t *camera, unsigned char *buf, uint32_t size)
{
    dc1394error_t                               err;
    uint32_t                                    GPDataSize;

    err = dc1394_avt_get_gpdata_info(camera, &GPDataSize);
    DC1394_ERR_RTN(err,"Could not get AVT GPData Info");

    /* GP Data buffer size should be a multiple of four */
    GPDataSize -= GPDataSize%4;

    uint32_t nSize_WholeQuads = size - (size%4);
    uint32_t nSize_Remainder = size%4;

    uint32_t nDataRead = 0;
    uint32_t nChunkSize;

    while (nDataRead < nSize_WholeQuads)
    {
        // Compute count of data to read in this run.
        nChunkSize = MIN (GPDataSize, nSize_WholeQuads - nDataRead);

        // Read chunk from camera.
        err = dc1394_get_adv_control_registers( camera,
                                                REG_CAMERA_AVT_GPDATA_BUFFER,
                                                ((uint32_t*) buf) + (nDataRead/4),
                                                nChunkSize/4 );
        DC1394_ERR_RTN(err,"Can't read GPData buffer!");

        // Prepare to read next chunk.
        nDataRead += nChunkSize;
    }

    /* read remaining bytes when size is not a multiple of four */
    if( nSize_Remainder>0 )
    {
        unsigned char quadbuf[4];
        err = dc1394_get_adv_control_registers( camera,
                                                REG_CAMERA_AVT_GPDATA_BUFFER,
                                                (uint32_t*) quadbuf,
                                                1 );
        DC1394_ERR_RTN(err,"Can't write data buffer!");

        buf[nDataRead] = quadbuf[0];
        if( nSize_Remainder>1 )
        {
            buf[nDataRead+1] = quadbuf[1];
        }
        if( nSize_Remainder>2 )
        {
            buf[nDataRead+2] = quadbuf[2];
        }
    }

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Write size number of bytes to GPData buffer                          */
/************************************************************************/
dc1394error_t
dc1394_avt_write_gpdata(dc1394camera_t *camera, unsigned char *buf, uint32_t size)
{
    dc1394error_t                               err;
    uint32_t                                    GPDataSize;

    err = dc1394_avt_get_gpdata_info(camera, &GPDataSize);
    DC1394_ERR_RTN(err,"Could not get AVT GPData Info");

    /* GP Data buffer size should be a multiple of four */
    GPDataSize -= GPDataSize%4;

    uint32_t nSize_WholeQuads = size - (size%4);
    uint32_t nSize_Remainder = size%4;

    uint32_t nDataWritten = 0;
    uint32_t nChunkSize;

    while (nDataWritten < nSize_WholeQuads)
    {
        // Compute count of data to write in this run.
        nChunkSize = MIN (GPDataSize, nSize_WholeQuads - nDataWritten);

        // Write chunk to camera.
        err = dc1394_set_adv_control_registers( camera,
                                                REG_CAMERA_AVT_GPDATA_BUFFER,
                                                ((uint32_t*) buf) + (nDataWritten/4),
                                                nChunkSize/4 );
        DC1394_ERR_RTN(err,"Can't write data buffer!");

        // Prepare to read next chunk.
        nDataWritten += nChunkSize;
    }

    /* write remaining bytes when size is not a multiple of four */
    if( nSize_Remainder>0 )
    {
        unsigned char quadbuf[4];
        quadbuf[0] = buf[nDataWritten];
        quadbuf[1] = (nSize_Remainder>1)? buf[nDataWritten+1] : 0;
        quadbuf[2] = (nSize_Remainder>2)? buf[nDataWritten+2] : 0;
        quadbuf[3] = 0;
        err = dc1394_set_adv_control_registers( camera,
                                                REG_CAMERA_AVT_GPDATA_BUFFER,
                                                (uint32_t*) quadbuf,
                                                1 );
        DC1394_ERR_RTN(err,"Can't write data buffer!");
    }

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Read shading image from camera into buffer                           */
/************************************************************************/
dc1394error_t
dc1394_avt_read_shading_img(dc1394camera_t *camera, unsigned char *buf,
                            uint32_t size)
{
    dc1394error_t err;
    dc1394bool_t en_write;
    uint32_t addr;

    /* Enable read at address 0 */
    err = dc1394_avt_get_shading_mem_ctrl(camera, &en_write, NULL, NULL);
    DC1394_ERR_RTN(err,"Could not read AVT shading mem ctrl");
    err = dc1394_avt_set_shading_mem_ctrl(camera, en_write, DC1394_TRUE, 0);
    DC1394_ERR_RTN(err,"Could not write AVT shading mem ctrl");

    /* Read data */
    err = dc1394_avt_read_gpdata(camera, buf, size);
    DC1394_ERR_RTN(err,"Could not read AVT gpdata");

    /* Disable read */
    err = dc1394_avt_get_shading_mem_ctrl(camera, &en_write, NULL, &addr);
    DC1394_ERR_RTN(err,"Could not read AVT shading mem ctrl");
    err = dc1394_avt_set_shading_mem_ctrl(camera, en_write, DC1394_FALSE, addr);
    DC1394_ERR_RTN(err,"Could not write AVT shading mem ctrl");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Write shading image from buffer to camera                            */
/************************************************************************/
dc1394error_t
dc1394_avt_write_shading_img(dc1394camera_t *camera, unsigned char *buf,
                             uint32_t size)
{
    dc1394error_t err;

    /* Enable write at address 0 */
    err = dc1394_avt_set_shading_mem_ctrl(camera, DC1394_TRUE, DC1394_FALSE, 0);
    DC1394_ERR_RTN(err,"Could not write AVT shading mem ctrl");

    /* Write data */
    err = dc1394_avt_write_gpdata(camera, buf, size);
    DC1394_ERR_RTN(err,"Could not write AVT gpdata");

    /* Disable write */
    err = dc1394_avt_set_shading_mem_ctrl(camera, DC1394_FALSE, DC1394_FALSE, 0);
    DC1394_ERR_RTN(err,"Could not write AVT shading mem ctrl");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Write lookup table from buffer to camera                             */
/************************************************************************/
dc1394error_t
dc1394_avt_write_lut(dc1394camera_t *camera, uint32_t LutNo, unsigned char *buf,
                     uint32_t size)
{
    dc1394error_t err;

    /* Enable write at address 0 */
    err = dc1394_avt_set_lut_mem_ctrl(camera, DC1394_TRUE, LutNo, 0);
    DC1394_ERR_RTN(err,"Could not write AVT LUT mem ctrl");

    /* Write data */
    err = dc1394_avt_write_gpdata(camera, buf, size);
    DC1394_ERR_RTN(err,"Could not write AVT gpdata");

    /* Disable write */
    err = dc1394_avt_set_lut_mem_ctrl(camera, DC1394_FALSE, LutNo, 0);
    DC1394_ERR_RTN(err,"Could not write AVT LUT mem ctrl");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Read channel adjust gain (AVT Pike)                                  */
/************************************************************************/
dc1394error_t dc1394_avt_get_channel_adjust(dc1394camera_t *camera, int16_t *channel_adjust)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current channel adjust */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_CHANNEL_ADJUST_VALUE, &value);
    DC1394_ERR_RTN(err,"Could not get AVT channel gain adjust");

    /* channel adjust: Bits 16..31 */
    *channel_adjust = (int16_t)value;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Write channel adjust gain (AVT Pike)                                 */
/************************************************************************/
dc1394error_t dc1394_avt_set_channel_adjust(dc1394camera_t *camera, int16_t channel_adjust)
{
    dc1394error_t err;

    /* Set new channel adjust */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_CHANNEL_ADJUST_VALUE, (uint32_t)channel_adjust);
    DC1394_ERR_RTN(err,"Could not set AVT channel gain adjust");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Read channel adjust offset (AVT Pike)                                */
/************************************************************************/
dc1394error_t dc1394_avt_get_channel_adjust_offset(dc1394camera_t *camera,
                                            int16_t *channel_adjust)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve current channel adjust */
    err=dc1394_get_adv_control_register(camera,REG_CAMERA_AVT_CHANNEL_ADJUST_OFFSET_VALUE, &value);
    DC1394_ERR_RTN(err,"Could not get AVT channel offset adjust");

    /* channel adjust: Bits 16..31 */
    *channel_adjust = (int16_t)value;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Write channel adjust offset (AVT Pike)                               */
/************************************************************************/
dc1394error_t dc1394_avt_set_channel_adjust_offset(dc1394camera_t *camera,
                                            int16_t channel_adjust)
{
    dc1394error_t err;

    /* Set new channel adjust */
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_CHANNEL_ADJUST_OFFSET_VALUE, (uint32_t)channel_adjust);
    DC1394_ERR_RTN(err,"Could not set AVT channel offset adjust");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Color Correction + Coefficients                                  */
/************************************************************************/
dc1394error_t dc1394_avt_set_color_corr(dc1394camera_t *camera, dc1394bool_t on_off, dc1394bool_t reset, int32_t Crr, int32_t Cgr, int32_t Cbr, int32_t Crg, int32_t Cgg, int32_t Cbg, int32_t Crb, int32_t Cgb, int32_t Cbb)
{
    dc1394error_t err;
    uint32_t curval;

    //retrieve color correction
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT color correction");

    //ON / OFF : Bit 6
    curval = (curval & 0xFDFFFFFFUL) | ((on_off) << 25);

    //reset coefficients to defaults : Bit 7
    curval = (curval & 0xFEFFFFFFUL) | ((reset) << 24);

    //set new parameters
    err=dc1394_set_adv_control_register(camera,REG_CAMERA_AVT_COLOR_CORR, curval);
    DC1394_ERR_RTN(err,"Could not set AVT color correction");

    if (!reset) {
        //red channel coefficients
        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CRR, Crr);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Crr");

        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CGR, Cgr);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Cgr");

        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CBR, Cbr);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Cbr");

        //green channel coefficients
        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CRG, Crg);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Crg");

        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CGG, Cgg);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Cgg");

        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CBG, Cbg);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Cbg");

        //blue channel coefficients
        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CRB, Crb);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Crb");

        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CGB, Cgb);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Cgb");

        err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CBB, Cbb);
        DC1394_ERR_RTN(err,"Could not set AVT color correction coefficient Cbb");
    }

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Color Correction + Coefficients                                  */
/************************************************************************/
dc1394error_t dc1394_avt_get_color_corr(dc1394camera_t *camera, dc1394bool_t *on_off, int32_t *Crr, int32_t *Cgr, int32_t *Cbr, int32_t *Crg, int32_t *Cgg, int32_t *Cbg, int32_t *Crb, int32_t *Cgb, int32_t *Cbb)
{
    dc1394error_t err;
    uint32_t value;

    //retrieve color correction
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR, &value);
    DC1394_ERR_RTN(err,"Could not get AVT color correction");

    //ON / OFF : Bit 6
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    //red channel coefficients
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CRR, (uint32_t *)Crr);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Crr");

    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CGR, (uint32_t *)Cgr);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Cgr");

    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CBR, (uint32_t *)Cbr);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Cbr");

    //green channel coefficients
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CRG, (uint32_t *)Crg);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Crg");

    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CGG, (uint32_t *)Cgg);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Cgg");

    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CBG, (uint32_t *)Cbg);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Cbg");

    //blue channel coefficients
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CRB, (uint32_t *)Crb);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Crb");

    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CGB, (uint32_t *)Cgb);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Cgb");

    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_COLOR_CORR_CBB, (uint32_t *)Cbb);
    DC1394_ERR_RTN(err,"Could not get AVT color correction coefficient Cbb");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get HSNR                                                             */
/************************************************************************/
dc1394error_t dc1394_avt_get_hsnr(dc1394camera_t *camera, dc1394bool_t *on_off, uint32_t *grabCount)
{
    dc1394error_t err;
    uint32_t value;

    /* Retrieve HSNRR */
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_HSNRR, &value);
    DC1394_ERR_RTN(err,"Could not get AVT HSNRR");

    /*ON / OFF : Bit 6 */
    *on_off = (uint32_t)((value & 0x2000000UL) >> 25);

    /* grabCount: Bits 23..31 */
    *grabCount =(uint32_t)((value & 0x1FFUL));

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set HSNR                                                             */
/************************************************************************/
dc1394error_t dc1394_avt_set_hsnr(dc1394camera_t *camera, dc1394bool_t on_off, uint32_t grabCount)
{
    dc1394error_t err;
    uint32_t curval;

    /* Retrieve HSNR */
    err=dc1394_get_adv_control_register(camera, REG_CAMERA_AVT_HSNRR, &curval);
    DC1394_ERR_RTN(err,"Could not get AVT HSNRR");

    /*ON / OFF : Bit 6 */
    curval = (curval & 0xFDFFFFFFUL) | ((on_off) << 25);

    /* grabCount: Bits 23..31 */
    curval = (curval & 0xFFFFFE00UL) | grabCount;

    /* Set new parameters */
    err=dc1394_set_adv_control_register(camera, REG_CAMERA_AVT_HSNRR, curval);
    DC1394_ERR_RTN(err,"Could not set AVT HSNRR");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get SIS                                                              */
/************************************************************************/
dc1394error_t dc1394_avt_get_sis(dc1394camera_t                        *camera,
                                 const dc1394_avt_smart_feature_info_t *feature_info,
                                 dc1394switch_t                        *on_off,
                                 int16_t                               *linePos,
                                 uint32_t                              *userVal)
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>feature_info->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394error_t err;
    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) feature_info;

    uint64_t offset;
    /*
    if( smart_feature_full->Internal.Inq1.m_bFrameInfo != 0 )
    {
        offset = REG_CAMERA_AVT_TIMESTAMP;
    }
    else*/ if( smart_feature_full->Internal.Inq1.m_bTimestamp2 != 0 )
    {
        offset = REG_CAMERA_AVT_TIMESTAMP_NEW;
    }
    else if( smart_feature_full->Internal.Inq3.m_bSIS != 0 )
    {
        offset = REG_CAMERA_AVT_SIS;
    }
    else
        return DC1394_FUNCTION_NOT_SUPPORTED;

    switch(offset)
    {
        case REG_CAMERA_AVT_TIMESTAMP:
        case REG_CAMERA_AVT_TIMESTAMP_NEW:
            {
                dc1394_avt_csradv_imagestamp_t sImageStamp;
                err = dc1394_get_adv_control_registers ( camera,
                                                         offset,
                                                         (uint32_t*) &sImageStamp,
                                                         sizeof(dc1394_avt_csradv_imagestamp_t)/4 );
                DC1394_ERR_RTN(err,"Could not get AVT advanced feature TimeStamp/SIS");

                *on_off = ( sImageStamp.m_Ctrl.m.m_bOnOff != 0 ) ? DC1394_ON : DC1394_OFF;
                *linePos = (int16_t) sImageStamp.m_Ctrl.m.m_nLinePos;
                break;
            }
        case REG_CAMERA_AVT_SIS:
        default:
            {
                dc1394_avt_csradv_sis_t sSIS;
                err = dc1394_get_adv_control_registers ( camera,
                                                         offset,
                                                         (uint32_t*) &sSIS,
                                                         sizeof(dc1394_avt_csradv_sis_t)/4 );
                DC1394_ERR_RTN(err,"Could not get AVT advanced feature TimeStamp/SIS");

                *on_off = ( sSIS.m_Ctrl.m.m_bOnOff != 0 ) ? DC1394_ON : DC1394_OFF;
                *linePos = (int16_t) sSIS.m_Ctrl.m.m_nLineNo;
                if(NULL!=userVal)
                {
                    *userVal = sSIS.m_nUserVal;
                }
                break;
            }
    }
    return err;
}


/************************************************************************/
/* Set SIS                                                              */
/************************************************************************/
dc1394error_t dc1394_avt_set_sis(dc1394camera_t                         *camera,
                                 const dc1394_avt_smart_feature_info_t  *feature_info,
                                 dc1394switch_t                         on_off,
                                 int16_t                                linePos,
                                 uint32_t                               userVal)
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>feature_info->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394error_t err;
    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) feature_info;

    uint64_t offset;
    /*if( smart_feature_full->Internal.Inq1.m_bTimestamp != 0 )
    {
        offset = REG_CAMERA_AVT_TIMESTAMP;
    }
    else*/ if( smart_feature_full->Internal.Inq1.m_bTimestamp2 != 0 )
    {
        offset = REG_CAMERA_AVT_TIMESTAMP_NEW;
    }
    else if( smart_feature_full->Internal.Inq3.m_bSIS != 0 )
    {
        offset = REG_CAMERA_AVT_SIS;
    }
    else
        return DC1394_FUNCTION_NOT_SUPPORTED;

    switch(offset)
    {
        case REG_CAMERA_AVT_TIMESTAMP:
        case REG_CAMERA_AVT_TIMESTAMP_NEW:
            {
                dc1394_avt_csradv_imagestamp_t sImageStamp;
                err = dc1394_get_adv_control_registers ( camera,
                                                         offset,
                                                         (uint32_t*) &sImageStamp,
                                                         sizeof(dc1394_avt_csradv_imagestamp_t)/4 );
                DC1394_ERR_RTN(err,"Could not get AVT advanced feature TimeStamp/SIS");

                sImageStamp.m_Ctrl.m.m_bOnOff = ( on_off == DC1394_ON )? 1 : 0;
                sImageStamp.m_Ctrl.m.m_nLinePos = (uint32_t) linePos;

                err = dc1394_set_adv_control_registers ( camera,
                                                         offset,
                                                         (uint32_t*) &sImageStamp,
                                                         sizeof(dc1394_avt_csradv_imagestamp_t)/4 );
                DC1394_ERR_RTN(err,"Could not set AVT advanced feature TimeStamp/SIS");

                break;
            }
        case REG_CAMERA_AVT_SIS:
        default:
            {
                dc1394_avt_csradv_sis_t sSIS;
                err = dc1394_get_adv_control_registers ( camera,
                                                         offset,
                                                         (uint32_t*) &sSIS,
                                                         sizeof(dc1394_avt_csradv_sis_t)/4 );
                DC1394_ERR_RTN(err,"Could not get AVT advanced feature TimeStamp/SIS");

                sSIS.m_Ctrl.m.m_bOnOff = ( on_off == DC1394_ON )? 1 : 0;
                sSIS.m_Ctrl.m.m_nLineNo = (uint32_t) linePos;
                sSIS.m_nUserVal = userVal;

                err = dc1394_set_adv_control_registers ( camera,
                                                         offset,
                                                         (uint32_t*) &sSIS,
                                                         sizeof(dc1394_avt_csradv_sis_t)/4 );
                DC1394_ERR_RTN(err,"Could not set AVT advanced feature TimeStamp/SIS");

                break;
            }
    }
    return err;
}


/************************************************************************/
/* Get SIS data inquiry                                                 */
/************************************************************************/
dc1394error_t dc1394_avt_get_sis_data_inquiry(const dc1394_avt_smart_feature_info_t *feature_info,
                                              dc1394_avt_sis_data                   *sis_data_inquiry )
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>feature_info->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) feature_info;

    memset(sis_data_inquiry, 0, sizeof(dc1394_avt_sis_data));

    /*if( smart_feature_full->Internal.Inq1.m_bTimestamp != 0 )
    {
        sis_data_inquiry->m_CycleTime = 1;
    }
    else*/ if( smart_feature_full->Internal.Inq1.m_bTimestamp2 != 0 )
    {
        sis_data_inquiry->CycleTime.m.Offset = 1;
        sis_data_inquiry->CycleTime.m.Cycles = 1;
        sis_data_inquiry->CycleTime.m.Seconds = 1;
        sis_data_inquiry->FrameCounter = 1;
        sis_data_inquiry->TriggerCounter = 1;
    }
    else if( smart_feature_full->Internal.Inq3.m_bSIS != 0 )
    {
        sis_data_inquiry->CycleTime.m.Offset = 1;
        sis_data_inquiry->CycleTime.m.Cycles = 1;
        sis_data_inquiry->CycleTime.m.Seconds = 1;
        sis_data_inquiry->FrameCounter = 1;
        sis_data_inquiry->TriggerCounter = 1;
        sis_data_inquiry->AOILeft = 1;
        sis_data_inquiry->AOITop = 1;
        sis_data_inquiry->AOIWidth = 1;
        sis_data_inquiry->AOIHeight = 1;
        sis_data_inquiry->Shutter = 1;
        sis_data_inquiry->Gain = 1;
        if( smart_feature_full->Internal.Inq2.m_bOutp_1 != 0 )
            sis_data_inquiry->OutputState[0] = 1;
        if( smart_feature_full->Internal.Inq2.m_bOutp_2 != 0 )
            sis_data_inquiry->OutputState[1] = 1;
        if( smart_feature_full->Internal.Inq2.m_bOutp_3 != 0 )
            sis_data_inquiry->OutputState[2] = 1;
        if( smart_feature_full->Internal.Inq2.m_bOutp_4 != 0 )
            sis_data_inquiry->OutputState[3] = 1;
        if( smart_feature_full->Internal.Inq2.m_bInp_1 != 0 )
            sis_data_inquiry->InputState[0] = 1;
        if( smart_feature_full->Internal.Inq2.m_bInp_2 != 0 )
            sis_data_inquiry->InputState[1] = 1;
        sis_data_inquiry->SequenceIndex = 1;
        sis_data_inquiry->ColorCoding = 1;
        sis_data_inquiry->SerialNumber = 1;
        sis_data_inquiry->UserValue = 1;
    }
    else
        return DC1394_FUNCTION_NOT_SUPPORTED;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get SIS data                                                         */
/************************************************************************/
dc1394error_t dc1394_avt_get_sis_data(const dc1394_avt_smart_feature_info_t *feature_info,
                                      dc1394video_frame_t                   *frame,
                                      int16_t                               linePos,
                                      dc1394_avt_sis_data                   *sis_data)
{
    if(SIZE_AVT_SMART_FEATURE_STRUCT_V1>feature_info->Size)
    {
        return DC1394_INVALID_ARGUMENT_VALUE;
    }

    dc1394error_t err;
    dc1394_avt_smart_feature_info_full_t* smart_feature_full = (dc1394_avt_smart_feature_info_full_t*) feature_info;

    int sisline = (int) MIN( linePos, linePos + (int16_t) frame->size[1] );

    uint32_t bitsPerPixel;
    err = dc1394_get_color_coding_bit_size( frame->color_coding, &bitsPerPixel );
    DC1394_ERR_RTN(err,"Could not get color coding bit size");

    unsigned char * sisImageData = frame->image + (sisline*frame->size[0]*bitsPerPixel/8);
    /*if( smart_feature_full->Internal.Inq1.m_bTimestamp != 0 )
    {
        sis_data->m_CycleTime = ((uint32_t*) sisImageData)[0];
    }
    else*/ if( smart_feature_full->Internal.Inq1.m_bTimestamp2 != 0 )
    {
        uint32_t cycleTime_dev = ((uint32_t*) sisImageData)[0];
        sis_data->CycleTime.m_all = (cycleTime_dev&0xff)<<24 | (cycleTime_dev&0xff00)<<8 |
                                    (cycleTime_dev&0xff0000)>>8 | (cycleTime_dev&0xff000000)>>24;
        uint32_t frameCounter_dev = ((uint32_t*) sisImageData)[1];
        sis_data->FrameCounter = (frameCounter_dev&0xff)<<24 | (frameCounter_dev&0xff00)<<8 |
                                 (frameCounter_dev&0xff0000)>>8 | (frameCounter_dev&0xff000000)>>24;
        uint32_t triggerCounter_dev = ((uint32_t*) sisImageData)[2];
        sis_data->TriggerCounter = (triggerCounter_dev&0xff)<<24 | (triggerCounter_dev&0xff00)<<8 |
                                   (triggerCounter_dev&0xff0000)>>8 | (triggerCounter_dev&0xff000000)>>24;
    }
    else if( smart_feature_full->Internal.Inq3.m_bSIS != 0 )
    {
        memcpy( sis_data, sisImageData, sizeof( dc1394_avt_sis_data ) );
    }
    else
        return DC1394_FUNCTION_NOT_SUPPORTED;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get max iso size                                                     */
/************************************************************************/
dc1394error_t dc1394_avt_get_max_iso_size(dc1394camera_t *camera,
                                          uint32_t       speed_mode,
                                          dc1394switch_t *on_off,
                                          uint32_t       *max_size)
{
    dc1394error_t err;
    dc1394_avt_csradv_max_isosize_t sMaxIso;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_MAX_ISO_SIZE,
                                             (uint32_t*) &sMaxIso,
                                             sizeof(dc1394_avt_csradv_max_isosize_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Max Iso Size");

    /* read config */
    switch( speed_mode )
    {
        case 2:
            *on_off = (sMaxIso.m_S400.m.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;
            *max_size = sMaxIso.m_S400.m.m_nMaxSize;
            break;
        case 3:
            *on_off = (sMaxIso.m_S800.m.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;
            *max_size = sMaxIso.m_S800.m.m_nMaxSize;
            break;
        case 4:
            *on_off = (sMaxIso.m_S1600.m.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;
            *max_size = sMaxIso.m_S1600.m.m_nMaxSize;
            break;
        case 5:
            *on_off = (sMaxIso.m_S3200.m.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;
            *max_size = sMaxIso.m_S3200.m.m_nMaxSize;
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set max iso size                                                     */
/************************************************************************/
dc1394error_t dc1394_avt_set_max_iso_size(dc1394camera_t *camera,
                                          uint32_t       speed_mode,
                                          dc1394switch_t on_off,
                                          dc1394switch_t set_to_max,
                                          uint32_t       max_size)
{
    dc1394error_t err;
    dc1394_avt_csradv_max_isosize_t sMaxIso;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_MAX_ISO_SIZE,
                                             (uint32_t*) &sMaxIso,
                                             sizeof(dc1394_avt_csradv_max_isosize_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Max Iso Size");

    /* set config */
    switch( speed_mode )
    {
        case 2:
            sMaxIso.m_S400.m.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;
            sMaxIso.m_S400.m.m_nMaxSize = max_size;
            sMaxIso.m_S400.m.m_bSet2Max = (set_to_max==DC1394_ON)? 1 : 0;
            break;
        case 3:
            sMaxIso.m_S800.m.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;
            sMaxIso.m_S800.m.m_nMaxSize = max_size;
            sMaxIso.m_S800.m.m_bSet2Max = (set_to_max==DC1394_ON)? 1 : 0;
            break;
        case 4:
            sMaxIso.m_S1600.m.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;
            sMaxIso.m_S1600.m.m_nMaxSize = max_size;
            sMaxIso.m_S1600.m.m_bSet2Max = (set_to_max==DC1394_ON)? 1 : 0;
            break;
        case 5:
            sMaxIso.m_S3200.m.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;
            sMaxIso.m_S3200.m.m_nMaxSize = max_size;
            sMaxIso.m_S3200.m.m_bSet2Max = (set_to_max==DC1394_ON)? 1 : 0;
            break;
        default:
            return DC1394_INVALID_ARGUMENT_VALUE;
    }

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_MAX_ISO_SIZE,
                                             (uint32_t*) &sMaxIso,
                                             sizeof(dc1394_avt_csradv_max_isosize_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Max Iso Size");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Parameter Update Timing                                          */
/************************************************************************/
dc1394error_t dc1394_avt_get_param_upd_timing(dc1394camera_t *camera,
                                              uint32_t       *update_timing_mode)
{
    dc1394error_t err;
    dc1394_avt_csradv_paramupd_timing_t sParamUpdTiming;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_PARAMUPD_MODE,
                                             (uint32_t*) &sParamUpdTiming,
                                             sizeof(dc1394_avt_csradv_paramupd_timing_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Parameter Update Timing");

    /* read config */
    *update_timing_mode = sParamUpdTiming.m.m_nFpgaUpdMode;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Parameter Update Timing                                          */
/************************************************************************/
dc1394error_t dc1394_avt_set_param_upd_timing(dc1394camera_t *camera,
                                              uint32_t       update_timing_mode)
{
    dc1394error_t err;
    dc1394_avt_csradv_paramupd_timing_t sParamUpdTiming;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_PARAMUPD_MODE,
                                             (uint32_t*) &sParamUpdTiming,
                                             sizeof(dc1394_avt_csradv_paramupd_timing_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Parameter Update Timing");

    /* set config */
    sParamUpdTiming.m.m_nFpgaUpdMode = update_timing_mode;

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_PARAMUPD_MODE,
                                             (uint32_t*) &sParamUpdTiming,
                                             sizeof(dc1394_avt_csradv_paramupd_timing_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Parameter Update Timing");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Low Smear                                                        */
/************************************************************************/
dc1394error_t dc1394_avt_get_low_smear(dc1394camera_t *camera,
                                       dc1394switch_t *on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_low_smear_t sLowSmear;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LOW_SMEAR,
                                             (uint32_t*) &sLowSmear,
                                             sizeof(dc1394_avt_csradv_low_smear_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Low Smear");

    /* read config */
    *on_off = (sLowSmear.m_Ctrl.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Low Smear                                                        */
/************************************************************************/
dc1394error_t dc1394_avt_set_low_smear(dc1394camera_t *camera,
                                       dc1394switch_t on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_low_smear_t sLowSmear;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LOW_SMEAR,
                                             (uint32_t*) &sLowSmear,
                                             sizeof(dc1394_avt_csradv_low_smear_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Low Smear");

    /* set config */
    sLowSmear.m_Ctrl.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LOW_SMEAR,
                                             (uint32_t*) &sLowSmear,
                                             sizeof(dc1394_avt_csradv_low_smear_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Low Smear");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Low Noise Binning                                                */
/************************************************************************/
dc1394error_t dc1394_avt_get_low_noise_binning(dc1394camera_t *camera,
                                               dc1394switch_t *on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_global_res_rel_shutter_t sLowNoiseBinning; // global res rel shutter has same reg layout

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LOW_NOISE_BINNING,
                                             (uint32_t*) &sLowNoiseBinning,
                                             sizeof(dc1394_avt_csradv_global_res_rel_shutter_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Low Noise Binning");

    /* read config */
    *on_off = (sLowNoiseBinning.m_Ctrl.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Low Noise Binning                                                */
/************************************************************************/
dc1394error_t dc1394_avt_set_low_noise_binning(dc1394camera_t *camera,
                                               dc1394switch_t on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_global_res_rel_shutter_t sLowNoiseBinning; // global res rel shutter has same reg layout

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LOW_NOISE_BINNING,
                                             (uint32_t*) &sLowNoiseBinning,
                                             sizeof(dc1394_avt_csradv_global_res_rel_shutter_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Low Noise Binning");

    /* set config */
    sLowNoiseBinning.m_Ctrl.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_LOW_NOISE_BINNING,
                                             (uint32_t*) &sLowNoiseBinning,
                                             sizeof(dc1394_avt_csradv_global_res_rel_shutter_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Low Noise Binning");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Global Reset Release Shutter                                     */
/************************************************************************/
dc1394error_t dc1394_avt_get_global_res_rel_shutter(dc1394camera_t *camera,
                                                    dc1394switch_t *on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_global_res_rel_shutter_t sGlobalRRShutter;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_GLOBAL_RESET_RELEASE_SHUTTER,
                                             (uint32_t*) &sGlobalRRShutter,
                                             sizeof(dc1394_avt_csradv_global_res_rel_shutter_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Global Reset Release Shutter");

    /* read config */
    *on_off = (sGlobalRRShutter.m_Ctrl.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Global Reset Release Shutter                                     */
/************************************************************************/
dc1394error_t dc1394_avt_set_global_res_rel_shutter(dc1394camera_t *camera,
                                                    dc1394switch_t on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_global_res_rel_shutter_t sGlobalRRShutter;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_GLOBAL_RESET_RELEASE_SHUTTER,
                                             (uint32_t*) &sGlobalRRShutter,
                                             sizeof(dc1394_avt_csradv_global_res_rel_shutter_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Global Reset Release Shutter");

    /* set config */
    sGlobalRRShutter.m_Ctrl.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_GLOBAL_RESET_RELEASE_SHUTTER,
                                             (uint32_t*) &sGlobalRRShutter,
                                             sizeof(dc1394_avt_csradv_global_res_rel_shutter_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Global Reset Release Shutter");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get User Profile                                                     */
/************************************************************************/
dc1394error_t dc1394_avt_get_user_profile(dc1394camera_t *camera,
                                          uint32_t       *profile_id,
                                          dc1394bool_t   *error,
                                          uint32_t       *err_code )
{
    dc1394error_t err;
    dc1394_avt_csradv_userprofile_t sProfile;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_USER_PROFILE,
                                             (uint32_t*) &sProfile,
                                             sizeof(dc1394_avt_csradv_userprofile_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature User Profile");

    /* read config */
    *profile_id = sProfile.m.m_nProfileID;
    *error = (sProfile.m.m_bError!=0)? DC1394_ON : DC1394_OFF;
    *err_code = sProfile.m.m_nErrCode;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set User Profile                                                     */
/************************************************************************/
dc1394error_t dc1394_avt_set_user_profile(dc1394camera_t *camera,
                                          uint32_t       profile_id,
                                          dc1394switch_t load_profile,
                                          dc1394switch_t save_profile,
                                          dc1394switch_t set_default )
{
    dc1394error_t err;
    dc1394_avt_csradv_userprofile_t sProfile;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_USER_PROFILE,
                                             (uint32_t*) &sProfile,
                                             sizeof(dc1394_avt_csradv_userprofile_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature User Profile");

    /* set config */
    sProfile.m.m_nProfileID = profile_id;
    sProfile.m.m_bSetAsDef = (set_default==DC1394_ON)? 1 : 0;
    sProfile.m.m_bMemLoad = (load_profile==DC1394_ON)? 1 : 0;
    sProfile.m.m_bMemSave = (save_profile==DC1394_ON)? 1 : 0;

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_USER_PROFILE,
                                             (uint32_t*) &sProfile,
                                             sizeof(dc1394_avt_csradv_userprofile_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature User Profile");

    /* poll camera until busy flag is cleared */
    do
    {
        usleep( 50000 );
        err = dc1394_get_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_USER_PROFILE,
                                                 (uint32_t*) &sProfile,
                                                 sizeof(dc1394_avt_csradv_userprofile_t)/4 );
        DC1394_ERR_RTN(err,"Could not get AVT advanced feature User Profile");
    }while( sProfile.m.m_bBusy!= 0 );

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get LED                                                              */
/************************************************************************/
dc1394error_t dc1394_avt_get_led(dc1394camera_t *camera,
                                 dc1394switch_t *on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_swfeature_t sSWFeature;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_SWFEATURE,
                                             (uint32_t*) &sSWFeature,
                                             sizeof(dc1394_avt_csradv_swfeature_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature SW Feature");

    /* read config */
    *on_off = (sSWFeature.m.m_bBlankLED!=0)? DC1394_OFF : DC1394_ON; // value is inverted! enable led <> disable led

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set LED                                                              */
/************************************************************************/
dc1394error_t dc1394_avt_set_led(dc1394camera_t *camera,
                                 dc1394switch_t on_off)
{
    dc1394error_t err;
    dc1394_avt_csradv_swfeature_t sSWFeature;

    /* get register */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_SWFEATURE,
                                             (uint32_t*) &sSWFeature,
                                             sizeof(dc1394_avt_csradv_swfeature_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature SW Feature");

    /* set config */
    sSWFeature.m.m_bBlankLED = (on_off==DC1394_ON)? 0 : 1; // value is inverted! enable led <> disable led

    /* write back register */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_SWFEATURE,
                                             (uint32_t*) &sSWFeature,
                                             sizeof(dc1394_avt_csradv_swfeature_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature SW Feature");

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Defect Pixel Correction Info                                     */
/************************************************************************/
dc1394error_t dc1394_avt_get_dpc_info(dc1394camera_t *camera,
                                      uint32_t       *MinThreshold,
                                      uint32_t       *MaxThreshold,
                                      uint32_t       *MaxSize)
{
    dc1394error_t err;
    dc1394_avt_csradv_defect_pixel_correction_t sDPC;

    /* get registers */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL,
                                             (uint32_t*) &sDPC,
                                             sizeof(dc1394_avt_csradv_defect_pixel_correction_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Defect Pixel Correction");

    *MinThreshold = sDPC.m_Info.m_nMinThreshold;
    *MaxThreshold = sDPC.m_Info.m_nMaxThreshold;
    *MaxSize = sDPC.m_Info.m_nMaxSize;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Get Defect Pixel Correction                                          */
/************************************************************************/
dc1394error_t dc1394_avt_get_dpc(dc1394camera_t *camera,
                                 dc1394switch_t *on_off,
                                 uint32_t       *threshold,
                                 uint32_t       *mean_value,
                                 uint32_t       *data_size)
{
    dc1394error_t err;
    dc1394_avt_csradv_defect_pixel_correction_t sDPC;

    /* get registers */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL,
                                             (uint32_t*) &sDPC,
                                             sizeof(dc1394_avt_csradv_defect_pixel_correction_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Defect Pixel Correction");

    /* read status */
    *on_off = (sDPC.m_Ctrl.m_bOnOff!=0)? DC1394_ON : DC1394_OFF;
    *threshold = sDPC.m_Ctrl.m_nThreshold;
    *mean_value = sDPC.m_Ctrl.m_nMean;
    *data_size = sDPC.m_Mem.m_nDPDataSize;

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Set Defect Pixel Correction                                          */
/************************************************************************/
dc1394error_t dc1394_avt_set_dpc(dc1394camera_t *camera,
                                 dc1394switch_t on_off,
                                 dc1394switch_t build_data,
                                 dc1394switch_t zero_data,
                                 dc1394switch_t mem_save,
                                 dc1394switch_t mem_load,
                                 uint32_t       threshold)
{
    dc1394error_t err;
    dc1394_avt_csradv_defect_pixel_correction_t sDPC;

    /* get registers */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL,
                                             (uint32_t*) &sDPC,
                                             sizeof(dc1394_avt_csradv_defect_pixel_correction_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Defect Pixel Correction");

    /* read status */
    sDPC.m_Ctrl.m_bOnOff = (on_off==DC1394_ON)? 1 : 0;
    sDPC.m_Ctrl.m_bZeroDPData = (zero_data==DC1394_ON)? 1 : 0;
    sDPC.m_Ctrl.m_bMemLoad = (mem_load==DC1394_ON)? 1 : 0;
    sDPC.m_Ctrl.m_bMemSave = (mem_save==DC1394_ON)? 1 : 0;
    sDPC.m_Ctrl.m_bBuildDPData = (build_data==DC1394_ON)? 1 : 0;
    sDPC.m_Ctrl.m_nThreshold = threshold;

    /* write back registers */
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL,
                                             (uint32_t*) &sDPC,
                                             sizeof(dc1394_avt_csradv_defect_pixel_correction_t)/4 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Defect Pixel Correction");

    /* Poll register until busy is cleared */
    do
    {
        usleep(50000);
        err = dc1394_get_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_DPC_CONTROL,
                                                 (uint32_t*) &sDPC,
                                                 1 );
        DC1394_ERR_RTN(err,"Could not get AVT advanced feature Defect Pixel Correction");
    } while( sDPC.m_Ctrl.m_bBusy != 0 );

    return DC1394_SUCCESS;
}


/************************************************************************/
/* Read Defect Pixel Correction Data                                    */
/************************************************************************/
dc1394error_t dc1394_avt_read_dpc_data(dc1394camera_t                *camera,
                                       dc1394_avt_dpc_pixel_position *DestBuffer,
                                       uint32_t                      *PixelCount,
                                       uint32_t                      BufferSize )
{
    dc1394error_t                               err;
    dc1394_avt_csradv_defect_pixel_correction_t sDPC;

    /* get registers */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL,
                                             (uint32_t*) &sDPC.m_Ctrl,
                                             sizeof(dc1394_avt_csradv_defect_pixel_correction_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Defect Pixel Correction");

    /* Get amount of data to load */
    uint32_t nDpcDataSize = sDPC.m_Mem.m_nDPDataSize;
    /* DPDataSize indicates size in bytes, so it should be a multiple of 4 */
    nDpcDataSize -= nDpcDataSize%4;

    /* make sure not to write beyond the user buffer */
    nDpcDataSize = MIN( nDpcDataSize, BufferSize*4 );

    if (nDpcDataSize != 0)
    {
        // Enable read access.
        sDPC.m_Mem.m_bEnaMemRD = 1;
        sDPC.m_Mem.m_bEnaMemWR = 0;
        sDPC.m_Mem.m_nAddrOffset = 0;
        err = dc1394_set_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_DPC_CONTROL+4,
                                                 (uint32_t*) &sDPC.m_Mem,
                                                 1 );
        DC1394_ERR_RTN(err,"Could not set AVT advanced feature Defect Pixel Correction - Mem Control");

        err = dc1394_avt_read_gpdata(camera, (unsigned char *) &DestBuffer[0].m_all, nDpcDataSize);
        DC1394_ERR_RTN(err,"Could not download Data via AVT GPData buffer");

        // Disable read access.
        sDPC.m_Mem.m_bEnaMemRD = 0;
        sDPC.m_Mem.m_bEnaMemWR = 0;
        sDPC.m_Mem.m_nAddrOffset = 0;
        err = dc1394_set_adv_control_registers ( camera,
                                                 REG_CAMERA_AVT_DPC_CONTROL+4,
                                                 (uint32_t*) &sDPC.m_Mem,
                                                 1 );
        DC1394_ERR_RTN(err,"Could not set AVT advanced feature Defect Pixel Correction - Mem Control");
    }
    *PixelCount = nDpcDataSize/4;
    return DC1394_SUCCESS;
}


/************************************************************************/
/* Write Defect Pixel Correction Data                                   */
/************************************************************************/
dc1394error_t dc1394_avt_write_dpc_data(dc1394camera_t                *camera,
                                        dc1394_avt_dpc_pixel_position *SourceBuffer,
                                        uint32_t                      PixelCount)
{
    dc1394error_t                               err;
    dc1394_avt_csradv_defect_pixel_correction_t sDPC;

    /* get registers */
    err = dc1394_get_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL,
                                             (uint32_t*) &sDPC.m_Ctrl,
                                             sizeof(dc1394_avt_csradv_defect_pixel_correction_t)/4 );
    DC1394_ERR_RTN(err,"Could not get AVT advanced feature Defect Pixel Correction");


    // Get amount of data to store and size of GP data buffer.
    uint32_t nDpcDataSize = PixelCount * 4;

    // Enable write access.
    sDPC.m_Mem.m_bEnaMemRD = 0;
    sDPC.m_Mem.m_bEnaMemWR = 1;
    sDPC.m_Mem.m_nAddrOffset = 0;
    err = dc1394_set_adv_control_registers ( camera,
                                             REG_CAMERA_AVT_DPC_CONTROL+4,
                                             (uint32_t*) &sDPC.m_Mem,
                                             1 );
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Defect Pixel Correction - Mem Control");

    err = dc1394_avt_write_gpdata (camera, (unsigned char *) &SourceBuffer[0].m_all, nDpcDataSize);
    DC1394_ERR_RTN(err,"Could not upload data via AVT GPData buffer");

    // Disable write access.
    sDPC.m_Mem.m_bEnaMemRD = 0;
    sDPC.m_Mem.m_bEnaMemWR = 0;
    sDPC.m_Mem.m_nAddrOffset = 0;
    DC1394_ERR_RTN(err,"Could not set AVT advanced feature Defect Pixel Correction - Mem Control");
    return DC1394_SUCCESS;
}
