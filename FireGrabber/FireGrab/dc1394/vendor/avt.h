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

#ifndef __DC1394_VENDOR_AVT_H__
#define __DC1394_VENDOR_AVT_H__

#include <stdint.h>
#include <dc1394/types.h>
#include <dc1394/log.h>


/*! \file dc1394/vendor/avt.h
    \brief Definition of AVT-specific features

    More details soon
*/

#define DC1394_AVT_VENDOR_ID 0xa47

/* defines used for dc1394_avt_get_io, dc1394_avt_set_io, dc1394_avt_get_io_pwmout... */
#define DC1394_AVT_IO_INP_CTRL1                     0x300U
#define DC1394_AVT_IO_INP_CTRL2                     0x304U
#define DC1394_AVT_IO_INP_CTRL3                     0x308U
#define DC1394_AVT_IO_INP_CTRL4                     0x30CU
#define DC1394_AVT_IO_INP_CTRL5                     0x310U
#define DC1394_AVT_IO_INP_CTRL6                     0x314U
#define DC1394_AVT_IO_INP_CTRL7                     0x318U
#define DC1394_AVT_IO_INP_CTRL8                     0x31CU
#define DC1394_AVT_IO_OUTP_CTRL1                    0x320U
#define DC1394_AVT_IO_OUTP_CTRL2                    0x324U
#define DC1394_AVT_IO_OUTP_CTRL3                    0x328U
#define DC1394_AVT_IO_OUTP_CTRL4                    0x32CU
#define DC1394_AVT_IO_OUTP_CTRL5                    0x330U
#define DC1394_AVT_IO_OUTP_CTRL6                    0x334U
#define DC1394_AVT_IO_OUTP_CTRL7                    0x338U
#define DC1394_AVT_IO_OUTP_CTRL8                    0x33CU
#define DC1394_AVT_IO_OUTP_PWM_CTRL1                0x800U
#define DC1394_AVT_IO_OUTP_PWM_CTRL2                0x804U
#define DC1394_AVT_IO_OUTP_PWM_CTRL3                0x808U
#define DC1394_AVT_IO_OUTP_PWM_CTRL4                0x80CU
#define DC1394_AVT_IO_OUTP_PWM_CTRL5                0x810U
#define DC1394_AVT_IO_OUTP_PWM_CTRL6                0x814U
#define DC1394_AVT_IO_OUTP_PWM_CTRL7                0x818U
#define DC1394_AVT_IO_OUTP_PWM_CTRL8                0x81CU
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL1            0x840U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL2            0x850U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL3            0x860U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL4            0x870U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL5            0x880U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL6            0x890U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL7            0x8A0U
#define DC1394_AVT_IO_INP_DEBOUNCE_CTRL8            0x8B0U


/**
 * Enumeration of AVT camera family IDs. Value is stored in bits 24-31 of the cameras serial number.
 */
typedef enum {
    DC1394_AVT_FAMILY_DOLPHIN_OLD= 0,
    DC1394_AVT_FAMILY_MARLIN_OLD= 1,
    DC1394_AVT_FAMILY_ZK_1_5_8_OLD= 2,
    DC1394_AVT_FAMILY_OSCAR_OLD= 3,
    DC1394_AVT_FAMILY_MARLIN_2_OLD= 4,
    DC1394_AVT_FAMILY_GUPPY_OLD= 5,
    DC1394_AVT_FAMILY_PIKE_OLD= 6,
    DC1394_AVT_FAMILY_DOLPHIN= 10,
    DC1394_AVT_FAMILY_MARLIN= 11,
    DC1394_AVT_FAMILY_ZK_1_5_8= 12,
    DC1394_AVT_FAMILY_OSCAR= 13,
    DC1394_AVT_FAMILY_MARLIN_2= 14,
    DC1394_AVT_FAMILY_GUPPY= 15,
    DC1394_AVT_FAMILY_PIKE= 16,
    DC1394_AVT_FAMILY_STINGRAY= 17,
    DC1394_AVT_FAMILY_GUPPYPRO= 18
} dc1394_avt_family_t;

/**
 * Enumeration of AVT camera IDs, as returned by dc1394_avt_get_version.
 */
typedef enum {
    /* Dolphin Family */
    DC1394_AVT_CAMERA_ID_DF145B= 1,
    DC1394_AVT_CAMERA_ID_DF145C= 2,
    DC1394_AVT_CAMERA_ID_DF201B= 3,
    DC1394_AVT_CAMERA_ID_DF201C= 4,
    DC1394_AVT_CAMERA_ID_DF145B_1= 5,
    DC1394_AVT_CAMERA_ID_DF145C_1= 6,
    DC1394_AVT_CAMERA_ID_DF201B_1= 7,
    DC1394_AVT_CAMERA_ID_DF201C_1= 8,

    /* Marlin I Family */
    DC1394_AVT_CAMERA_ID_MF033B= 9,
    DC1394_AVT_CAMERA_ID_MF033C= 10,
    DC1394_AVT_CAMERA_ID_MF046B= 11,
    DC1394_AVT_CAMERA_ID_MF046C= 12,
    DC1394_AVT_CAMERA_ID_MF080B= 13,
    DC1394_AVT_CAMERA_ID_MF080C= 14,
    DC1394_AVT_CAMERA_ID_MF145B= 15,
    DC1394_AVT_CAMERA_ID_MF145C= 16,
    DC1394_AVT_CAMERA_ID_MF131B= 17,
    DC1394_AVT_CAMERA_ID_MF131C= 18,
    DC1394_AVT_CAMERA_ID_MF145B3= 19,
    DC1394_AVT_CAMERA_ID_MF145C3= 20,

    /* Marlin II Family */
    DC1394_AVT_CAMERA_ID_M2F033B= 21,
    DC1394_AVT_CAMERA_ID_M2F033C= 22,
    DC1394_AVT_CAMERA_ID_M2F046B= 23,
    DC1394_AVT_CAMERA_ID_M2F046C= 24,
    DC1394_AVT_CAMERA_ID_M2F080B= 25,
    DC1394_AVT_CAMERA_ID_M2F080C= 26,
    DC1394_AVT_CAMERA_ID_M2F145B= 27,
    DC1394_AVT_CAMERA_ID_M2F145C= 28,
    DC1394_AVT_CAMERA_ID_M2F145B3= 31,
    DC1394_AVT_CAMERA_ID_M2F145C3= 32,

    /* Oscar Family */
    DC1394_AVT_CAMERA_ID_OF320B= 37,
    DC1394_AVT_CAMERA_ID_OF320C= 38,
    DC1394_AVT_CAMERA_ID_OF510B= 39,
    DC1394_AVT_CAMERA_ID_OF510C= 40,
    DC1394_AVT_CAMERA_ID_OF810B= 41,
    DC1394_AVT_CAMERA_ID_OF810C= 42,

    /* Marlin II Family */
    DC1394_AVT_CAMERA_ID_M2F080B_30FPS= 43,
    DC1394_AVT_CAMERA_ID_M2F080C_30FPS= 44,
    DC1394_AVT_CAMERA_ID_M2F145B4= 45,
    DC1394_AVT_CAMERA_ID_M2F145C4= 46,
    DC1394_AVT_CAMERA_ID_M2F201B= 47,
    DC1394_AVT_CAMERA_ID_M2F201C= 48,
    DC1394_AVT_CAMERA_ID_M2F146B= 49,
    DC1394_AVT_CAMERA_ID_M2F146C= 50,

    /* Pike Family */
    DC1394_AVT_CAMERA_ID_PF032B= 101,
    DC1394_AVT_CAMERA_ID_PF032C= 102,
    DC1394_AVT_CAMERA_ID_PF100B= 103,
    DC1394_AVT_CAMERA_ID_PF100C= 104,
    DC1394_AVT_CAMERA_ID_PF145B= 105,
    DC1394_AVT_CAMERA_ID_PF145C= 106,
    DC1394_AVT_CAMERA_ID_PF201B= 107,
    DC1394_AVT_CAMERA_ID_PF201C= 108,
    DC1394_AVT_CAMERA_ID_PF411B= 109,
    DC1394_AVT_CAMERA_ID_PF411C= 110,
    DC1394_AVT_CAMERA_ID_PF421B= 111,
    DC1394_AVT_CAMERA_ID_PF421C= 112,
    DC1394_AVT_CAMERA_ID_PF210BM= 113,
    DC1394_AVT_CAMERA_ID_PF210CM= 114,
    DC1394_AVT_CAMERA_ID_PF145B_15= 115,
    DC1394_AVT_CAMERA_ID_PF145C_15= 116,
    DC1394_AVT_CAMERA_ID_PF505B= 117,
    DC1394_AVT_CAMERA_ID_PF505C= 118,
    DC1394_AVT_CAMERA_ID_PF1100B= 119,
    DC1394_AVT_CAMERA_ID_PF1100C= 120,
    DC1394_AVT_CAMERA_ID_PF422B= 121,
    DC1394_AVT_CAMERA_ID_PF422C= 122,
    DC1394_AVT_CAMERA_ID_PF1102B= 123,
    DC1394_AVT_CAMERA_ID_PF1102C= 124,
    DC1394_AVT_CAMERA_ID_PF1600B= 125,
    DC1394_AVT_CAMERA_ID_PF1600C= 126,

    /* Guppy Family */
    DC1394_AVT_CAMERA_ID_GF033B= 201,
    DC1394_AVT_CAMERA_ID_GF033C= 202,
    DC1394_AVT_CAMERA_ID_GF036B= 203,
    DC1394_AVT_CAMERA_ID_GF036C= 204,
    DC1394_AVT_CAMERA_ID_GF046B= 205,
    DC1394_AVT_CAMERA_ID_GF046C= 206,
    DC1394_AVT_CAMERA_ID_GF080B= 207,
    DC1394_AVT_CAMERA_ID_GF080C= 208,
    DC1394_AVT_CAMERA_ID_GF146B= 209,
    DC1394_AVT_CAMERA_ID_GF146C= 210,
    DC1394_AVT_CAMERA_ID_GF033B_GP1= 211,
    DC1394_AVT_CAMERA_ID_GF033B_BL= 213,
    DC1394_AVT_CAMERA_ID_GF033C_BL= 214,
    DC1394_AVT_CAMERA_ID_GF025B= 215,
    DC1394_AVT_CAMERA_ID_GF025C= 216,
    DC1394_AVT_CAMERA_ID_GF029B= 217,
    DC1394_AVT_CAMERA_ID_GF029C= 218,
    DC1394_AVT_CAMERA_ID_GF038B= 219,
    DC1394_AVT_CAMERA_ID_GF038C= 220,
    DC1394_AVT_CAMERA_ID_GF038B_NIR= 221,
    DC1394_AVT_CAMERA_ID_GF038C_NIR= 222,
    DC1394_AVT_CAMERA_ID_GF044B_NIR= 223,
    DC1394_AVT_CAMERA_ID_GF044C_NIR= 224,
    DC1394_AVT_CAMERA_ID_GF080B_BL= 225,
    DC1394_AVT_CAMERA_ID_GF080C_BL= 226,
    DC1394_AVT_CAMERA_ID_GF044B= 227,
    DC1394_AVT_CAMERA_ID_GF044C= 228,
    DC1394_AVT_CAMERA_ID_OC030B_BL= 229,
    DC1394_AVT_CAMERA_ID_OC030C_BL= 230,
    DC1394_AVT_CAMERA_ID_GP145B_BL= 231,
    DC1394_AVT_CAMERA_ID_GP145C_BL= 232,
    DC1394_AVT_CAMERA_ID_GF503B= 233,
    DC1394_AVT_CAMERA_ID_GF503C= 234,

    /* Stingray Family */
    DC1394_AVT_CAMERA_ID_SF033B= 401,
    DC1394_AVT_CAMERA_ID_SF033C= 402,
    DC1394_AVT_CAMERA_ID_SF03XB= 403,
    DC1394_AVT_CAMERA_ID_SF03XC= 404,
    DC1394_AVT_CAMERA_ID_SF046B= 405,
    DC1394_AVT_CAMERA_ID_SF046C= 406,
    DC1394_AVT_CAMERA_ID_SF080B= 407,
    DC1394_AVT_CAMERA_ID_SF080C= 408,
    DC1394_AVT_CAMERA_ID_SF125B= 409,
    DC1394_AVT_CAMERA_ID_SF125C= 410,
    DC1394_AVT_CAMERA_ID_SF131B= 411,
    DC1394_AVT_CAMERA_ID_SF131C= 412,
    DC1394_AVT_CAMERA_ID_SF145B= 413,
    DC1394_AVT_CAMERA_ID_SF145C= 414,
    DC1394_AVT_CAMERA_ID_SF146B= 415,
    DC1394_AVT_CAMERA_ID_SF146C= 416,
    DC1394_AVT_CAMERA_ID_SF201B= 417,
    DC1394_AVT_CAMERA_ID_SF201C= 418,
    DC1394_AVT_CAMERA_ID_SF505B= 419,
    DC1394_AVT_CAMERA_ID_SF505C= 420,
    DC1394_AVT_CAMERA_ID_SF145B_30= 421,
    DC1394_AVT_CAMERA_ID_SF145C_30= 422,
    DC1394_AVT_CAMERA_ID_SF504B= 423,
    DC1394_AVT_CAMERA_ID_SF504C= 424,
    DC1394_AVT_CAMERA_ID_SF145_25_CL= 425,
    DC1394_AVT_CAMERA_ID_SF146_ICM1= 427,
    DC1394_AVT_CAMERA_ID_SF146_ICC1= 428,
    DC1394_AVT_CAMERA_ID_SF201B_CSO= 429,
    DC1394_AVT_CAMERA_ID_SF201C_CSO= 430,
    DC1394_AVT_CAMERA_ID_SF504B_AB= 431,
    DC1394_AVT_CAMERA_ID_SF504C_AB= 432,
    DC1394_AVT_CAMERA_ID_SF504B_FF= 433,
    DC1394_AVT_CAMERA_ID_SF504C_FF= 434,

    /* Guppy Pro Family */
    DC1394_AVT_CAMERA_ID_GPF031B= 501,
    DC1394_AVT_CAMERA_ID_GPF031C= 502,
    DC1394_AVT_CAMERA_ID_GPF033B= 503,
    DC1394_AVT_CAMERA_ID_GPF033C= 504,
    DC1394_AVT_CAMERA_ID_GPF046B= 507,
    DC1394_AVT_CAMERA_ID_GPF046C= 508,
    DC1394_AVT_CAMERA_ID_GPF080B= 509,
    DC1394_AVT_CAMERA_ID_GPF080C= 510,
    DC1394_AVT_CAMERA_ID_GPF125B= 511,
    DC1394_AVT_CAMERA_ID_GPF125C= 512,
    DC1394_AVT_CAMERA_ID_GPF201B= 517,
    DC1394_AVT_CAMERA_ID_GPF201C= 518,
    DC1394_AVT_CAMERA_ID_GPF503B= 519,
    DC1394_AVT_CAMERA_ID_GPF503C= 520,
    DC1394_AVT_CAMERA_ID_GPF504B= 521,
    DC1394_AVT_CAMERA_ID_GPF504C= 522
} dc1394_avt_camera_id_t;

/**
 * Inquiry of available advanced features - This struct is intended to be used as a replacement for dc1394_avt_adv_feature_info_t.
 * For initialization see dc1394_avt_get_smart_feature_inquiry().
 */
typedef struct __dc1394_avt_smart_feature_info_struct
{
    uint32_t        Size;

    /* Camera capabilities provided as user information. */
    dc1394bool_t    MaxResolution;
    dc1394bool_t    TimeBase;
    dc1394bool_t    ExtdShutter;
    dc1394bool_t    TestImage;
    dc1394bool_t    FrameCounter;
    dc1394bool_t    Sequences;
    dc1394bool_t    VersionInfo;
    dc1394bool_t    Lookup_Tables;
    dc1394bool_t    Shading;
    dc1394bool_t    DeferredTransport;
    dc1394bool_t    HDR_Mode;
    dc1394bool_t    DSNU;
    dc1394bool_t    BlemishCorrection;
    dc1394bool_t    TriggerDelay;
    dc1394bool_t    MirrorImage;
    dc1394bool_t    SoftReset;
    dc1394bool_t    HSNR;
    dc1394bool_t    ColorCorrection;
    dc1394bool_t    ColorAvg;
    dc1394bool_t    SIS;
    dc1394bool_t    UserProfiles;
    dc1394bool_t    TriggerCounter;
    dc1394bool_t    ParamListBuffer;
    dc1394bool_t    GP_Buffer;

    dc1394bool_t    Input_1;
    dc1394bool_t    Input_2;
    dc1394bool_t    Input_3;
    dc1394bool_t    Input_4;
    dc1394bool_t    Input_5;
    dc1394bool_t    Input_6;
    dc1394bool_t    Input_7;
    dc1394bool_t    Input_8;
    dc1394bool_t    Output_1;
    dc1394bool_t    Output_2;
    dc1394bool_t    Output_3;
    dc1394bool_t    Output_4;
    dc1394bool_t    Output_5;
    dc1394bool_t    Output_6;
    dc1394bool_t    Output_7;
    dc1394bool_t    Output_8;
    dc1394bool_t    IntEnaDelay;
    dc1394bool_t    IncDecoder;
    dc1394bool_t    Output_1_PWM;
    dc1394bool_t    Output_2_PWM;
    dc1394bool_t    Output_3_PWM;
    dc1394bool_t    Output_4_PWM;
    dc1394bool_t    Output_5_PWM;
    dc1394bool_t    Output_6_PWM;
    dc1394bool_t    Output_7_PWM;
    dc1394bool_t    Output_8_PWM;

    dc1394bool_t    CameraStatus;
    dc1394bool_t    MaxIsoSize_S400;
    dc1394bool_t    MaxIsoSize_S800;
    dc1394bool_t    ParamUpdTiming;
    dc1394bool_t    F7ModeMapping;
    dc1394bool_t    AutoShutter;
    dc1394bool_t    AutoGain;
    dc1394bool_t    AutoFunctionAOI;
    dc1394bool_t    SequenceStep;
    dc1394bool_t    LowNoiseBinning;
    dc1394bool_t    GlobalResetReleaseShutter;
    dc1394bool_t    DefectPixelCorrection;
    dc1394bool_t    SWFeatureControl;
    dc1394bool_t    LedBlanking;
    dc1394bool_t    InputDebounce_1;
    dc1394bool_t    InputDebounce_2;
    dc1394bool_t    InputDebounce_3;
    dc1394bool_t    InputDebounce_4;
    dc1394bool_t    InputDebounce_5;
    dc1394bool_t    InputDebounce_6;
    dc1394bool_t    InputDebounce_7;
    dc1394bool_t    InputDebounce_8;

    dc1394bool_t    HDRPike;
    dc1394bool_t    ChannelAdjustGain;
    dc1394bool_t    LowSmear;
    dc1394bool_t    AdvWhiteBal;
    dc1394bool_t    ChannelAdjustOffset;

    dc1394bool_t    reserved[64];

    /* Internal information used by libdc functions. */
    struct __tagCsrAdvInquiryHidden
    {
        uint32_t Inq1;
        uint32_t Inq2;
        uint32_t Inq3;
        uint32_t Inq4;
    } Internal;
} dc1394_avt_smart_feature_info_t;

/**
 * Inquiry of available advanced features - This struct is provided for backwards compatibility. New code
 * should use dc1394_avt_adv_function_inquiry_t instead.
 */
typedef struct __dc1394_avt_adv_feature_info_struct
{
    uint32_t feature_id;
    dc1394bool_t features_requested;
    /************************************************************************/
    dc1394bool_t MaxResolution;      //ADV_INQ_1 0
    dc1394bool_t TimeBase;           //ADV_INQ_1 1
    dc1394bool_t ExtdShutter;        //ADV_INQ_1 2
    dc1394bool_t TestImage;          //ADV_INQ_1 3
    dc1394bool_t FrameInfo;          //ADV_INQ_1 4
    dc1394bool_t Sequences;          //ADV_INQ_1 5
    dc1394bool_t VersionInfo;        //ADV_INQ_1 6
    //ADV_INQ_1 7
    dc1394bool_t Lookup_Tables;      //ADV_INQ_1 8
    dc1394bool_t Shading;            //ADV_INQ_1 9
    dc1394bool_t DeferredTrans;      //ADV_INQ_1 10
    dc1394bool_t HDR_Mode;           //ADV_INQ_1 11
    dc1394bool_t DSNU;               //ADV_INQ_1 12
    dc1394bool_t BlemishCorrection;  //ADV_INQ_1 13
    dc1394bool_t TriggerDelay;       //ADV_INQ_1 14
    dc1394bool_t MirrorImage;        //ADV_INQ_1 15
    dc1394bool_t SoftReset;          //ADV_INQ_1 16
    dc1394bool_t HSNR;               //ADV_INQ_1 17
    dc1394bool_t ColorCorrection;    //ADV_INQ_1 18
    dc1394bool_t UserProfiles;       //ADV_INQ_1 19
    //ADV_INQ_1 20
    dc1394bool_t UserSets;           //ADV_INQ_1 21
    dc1394bool_t TimeStamp;          //ADV_INQ_1 22
    dc1394bool_t FrmCntStamp;        //ADV_INQ_1 23
    dc1394bool_t TrgCntStamp;        //ADV_INQ_1 24
    //ADV_INQ_1 25-30
    dc1394bool_t GP_Buffer;          //ADV_INQ_1 31
    /************************************************************************/
    dc1394bool_t Input_1;            //ADV_INQ_2 0
    dc1394bool_t Input_2;            //ADV_INQ_2 1
    //ADV_INQ_2 2-7
    dc1394bool_t Output_1;           //ADV_INQ_2 8
    dc1394bool_t Output_2;           //ADV_INQ_2 9
    dc1394bool_t Output_3;           //ADV_INQ_2 10
    dc1394bool_t Output_4;           //ADV_INQ_2 11
    //ADV_INQ_2 12-15
    dc1394bool_t IntEnaDelay;        //ADV_INQ_2 16
    dc1394bool_t IncDecoder;         //ADV_INQ_2 17
    //ADV_INQ_2 18-31
    /************************************************************************/
    dc1394bool_t CameraStatus;       //ADV_INQ_3 0
    //ADV_INQ_3 1-3
    dc1394bool_t AutoShutter;        //ADV_INQ_3 4
    dc1394bool_t AutoGain;           //ADV_INQ_3 5
    dc1394bool_t AutoFunctionAOI;    //ADV_INQ_3 6
    //ADV_INQ_3 7-31
    /************************************************************************/
    dc1394bool_t HDRPike;            //ADV_INQ_4 0
    //ADV_INQ_4 1-31


} dc1394_avt_adv_feature_info_t;


/**
 * SIS data structure
 * See also dc1394_avt_set_sis, dc1394_avt_get_sis_data_inquiry and dc1394_avt_get_sis_data.
 */
typedef struct __dc1394_avt_sis_data_struct
{
    union
    {
        struct
        {
            uint32_t    Offset : 12;       //!< elapsed time within the current bus cycle, unit 40.69 ns (range 0-3071)
            uint32_t    Cycles : 13;       //!< elapsed time in firewire bus cycles, unit 125 us, will be reset every second (range 0-7999)
            uint32_t    Seconds: 7;        //!< elapsed time in seconds
        }m;
        uint32_t m_all;
    } CycleTime;
    uint32_t     FrameCounter;         //!< frame number
    uint32_t     TriggerCounter;       //!< number of detected trigger events
    uint16_t     AOILeft;              //!< AOI left position
    uint16_t     AOITop;               //!< AOI top position
    uint16_t     AOIWidth;             //!< AOI width
    uint16_t     AOIHeight;            //!< AOI height
    uint32_t     Shutter;              //!< shutter setting
    uint16_t     Gain;                 //!< gain setting
    uint16_t     Reserved0;            //!< reserved for future use
    uint8_t      OutputState[4];       //!< output pin states - idle:0 active:255
    uint8_t      InputState[2];        //!< input pin states - idle:0 active:255
    uint8_t      Reserved1[2];         //!< reserved for future use
    uint8_t      SequenceIndex;        //!< sequence position (sequence mode)
    uint8_t      Reserved2a;           //!< reserved for future use
    uint8_t      ColorCoding;          //!< IIDC color coding
    uint8_t      Reserved2b;           //!< reserved for future use
    uint32_t     SerialNumber;         //!< camera serial number
    uint32_t     UserValue;            //!< user defined value
} dc1394_avt_sis_data;

/*
 * Pixel Position for Defect Pixel Correction
 * See also dc1394_avt_set_dpc, dc1394_avt_read_dpc_data and dc1394_avt_write_dpc_data
 */
typedef union __dc1394_avt_dpc_pixel_position_union
{
    struct
    {
        uint32_t    x : 16;       //!< Defect Pixel X-Coordinate
        uint32_t    y : 16;       //!< Defect Pixel Y-Coordinate
    }m;
    uint32_t m_all;
} dc1394_avt_dpc_pixel_position;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Retrieve the firmware version, FPGA version and the camera ID
 */
dc1394error_t dc1394_avt_get_version(dc1394camera_t *camera,
                                     uint32_t *UCType,
                                     uint32_t *Version,
                                     uint32_t *Camera_ID,
                                     uint32_t *FPGA_Version);

/**
 * Adjust the frame's color code (in some situations where raw is declared as mono).
 * It is safe to call this function for any received frame.
 */
dc1394error_t dc1394_avt_adjust_frames(uint32_t Camera_ID, dc1394video_frame_t *frame);

/**
 * Retrieve a list with supported features
 * This function is provided for backwards compatibility. New code should call
 * dc1394_avt_get_smart_feature_inquiry instead.
 */
dc1394error_t dc1394_avt_get_advanced_feature_inquiry(dc1394camera_t *camera,
                                                      dc1394_avt_adv_feature_info_t *adv_feature);

/**
 * Retrieve a list with supported features
 * Parameter 'size' is expected to reflect the size of struct smart_feature in bytes.
 * It is recommended to pass 'sizeof(dc1394_avt_smart_feature_inquiry_t)' as size.
 */
dc1394error_t dc1394_avt_get_smart_feature_inquiry(dc1394camera_t *camera,
                                                   dc1394_avt_smart_feature_info_t *smart_feature,
                                                   int size );

/**
 * Print the list of supported features
 * See also dc1394_avt_get_advanced_feature_inquiry
 */
dc1394error_t dc1394_avt_print_advanced_feature(dc1394_avt_adv_feature_info_t *adv_feature);

/**
 * Print the list of supported features
 * See also dc1394_avt_get_smart_feature_inquiry
 */
dc1394error_t dc1394_avt_print_smart_features(dc1394_avt_smart_feature_info_t *adv_feature);

/**
 * Get Shading Correction settings.
 * This function is provided for backwards compatibility - new code
 * should call dc1394_avt_get_shading_correction instead.
 */
dc1394error_t dc1394_avt_get_shading(dc1394camera_t *camera,
                                     dc1394bool_t *on_off,
                                     dc1394bool_t *compute,
                                     dc1394bool_t *show, uint32_t *frame_nb);

/**
 * Get Shading Correction settings
 */
dc1394error_t dc1394_avt_get_shading_correction(dc1394camera_t *camera,
                                                dc1394switch_t *on_off,
                                                dc1394bool_t *build_err,
                                                dc1394switch_t *show,
                                                uint32_t *frame_nb,
                                                uint32_t *mem_channel,
                                                uint32_t *mem_channel_err);

/**
 * Set Shading Correction feature
 * This function is provided for backwards compatibility - new code
 * should call dc1394_avt_set_shading_correction instead.
 */
dc1394error_t dc1394_avt_set_shading(dc1394camera_t *camera,
                                     dc1394bool_t on_off, dc1394bool_t compute,
                                     dc1394bool_t show, uint32_t frame_nb);

/**
 * Set Shading Correction feature
 */
dc1394error_t dc1394_avt_set_shading_correction(dc1394camera_t *camera,
                                                dc1394switch_t on_off,
                                                dc1394switch_t compute,
                                                dc1394switch_t show,
                                                uint32_t frame_nb,
                                                uint32_t mem_channel,
                                                dc1394switch_t mem_clear,
                                                dc1394switch_t mem_load,
                                                dc1394switch_t mem_save);

/**
 * Get the current access mode (none/read/write) for Shading Correction image
 */
dc1394error_t dc1394_avt_get_shading_mem_ctrl(dc1394camera_t *camera,
                                              dc1394bool_t *en_write,
                                              dc1394bool_t *en_read,
                                              uint32_t *addroffset);

/**
 * Set access mode (none/read/write) for Shading Correction image
 */
dc1394error_t dc1394_avt_set_shading_mem_ctrl(dc1394camera_t *camera,
                                              dc1394bool_t en_write,
                                              dc1394bool_t en_read,
                                              uint32_t addroffset);

/**
 * Retrieve the max size of a Shading Correction image
 * This function is provided for backwards compatibility - new code
 * should call dc1394_avt_get_shading_correction_info instead.
 */
dc1394error_t dc1394_avt_get_shading_info(dc1394camera_t *camera,
                                          uint32_t *MaxImageSize);

/**
 * Retrieve the max size of a Shading Correction image and the number of
 * supported memory channels to store shading images inside the camera.
 */
dc1394error_t dc1394_avt_get_shading_correction_info(dc1394camera_t *camera,
                                                     uint32_t *MaxImageSize,
                                                     uint32_t *MemChannelCount);

/**
 * Get HDR mode (multiple slope) configuration
 * (on/off, the nb of kneepoints used and kneepoints values)
 */
dc1394error_t dc1394_avt_get_multiple_slope(dc1394camera_t *camera,
                                            dc1394bool_t *on_off,
                                            uint32_t *points_nb,
                                            uint32_t *kneepoint1,
                                            uint32_t *kneepoint2,
                                            uint32_t *kneepoint3);

/**
 * Set HDR mode (multiple slope) configuration
 * (on/off, the nb of kneepoints used and kneepoints values)
 */
dc1394error_t dc1394_avt_set_multiple_slope(dc1394camera_t *camera,
                                            dc1394bool_t on_off,
                                            uint32_t points_nb,
                                            uint32_t kneepoint1,
                                            uint32_t kneepoint2,
                                            uint32_t kneepoint3);

/**
 * Get the timebase used for 'shutter' feature.
 * Possible values:
 * 0:    1us
 * 1:    2us
 * 2:    5us
 * 3:   10us
 * 4:   20us
 * 5:   50us
 * 6:  100us
 * 7:  200us
 * 8:  500us
 * 9: 1000us
 */
dc1394error_t dc1394_avt_get_timebase(dc1394camera_t *camera,
                                      uint32_t *timebase_id);

/**
 * Set the timebase used for 'shutter' feature. See dc1394_avt_get_timebase for possible values.
 */
dc1394error_t dc1394_avt_set_timebase(dc1394camera_t *camera,
                                      uint32_t timebase_id);

/**
 * Get the Extented Shutter value in us
 */
dc1394error_t dc1394_avt_get_extented_shutter(dc1394camera_t *camera,
                                              uint32_t *timebase_id);

/**
 * Set the Extented Shutter value in us
 */
dc1394error_t dc1394_avt_set_extented_shutter(dc1394camera_t *camera,
                                              uint32_t timebase_id);

/**
 * Get the Max achievable resolution
 */
dc1394error_t dc1394_avt_get_MaxResolution(dc1394camera_t *camera,
                                           uint32_t *MaxHeight,
                                           uint32_t *MaxWidth);

/**
 * Get min and max shutter values for Autoshutter
 */
dc1394error_t dc1394_avt_get_auto_shutter(dc1394camera_t *camera,
                                          uint32_t *MinValue,
                                          uint32_t *MaxValue);

/**
 * Set min and max shutter values for Autoshutter
 */
dc1394error_t dc1394_avt_set_auto_shutter(dc1394camera_t *camera,
                                          uint32_t MinValue,
                                          uint32_t MaxValue);

/**
 * Get min and max gain values for Autogain
 */
dc1394error_t dc1394_avt_get_auto_gain(dc1394camera_t *camera,
                                       uint32_t *MinValue,
                                       uint32_t *MaxValue);

/**
 * Set min and max gain values for Autogain
 */
dc1394error_t dc1394_avt_set_auto_gain(dc1394camera_t *camera,
                                       uint32_t MinValue,
                                       uint32_t MaxValue);

/**
 * Get Trigger Delay configuration (on/off and the actual delay)
 */
dc1394error_t dc1394_avt_get_trigger_delay(dc1394camera_t *camera,
                                           dc1394bool_t *on_off,
                                           uint32_t *DelayTime);

/**
 * Set Trigger Delay configuration (on/off and the actual delay)
 */
dc1394error_t dc1394_avt_set_trigger_delay(dc1394camera_t *camera,
                                           dc1394bool_t on_off,
                                           uint32_t DelayTime);

/**
 * Get 'Mirror Image' configuration (horizontal)
 */
dc1394error_t dc1394_avt_get_mirror(dc1394camera_t *camera,
                                    dc1394bool_t *on_off);

/**
 * Set 'Mirror Image' configuration (horizontal)
 */
dc1394error_t dc1394_avt_set_mirror(dc1394camera_t *camera,
                                    dc1394bool_t on_off);

/**
 * Get DSNU mode and num of frames used for correction data computation.
 * This function is provided for backwards compatibility - new code
 * should call dc1394_avt_get_dsnu_correction instead.
 */
dc1394error_t dc1394_avt_get_dsnu(dc1394camera_t *camera,
                                  dc1394bool_t *on_off,
                                  uint32_t *frame_nb);

/**
 * Get DSNU correction configuration and error status of the last correction data computation.
 */
dc1394error_t dc1394_avt_get_dsnu_correction(dc1394camera_t *camera,
                                             dc1394switch_t *on_off,
                                             dc1394bool_t   *build_error,
                                             uint32_t       *frame_nb,
                                             dc1394switch_t *show_image);

/**
 * Set DSNU mode, number of frames used for correction data computation and launch the the computation of the dsnu frame
 * This function is provided for backwards compatibility - new code should call
 * dc1394_avt_set_dsnu_correction instead.
 */
dc1394error_t dc1394_avt_set_dsnu(dc1394camera_t *camera,
                                  dc1394bool_t on_off, dc1394bool_t compute,
                                  uint32_t frame_nb);

/**
 * Configure DSNU correction.
 * ( Enable/Disable, number of frames used for correction data computation, launch the computation of correction data,
 *   enable show_image to receive correction data instead of images, load or save data from/to flash )
 */
dc1394error_t dc1394_avt_set_dsnu_correction(dc1394camera_t *camera,
                                             dc1394switch_t on_off,
                                             dc1394switch_t compute_image,
                                             uint32_t       frame_nb,
                                             dc1394switch_t show_image,
                                             dc1394switch_t load_image,
                                             dc1394switch_t save_image);

/**
 * Get Blemish mode and num of frames used for correction data computation
 * This function is provided for backwards compatibility - new code
 * should call dc1394_avt_get_blemish_correction instead.
 */
dc1394error_t dc1394_avt_get_blemish(dc1394camera_t *camera,
                                     dc1394bool_t *on_off, uint32_t *frame_nb);

/**
 * Get Blemish Correction configuration and error status of the last correction data computation
 */
dc1394error_t dc1394_avt_get_blemish_correction(dc1394camera_t *camera,
                                                dc1394switch_t *on_off,
                                                dc1394bool_t   *build_error,
                                                uint32_t       *frame_nb,
                                                dc1394switch_t *show_image);

/**
 * Set Blemish mode, num of frames used for correction data computation and launch the the computation of correction data.
 * This function is provided for backwards compatibility - new code should call dc1394_avt_set_blemish_correction
 * instead.
 */
dc1394error_t dc1394_avt_set_blemish(dc1394camera_t *camera,
                                     dc1394bool_t on_off, dc1394bool_t compute,
                                     uint32_t frame_nb);

/**
 * Configure Blemish Correction.
 * ( On/Off, number of frames used for computation, launch computation, enable show_image to receive correction data instead of images,
 *   load or save data from/to flash)
 */
dc1394error_t dc1394_avt_set_blemish_correction(dc1394camera_t *camera,
                                                dc1394switch_t on_off,
                                                dc1394switch_t compute_image,
                                                uint32_t       frame_nb,
                                                dc1394switch_t show_image,
                                                dc1394switch_t load_image,
                                                dc1394switch_t save_image);

/**
 * Get the polarity, the mode, the state of the IO.
 * Parameter 'IO' specifies the Input-/Output Pin and should be set
 * to DC1394_AVT_IO_INP_CTRLx or DC1394_AVT_IO_OUTP_CTRLx
 */
dc1394error_t dc1394_avt_get_io(dc1394camera_t *camera, uint32_t IO,
                                dc1394bool_t *polarity, uint32_t *mode,
                                dc1394bool_t *pinstate);

/**
 * Set the polarity, the mode and the state of the IO
 * Parameter 'IO' specifies the Input-/Output Pin and should be set
 * to DC1394_AVT_IO_INP_CTRLx or DC1394_AVT_IO_OUTP_CTRLx
 */
dc1394error_t dc1394_avt_set_io(dc1394camera_t *camera,uint32_t IO,
                                dc1394bool_t polarity, uint32_t mode,
                                dc1394bool_t pinstate);

/**
 * Get minimum PWM (Pulse-Width Modulation) period in us.
 * output_pin should be set to DC1394_AVT_IO_OUTP_PWM_CTRLx
 */
dc1394error_t dc1394_avt_get_io_pwmout_info(dc1394camera_t *camera,
                                            uint32_t       pwm_output_pin,
                                            uint32_t       *min_period);

/**
 * Get timebase period and pulse width for PWM (Pulse-Width Modulation) output
 * output_pin should be set to DC1394_AVT_IO_OUTP_PWM_CTRLx
 */
dc1394error_t dc1394_avt_get_io_pwmout(dc1394camera_t *camera,
                                       uint32_t       pwm_output_pin,
                                       uint32_t       *period,
                                       uint32_t       *pulse_width);

/**
 * Set period and pulse width for PWM (Pulse-Width Modulation) output.
 * output_pin should be set to DC1394_AVT_IO_OUTP_PWM_CTRLx
 */
dc1394error_t dc1394_avt_set_io_pwmout(dc1394camera_t *camera,
                                       uint32_t       pwm_output_pin,
                                       uint32_t       period,
                                       uint32_t       pulse_width);

/**
 * Get valid Debounce time range for a certain input pin.
 * debounce_inp_pin should be set to DC1394_AVT_IO_INP_DEBOUNCE_CTRLx
 */
dc1394error_t dc1394_avt_get_io_inp_debounce_info(dc1394camera_t *camera,
                                                  uint32_t       debounce_inp_pin,
                                                  uint32_t       *min_debounce_time,
                                                  uint32_t       *max_debounce_time);

/**
 * Get Debounce time for a certain input pin.
 * debounce_inp_pin should be set to DC1394_AVT_IO_INP_DEBOUNCE_CTRLx
 */
dc1394error_t dc1394_avt_get_io_inp_debounce(dc1394camera_t *camera,
                                             uint32_t       debounce_inp_pin,
                                             uint32_t       *debounce_time);

/**
 * Set Debounce time for a certain input pin.
 * debounce_inp_pin should be set to DC1394_AVT_IO_INP_DEBOUNCE_CTRLx
 */
dc1394error_t dc1394_avt_set_io_inp_debounce(dc1394camera_t *camera,
                                             uint32_t       debounce_inp_pin,
                                             uint32_t       debounce_time);

/**
 * Reset the bus and the fpga
 */
dc1394error_t dc1394_avt_reset(dc1394camera_t *camera);

/**
 * Get LUT (look-up table) configuration (on/off and the index of the current LUT)
 */
dc1394error_t dc1394_avt_get_lut(dc1394camera_t *camera,
                                 dc1394bool_t *on_off, uint32_t *lutnb  );

/**
 * Set LUT (look-up table) configuration (on/off and the index of the current LUT)
 */
dc1394error_t dc1394_avt_set_lut(dc1394camera_t *camera,
                                 dc1394bool_t on_off, uint32_t lutnb);

/**
 * Get memory access mode of LUT (look-up table) data
 */
dc1394error_t dc1394_avt_get_lut_mem_ctrl(dc1394camera_t *camera,
                                          dc1394bool_t *en_write,
                                          uint32_t * AccessLutNo,
                                          uint32_t *addroffset);

/**
 * Set memory access mode of LUT (look-up table) data
 */
dc1394error_t dc1394_avt_set_lut_mem_ctrl(dc1394camera_t *camera,
                                          dc1394bool_t en_write,
                                          uint32_t AccessLutNo,
                                          uint32_t addroffset);

/**
 * Get num of LUTs (look-up tables) present and the max size
 * This function is provided for backwards compatibility - new code should call dc1394_avt_get_lut_extd_info
 * instead.
 */
dc1394error_t dc1394_avt_get_lut_info(dc1394camera_t *camera,
                                      uint32_t *NumOfLuts, uint32_t *MaxLutSize);

/**
 * Get num of LUTs (look-up tables) present, the maximum value for lut entries, the number of values per lut
 * and the lut size in bytes.
 */
dc1394error_t dc1394_avt_get_lut_extd_info(dc1394camera_t *camera,
                                           uint32_t       *NumOfLuts,
                                           uint32_t       *MaxValue,
                                           uint32_t       *NumOfValues,
                                           uint32_t       *MaxLutSize);

/**
 * Get Autofunction AOI unit sizes. Only multiples of these units are allowed for
 * area position and size.
 */
dc1394error_t dc1394_avt_get_autofunc_aoi_info(dc1394camera_t *camera,
                                               uint32_t       *unit_x,
                                               uint32_t       *unit_y);

/**
 * Get Autofunction AOI configuration (on/off and area).
 * This function is provided for backwards compatibility - new code should call dc1394_avt_get_autofunc_aoi
 * instead.
 */
dc1394error_t dc1394_avt_get_aoi(dc1394camera_t *camera,
                                 dc1394bool_t *on_off, int *left, int *top,
                                 int *width, int *height);

/**
 * Get Autofunction AOI configuration (on/off state, show area mode and area)
 */
dc1394error_t dc1394_avt_get_autofunc_aoi(dc1394camera_t *camera,
                                          dc1394switch_t *on_off,
                                          dc1394switch_t *show_area,
                                          uint32_t       *left,
                                          uint32_t       *top,
                                          uint32_t       *width,
                                          uint32_t       *height);

/**
 * Set  Autofunction AOI configuration (on/off and area).
 * This function is provided for backwards compatibility - new code should call dc1394_avt_set_autofunc_aoi
 * instead.
 */
dc1394error_t dc1394_avt_set_aoi(dc1394camera_t *camera,
                                 dc1394bool_t on_off,int left, int top,
                                 int width, int height);

/**
 * Set Autofunction AOI configuration (on/off state, show area mode and area)
 */
dc1394error_t dc1394_avt_set_autofunc_aoi(dc1394camera_t *camera,
                                          dc1394switch_t on_off,
                                          dc1394switch_t show_area,
                                          uint32_t       left,
                                          uint32_t       top,
                                          uint32_t       width,
                                          uint32_t       height);

/**
 * Get supported Test Images
 */
dc1394error_t dc1394_avt_get_test_images_info(dc1394camera_t *camera,
                                              dc1394bool_t   *TestImage1,
                                              dc1394bool_t   *TestImage2,
                                              dc1394bool_t   *TestImage3,
                                              dc1394bool_t   *TestImage4,
                                              dc1394bool_t   *TestImage5,
                                              dc1394bool_t   *TestImage6,
                                              dc1394bool_t   *TestImage7);

/**
 * Get Test Image configuration (Index 0 disables the feature)
 */
dc1394error_t dc1394_avt_get_test_images(dc1394camera_t *camera,
                                         uint32_t *image_no);

/**
 * Set Test Image configuration (Index 0 disables the feature)
 */
dc1394error_t dc1394_avt_set_test_images(dc1394camera_t *camera,
                                         uint32_t image_no);

/**
 * Get the number of captured frames
 * This function is provided for backwards compatibility - new code should call dc1394_avt_get_frame_counter
 * instead.
 */
dc1394error_t dc1394_avt_get_frame_info(dc1394camera_t *camera,
                                        uint32_t *framecounter);

/**
 * Frame Counter: Get the number of captured frames
 * feature_info is needed to point to a properly filled info struct - see dc1394_avt_get_smart_feature_inquiry
 */
dc1394error_t dc1394_avt_get_frame_counter(dc1394camera_t                           *camera,
                                           const dc1394_avt_smart_feature_info_t    *feature_info,
                                           uint32_t                                 *framecounter);

/**
 * Reset Frame Counter
 * This function is provided for backwards compatibility - new code should call dc1394_avt_reset_frame_counter
 * instead.
 */
dc1394error_t dc1394_avt_reset_frame_info(dc1394camera_t *camera);

/**
 * Reset Frame Counter
 * feature_info is needed to point to a properly filled info struct - see dc1394_avt_get_smart_feature_inquiry
 */
dc1394error_t dc1394_avt_reset_frame_counter(dc1394camera_t                         *camera,
                                             const dc1394_avt_smart_feature_info_t  *feature_info);

/**
 * Trigger Counter: Get the number detected trigger events
 */
dc1394error_t dc1394_avt_get_trigger_counter(dc1394camera_t *camera,
                                             uint32_t       *triggercounter);

/**
 * Reset trigger counter
 */
dc1394error_t dc1394_avt_reset_trigger_counter(dc1394camera_t *camera);

/**
 * Get the size of the GP Data buffer
 */
dc1394error_t dc1394_avt_get_gpdata_info(dc1394camera_t *camera,
                                         uint32_t *BufferSize);

/**
 * Get the fifo control mode
 */
dc1394error_t dc1394_avt_get_deferred_trans(dc1394camera_t *camera,
                                            dc1394bool_t *HoldImage,
                                            dc1394bool_t * FastCapture,
                                            uint32_t *FifoSize,
                                            uint32_t *NumOfImages );

/**
 * Set the fifo control mode
 */
dc1394error_t dc1394_avt_set_deferred_trans(dc1394camera_t *camera,
                                            dc1394bool_t HoldImage,
                                            dc1394bool_t  FastCapture,
                                            uint32_t FifoSize,
                                            uint32_t NumOfImages,
                                            dc1394bool_t SendImage );

/**
 * Read size number of bytes from GPData buffer
 */
dc1394error_t dc1394_avt_read_gpdata(dc1394camera_t *camera, unsigned char *buf,
                                     uint32_t size);

/**
 * Write size number of bytes to GPData buffer
 */
dc1394error_t dc1394_avt_write_gpdata(dc1394camera_t *camera,
                                      unsigned char *buf, uint32_t size);

/**
 * Read Shading Correction image from camera into buffer
 */
dc1394error_t dc1394_avt_read_shading_img(dc1394camera_t *camera,
                                          unsigned char *buf, uint32_t size);

/**
 * Write Shading Correction image from buffer to camera
 */
dc1394error_t dc1394_avt_write_shading_img(dc1394camera_t *camera,
                                           unsigned char *buf, uint32_t size);

/**
 * Write LUT (look-up table) data from buffer to camera
 */
dc1394error_t dc1394_avt_write_lut(dc1394camera_t *camera, uint32_t LutNo, unsigned char *buf,
                                   uint32_t size);

/**
 * Channel Balance: Read channel (gain) adjust (AVT Pike)
 */
dc1394error_t dc1394_avt_get_channel_adjust(dc1394camera_t *camera,
                                            int16_t *channel_adjust);

/**
 * Channel Balance: Write channel (gain) adjust (AVT Pike)
 */
dc1394error_t dc1394_avt_set_channel_adjust(dc1394camera_t *camera,
                                            int16_t channel_adjust);

/**
 * Channel Balance: Read channel (offset) adjust (AVT Pike)
 */
dc1394error_t dc1394_avt_get_channel_adjust_offset(dc1394camera_t *camera,
                                                   int16_t *channel_adjust);

/**
 * Channel Balance: Write channel (offset) adjust (AVT Pike)
 */
dc1394error_t dc1394_avt_set_channel_adjust_offset(dc1394camera_t *camera,
                                                   int16_t channel_adjust);

/**
 * Set Color Correction + Coefficients
 */
dc1394error_t dc1394_avt_set_color_corr(dc1394camera_t *camera, dc1394bool_t on_off, dc1394bool_t reset,
                int32_t Crr, int32_t Cgr, int32_t Cbr, int32_t Crg, int32_t Cgg, int32_t Cbg, int32_t Crb, int32_t Cgb, int32_t Cbb);

/**
 * Get Color Correction + Coefficients
 */
dc1394error_t dc1394_avt_get_color_corr(dc1394camera_t *camera, dc1394bool_t *on_off,
                int32_t *Crr, int32_t *Cgr, int32_t *Cbr, int32_t *Crg, int32_t *Cgg, int32_t *Cbg, int32_t *Crb, int32_t *Cgb, int32_t *Cbb);

/**
 * Get HSNR configuration
 * ( on/off, num of frames used for averaging )
 */
dc1394error_t dc1394_avt_get_hsnr(dc1394camera_t *camera, dc1394bool_t *on_off, uint32_t *grabCount);

/**
 * Set HSNR configuration
 * ( on/off, num of frames used for averaging )
 */
dc1394error_t dc1394_avt_set_hsnr(dc1394camera_t *camera, dc1394bool_t on_off, uint32_t grabCount);


/**
 * Get SIS configuration
 * feature_info is needed to point to a properly filled info struct - see dc1394_avt_get_smart_feature_inquiry.
 * Parameter userVal is only supported for newer camera families like Pike and Stingray and may be set to NULL.
 */
dc1394error_t dc1394_avt_get_sis(dc1394camera_t                         *camera,
                                 const dc1394_avt_smart_feature_info_t  *feature_info,
                                 dc1394switch_t                         *on_off,
                                 int16_t                                *linePos,
                                 uint32_t                               *userVal);

/**
 * Set SIS configuration
 * feature_info is needed to point to a properly filled info struct - see dc1394_avt_get_smart_feature_inquiry.
 * Changing linePos during image acquisition may lead to synchronization issues with dc1394_avt_get_sis_data.
 */
dc1394error_t dc1394_avt_set_sis(dc1394camera_t                         *camera,
                                 const dc1394_avt_smart_feature_info_t  *feature_info,
                                 dc1394switch_t                         on_off,
                                 int16_t                                linePos,
                                 uint32_t                               userVal);

/**
 * Get supported SIS elements for a certain camera. Supported elements of sis_data_inquiry will be set to '1',
 * '0' otherwise. Elements marked as 'reserved' are generally unsupported.
 * feature_info is needed to point to a properly filled info struct - see dc1394_avt_get_smart_feature_inquiry
 */
dc1394error_t dc1394_avt_get_sis_data_inquiry(const dc1394_avt_smart_feature_info_t *feature_info,
                                              dc1394_avt_sis_data                   *sis_data_inquiry );


/**
 * Retrieve SIS data from image data. 'linePos' needs to reflect the setting of the corresponding SIS parameter
 * at the time the image was aquired. Not all elements of sis_data are necessarily filled with valid data, depending
 * on the camera model. See dc1394_avt_get_sis_data_inquiry for supported elements.
 * feature_info is needed to point to a properly filled info struct - see dc1394_avt_get_smart_feature_inquiry
 */
dc1394error_t dc1394_avt_get_sis_data(const dc1394_avt_smart_feature_info_t *feature_info,
                                      dc1394video_frame_t                   *frame,
                                      int16_t                               linePos,
                                      dc1394_avt_sis_data                   *sis_data);

/**
 * Get 'Max Iso Size' settings. This feature overrides the maximum packet size for isochronuos
 * transfers for a certain FireWire speed mode. The following speed modes are supported:
 * 2: S400 setting
 * 3: S800 setting
 * 4: S1600 setting (future use)
 * 5: S3200 setting (future use)
 */
dc1394error_t dc1394_avt_get_max_iso_size(dc1394camera_t *camera,
                                          uint32_t       speed_mode,
                                          dc1394switch_t *on_off,
                                          uint32_t       *max_size);

/**
 * Set 'Max Iso Size' settings. This feature overrides the maximum packet size for isochronuos
 * transfers for a certain FireWire speed mode. The following speed modes are supported:
 * 2: S400 setting
 * 3: S800 setting
 * 4: S1600 setting (future use)
 * 5: S3200 setting (future use)
 * Parameter 'set_to_max' configures the maximum iso size possible.
 *
 * Caution: Activating this feature leads to bus utilization beyond FireWire Spec. Higher
 * framerates can be achieved, but asynchronuos transfers may be delayed.
 */
dc1394error_t dc1394_avt_set_max_iso_size(dc1394camera_t *camera,
                                          uint32_t       speed_mode,
                                          dc1394switch_t on_off,
                                          dc1394switch_t set_to_max,
                                          uint32_t       max_size);

/**
 * Get Parameter Update Timing. The following modes are supported:
 * 0: Standard Parameter Update Timing
 * 2: Quick Format Change Mode - A running image integration will be stopped when
 * new settings have to be applied
 */
dc1394error_t dc1394_avt_get_param_upd_timing(dc1394camera_t *camera,
                                              uint32_t       *update_timing_mode);

/**
 * Set Parameter Update Timing. The following modes are supported:
 * 0: Standard Parameter Update Timing
 * 2: Quick Format Change Mode - A running image integration will be stopped when
 * new settings have to be applied
 */
dc1394error_t dc1394_avt_set_param_upd_timing(dc1394camera_t *camera,
                                              uint32_t       update_timing_mode);

/**
 * Get Low Smear configuration
 */
dc1394error_t dc1394_avt_get_low_smear(dc1394camera_t *camera,
                                       dc1394switch_t *on_off);

/**
 * Set Low Smear configuration
 */
dc1394error_t dc1394_avt_set_low_smear(dc1394camera_t *camera,
                                       dc1394switch_t on_off);

/**
 * Get Low Noise Binning
 */
dc1394error_t dc1394_avt_get_low_noise_binning(dc1394camera_t *camera,
                                               dc1394switch_t *on_off);

/**
 * Set Low Noise Binning
 */
dc1394error_t dc1394_avt_set_low_noise_binning(dc1394camera_t *camera,
                                               dc1394switch_t on_off);

/**
 * Get Global Reset Release Shutter
 */
dc1394error_t dc1394_avt_get_global_res_rel_shutter(dc1394camera_t *camera,
                                                    dc1394switch_t *on_off);

/**
 * Set Global Reset Release Shutter
 */
dc1394error_t dc1394_avt_set_global_res_rel_shutter(dc1394camera_t *camera,
                                                    dc1394switch_t on_off);

/**
 * Get User Profile settings. This feature is an AVT-specific extension of the standard IIDC memory feature.
 */
dc1394error_t dc1394_avt_get_user_profile(dc1394camera_t *camera,
                                          uint32_t       *profile_id,
                                          dc1394bool_t   *error,
                                          uint32_t       *err_code );

/**
 * Set User Profile settings. This feature is an AVT-specific extension of the standard IIDC memory feature.
 * The number of supported user profiles is indicated by the max_mem_channel member of dc1394camera_t.
 */
dc1394error_t dc1394_avt_set_user_profile(dc1394camera_t *camera,
                                          uint32_t       profile_id,
                                          dc1394switch_t load_profile,
                                          dc1394switch_t save_profile,
                                          dc1394switch_t set_default);

/**
 * Get LED configuration - controls 'SW Feature - LED' functionality to blank the camera's status LEDs
 */
dc1394error_t dc1394_avt_get_led(dc1394camera_t *camera,
                                 dc1394switch_t *on_off);

/**
 * Set LED configuration - controls 'SW Feature - LED' functionality to blank the camera's status LEDs
 */
dc1394error_t dc1394_avt_set_led(dc1394camera_t *camera,
                                 dc1394switch_t on_off);

/**
 * Get info for feature Defect Pixel Correction: valid range for threshold, maximum size of defect pixel data
 */
dc1394error_t dc1394_avt_get_dpc_info(dc1394camera_t *camera,
                                      uint32_t       *MinThreshold,
                                      uint32_t       *MaxThreshold,
                                      uint32_t       *MaxSize);

/**
 * Get configuration for Defect Pixel Correction: on/off, threshold for computation, mean value and size of computed data
 */
dc1394error_t dc1394_avt_get_dpc(dc1394camera_t *camera,
                                 dc1394switch_t *on_off,
                                 uint32_t       *threshold,
                                 uint32_t       *mean_value,
                                 uint32_t       *data_size);

/**
 * Set configuration for Defect Pixel Correction: on/off, threshold for computation, mean value and size of computed data
 */
dc1394error_t dc1394_avt_set_dpc(dc1394camera_t *camera,
                                 dc1394switch_t on_off,
                                 dc1394switch_t build_data,
                                 dc1394switch_t zero_data,
                                 dc1394switch_t mem_save,
                                 dc1394switch_t mem_load,
                                 uint32_t       threshold);

/**
 * Download Defect Pixel Correction data to PC. PixelCount will return the number of pixels that have been read.
 * BufferSize indicates the size of DestBuffer in Pixels
 */
dc1394error_t dc1394_avt_read_dpc_data(dc1394camera_t                *camera,
                                       dc1394_avt_dpc_pixel_position *DestBuffer,
                                       uint32_t                      *PixelCount,
                                       uint32_t                      BufferSize );

/**
 * Upload Defect Pixel Correction data to camera. PixelCount indicates the number of pixels to be written.
 */
dc1394error_t dc1394_avt_write_dpc_data(dc1394camera_t                *camera,
                                        dc1394_avt_dpc_pixel_position *SourceBuffer,
                                        uint32_t                      PixelCount);
#ifdef __cplusplus
}
#endif

#endif
