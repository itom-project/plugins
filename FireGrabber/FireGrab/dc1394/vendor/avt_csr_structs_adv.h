/*
 * 1394-Based Digital Camera Control Library
 *
 * AVT Advanced Feature Helper Struct Definitions
 *
 * The original version of this file is distributed with the AVT FirePackage
 * for Windows, as part of the 'FireGrab' programming examples.
 *
 * Copyright (C) 2010 Allied Vision Technologies GmbH
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


#ifndef __CSR_STRUCTS_ADV_H
#define __CSR_STRUCTS_ADV_H

#include <types.h>

#define DC1394_AVT_NUM_FORMAT7_MODES       8

/*****************************************************************************
    Helper struct definitions originally used for standard IIDC features
*****************************************************************************/

typedef struct __dc1394_avt_imagepos_struct
{
    uint16_t                m_nTop;
    uint16_t                m_nLeft;
} dc1394_avt_imagepos_t;

typedef struct __dc1394_avt_imagesize_struct
{
    uint16_t                m_nHeight;
    uint16_t                m_nWidth;
} dc1394_avt_imagesize_t;

typedef struct __dc1394_avt_aoi_struct
{
    dc1394_avt_imagepos_t             m_imgPos;
    dc1394_avt_imagesize_t            m_imgSize;
} dc1394_avt_aoi_t;

typedef union __dc1394_avt_csr_whitebal_inq_union
{
    struct __tagBrightnessInqElems
    {
        uint32_t                m_nMaxVal       : 12;
        uint32_t                m_nMinVal       : 12;
        uint32_t                m_bManual       :  1;
        uint32_t                m_bAuto         :  1;
        uint32_t                m_bOnOff        :  1;
        uint32_t                m_bReadOut      :  1;
        uint32_t                m_bOnePush      :  1;
        uint32_t                                :  1;
        uint32_t                m_bAbsControl   :  1;
        uint32_t                m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csr_whitebal_inq_t;

/*****************************************************************************
    Advanced feature registers
*****************************************************************************/

#define DC1394_AVT_CSR_MAX_AFECHN      4


// ---------------------------------------------------------------------------
//
typedef struct __dc1394_avt_csradv_version_info_struct
{
    uint16_t                    m_nArmVersion;      //  2 Bytes
    uint16_t                    m_nArmSpecID;       //  2 Bytes
    uint32_t                    m_nArmVersionEx;    //  4 Bytes
    uint16_t                    m_nFpgaVersion;     //  2 Bytes
    uint16_t                    m_nFpgaSpecID;      //  2 Bytes
    uint32_t                    m_nFpgaVersionEx;   //  4 Bytes
} dc1394_avt_csradv_version_info_t;

typedef struct __dc1394_avt_csradv_version_info_ex_struct
{
    dc1394_avt_csradv_version_info_t    m_Rev;

    uint64_t                            gap3[2];            // 16 Bytes

    uint64_t                            m_nOrderID;         //  8 Bytes
    uint64_t                            m_nCustomerKey;     //  8 Bytes
} dc1394_avt_csradv_version_info_ex_t;

// ---------------------------------------------------------------------------
// Inquiry of available advanced features
typedef struct __dc1394_avt_smart_feature_info_full_struct
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

    /* reserved space for future features */
    dc1394bool_t    reserved[64];

    /* Internal information used by libdc functions. */
    struct __tagCsrAdvInfoInternal
    {
        struct __tagCsrAdvInq1
        {
            uint32_t                m_bGPBuffer         :  1;   // BIT31
            uint32_t                m_bParamListBuffer  :  1;   // BIT30
            uint32_t                                    :  5;   // BIT[25..29]
            uint32_t                m_bTrgCounter       :  1;   // BIT24
            uint32_t                m_bFrmCounter2      :  1;   // BIT23
            uint32_t                m_bTimestamp2       :  1;   // BIT22
            uint32_t                m_bUserProfiles     :  1;   // BIT21
            uint32_t                m_bTimestamp        :  1;   // BIT20
            uint32_t                m_bColorAvg         :  1;   // BIT19 (DF145C only)
            uint32_t                m_bColorCorr        :  1;   // BIT18
            uint32_t                m_bHighSNR          :  1;   // BIT17
            uint32_t                m_bSoftReset        :  1;   // BIT16
            uint32_t                m_bImageMirror      :  1;   // BIT15
            uint32_t                m_bTriggerDelay     :  1;   // BIT14
            uint32_t                m_bBlemishCorr      :  1;   // BIT13
            uint32_t                m_bFpnCorrection    :  1;   // BIT12
            uint32_t                m_bCmosHdrMode      :  1;   // BIT11
            uint32_t                m_bDeferredTrans    :  1;   // BIT10
            uint32_t                m_bShading          :  1;   // BIT9
            uint32_t                m_bLut              :  1;   // BIT8
            uint32_t                                    :  1;   // BIT7
            uint32_t                m_bVersionInfo      :  1;   // BIT6
            uint32_t                m_bSequences        :  1;   // BIT5
            uint32_t                m_bFrameInfo        :  1;   // BIT4
            uint32_t                m_bTestImage        :  1;   // BIT3
            uint32_t                m_bExtdShutter      :  1;   // BIT2
            uint32_t                m_bTimeBase         :  1;   // BIT1
            uint32_t                m_bMaxResolution    :  1;   // BIT0
        }                           Inq1;

        struct __tagCsrAdvInq2
        {
            uint32_t                m_bOutp_8_PWM       :  1;
            uint32_t                m_bOutp_7_PWM       :  1;
            uint32_t                m_bOutp_6_PWM       :  1;
            uint32_t                m_bOutp_5_PWM       :  1;
            uint32_t                m_bOutp_4_PWM       :  1;
            uint32_t                m_bOutp_3_PWM       :  1;
            uint32_t                m_bOutp_2_PWM       :  1;
            uint32_t                m_bOutp_1_PWM       :  1;
            uint32_t                                    :  6;
            uint32_t                m_bIncDecoder       :  1;
            uint32_t                m_bIntEnaDelay      :  1;
            uint32_t                m_bOutp_8           :  1;
            uint32_t                m_bOutp_7           :  1;
            uint32_t                m_bOutp_6           :  1;
            uint32_t                m_bOutp_5           :  1;
            uint32_t                m_bOutp_4           :  1;
            uint32_t                m_bOutp_3           :  1;
            uint32_t                m_bOutp_2           :  1;
            uint32_t                m_bOutp_1           :  1;
            uint32_t                m_bInp_8            :  1;
            uint32_t                m_bInp_7            :  1;
            uint32_t                m_bInp_6            :  1;
            uint32_t                m_bInp_5            :  1;
            uint32_t                m_bInp_4            :  1;
            uint32_t                m_bInp_3            :  1;
            uint32_t                m_bInp_2            :  1;
            uint32_t                m_bInp_1            :  1;
        }                           Inq2;

        struct __tagCsrAdvInq3
        {
            uint32_t                m_bInpDebounce_8    :  1;
            uint32_t                m_bInpDebounce_7    :  1;
            uint32_t                m_bInpDebounce_6    :  1;
            uint32_t                m_bInpDebounce_5    :  1;
            uint32_t                m_bInpDebounce_4    :  1;
            uint32_t                m_bInpDebounce_3    :  1;
            uint32_t                m_bInpDebounce_2    :  1;
            uint32_t                m_bInpDebounce_1    :  1;
            uint32_t                                    : 10;
            uint32_t                m_bSWFeatureCtrl    :  1;
            uint32_t                m_bDpc              :  1;
            uint32_t                m_bGlobResRelShutter:  1;
            uint32_t                                    :  1;
            uint32_t                m_bLowNoiseBinning  :  1;
            uint32_t                m_bSIS              :  1;
            uint32_t                m_bSequenceStep     :  1;
            uint32_t                m_bAutoFncAOI       :  1;
            uint32_t                m_bAutoGain         :  1;
            uint32_t                m_bAutoShutter      :  1;
            uint32_t                m_bF7ModeMapping    :  1;
            uint32_t                m_bParamUpdTiming   :  1;
            uint32_t                m_bMaxIsoSize       :  1;
            uint32_t                m_bCameraStatus     :  1;
        }                           Inq3;

        struct __tagCsrAdvInq4
        {
            uint32_t                                    : 27;
            uint32_t                m_bPikeChannelOffset:  1;
            uint32_t                m_bAdvWhiteBal      :  1;
            uint32_t                m_bLowSmear         :  1;
            uint32_t                m_bPikeChannelComp  :  1;
            uint32_t                m_bPikeHdrMode      :  1;
        }                           Inq4;
    } Internal;
} dc1394_avt_smart_feature_info_full_t;

/* Camera status register
 */
typedef struct __dc1394_avt_csradv_camstatus_struct
{
    uint32_t                    m_nImplID;          //!< presence + impl ID
    uint32_t                    m_nStatus;
} dc1394_avt_csradv_camstatus_t;

/* Max resolution information register
 */
typedef union __dc1394_avt_csradv_max_resolution_union
{
    dc1394_avt_imagesize_t      m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_max_resolution_t;

// ---------------------------------------------------------------------------
//
typedef union __dc1394_avt_csradv_timebase_union
{
    struct __tagCsrAdvTimebaseElems
    {
        uint32_t                m_nBase         :  4;   //! shutter time base ID
        uint32_t                                :  8;
        uint32_t                m_nExpOffset    : 12;   //! camera specific exposure offset in uSec
        uint32_t                                :  7;
        uint32_t                m_bPresence     :  1;   //! presence of this feature
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_timebase_t;

typedef union __dc1394_avt_csradv_extd_shutter_union
{
    struct __tagCsrAdvExtdShutterElems
    {
        uint32_t                m_nShutter      : 26;
        uint32_t                                :  5;
        uint32_t                m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_extd_shutter_t;

typedef union __dc1394_avt_csradv_trigger_delay_union
{
    struct __tagCsrAdvTriggerDelayElems
    {
        uint32_t                m_nDelay        : 21;
        uint32_t                                :  4;
        uint32_t                m_bOnOff        :  1;
        uint32_t                                :  5;
        uint32_t                m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_trigger_delay_t;

// ---------------------------------------------------------------------------
//
typedef union __dc1394_avt_csradv_testpix_union
{
    struct __tagCsrAdvTestPixElems
    {
        uint32_t                m_nTestPix      :  4;
        uint32_t                                : 13;
        uint32_t                m_bImg7Inq      :  1;   // Image 7 present
        uint32_t                m_bImg6Inq      :  1;   // Image 6 present
        uint32_t                m_bImg5Inq      :  1;   // Image 5 present
        uint32_t                m_bImg4Inq      :  1;   // Image 4 present
        uint32_t                m_bImg3Inq      :  1;   // Image 3 present
        uint32_t                m_bImg2Inq      :  1;   // Image 2 present
        uint32_t                m_bImg1Inq      :  1;   // Image 1 present
        uint32_t                                :  7;
        uint32_t                m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_testpix_t;

// ---------------------------------------------------------------------------
// Sequence control and parameter
typedef union __dc1394_avt_csradv_seqctrl_union
{
    struct __tagCsrAdvSeqParamElems
    {
        uint32_t                m_nSeqLength    :  8;   //!< user selected sequence length
        uint32_t                m_nMaxLength    :  8;   //!< max sequence length (read-only)
        uint32_t                m_nSeqMode      :  8;   //!< sequence mode
        uint32_t                m_bSetupMode    :  1;   //!< sequence setup mode
        uint32_t                m_bOnOff        :  1;   //!< sequence enabled/disabled
        uint32_t                m_bAutoRewind   :  1;   //!< auto reqwind sequence at the end
        uint32_t                                :  4;
        uint32_t                m_bPresence     :  1;   //!< presence of this feature
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_seqctrl_t;

typedef union __dc1394_avt_csradv_seqparam_union
{
    struct __tagCsrAdvSeqCtrlElems
    {
        uint32_t                m_nImageNo      :  8;   //!< sequence item index (0..n)
        uint32_t                m_nImgRep       :  8;   //!< repeat count of sequence item
        uint32_t                m_nSeqMode      :  8;   //!< sequence mode
        uint32_t                                :  1;
        uint32_t                m_bIncImgNo     :  1;   //!< increment m_nImageNo on m_bApply
        uint32_t                m_bApply        :  1;
        uint32_t                                :  5;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_seqparam_t;

typedef union __dc1394_avt_csradv_seqstep_union
{
    struct __tagCsrAdvSeqStepElems
    {
        uint32_t                m_nSeqPos       :  8;   //!< current sequence position
        uint32_t                                : 17;
        uint32_t                m_bReset        :  1;   //!< perform sequence reset
        uint32_t                m_bStep         :  1;   //!< perform sequence step
        uint32_t                                :  4;
        uint32_t                m_bPresence     :  1;   //!< presence of this feature
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_seqstep_t;

typedef struct __dc1394_avt_csradv_sequence_struct
{
    dc1394_avt_csradv_seqctrl_t      m_Ctrl;
    dc1394_avt_csradv_seqparam_t     m_Param;
} dc1394_avt_csradv_sequence_t;

// ---------------------------------------------------------------------------
// Lut control
//#define DC1394_AVT_LUT_TABLE_SIZE          8192
//#define DC1394_AVT_LUT_NUMOFTABLES         64

typedef struct __dc1394_avt_csradv_lut_ctrl_struct
{
    // Lut control
    struct __tagCsrAdvLutCtrlElems
    {
        uint32_t                m_nLutNo        :  6;   //! number of table to use
        uint32_t                                :  2;
        uint32_t                m_nMemChn       :  8;   //! memory channel to save/load data to/from
        uint32_t                                :  6;
        uint32_t                m_bMemLoad      :  1;   //! load LUT data from channel n
        uint32_t                m_bMemSave      :  1;   //! save LUT data to channel n
        uint32_t                                :  1;
        uint32_t                m_bOnOff        :  1;
        uint32_t                                :  5;
        uint32_t                m_bPresence     :  1;
    }                           m_Ctrl;

    struct __tagCsrAdvLutMemElems
    {
        uint32_t                m_nAddrOffset   : 16;   //! address offset to selected LUT
        uint32_t                m_nLut2WR       :  8;   //! number of LUT to access
        uint32_t                                :  2;
        uint32_t                m_bEnaMemWR     :  1;
        uint32_t                                :  4;
        uint32_t                m_bPresence     :  1;
    }                           m_Mem;

    struct __tagCsrAdvLutInfoElems
    {
        uint32_t                m_nMaxSize      : 16;   //! max size of one LUT
        uint32_t                m_nNumOfLuts    :  8;   //! number of available LUTs
        uint32_t                m_nBitsPerValue :  5;   //! number of bits/grey value
        uint32_t                                :  2;
        uint32_t                m_bPresence     :  1;   //
    }                           m_Info;

} dc1394_avt_csradv_lut_ctrl_t;

// ---------------------------------------------------------------------------
// Deferred transport
typedef union __dc1394_avt_csradv_deferredtrans_union
{
    struct __tagCsrAdvDeferredTransElems
    {
        uint32_t                m_nSendPix      :  8;   //! number of images WR:to send, RD: left
        uint32_t                m_nFifoDepth    :  8;   //! depth of image FIFO
        uint32_t                                :  8;
        uint32_t                m_bFastCapture  :  1;   //!
        uint32_t                m_bHoldImg      :  1;   //!
        uint32_t                m_bSendPix      :  1;   //!
        uint32_t                                :  4;
        uint32_t                m_bPresence     :  1;   //!
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_deferredtrans_t;

// ---------------------------------------------------------------------------
// IO input/output control
#define DC1394_AVT_MAX_NUM_OF_INPUTS            8
#define DC1394_AVT_MAX_NUM_OF_OUTPUTS           8

typedef enum __dc1394_avt_csradv_inp_modes_enum
{
    DC1394_AVT_GPIO_INP_OFF                = 0,
    DC1394_AVT_GPIO_INP_TRIGGER            = 0x02,
    DC1394_AVT_GPIO_INP_DECODER            = 0x03,
    DC1394_AVT_GPIO_INP_SEQSTEP_EDGE       = 0x04,     //!< sequence stepping on edge signal
    DC1394_AVT_GPIO_INP_SEQSTEP_LEVEL      = 0x05,     //!< sequence stepping on level signal
    DC1394_AVT_GPIO_INP_SEQ_RESET          = 0x06,     //!< sequence reset on edge signal
} dc1394_avt_csradv_inp_modes_t;

typedef union __dc1394_avt_csradv_io_inp_ctrlx_union
{
    struct __tagCsrAdvIoInpCtrlxElems
    {
        uint32_t                m_bPinState     :  1;   //!< pin input state
        uint32_t                                : 15;
        uint32_t                m_nMode         :  5;   //!< pin input mode
        uint32_t                                :  3;
        uint32_t                m_bPolarity     :  1;   //!< pin polarity
        uint32_t                                :  5;
        uint32_t                m_bIsBidir      :  1;   //!< pin is input & output pin
        uint32_t                m_bPresence     :  1;   //!< pin is present
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_io_inp_ctrlx_t;

typedef enum __dc1394_avt_csradv_io_outp_modes_enum
{
    DC1394_AVT_GPIO_OUTP_OFF               = 0,
    DC1394_AVT_GPIO_OUTP_DIRECT            = 0x01,
    DC1394_AVT_GPIO_OUTP_INTENA            = 0x02,
    DC1394_AVT_GPIO_OUTP_DECODER           = 0x03,
    DC1394_AVT_GPIO_OUTP_FVAL              = 0x06,
    DC1394_AVT_GPIO_OUTP_BUSY              = 0x07,
    DC1394_AVT_GPIO_OUTP_FOLLOW_INP        = 0x08,
    DC1394_AVT_GPIO_OUTP_PWM               = 0x09,
} dc1394_avt_csradv_io_outp_modes_t;

typedef union __dc1394_avt_csradv_io_outp_ctrlx_union
{
    struct __tagCsrAdvIoOutpCtrlxElems
    {
        uint32_t                m_bPinState     :  1;   //!< pin readback state
        uint32_t                                : 15;
        uint32_t                m_nMode         :  5;   //!< pin mode
        uint32_t                                :  3;
        uint32_t                m_bPolarity     :  1;   //!< pin polartiy
        uint32_t                                :  5;
        uint32_t                m_bHasPwm       :  1;   //!< pin has PWM capability
        uint32_t                m_bPresence     :  1;   //!< pin is present
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_io_outp_ctrlx_t;

typedef struct __dc1394_avt_csradv_io_outp_pwmx_struct
{
    struct
    {
        uint32_t                                : 12;
        uint32_t                m_nMinPeriod    : 16;   //! minimum PWM period in microseconds (aka max frequency)
        uint32_t                                :  3;
        uint32_t                m_bPresence     :  1;   //! presence of this feature
    }                           m_Ctrl;

    struct
    {
        uint32_t                m_nPeriod       : 16;   //!< PWM period in microseconds * timebase
        uint32_t                m_nPulseWidth   : 16;   //!< PWM pulse width in microseconds * timebase
    }                           m_Pwm;
} dc1394_avt_csradv_io_outp_pwmx_t;

// integration enable delay
typedef union __dc1394_avt_csradv_intena_delay_union
{
    struct __tagCsrAdvIntEnaDelayElems
    {
        uint32_t                m_nDelay1us     : 20;
        uint32_t                                :  5;
        uint32_t          m_bOnOff        :  1;
        uint32_t                                :  5;
        uint32_t            m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_intena_delay_t;

// incremental decoder
typedef struct __dc1394_avt_csradv_decoder_struct
{
    struct __tagCsrAdvDecoderCtrl
    {
        uint32_t                                : 24;
        uint32_t          m_bClearCounter :  1;
        uint32_t          m_bOnOff        :  1;
        uint32_t                                :  5;
        uint32_t            m_bPresence     :  1;
    }                           m_Ctrl;

    struct __tagCsrAdvDecoderValues
    {
        uint32_t                m_nCounter      : 12;
        uint32_t                                :  4;
        uint32_t                m_nCompare      : 12;
        uint32_t                                :  4;
    }                           m_Val;

} dc1394_avt_csradv_decoder_t;

typedef struct
{
    dc1394_avt_csradv_io_inp_ctrlx_t         m_ioInp[DC1394_AVT_MAX_NUM_OF_INPUTS];
} dc1394_avt_csradv_io_inp_ctrls_t;

typedef struct
{
    dc1394_avt_csradv_io_outp_ctrlx_t        m_ioOutp[DC1394_AVT_MAX_NUM_OF_OUTPUTS];
} dc1394_avt_csradv_io_outp_ctrls_t;

typedef struct
{
    dc1394_avt_csradv_io_inp_ctrlx_t         m_ioInp[DC1394_AVT_MAX_NUM_OF_INPUTS];
    dc1394_avt_csradv_io_outp_ctrlx_t        m_ioOutp[DC1394_AVT_MAX_NUM_OF_OUTPUTS];
} dc1394_avt_csradv_io_ctrl_t;

typedef struct
{
    dc1394_avt_csradv_io_outp_pwmx_t         m_ioOutp[DC1394_AVT_MAX_NUM_OF_OUTPUTS];
} dc1394_avt_csradv_io_pwm_t;


/**
 *  Advanced feature input debounce control
 */
typedef struct __dc1394_avt_csradv_io_inp_debounce_x_struct
{
    union __tagCsrAdvIoInpDebounceCtrl
    {
        struct __tagCsrAdvIoInpDebounceElems
        {
            uint32_t            m_nDebTime      : 24;   //!< input debounce time in 500ns steps
            uint32_t                            :  7;
            uint32_t            m_bPresence     :  1;   //!< presence of this feature
        }                       m;
        uint32_t                m_nAll;
    }                           m_Ctrl;

    uint32_t                    m_nMinDebTime;          //!< minimum debounce time
    uint32_t                    m_nMaxDebTime;          //!< maximum debounce time
    uint32_t                    m_gap;
} dc1394_avt_csradv_io_inp_debounce_x;

typedef struct
{
    dc1394_avt_csradv_io_inp_debounce_x    m_ioInp[DC1394_AVT_MAX_NUM_OF_INPUTS];
} dc1394_avt_csradv_io_inp_debounce;

// ---------------------------------------------------------------------------
// Serial function control
typedef union __dc1394_avt_csradv_serialfunction_union
{
    struct __tagCsrAdvSerialFunctionElems
    {
        uint32_t                m_nFuncID       : 16;
        uint32_t                m_nBitrateID    :  8;
        uint32_t                                :  7;
        uint32_t            m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_serialfunction_t;

// ---------------------------------------------------------------------------
// Shading control
typedef struct __dc1394_avt_csradv_shading_struct
{
    struct __tagCsrAdvShadingCtrl
    {
        uint32_t                m_nGrabCount    :  8;   //! number of images to build shading image
        uint32_t                m_nMemChn       :  4;   //! memory channel to save/load shading image to/from
        uint32_t                m_nMemChnError  :  4;   //! indicates any memory channel errors
        uint32_t                                :  5;
        uint32_t          m_bMemClear     :  1;   //! clear shading image from channel n
        uint32_t          m_bMemLoad      :  1;   //! load shading image from channel n
        uint32_t          m_bMemSave      :  1;   //! save shading image to channel n
        uint32_t            m_bBusy         :  1;   //! build shading image in progress
        uint32_t          m_bOnOff        :  1;   //! shading on/off
        uint32_t          m_bBuildTable   :  1;   //! build shading image now
        uint32_t          m_bShowImg      :  1;   //! show shading data as image
        uint32_t                                :  2;
        uint32_t            m_bBuildError   :  1;   //! build shading image reports an error
        uint32_t            m_bPresence     :  1;   //! Presence of this feature
    }                           m_Ctrl;

    struct __tagCsrAdvShadingMem
    {
        uint32_t                m_nAddrOffset   : 24;   //! wr: set address offset
                                                        //! rd: get address offset
        uint32_t                                :  1;
        uint32_t          m_bEnaMemRD     :  1;   //! enable RD access
        uint32_t          m_bEnaMemWR     :  1;   //! enable WR access
        uint32_t                                :  4;
        uint32_t            m_bPresence     :  1;   //! Presence of this feature
    }                           m_Mem;

    struct __tagCsrAdvShadingInfo
    {
        uint32_t                m_nMaxSize      : 24;   // max size of shading image
        uint32_t                m_nMemChnCount  :  4;   // number of available memory channels
        uint32_t                                :  3;
        uint32_t            m_bPresence     :  1;   // Presence of this feature
    }                           m_Info;
} dc1394_avt_csradv_shading_t;

// ---------------------------------------------------------------------------
// FPN & Blemish correction control
typedef union __dc1394_avt_csradv_dsnucorrection_union
{
    struct __tagCsrAdvDsnuCorrectionCtrl
    {
        uint32_t                m_nGrabCount    :  8;   // number of images to build FPN image
        uint32_t                m_nMemChn       :  4;   // memory channel to save/load FPN image to/from
        uint32_t                                :  9;
        uint32_t                m_bZeroTable    :  1;   // zero the FPN image buffer
        uint32_t                m_bMemLoad      :  1;   // load FPN image from storage
        uint32_t                m_bMemSave      :  1;   // save FPN image to storage
        uint32_t                m_bBusy         :  1;   // build FPN image in progress
        uint32_t                m_bOnOff        :  1;   // FPN correction on/off
        uint32_t                m_bBuildTable   :  1;   // build shading image now
        uint32_t                m_bShowImg      :  1;   // show shading data as image
        uint32_t                                :  2;
        uint32_t                m_bBuildError   :  1;   // build FPN image reports an error
        uint32_t                m_bPresence     :  1;   // Presence of this feature
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_dsnucorrection_t, dc1394_avt_csradv_blemishcorrection_t;

// ---------------------------------------------------------------------------
// Defect Pixel Correction control
typedef struct __dc1394_avt_csradv_defect_pixel_correction_struct
{
    struct __tagCsrAdvDefectPixelCorrectionCtrlElems
    {
        uint32_t                m_nThreshold    :  7;    //!< threshold for DP correction
        uint32_t                m_nMean         :  7;      //!< calculated mean value
        uint32_t                                :  7;
        uint32_t                m_bZeroDPData   :  1;    //!< zero DP data
        uint32_t                m_bMemLoad      :  1;    //!< load DP data from storage
        uint32_t                m_bMemSave      :  1;    //!< save DP data to storage
        uint32_t                m_bBusy         :  1;    //!< build DP data in progress
        uint32_t                m_bOnOff        :  1;    //!< DP correction on/off
        uint32_t                m_bBuildDPData  :  1;    //!< build DP data now
        uint32_t                                :  3;
        uint32_t                m_bBuildError   :  1;    //!< build DP data reports an error
        uint32_t                m_bPresence     :  1;    //!< presence of this feature
    }                           m_Ctrl;

    struct __tagCsrAdvDefectPixelCorrectionMemElems
    {
        uint32_t                m_nAddrOffset    : 14;   //!< address offset to selected DP data
        uint32_t                m_nDPDataSize    : 14;   //!< size of DP data to read (from Ram to Host)
        uint32_t                m_bEnaMemRD      :  1;   //!< enable RD access (from Ram to Host)
        uint32_t                m_bEnaMemWR      :  1;   //!< enable WR access (from Host to Ram)
        uint32_t                                 :  1;
        uint32_t                m_bPresence      :  1;   //!< presence of this feature
    }                           m_Mem;

    struct __tagCsrAdvDefectPixelCorrectionInfoElems
    {
        uint32_t                m_nMaxSize       : 14;   //!< max size of the DP data
        uint32_t                m_nMaxThreshold  :  7;   //!< max value for threshold
        uint32_t                m_nMinThreshold  :  7;   //!< min value for threshold
        uint32_t                                 :  3;
        uint32_t                m_bPresence      :  1;   //!< presence of this feature
    }                              m_Info;
} dc1394_avt_csradv_defect_pixel_correction_t;

// ---------------------------------------------------------------------------
// soft reset
typedef union __dc1394_avt_csradv_softreset_union
{
    struct __tagCsrAdvSoftResetElems
    {
        uint32_t                m_nDelay10ms    : 12;
        uint32_t                                : 13;
        uint32_t                m_bReset        :  1;   //! restart current firmware set
        uint32_t                                :  5;
        uint32_t                m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_softreset_t;

// ---------------------------------------------------------------------------
// Local Color Anti Aliasing
typedef union __dc1394_avt_csradv_coloravg_union
{
    struct __tagCsrAdvColorAvgElems
    {
        uint32_t                                : 25;
        uint32_t                m_bOnOff        :  1;
        uint32_t                                :  5;
        uint32_t                m_bPresence     :  1;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_coloravg_t;

// ---------------------------------------------------------------------------
// general purpose data buffer

#define DC1394_AVT_CSRADV_GPDATABUFFER_SIZE    2048

typedef union __dc1394_avt_csradv_gpdatainfo_union
{
    struct __tagCsrAdvGpDataInfoElems
    {
        uint32_t                m_nSize         : 16;
        uint32_t                                : 16;
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_gpdatainfo_t;

typedef union __dc1394_avt_csradv_gpdatabuffer_union
{
    uint16_t                    m_int16[DC1394_AVT_CSRADV_GPDATABUFFER_SIZE/sizeof(uint16_t)];
    uint32_t                    m_int32[DC1394_AVT_CSRADV_GPDATABUFFER_SIZE/sizeof(uint32_t)];
} dc1394_avt_csradv_gpdatabuffer_t;

// ---------------------------------------------------------------------------
//
typedef struct __dc1394_avt_csradv_frameinfo_struct
{
    struct __tagCsrAdvFrameInfoElems
    {
        uint32_t                                        : 30;
        uint32_t                m_bClearFrameCounter    :  1;
        uint32_t                m_bPresence             :  1;
    }                           m_Cmd;

    uint32_t                    m_nFrameCounter;

} dc1394_avt_csradv_frameinfo_t;

/*
 *  IBIS5x HDR control register - extended for the MT9V022
 *
 *  @remarks The MT9V022 HDR mode requires multiple values per knee-point
 *  @arg 3x voltage and 1x shutter value for a single knee-point
 *  @arg 4x voltage and 2x shutter values for two knee-points
 */
typedef struct __dc1394_avt_csradv_hdr_struct
{
    struct __tagCsrAdvHdrElems
    {
        uint32_t                m_nKneePoints       :  4;       //!< selected number of knee-points
        uint32_t                                    :  4;
        uint32_t                m_nMaxKneePoints    :  4;       //!< max supported number of knee-points
        uint32_t                                    : 13;
        uint32_t                m_bOnOff            :  1;       //!< HDR mode on/off
        uint32_t                                    :  5;
        uint32_t                m_bPresence         :  1;       //!< feature presence
    }                           m_Cmd;

    struct __tagCsrAdvHdrKneePoints
    {
        uint16_t                m_nTime;                        //!< knee-point time
        uint8_t                 m_nVoltage2;                    //!< knee-point voltage 2, 4, 6
        uint8_t                 m_nVoltage1;                    //!< knee-point voltage 1, 3, 5
    }                           m_kneePoint[3];

} dc1394_avt_csradv_hdr_t;

/*
 *  Pike HDR control register
 */
typedef struct __dc1394_avt_csradv_hdr_pike_struct
{
    struct __tagCsrAdvHdrPikeElems
    {
        uint32_t                m_nKneeVoltage      :  8;       //!< reset voltage
        uint32_t                                    : 17;
        uint32_t                m_bOnOff            :  1;       //!< HDR mode on/off
        uint32_t                                    :  5;
        uint32_t                m_bPresence         :  1;       //!< feature presence
    }                           m_Cmd;

    uint32_t                    m_kneeTime1us;                  //!
} dc1394_avt_csradv_hdr_pike_t;

// ---------------------------------------------------------------------------
// Global Reset Release shutter mode register
typedef struct __dc1394_avt_csradv_global_res_rel_shutter_struct
{
    struct __tagCsrAdvGlobalResRelShutterElems
    {
        uint32_t                                 :  25;
        uint32_t                m_bOnOff         :  1;      //!< enable global reset release shutter
        uint32_t                                 :  5;
        uint32_t                m_bPresence      :  1;      //!< presence of this feature
    }                           m_Ctrl;
} dc1394_avt_csradv_global_res_rel_shutter_t;

// ---------------------------------------------------------------------------
// color correction (8 quadlets)
typedef struct __dc1394_avt_csradv_colorcorrection_struct
{
    struct __tagCsrAdvColorCorrectionElems
    {
        uint32_t                                    : 24;
        uint32_t                m_bReset            :  1;
        uint32_t                m_bOff              :  1;
        uint32_t                                    :  5;
        uint32_t                m_bPresence         :  1;
    }                           m;

    struct __tagCsrAdvColorCorrectionMatrix
    {
        uint32_t                m_nR1[3];
        uint32_t                m_nR2[3];
        uint32_t                m_nR3[3];
    }                           m_Matrix;
} dc1394_avt_csradv_colorcorrection_t;

// ---------------------------------------------------------------------------
// AutoShutterControl
typedef struct __dc1394_avt_csradv_autoshutter_struct
{
    struct __tagCsrAdvAutoShutterElems
    {
        uint32_t                                    : 31;
        uint32_t                m_bPresence         :  1;
    }                           m;

    struct __tagCsrAdvAutoShutterMinElems
    {
        uint32_t                m_nValue            : 26;
        uint32_t                                    :  6;
    }                           m_Low;

    struct __tagCsrAdvAutoShutterMaxElems
    {
        uint32_t                m_nValue            : 26;
        uint32_t                                    :  6;
    }                           m_High;
} dc1394_avt_csradv_autoshutter_t;

// ---------------------------------------------------------------------------
// AutoGainControl
typedef union __dc1394_avt_csradv_autogain_union
{
    struct __tagCsrAdvAutoGainElems
    {
        uint32_t                m_nLoVal            : 12;
        uint32_t                                    :  4;
        uint32_t                m_nHiVal            : 12;
        uint32_t                                    :  3;
        uint32_t                m_bPresence         :  1;
    }                           m;

    uint32_t                    m_All;
} dc1394_avt_csradv_autogain_t;

// ---------------------------------------------------------------------------
// AutoFeatureAOI
typedef struct __dc1394_avt_csradv_autofnc_aoi_struct
{
    struct __tagCsrAdvAutoFncAOIElems
    {
        uint32_t                m_nXUnits           :  12;  //!< allowed position & size increment
        uint32_t                m_nYUnits           :  12;  //!< allowed position & size increment
        uint32_t                                    :  1;
        uint32_t                m_bOnOff            :  1;
        uint32_t                                    :  1;
        uint32_t                m_bShowWorkArea     :  1;
        uint32_t                                    :  3;
        uint32_t                m_bPresence         :  1;
    }                           m_Ctrl;

    dc1394_avt_imagepos_t                  m_ImagePos;                 //!< area position
    dc1394_avt_imagesize_t                 m_ImageSize;                //!< area size
} dc1394_avt_csradv_autofnc_aoi_t;

// ---------------------------------------------------------------------------
// Low Smear feature
typedef struct __dc1394_avt_csradv_low_smear_struct
{
    struct __tagCsrAdvLowSmearElems
    {
        uint32_t                                    :  25;
        uint32_t                m_bOnOff            :  1;
        uint32_t                                    :  5;
        uint32_t                m_bPresence         :  1;
    }                           m_Ctrl;
} dc1394_avt_csradv_low_smear_t;

/**
 *  Image mirror register definition.
 *
 *  @remarks
 *  \arg The former name of this register was MISC_FEATURES
 *  \arg The m_MNR member has been kept for compatibility with older Marlin
 *  software versions. Nevertheless the MNR features has been removed from
 *  recent Marlin software versions.
 */
typedef struct __dc1394_avt_csradv_image_mirror_struct
{
    struct __tagCsrAdvImageMirrorElems
    {
        uint32_t                                    :  14;
        uint32_t                m_bVertInq          :  1;   //!< vertical mirror presence
        uint32_t                m_bHorzInq          :  1;   //!< horizontal mirror presence
        uint32_t                                    :  8;
        uint32_t                m_bVertOnOff        :  1;   //!< vertical mirror on/off
        uint32_t                m_bHorzOnOff        :  1;   //!< horizontal mirror on/off
        uint32_t                                    :  5;
        uint32_t                m_bPresence         :  1;   //!< feature presence
    }                           m_Mirror;

    //! preserve compatibility with Marlin series
    uint32_t                    m_MNR;

} dc1394_avt_csradv_image_mirror_t;

// ---------------------------------------------------------------------------
//
typedef struct __dc1394_avt_csradv_high_snr_struct
{
    union __tagCsrAdvHighSNRCtrl
    {
        struct __tagCsrAdvHighSNRCtrlElems
        {
            uint32_t            m_nGrabCount    :  9;
            uint32_t                            : 16;
            uint32_t            m_bOnOff        :  1;
            uint32_t                            :  5;
            uint32_t            m_bPresence     :  1;
        }                       m;
        uint32_t                m_nAll;
    }                           m_Ctrl;

    uint16_t                    m_nReserved[4];
} dc1394_avt_csradv_high_snr_t;

typedef struct __dc1394_avt_csradv_imagestamp_struct
{
    union __tagCsrAdvImageStampCtrl
    {
        struct __tagCsrAdvImageStampElems
        {
            uint32_t            m_nLinePos              : 16;   // signed line position of imagestamp
            uint32_t            m_nTMode                :  3;   // format/mode of imagestamp data
            uint32_t            m_bMode4Inq             :  1;   // b12: Mode 4 present
            uint32_t            m_bMode3Inq             :  1;   // b11: Mode 3 present
            uint32_t            m_bMode2Inq             :  1;   // b10: Mode 2 present
            uint32_t            m_bMode1Inq             :  1;   // b09: Mode 1 present
            uint32_t            m_bMode0Inq             :  1;   // b08: Mode 0 present -> GPLX CYCLE_TIMER
            uint32_t            m_bSwap                 :  1;   // b07: byte swap of timestamp data
            uint32_t            m_bOnOff                :  1;   // b06: enable insertion of timestamp data into image data
            uint32_t                                    :  4;
            uint32_t            m_bReset                :  1;   // b01: reset counter
            uint32_t            m_bPresence             :  1;   // b00: presence of this feature
        }                       m;
        uint32_t                m_nAll;
    }                           m_Ctrl;

    uint32_t                    m_nStampData;

} dc1394_avt_csradv_imagestamp_t;

/**
 * 	SIS control structure
 */
typedef struct __dc1394_avt_csradv_sis_struct
{
    union __tagCsrAdvSisCtrl
    {
        struct __tagCsrAdvSisElems
        {
            uint32_t                m_nLineNo       : 16;   //! SIS data position inside an image
            uint32_t                                :  9;
            uint32_t                m_bOnOff        :  1;   //! SIS on/off
            uint32_t                                :  5;
            uint32_t                m_bPresence     :  1;   //!< Presence of this feature
        }                           m;
        uint32_t                    m_nAll;
    }                               m_Ctrl;

    uint32_t                        m_nUserVal;             //!< user provided SIS value
} dc1394_avt_csradv_sis_t;

// ---------------------------------------------------------------------------
typedef union __dc1394_avt_csradv_swfeature_union
{
        struct __tagCsrAdvSwFeatureCtrlElems
        {
            uint32_t                            : 14;
            uint32_t            m_bBlankLED     :  1;   //!< BlankLED on/off
            uint32_t                            :  1;
            uint32_t                            : 14;
            uint32_t            m_bBlankLED_Inq :  1;   //!< BlankLED inquiry
            uint32_t            m_bPresence     :  1;   //!< Presence of this feature
        }                       m;
        uint32_t                m_nAll;
} dc1394_avt_csradv_swfeature_t;
// ---------------------------------------------------------------------------
//
typedef union __dc1394_avt_csradv_userprofile_union
{
    struct __tagCsrAdvUserProfileElems
    {
        uint32_t                m_nProfileID    :  4;   //!< load/save from/to profile with the id ID
        uint32_t                                :  4;
        uint32_t                m_nErrCode      :  4;   //!<
        uint32_t                                :  9;
        uint32_t                m_bSetAsDef     :  1;   //!< set Profile ID as default
        uint32_t                m_bMemLoad      :  1;   //!< load settings
        uint32_t                m_bMemSave      :  1;   //!< save settings
        uint32_t                m_bBusy         :  1;   //!< load/save operation in progress
        uint32_t                                :  5;
        uint32_t                m_bError        :  1;   //!< load/save error
        uint32_t                m_bPresence     :  1;   //!< Presence of this feature
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_userprofile_t;

/**
 *  Extended/mapped format 7 modes
 */
typedef struct __dc1394_avt_csradv_f7mode_mapping_struct
{
    union __tagCsrAdvExtdF7ModesElems
    {
        struct __tagCsrAdvExtdF7ModesBitElems
        {
            uint32_t                            : 25;
            uint32_t            m_bOnOff        :  1;   //!< enable this feature
            uint32_t                            :  5;
            uint32_t            m_bPresence     :  1;   //!< presence of this feature
        }                       m;
        uint32_t                m_nAll;
    }                           m_Ctrl;

    uint32_t                    m_Inq[2];                 //!< available extended modes
    uint32_t                    m_Gap3;
                                                        //! mapped format 7 modes
    uint32_t                    m_nMappedModes[DC1394_AVT_NUM_FORMAT7_MODES];
} dc1394_avt_csradv_f7mode_mapping_t;

/**
 *  Advanced feature to override the maximum ISO packet size. Because we
 *  want speed, we don't obey S100 and S200 here...
 */
typedef struct __dc1394_avt_csradv_max_isosize_struct
{
    union __tagCsrAdvMaxIsoSizeElems
    {
        struct __tagCsrAdvIsoSizeBitElems
        {
            uint32_t            m_nMaxSize      : 16;    //!< MaxBytePerPacket for format 7 modes
            uint32_t                            :  8;
            uint32_t            m_bSet2Max      :  1;    //!< set to maximum supported packet size
            uint32_t            m_bOnOff        :  1;    //!< enable MaxBytePerPacket
            uint32_t                            :  5;
            uint32_t            m_bPresence     :  1;    //!< presence of this feature
        }                       m;
        uint32_t                m_nAll;
    }                           m_S400,                    //!< ISO packet size S400
                                m_S800,                    //!< ISO packet size S800
                                m_S1600,
                                m_S3200;
} dc1394_avt_csradv_max_isosize_t;

/**
 *  Advanced parameter update control
 *  This feature controls the update behaviour of the camera between the
 *  host and the uC and also between the uC and the FPGA. It controls when
 *  and how new parameters become active
 */
typedef union __dc1394_avt_csradv_paramupd_timing_union
{
    struct __tagCsrAdvParamUpdTimingElems
    {
        uint32_t                m_nFpgaUpdMode  :  8;   //!< FPGA parameter update behaviour
        uint32_t                                : 17;
        uint32_t                m_bUpdActive    :  1;   //!< indicates an parameter update cycle
        uint32_t                                :  5;
        uint32_t                m_bPresence     :  1;   //!< Presence of this feature
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_paramupd_timing_t;

/**
 *  Advanced white balance - extends the standard white balance feature to 4
 *  independent color controllers
 */
typedef struct __dc1394_avt_csradv_whitebal_struct
{
    // advanced white balance inquiry
    dc1394_avt_csr_whitebal_inq_t           m_Inq;

    // independent color controllers
    union __tagAdvWhiteBal1
    {
        struct __tagAdvWhiteBal1Elems
        {
            uint32_t            m_nPxga1        : 12;   //!< PXGA 1 or 3
            uint32_t            m_nPxga2        : 12;   //!< PXGA 2 or 4
            uint32_t            m_bAMMode       :  1;   //!< auto mode on/off
            uint32_t                            :  1;
            uint32_t            m_bOnePush      :  1;   //!< one push operation
            uint32_t                            :  3;
            uint32_t                            :  1;
            uint32_t            m_bPresence     :  1;   //!< feature presence
        }                       m;
        uint32_t                m_nAll;
    }                           m_WhiteBal12,           //!< white balance control PXGA 1+2
                                m_WhiteBal34;           //!< white balance control PXGA 3+4
} dc1394_avt_csradv_whitebal_t;

/**
 *  Advanced parameter list info
 *  @remarks GPDATA_BUFFER and PARAMLIST_BUFFER share the same memory!
 */
typedef union __dc1394_avt_csradv_paramlist_info_union
{
    struct __tagCsrAdvParamListInfoElems
    {
        uint32_t                m_nSize         : 16;   //!< size of the parameter list buffer in bytes
        uint32_t                m_nLastItem     :  8;   //!< position of last processed item
        uint32_t                m_nErrorCode    :  4;   //!< parameter list operation errors
        uint32_t                                :  3;
        uint32_t                m_bPresence     :  1;   //!< feature presence
    }                           m;
    uint32_t                    m_nAll;
} dc1394_avt_csradv_paramlist_info_t;

#define DC1394_AVT_PARAMLIST_ADDRESS_ERR     1       //!< unknown CSR address
#define DC1394_AVT_PARAMLIST_CSR_NWR_ERR     2       //!< CSR not writable
#define DC1394_AVT_PARAMLIST_CSR_VOID_ERR    3       //!< CSR not allowed here
#define DC1394_AVT_PARAMLIST_CSR_NMEM_ERR    4       //!< CSR has not writable memory





#endif /// #ifndef  __CSR_STRUCTS_ADV_H
