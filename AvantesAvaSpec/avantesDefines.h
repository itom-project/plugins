/* ********************************************************************
    Plugin "AvantesAvaSpec" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef AVANTESDEFINES_H
#define AVANTESDEFINES_H

#pragma pack(push,1)

typedef signed char     int8;
typedef unsigned char   uint8;
typedef signed short    int16;
typedef unsigned short  uint16;
typedef unsigned int    uint32;

uint8 const     USER_ID_LEN             = 64;
uint8 const     NR_WAVELEN_POL_COEF     = 5;
uint8 const     NR_NONLIN_POL_COEF      = 8;
uint8 const     NR_DEFECTIVE_PIXELS     = 30;
uint16 const    MAX_NR_PIXELS           = 4096;
uint8 const     NR_TEMP_POL_COEF        = 5;
uint8 const     MAX_TEMP_SENSORS        = 3;
uint8 const     ROOT_NAME_LEN           = 6;
uint8 const     AVS_SERIAL_LEN          = 9; //was 10, but AvaSpec-3648 and AvaSpec-ULS3648 showed only 9 bit
uint16 const    MAX_PIXEL_VALUE         = 0xFFFC;
uint8 const     MAX_VIDEO_CHANNELS      = 2;
uint16 const    MAX_LASER_WIDTH         = 0xFFFF;
uint8 const     HW_TRIGGER_MODE            = 1;
uint8 const     SW_TRIGGER_MODE            = 0;
uint8 const     EDGE_TRIGGER_SOURCE      = 0;
uint8 const     LEVEL_TRIGGER_SOURCE    = 1;
uint8 const     MAX_TRIGGER_MODE        = 1;
uint8 const     MAX_TRIGGER_SOURCE      = 1;
uint8 const     MAX_TRIGGER_SOURCE_TYPE = 1;
uint32 const    MAX_INTEGRATION_TIME    = 600000;    // 600 seconds
uint8 const     SAT_DISABLE_DET         = 0;
uint8 const     SAT_ENABLE_DET          = 1;
uint8 const     SAT_PEAK_INVERSION      = 2;
uint8 const     NR_DAC_POL_COEF         = 2;
uint8 const     MAX_NR_DARKPIXELS       = 20;

typedef struct
{
    uint16                  m_StrobeControl;
    uint32                  m_LaserDelay;
    uint32                  m_LaserWidth;
    float                   m_LaserWaveLength;
    uint16                  m_StoreToRam;
} ControlSettingsType;

typedef struct
{
    uint8                   m_Enable;
    uint8                   m_ForgetPercentage;
} DarkCorrectionType;

typedef enum
{
    SENS_HAMS8378_256 = 1,
    SENS_HAMS8378_1024,
    SENS_ILX554,
    SENS_HAMS9201,
    SENS_TCD1304,
    SENS_TSL1301,
    SENS_TSL1401,
    SENS_HAMS8378_512,
    SENS_HAMS9840,
    SENS_ILX511,
    SENS_HAMS10420_2048X64,
    SENS_HAMS11071_2048X64,
    SENS_HAMS7031_1024X122,
    SENS_HAMS7031_1024X58,
    SENS_HAMS11071_2048X16,
    SENS_HAMS11155,
    SENS_SU256LSB,
    SENS_SU512LDB,
    SENS_HAMS11638 = 21,
    SENS_HAMS11639 = 22,
    SENS_HAMS12443 = 23,
    SENS_HAMG9208_512 = 24,
    SENS_HAMG13913 = 25,
    SENS_HAMS13496 = 26
} SensorType;

typedef struct
{
    uint8                   m_SensorType; //value is of enumeration SensorType
    uint16                  m_NrPixels;
    float                   m_aFit[NR_WAVELEN_POL_COEF];
    bool                    m_NLEnable;
    double                  m_aNLCorrect[NR_NONLIN_POL_COEF];
    double                  m_aLowNLCounts;
    double                  m_aHighNLCounts;
    float                   m_Gain[MAX_VIDEO_CHANNELS];
    float                   m_Reserved;
    float                   m_Offset[MAX_VIDEO_CHANNELS];
    float                   m_ExtOffset;
    uint16                  m_DefectivePixels[NR_DEFECTIVE_PIXELS];
} DetectorType;

typedef struct
{
    uint16                  m_SmoothPix;
    uint8                   m_SmoothModel;
} SmoothingType;

typedef struct
{
    SmoothingType           m_Smoothing;
    float                   m_CalInttime;
    float                   m_aCalibConvers[MAX_NR_PIXELS];
} SpectrumCalibrationType;

typedef struct
{
    SpectrumCalibrationType m_IntensityCalib;
    uint8                   m_CalibrationType;
    uint32                  m_FiberDiameter;
} IrradianceType;

typedef struct
{
    uint8                   m_Mode;
    uint8                   m_Source;
    uint8                   m_SourceType;
} TriggerType;

typedef struct
{
    uint16                  m_StartPixel;
    uint16                  m_StopPixel;
    float                   m_IntegrationTime;
    uint32                  m_IntegrationDelay;
    uint32                  m_NrAverages;
    DarkCorrectionType      m_CorDynDark;
    SmoothingType           m_Smoothing;
    uint8                   m_SaturationDetection;
    TriggerType             m_Trigger;
    ControlSettingsType     m_Control;
} MeasConfigType;

// bugfix 09-09-08: MeasConfigType is nested in StandAloneType
// -> do not add prefix, define a new type

typedef struct
{
    uint8                   prefix[6];
    MeasConfigType          m_Meas;
} SendMeasConfigType;

typedef struct
{
    uint16                  m_Date;
    uint16                  m_Time;
} TimeStampType;

typedef struct
{
    bool                    m_Enable;
    uint8                   m_SpectrumType;
    char                    m_aFileRootName[ROOT_NAME_LEN];
    TimeStampType           m_TimeStamp;
} SDCardType;

typedef struct
{
    float                   m_aSpectrumCorrect[MAX_NR_PIXELS];
} SpectrumCorrectionType;

typedef struct
{
    bool                    m_Enable;
    MeasConfigType          m_Meas;
    int16                   m_Nmsr;
    SDCardType              m_SDCard;
} StandAloneType;

typedef struct
{
    float                   m_aFit[NR_TEMP_POL_COEF];
} TempSensorType;

typedef struct
{
    bool                    m_Enable;
    float                   m_Setpoint;     // [degree Celsius]
    float                   m_aFit[NR_DAC_POL_COEF];
} TecControlType;

typedef struct
{
    float                   AnalogLow[2];
    float                   AnalogHigh[2];
    float                   DigitalLow[10];
    float                   DigitalHigh[10];
} ProcessControlType;

uint16 const    SETTINGS_RESERVED_LEN   = ((62*1024) -  sizeof(uint32) -
                                                        (sizeof(uint16) +   // m_Len
                                                         sizeof(uint16) +  // m_ConfigVersion
                                                         USER_ID_LEN +
                                                         sizeof(DetectorType) +
                                                         sizeof(IrradianceType) +
                                                         sizeof(SpectrumCalibrationType) +
                                                         sizeof(SpectrumCorrectionType) +
                                                         sizeof(StandAloneType) +
                                                        (sizeof(TempSensorType)*MAX_TEMP_SENSORS) +
                                                         sizeof(TecControlType) +
                                                         sizeof(ProcessControlType)
                                                        )
                                           );


typedef struct
{
    uint8                   prefix[6];
    uint16                  m_Len;
    uint16                  m_ConfigVersion;
    char                    m_aUserFriendlyId[USER_ID_LEN];
    DetectorType            m_Detector;
    IrradianceType          m_Irradiance;
    SpectrumCalibrationType m_Reflectance;
    SpectrumCorrectionType  m_SpectrumCorrect;
    StandAloneType          m_StandAlone;
    TempSensorType          m_aTemperature[MAX_TEMP_SENSORS];
    TecControlType          m_TecControl;
    ProcessControlType      m_ProcessControl;
    uint8                   m_aReserved[SETTINGS_RESERVED_LEN];
} DeviceConfigType;

typedef struct
{
    uint8           prefix[6];
    uint32          version1;
    uint32          version2;
    uint32          version3;
    char            SerialNumber[AVS_SERIAL_LEN];
    char            UserFriendlyName[USER_ID_LEN];
    unsigned char   Status;
} AvsIdentityType;

typedef struct
{
    uint8    prefix[6];
    uint32    timestamp;
    uint16    pixels[3648 + MAX_NR_DARKPIXELS]; //the biggest detector has 3648 pixels, at the beginning there might be some dark-pixels (depending on the detector, therefore a buffer of up to 20px is reserved for this, too)
} AvsSingleMeasdata;

typedef struct
{
    uint8    prefix[6];
    uint32    timestamp;
    uint16    averages;
    uint32    pixels[3648 + MAX_NR_DARKPIXELS]; //the biggest detector has 3648 pixels, at the beginning there might be some dark-pixels (depending on the detector, therefore a buffer of up to 20px is reserved for this, too)
} AvsMultiMeasdata;

//----------------------------------------------------------------------------------------------------------------------------------
#pragma pack(pop)

#endif // AVANTESDEFINES_H
