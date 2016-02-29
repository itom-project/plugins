project(itom_plugins)

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET(ITOM_DIR "" CACHE PATH "base path to itom") 

# AerotechA3200
IF (WIN32)
    OPTION(PLUGIN_aerotechA3200 "Build with this plugin." ON)
    if (PLUGIN_aerotechA3200)
            ADD_SUBDIRECTORY(AerotechA3200)
    endif (PLUGIN_aerotechA3200)
ENDIF (WIN32)

# AerotechEnsemble
IF (WIN32)
    OPTION(PLUGIN_aerotechEnsemble "Build with this plugin." ON)
    if (PLUGIN_aerotechEnsemble)
            ADD_SUBDIRECTORY(AerotechEnsemble)
    endif (PLUGIN_aerotechEnsemble)
ENDIF (WIN32)

# AndorSDK3
OPTION(PLUGIN_andorSDK3 "Build with this plugin." ON)
if (PLUGIN_andorSDK3)
        ADD_SUBDIRECTORY(AndorSDK3)
endif (PLUGIN_andorSDK3)

# AvantesAvaSpec
OPTION(PLUGIN_AvantesAvaSpec "Build with this plugin." ON)
if (PLUGIN_AvantesAvaSpec)
        ADD_SUBDIRECTORY(AvantesAvaSpec)
endif (PLUGIN_AvantesAvaSpec)

# AVTVimba
IF (WIN32)
    OPTION(PLUGIN_AVTVimba "Build with this plugin." ON)
    if (PLUGIN_AVTVimba)
            ADD_SUBDIRECTORY(AVTVimba)
    endif (PLUGIN_AVTVimba)
ENDIF (WIN32)

# cmu1394
IF (WIN32)
    OPTION(PLUGIN_cmu1394 "Build with this plugin." ON)
    if (PLUGIN_cmu1394)
            ADD_SUBDIRECTORY(cmu1394)
    endif (PLUGIN_cmu1394)
ENDIF (WIN32)

# CommonVisionBlox
IF (WIN32)
    OPTION(PLUGIN_CommonVisionBlox "Build with this plugin." OFF)
    if (PLUGIN_CommonVisionBlox)
            ADD_SUBDIRECTORY(CommonVisionBlox)
    endif (PLUGIN_CommonVisionBlox)
ENDIF (WIN32)

# dataobjectarithmetic
OPTION(PLUGIN_dataobjectarithmetic "Build with this plugin." ON)
if (PLUGIN_dataobjectarithmetic)
    ADD_SUBDIRECTORY(dataobjectarithmetic)
endif (PLUGIN_dataobjectarithmetic)

# DataObjectIO
OPTION(PLUGIN_DataObjectIO "Build with this plugin." ON)
if (PLUGIN_DataObjectIO)
    ADD_SUBDIRECTORY(DataObjectIO)
endif (PLUGIN_DataObjectIO)

# dispWindow
OPTION(PLUGIN_dispWindow "Build with this plugin." ON)
if (PLUGIN_dispWindow)
    ADD_SUBDIRECTORY(dispWindow)
endif (PLUGIN_dispWindow)

# dslrRemote - not yet
OPTION(PLUGIN_DslrRemote "Build with this plugin." ON)
if (PLUGIN_DslrRemote)
    ADD_SUBDIRECTORY(DslrRemote)
endif (PLUGIN_DslrRemote)

# DummyGrabber
OPTION(PLUGIN_DummyGrabber "Build with this plugin." ON)
if (PLUGIN_DummyGrabber)
    ADD_SUBDIRECTORY(DummyGrabber)
endif (PLUGIN_DummyGrabber)

# DummyMotor
OPTION(PLUGIN_DummyMotor "Build with this plugin." ON)
if (PLUGIN_DummyMotor)
    ADD_SUBDIRECTORY(DummyMotor)
endif (PLUGIN_DummyMotor)

# DemoAlgorithms
OPTION(PLUGIN_DemoAlgorithms "Build with this plugin." ON)
if (PLUGIN_DemoAlgorithms)
    ADD_SUBDIRECTORY(DemoAlgorithms)
endif (PLUGIN_DemoAlgorithms)

# FFTW-Wrapper
OPTION(PLUGIN_FFTWFilters "Build with this plugin." ON)
if (PLUGIN_FFTWFilters)
    ADD_SUBDIRECTORY(FFTWfilters)
endif (PLUGIN_FFTWFilters)

# FileGrabber
OPTION(PLUGIN_FileGrabber "Build with this plugin." ON)
if (PLUGIN_FileGrabber)
    ADD_SUBDIRECTORY(FileGrabber)
endif (PLUGIN_FileGrabber)

# FittingFilters
OPTION(PLUGIN_FittingFilters "Build with this plugin." ON)
if (PLUGIN_FittingFilters)
    ADD_SUBDIRECTORY(FittingFilters)
endif (PLUGIN_FittingFilters)

# FireGrabber
OPTION(PLUGIN_FireGrabber "Build with this plugin." ON)
if (PLUGIN_FireGrabber)
    ADD_SUBDIRECTORY(FireGrabber)
endif (PLUGIN_FireGrabber)

# FringeProj
OPTION(PLUGIN_FringeProj "Build with this plugin." ON)
if (PLUGIN_FringeProj)
    ADD_SUBDIRECTORY(FringeProj)
endif (PLUGIN_FringeProj)

# glDisplay
OPTION(PLUGIN_GLDisplay "Build with this plugin." ON)
if (PLUGIN_GLDisplay)
    ADD_SUBDIRECTORY(glDisplay)
endif (PLUGIN_GLDisplay)

# GenICam
#IF (WIN32)
#    OPTION(PLUGIN_GenICam "Build with this plugin." OFF)
#    if (PLUGIN_GenICam)
#        ADD_SUBDIRECTORY(GenICam)
#    endif (PLUGIN_GenICam)
#ENDIF (WIN32)

# GWInstekPSP
OPTION(PLUGIN_GWInstekPSP "Build with this plugin." ON)
if (PLUGIN_GWInstekPSP)
    ADD_SUBDIRECTORY(GWInstekPSP)
endif (PLUGIN_GWInstekPSP)

# IDSuEye
OPTION(PLUGIN_IDSuEye "Build with this plugin." ON)
if (PLUGIN_IDSuEye)
    ADD_SUBDIRECTORY(IDSuEye)
endif (PLUGIN_IDSuEye)

# BasicFilters
OPTION(PLUGIN_BasicFilters "Build with this plugin." ON)
if (PLUGIN_BasicFilters)
    ADD_SUBDIRECTORY(BasicFilters)
endif (PLUGIN_BasicFilters)

# BasicGPLFilters
OPTION(PLUGIN_BasicGPLFilters "Build with this plugin." ON)
if (PLUGIN_BasicGPLFilters)
    ADD_SUBDIRECTORY(BasicGPLFilters)
endif (PLUGIN_BasicGPLFilters) 

# BasicGPLFilters
OPTION(PLUGIN_HidApi "Build with this plugin." ON)
if (PLUGIN_HidApi)
    ADD_SUBDIRECTORY(hidapi)
endif (PLUGIN_HidApi) 

# LeicaMotorFocus
OPTION(PLUGIN_LeicaMotorFocus "Build with this plugin." ON)
if (PLUGIN_LeicaMotorFocus)
    ADD_SUBDIRECTORY(LeicaMotorFocus)
endif (PLUGIN_LeicaMotorFocus)

# libmodbus
OPTION(PLUGIN_LibModbus "Build with this plugin." ON)
if (PLUGIN_LibModbus)
    ADD_SUBDIRECTORY(libmodbus)
endif (PLUGIN_LibModbus)

# LibUSB
OPTION(PLUGIN_LIBUSB "Build with this plugin." ON)
if (PLUGIN_LIBUSB)
    ADD_SUBDIRECTORY(LibUSB)
endif (PLUGIN_LIBUSB)

# MeasurementComputing
IF(WIN32)
OPTION(PLUGIN_MeasurementComputing "Build with this plugin." ON)
if (PLUGIN_MeasurementComputing)
	ADD_SUBDIRECTORY(MeasurementComputing)
endif (PLUGIN_MeasurementComputing)
ENDIF(WIN32)

# MSMediaFoundation
IF(WIN32)
OPTION(PLUGIN_MSMediaFoundation "Build with this plugin." ON)
if (PLUGIN_MSMediaFoundation)
    ADD_SUBDIRECTORY(MSMediaFoundation)
endif (PLUGIN_MSMediaFoundation)
ENDIF(WIN32)

# NanotecStepMotor
OPTION(PLUGIN_NanotecStepMotor "Build with this plugin." ON)
if (PLUGIN_NanotecStepMotor)
    ADD_SUBDIRECTORY(NanotecStepMotor)
endif (PLUGIN_NanotecStepMotor)

# NewportSMC100
OPTION(PLUGIN_NEWPORT_SMC100 "Build with this plugin." ON)
if (PLUGIN_NEWPORT_SMC100)
    ADD_SUBDIRECTORY(NewportSMC100)
endif (PLUGIN_NEWPORT_SMC100)

# NI-DAQmx
OPTION(PLUGIN_NI-DAQmx "Build with this plugin." OFF)
if (PLUGIN_NI-DAQmx)
    ADD_SUBDIRECTORY(NI-DAQmx)
endif (PLUGIN_NI-DAQmx)

# OpenCVFilters
OPTION(PLUGIN_OpenCVFilters "Build with this plugin." ON)
if (PLUGIN_OpenCVFilters)
    ADD_SUBDIRECTORY(OpenCVFilters)
endif (PLUGIN_OpenCVFilters)

# OpenCVFilters(nonfree)
OPTION(PLUGIN_OpenCVFilters_Nonfree "Build with this plugin." OFF)
if (PLUGIN_OpenCVFilters_Nonfree)
    ADD_SUBDIRECTORY(OpenCVFiltersNonFree)
endif (PLUGIN_OpenCVFilters_Nonfree)

# OpenCVGrabber
OPTION(PLUGIN_OpenCVGrabber "Build with this plugin." ON)
if (PLUGIN_OpenCVGrabber)
    ADD_SUBDIRECTORY(OpenCVGrabber)
endif (PLUGIN_OpenCVGrabber)

# PclTools
OPTION(PLUGIN_PclTools "Build with this plugin." ON)
if (PLUGIN_PclTools)
  ADD_SUBDIRECTORY(PclTools)
endif (PLUGIN_PclTools)

# PCOCamera
IF (WIN32)
  OPTION(PLUGIN_PCOCamera "Build with this plugin." ON)
  if (PLUGIN_PCOCamera)
      ADD_SUBDIRECTORY(PCOCamera)
  endif (PLUGIN_PCOCamera)
ENDIF (WIN32)

# PCOSensicam
IF (WIN32)
  OPTION(PLUGIN_PCOSensicam "Build with this plugin." ON)
  if (PLUGIN_PCOSensicam)
      ADD_SUBDIRECTORY(PCOSensicam)
  endif (PLUGIN_PCOSensicam)
ENDIF (WIN32)

# PCOPixelFly
IF (WIN32)
  OPTION(PLUGIN_PCOPixelFly "Build with this plugin." ON)
  if (PLUGIN_PCOPixelFly)
      ADD_SUBDIRECTORY(PCOPixelFly)
  endif (PLUGIN_PCOPixelFly)
ENDIF (WIN32)

# PiezosystemJena_NV40_1
OPTION(PLUGIN_PiezosystemJena_NV40_1 "Build with this plugin." ON)
if (PLUGIN_PiezosystemJena_NV40_1)
  ADD_SUBDIRECTORY(PiezosystemJena_NV40_1)
endif (PLUGIN_PiezosystemJena_NV40_1)

# PGRFlyCapture
OPTION(PLUGIN_PGRFlyCapture "Build with this plugin." ON)
if (PLUGIN_PGRFlyCapture)
    ADD_SUBDIRECTORY(PGRFlyCapture)
endif (PLUGIN_PGRFlyCapture)

# PIPiezoCtrl
OPTION(PLUGIN_PIPiezoCtrl "Build with this plugin." ON)
if (PLUGIN_PIPiezoCtrl)
    ADD_SUBDIRECTORY(PIPiezoCtrl)
endif (PLUGIN_PIPiezoCtrl)

# PI_GCS2
OPTION(PLUGIN_PI_GCS2 "Build with this plugin." ON)
if (PLUGIN_PI_GCS2)
    ADD_SUBDIRECTORY(PI_GCS2)
endif (PLUGIN_PI_GCS2)

# QCam
IF (WIN32)
    OPTION(PLUGIN_QCam "Build with this plugin." OFF)
    if (PLUGIN_QCam)
        ADD_SUBDIRECTORY(QCam)
    endif (PLUGIN_QCam)
ENDIF (WIN32)

# rawImport
OPTION(PLUGIN_rawImport "Build with this plugin." ON)
if (PLUGIN_rawImport)
    ADD_SUBDIRECTORY(RawImport)
endif (PLUGIN_rawImport)

# SerialIO
OPTION(PLUGIN_SerialIO "Build with this plugin." ON)
if (PLUGIN_SerialIO)
    ADD_SUBDIRECTORY(SerialIO)
endif (PLUGIN_SerialIO)

# ST8SMC4USB
IF (WIN32)
    OPTION(PLUGIN_ST8SMC4USB "Build with this plugin." ON)
    if (PLUGIN_ST8SMC4USB)
            ADD_SUBDIRECTORY(ST8SMC4USB)
    endif (PLUGIN_ST8SMC4USB)
ENDIF (WIN32)

# SuperlumBS
OPTION(PLUGIN_SuperlumBS "Build with this plugin." ON)
if (PLUGIN_SuperlumBS)
    ADD_SUBDIRECTORY(SuperlumBS)
endif (PLUGIN_SuperlumBS)

# SuperlumBL
OPTION(PLUGIN_SuperlumBL "Build with this plugin." ON)
if (PLUGIN_SuperlumBL)
    ADD_SUBDIRECTORY(SuperlumBL)
endif (PLUGIN_SuperlumBL)

# ThorlabsCCS Spectrometer
IF (WIN32)
    OPTION(PLUGIN_ThorlabsCCS "Build with this plugin." ON)
    if (PLUGIN_ThorlabsCCS)
        ADD_SUBDIRECTORY(ThorlabsCCS)
    endif (PLUGIN_ThorlabsCCS)
ENDIF (WIN32)

# UhlRegister
OPTION(PLUGIN_UhlRegister "Build with this plugin." ON)
if (PLUGIN_UhlRegister)
    ADD_SUBDIRECTORY(UhlRegister)
endif (PLUGIN_UhlRegister)

# UhlText
OPTION(PLUGIN_UhlText "Build with this plugin." ON)
if (PLUGIN_UhlText)
    ADD_SUBDIRECTORY(UhlText)
endif (PLUGIN_UhlText)

# USBMotion3XIII
IF (WIN32)
    OPTION(PLUGIN_USBMotion3XIII "Build with this plugin." ON)
    if (PLUGIN_USBMotion3XIII)
        ADD_SUBDIRECTORY(USBMotion3XIII)
    endif (PLUGIN_USBMotion3XIII)
ENDIF (WIN32)

# Xeneth
IF (WIN32)
    OPTION(PLUGIN_Xeneth "Build with this plugin." ON)
    if (PLUGIN_Xeneth)
        ADD_SUBDIRECTORY(Xeneth)
    endif (PLUGIN_Xeneth)
ENDIF (WIN32)

# V4L2
IF (UNIX AND NOT APPLE)
    OPTION(PLUGIN_V4L2 "Build with this plugin." ON)
    if (PLUGIN_V4L2)
        ADD_SUBDIRECTORY(V4L2)
    endif (PLUGIN_V4L2)
ENDIF (UNIX AND NOT APPLE)

# Vistek
IF (WIN32)
    OPTION(PLUGIN_Vistek "Build with this plugin." ON)
    if (PLUGIN_Vistek)
        ADD_SUBDIRECTORY(Vistek)
    endif (PLUGIN_Vistek)
ENDIF (WIN32)

# VRMagic
IF (WIN32)
    OPTION(PLUGIN_VRMagic "Build with this plugin." ON)
    if (PLUGIN_VRMagic)
        ADD_SUBDIRECTORY(VRMagic)
    endif (PLUGIN_VRMagic)
ENDIF (WIN32)

# XIMEA
OPTION(PLUGIN_XIMEA "Build with this plugin." ON)
if (PLUGIN_XIMEA)
    ADD_SUBDIRECTORY(Ximea)
endif (PLUGIN_XIMEA)

# x3pio
OPTION(PLUGIN_x3pio "Build with this plugin." ON)
if (PLUGIN_x3pio)
    ADD_SUBDIRECTORY(x3pio)
endif (PLUGIN_x3pio)
