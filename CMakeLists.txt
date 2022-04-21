project(itom_plugins)

cmake_minimum_required(VERSION 3.1...3.15)

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT ITOM_SDK_DIR)
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
        HINTS "C:/itom/build/itom/SDK"
              "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
        DOC "path of SDK subfolder of itom root (build) directory")
endif()

if(NOT ITOM_SDK_DIR)
    message(SEND_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()


#this is to automatically detect the SDK subfolder of the itom build directory.
find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "C:/itom/build/itom/SDK"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "path of SDK subfolder of itom root (build) directory")

if(NOT ITOM_SDK_DIR)
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif()

# AerotechA3200
if(WIN32)
    option(PLUGIN_aerotechA3200 "Build with this plugin." OFF)
    if(PLUGIN_aerotechA3200)
            add_subdirectory(AerotechA3200)
    endif(PLUGIN_aerotechA3200)
endif()

# AerotechEnsemble
if(WIN32)
    option(PLUGIN_aerotechEnsemble "Build with this plugin." OFF)
    if(PLUGIN_aerotechEnsemble)
            add_subdirectory(AerotechEnsemble)
    endif(PLUGIN_aerotechEnsemble)
endif()

# AndorSDK3
option(PLUGIN_andorSDK3 "Build with this plugin." OFF)
if(PLUGIN_andorSDK3)
        add_subdirectory(AndorSDK3)
endif(PLUGIN_andorSDK3)

# AvantesAvaSpec
option(PLUGIN_AvantesAvaSpec "Build with this plugin." OFF)
if(PLUGIN_AvantesAvaSpec)
        add_subdirectory(AvantesAvaSpec)
endif(PLUGIN_AvantesAvaSpec)

# AVTVimba
if(WIN32)
    option(PLUGIN_AVTVimba "Build with this plugin." OFF)
    if(PLUGIN_AVTVimba)
            add_subdirectory(AVTVimba)
    endif(PLUGIN_AVTVimba)
endif()

# BasicFilters
option(PLUGIN_BasicFilters "Build with this plugin." ON)
if(PLUGIN_BasicFilters)
    add_subdirectory(BasicFilters)
endif(PLUGIN_BasicFilters)

# BasicGPLFilters
option(PLUGIN_BasicGPLFilters "Build with this plugin." ON)
if(PLUGIN_BasicGPLFilters)
    add_subdirectory(BasicGPLFilters)
endif(PLUGIN_BasicGPLFilters) 

# HidApi
option(PLUGIN_HidApi "Build with this plugin." OFF)
if(PLUGIN_HidApi)
    add_subdirectory(hidapi)
endif(PLUGIN_HidApi) 

# cmu1394
if(WIN32)
    option(PLUGIN_cmu1394 "Build with this plugin." OFF)
    if(PLUGIN_cmu1394)
            add_subdirectory(cmu1394)
    endif(PLUGIN_cmu1394)
endif()

# CommonVisionBlox
if(WIN32)
    option(PLUGIN_CommonVisionBlox "Build with this plugin." OFF)
    if(PLUGIN_CommonVisionBlox)
            add_subdirectory(CommonVisionBlox)
    endif(PLUGIN_CommonVisionBlox)
endif()

# CyUSB
option(PLUGIN_CyUSB "Build with this plugin." OFF)
if(PLUGIN_CyUSB)
        add_subdirectory(CyUSB)
endif(PLUGIN_CyUSB)

# dataobjectarithmetic
option(PLUGIN_dataobjectarithmetic "Build with this plugin." ON)
if(PLUGIN_dataobjectarithmetic)
    add_subdirectory(dataobjectarithmetic)
endif(PLUGIN_dataobjectarithmetic)

# DataObjectIO
option(PLUGIN_DataObjectIO "Build with this plugin." ON)
if(PLUGIN_DataObjectIO)
    add_subdirectory(DataObjectIO)
endif(PLUGIN_DataObjectIO)

# Digital Image Correlation - DIC
option(PLUGIN_DIC "Build with this plugin." OFF)
if(PLUGIN_DIC)
    add_subdirectory(DIC)
endif(PLUGIN_DIC)

# dispWindow
option(PLUGIN_dispWindow "Build with this plugin." OFF)
if(PLUGIN_dispWindow)
    add_subdirectory(dispWindow)
endif(PLUGIN_dispWindow)

# dslrRemote - not yet
option(PLUGIN_DslrRemote2 "Build with this plugin." OFF)
if(PLUGIN_DslrRemote2)
    add_subdirectory(DslrRemote2)
endif(PLUGIN_DslrRemote2)

# dslrRemote - not yet
option(PLUGIN_DslrRemote "Build with this plugin." OFF)
if(PLUGIN_DslrRemote)
    add_subdirectory(DslrRemote)
endif(PLUGIN_DslrRemote)

# DummyGrabber
option(PLUGIN_DummyGrabber "Build with this plugin." ON)
if(PLUGIN_DummyGrabber)
    add_subdirectory(DummyGrabber)
endif(PLUGIN_DummyGrabber)

# DummyMotor
option(PLUGIN_DummyMotor "Build with this plugin." ON)
if(PLUGIN_DummyMotor)
    add_subdirectory(DummyMotor)
endif(PLUGIN_DummyMotor)

# DemoAlgorithms
option(PLUGIN_DemoAlgorithms "Build with this plugin." OFF)
if(PLUGIN_DemoAlgorithms)
    add_subdirectory(DemoAlgorithms)
endif(PLUGIN_DemoAlgorithms)

# FFTW-Wrapper
option(PLUGIN_FFTWFilters "Build with this plugin." ON)
if(PLUGIN_FFTWFilters)
    add_subdirectory(FFTWfilters)
endif(PLUGIN_FFTWFilters)

# FileGrabber
option(PLUGIN_FileGrabber "Build with this plugin." ON)
if(PLUGIN_FileGrabber)
    add_subdirectory(FileGrabber)
endif(PLUGIN_FileGrabber)

# FittingFilters
option(PLUGIN_FittingFilters "Build with this plugin." ON)
if(PLUGIN_FittingFilters)
    add_subdirectory(FittingFilters)
endif(PLUGIN_FittingFilters)

# FireGrabber
option(PLUGIN_FireGrabber "Build with this plugin." OFF)
if(PLUGIN_FireGrabber)
    add_subdirectory(FireGrabber)
endif(PLUGIN_FireGrabber)

# FirgelliLAC
if(WIN32)
    option(PLUGIN_FirgelliLAC "Build with this plugin." OFF)
    if(PLUGIN_FirgelliLAC)
        add_subdirectory(FirgelliLAC)
    endif(PLUGIN_FirgelliLAC)
endif()

# FringeProj
option(PLUGIN_FringeProj "Build with this plugin." OFF)
if(PLUGIN_FringeProj)
    add_subdirectory(FringeProj)
endif(PLUGIN_FringeProj)

# GenICam
option(PLUGIN_GenICam "Build with this plugin." OFF)
if(PLUGIN_GenICam)
    add_subdirectory(GenICam)
endif(PLUGIN_GenICam)

# glDisplay
option(PLUGIN_GLDisplay "Build with this plugin." OFF)
if(PLUGIN_GLDisplay)
    add_subdirectory(glDisplay)
endif(PLUGIN_GLDisplay)

# GWInstekPSP
option(PLUGIN_GWInstekPSP "Build with this plugin." OFF)
if(PLUGIN_GWInstekPSP)
    add_subdirectory(GWInstekPSP)
endif(PLUGIN_GWInstekPSP)

# HBM Spider8
option(PLUGIN_HBMSpider8 "Build with this plugin." OFF)
if(PLUGIN_HBMSpider8)
    add_subdirectory(HBMSpider8)
endif(PLUGIN_HBMSpider8)

# Holography
option(PLUGIN_Holography "Build with this plugin." ON)
if(PLUGIN_Holography)
    add_subdirectory(Holography)
endif(PLUGIN_Holography)

# IDSuEye
option(PLUGIN_IDSuEye "Build with this plugin." OFF)
if(PLUGIN_IDSuEye)
    add_subdirectory(IDSuEye)
endif(PLUGIN_IDSuEye)

# LeicaMotorFocus
option(PLUGIN_LeicaMotorFocus "Build with this plugin." OFF)
if(PLUGIN_LeicaMotorFocus)
    add_subdirectory(LeicaMotorFocus)
endif(PLUGIN_LeicaMotorFocus)

# libmodbus
option(PLUGIN_LibModbus "Build with this plugin." OFF)
if(PLUGIN_LibModbus)
    add_subdirectory(libmodbus)
endif(PLUGIN_LibModbus)

# LibUSB
option(PLUGIN_LIBUSB "Build with this plugin." ON)
if(PLUGIN_LIBUSB)
    add_subdirectory(LibUSB)
endif(PLUGIN_LIBUSB)

# MeasurementComputing
if(WIN32)
option(PLUGIN_MeasurementComputing "Build with this plugin." OFF)
if(PLUGIN_MeasurementComputing)
    add_subdirectory(MeasurementComputing)
endif(PLUGIN_MeasurementComputing)
endif()

# MSMediaFoundation
if(WIN32)
option(PLUGIN_MSMediaFoundation "Build with this plugin." ON)
if(PLUGIN_MSMediaFoundation)
    add_subdirectory(MSMediaFoundation)
endif(PLUGIN_MSMediaFoundation)
endif()

# NerianSceneScanPro
option(PLUGIN_NerianSceneScanPro "Build with this plugin." OFF)
if(PLUGIN_NerianSceneScanPro)
    add_subdirectory(NerianSceneScanPro)
endif(PLUGIN_NerianSceneScanPro)

# NanotecStepMotor
option(PLUGIN_NanotecStepMotor "Build with this plugin." OFF)
if(PLUGIN_NanotecStepMotor)
    add_subdirectory(NanotecStepMotor)
endif(PLUGIN_NanotecStepMotor)

# Newport2936
option(PLUGIN_NEWPORT_2936 "Build with this plugin." OFF)
if(PLUGIN_NEWPORT_2936)
    add_subdirectory(Newport2936)
endif(PLUGIN_NEWPORT_2936)

# NewportSMC100
option(PLUGIN_NEWPORT_SMC100 "Build with this plugin." OFF)
if(PLUGIN_NEWPORT_SMC100)
    add_subdirectory(NewportSMC100)
endif(PLUGIN_NEWPORT_SMC100)

# niDAQmx
option(PLUGIN_niDAQmx "Build with this plugin." OFF)
if(PLUGIN_niDAQmx)
    add_subdirectory(niDAQmx)
endif(PLUGIN_niDAQmx)

# NITWidySWIR
option(PLUGIN_NITWidySWIR "Build with this plugin." OFF)
if(PLUGIN_NITWidySWIR)
    add_subdirectory(NITWidySWIR)
endif(PLUGIN_NITWidySWIR)

# OceanOpticsSpec
option(PLUGIN_OceanOpticsSpec "Build with this plugin." ON)
if(PLUGIN_OceanOpticsSpec)
    add_subdirectory(OceanOpticsSpec)
endif(PLUGIN_OceanOpticsSpec)

# OpenCVFilters
option(PLUGIN_OpenCVFilters "Build with this plugin." ON)
if(PLUGIN_OpenCVFilters)
    add_subdirectory(OpenCVFilters)
endif(PLUGIN_OpenCVFilters)

# OpenCVFilters(nonfree)
option(PLUGIN_OpenCVFilters_Nonfree "Build with this plugin." OFF)
if(PLUGIN_OpenCVFilters_Nonfree)
    add_subdirectory(OpenCVFiltersNonFree)
endif(PLUGIN_OpenCVFilters_Nonfree)

# OpenCVGrabber
option(PLUGIN_OpenCVGrabber "Build with this plugin." ON)
if(PLUGIN_OpenCVGrabber)
    add_subdirectory(OpenCVGrabber)
endif(PLUGIN_OpenCVGrabber)

# OphirPowermeter
if(WIN32)
    option(PLUGIN_OphirPowermeter "Build with this plugin." OFF)
    if(PLUGIN_OphirPowermeter)
        add_subdirectory(OphirPowermeter)
    endif(PLUGIN_OphirPowermeter)
endif()

# PclTools
option(PLUGIN_PclTools "Build with this plugin." ON)
if(PLUGIN_PclTools)
  add_subdirectory(PclTools)
endif(PLUGIN_PclTools)

# PCOCamera
if(WIN32)
  option(PLUGIN_PCOCamera "Build with this plugin." OFF)
  if(PLUGIN_PCOCamera)
      add_subdirectory(PCOCamera)
  endif(PLUGIN_PCOCamera)
endif()

# PCOSensicam
if(WIN32)
  option(PLUGIN_PCOSensicam "Build with this plugin." OFF)
  if(PLUGIN_PCOSensicam)
      add_subdirectory(PCOSensicam)
  endif(PLUGIN_PCOSensicam)
endif()

# PCOPixelFly
if(WIN32)
  option(PLUGIN_PCOPixelFly "Build with this plugin." OFF)
  if(PLUGIN_PCOPixelFly)
      add_subdirectory(PCOPixelFly)
  endif(PLUGIN_PCOPixelFly)
endif()

# PiezosystemJena_NV40_1
option(PLUGIN_PiezosystemJena_NV40_1 "Build with this plugin." OFF)
if(PLUGIN_PiezosystemJena_NV40_1)
  add_subdirectory(PiezosystemJena_NV40_1)
endif(PLUGIN_PiezosystemJena_NV40_1)

# PGRFlyCapture
option(PLUGIN_PGRFlyCapture "Build with this plugin." OFF)
if(PLUGIN_PGRFlyCapture)
    add_subdirectory(PGRFlyCapture)
endif(PLUGIN_PGRFlyCapture)

# PIHexapodCtrl
option(PLUGIN_PIHexapodCtrl "Build with this plugin." OFF)
if(PLUGIN_PIHexapodCtrl)
    add_subdirectory(PIHexapodCtrl)
endif(PLUGIN_PIHexapodCtrl)

# PIPiezoCtrl
option(PLUGIN_PIPiezoCtrl "Build with this plugin." OFF)
if(PLUGIN_PIPiezoCtrl)
    add_subdirectory(PIPiezoCtrl)
endif(PLUGIN_PIPiezoCtrl)

# PI_GCS2
option(PLUGIN_PI_GCS2 "Build with this plugin." OFF)
if(PLUGIN_PI_GCS2)
    add_subdirectory(PI_GCS2)
endif(PLUGIN_PI_GCS2)

# PmdPico
option(PLUGIN_PMD_PICO "Build with this plugin." OFF)
if(PLUGIN_PMD_PICO)
    add_subdirectory(PmdPico)
endif(PLUGIN_PMD_PICO)

# QCam
if(WIN32)
    option(PLUGIN_QCam "Build with this plugin." OFF)
    if(PLUGIN_QCam)
        add_subdirectory(QCam)
    endif(PLUGIN_QCam)
endif()

# QuantumComposer
if(WIN32)
    option(PLUGIN_QuantumComposer "Build with this plugin." ON)
    if(PLUGIN_QuantumComposer)
        add_subdirectory(QuantumComposer)
    endif(PLUGIN_QuantumComposer)
endif()

# rawImport
option(PLUGIN_rawImport "Build with this plugin." ON)
if(PLUGIN_rawImport)
    add_subdirectory(RawImport)
endif(PLUGIN_rawImport)

# Roughness
option(PLUGIN_Roughness "Build with this plugin." ON)
if(PLUGIN_Roughness)
    add_subdirectory(Roughness)
endif(PLUGIN_Roughness)

# SerialIO
option(PLUGIN_SerialIO "Build with this plugin." ON)
if(PLUGIN_SerialIO)
    add_subdirectory(SerialIO)
endif(PLUGIN_SerialIO)

# SuperlumBS
option(PLUGIN_SuperlumBS "Build with this plugin." OFF)
if(PLUGIN_SuperlumBS)
    add_subdirectory(SuperlumBS)
endif(PLUGIN_SuperlumBS)

# SuperlumBL
option(PLUGIN_SuperlumBL "Build with this plugin." OFF)
if(PLUGIN_SuperlumBL)
    add_subdirectory(SuperlumBL)
endif(PLUGIN_SuperlumBL)

# ST8SMC4USB
if(WIN32)
    option(PLUGIN_ST8SMC4USB "Build with this plugin." OFF)
    if(PLUGIN_ST8SMC4USB)
            add_subdirectory(ST8SMC4USB)
    endif(PLUGIN_ST8SMC4USB)
endif()

# ThorlabsBP benchtop piezo
if(WIN32)
    option(PLUGIN_ThorlabsBP "Build the Thorlabs benchtop piezo plugin." OFF)
    if(PLUGIN_ThorlabsBP)
        add_subdirectory(ThorlabsBP)
    endif(PLUGIN_ThorlabsBP)
endif()

# ThorlabsFF filter flipper
if(WIN32)
    option(PLUGIN_ThorlabsFF "Build the Thorlabs filter flipper plugin." OFF)
    if(PLUGIN_ThorlabsFF)
        add_subdirectory(ThorlabsFF)
    endif(PLUGIN_ThorlabsFF)
endif()

# ThorlabsTCubeTEC TCube TEC controller (temperature controller)
if(WIN32)
    option(PLUGIN_ThorlabsTCubeTEC "Build the Thorlabs TCube TEC plugin." OFF)
    if(PLUGIN_ThorlabsTCubeTEC)
        add_subdirectory(ThorlabsTCubeTEC)
    endif()
endif()

# ThorlabsKCubeDCServo KCube DC Servo
if(WIN32)
    option(PLUGIN_ThorlabsKCubeDCServo "Build the Thorlabs KCube DC Servo plugin." OFF)
    if(PLUGIN_ThorlabsKCubeDCServo)
        add_subdirectory(ThorlabsKCubeDCServo)
    endif(PLUGIN_ThorlabsKCubeDCServo)
endif()

# ThorlabsKCubeIM KCube inertial motor
if(WIN32)
    option(PLUGIN_ThorlabsKCubeIM "Build the Thorlabs KCube inertial motor plugin." OFF)
    if(PLUGIN_ThorlabsKCubeIM)
        add_subdirectory(ThorlabsKCubeIM)
    endif(PLUGIN_ThorlabsKCubeIM)
endif()

# ThorlabsKCubePA KCube position aligner
if(WIN32)
    option(PLUGIN_ThorlabsKCubePA "Build the Thorlabs KCube position aligner plugin." OFF)
    if(PLUGIN_ThorlabsKCubePA)
        add_subdirectory(ThorlabsKCubePA)
    endif(PLUGIN_ThorlabsKCubePA)
endif()

# ThorlabsCCS Spectrometer
if(WIN32)
    option(PLUGIN_ThorlabsCCS "Build with this plugin." OFF)
    if(PLUGIN_ThorlabsCCS)
        add_subdirectory(ThorlabsCCS)
    endif(PLUGIN_ThorlabsCCS)
endif()

# ThorlabsISM stepper motor
if(WIN32)
    option(PLUGIN_ThorlabsISM "Build with this plugin." OFF)
    if(PLUGIN_ThorlabsISM)
        add_subdirectory(ThorlabsISM)
    endif(PLUGIN_ThorlabsISM)
endif()

# ThorlabsDCxCam cameras
if(WIN32)
    option(PLUGIN_ThorlabsDCxCam "Build with this plugin." OFF)
    if(PLUGIN_ThorlabsDCxCam)
        add_subdirectory(ThorlabsDCxCam)
    endif(PLUGIN_ThorlabsDCxCam)
endif()

# Thorlabs Power Meter
if(WIN32)
    option(PLUGIN_ThorlabsPowerMeter "Build with this plugin." OFF)
    if(PLUGIN_ThorlabsPowerMeter)
        add_subdirectory(ThorlabsPowerMeter)
    endif(PLUGIN_ThorlabsPowerMeter)
endif()

# UhlRegister
option(PLUGIN_UhlRegister "Build with this plugin." OFF)
if(PLUGIN_UhlRegister)
    add_subdirectory(UhlRegister)
endif(PLUGIN_UhlRegister)

# UhlText
option(PLUGIN_UhlText "Build with this plugin." OFF)
if(PLUGIN_UhlText)
    add_subdirectory(UhlText)
endif(PLUGIN_UhlText)

# USBMotion3XIII
if(WIN32)
    option(PLUGIN_USBMotion3XIII "Build with this plugin." OFF)
    if(PLUGIN_USBMotion3XIII)
        add_subdirectory(USBMotion3XIII)
    endif(PLUGIN_USBMotion3XIII)
endif()

# V4L2
if(UNIX AND NOT APPLE)
    option(PLUGIN_V4L2 "Build with this plugin." ON)
    if(PLUGIN_V4L2)
        add_subdirectory(V4L2)
    endif(PLUGIN_V4L2)
endif(UNIX AND NOT APPLE)

# Vistek
if(WIN32)
    option(PLUGIN_Vistek "Build with this plugin." OFF)
    if(PLUGIN_Vistek)
        add_subdirectory(Vistek)
    endif(PLUGIN_Vistek)
endif()

# VRMagic
if(WIN32)
    option(PLUGIN_VRMagic "Build with this plugin." OFF)
    if(PLUGIN_VRMagic)
        add_subdirectory(VRMagic)
    endif(PLUGIN_VRMagic)
endif()

# x3pio
option(PLUGIN_x3pio "Build with this plugin." ON)
if(PLUGIN_x3pio)
    add_subdirectory(x3pio)
endif(PLUGIN_x3pio)

# Xeneth
if(WIN32)
    option(PLUGIN_Xeneth "Build with this plugin." OFF)
    if(PLUGIN_Xeneth)
        add_subdirectory(Xeneth)
    endif(PLUGIN_Xeneth)
endif()

# XIMEA
option(PLUGIN_XIMEA "Build with this plugin." OFF)
if(PLUGIN_XIMEA)
    add_subdirectory(Ximea)
endif(PLUGIN_XIMEA)
