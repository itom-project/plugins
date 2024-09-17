cmake_minimum_required(VERSION 3.12...3.29)

#this is to automatically detect the SDK subfolder of the itom build directory.
if(NOT EXISTS ${ITOM_SDK_DIR})
    find_path(ITOM_SDK_DIR "cmake/itom_sdk.cmake"
    HINTS "$ENV{ITOM_SDK_ROOT}"
          "${CMAKE_CURRENT_BINARY_DIR}/../itom/SDK"
    DOC "Path of SDK subfolder of itom root (build) directory")
endif(NOT EXISTS ${ITOM_SDK_DIR})

if(NOT EXISTS ${ITOM_SDK_DIR})
    message(FATAL_ERROR "ITOM_SDK_DIR is invalid. Provide itom SDK directory path first")
endif(NOT EXISTS ${ITOM_SDK_DIR})

message(
    STATUS "------------------- PROJECT itom_plugins ---------------------")
# Retrieve Version Number and Identifier from GIT-TAG
include(${ITOM_SDK_DIR}/cmake/VersionFromGit.cmake)
version_from_git(
  LOG       ON
  TIMESTAMP "%Y%m%d%H%M%S"
)

# define cmake project and version number
project(itom_plugins VERSION ${GIT_VERSION})
set(itom_plugins_VERSION_IDENTIFIERS ${GIT_IDENTIFIERS})

option(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." ON)
set(ITOM_SDK_DIR NOTFOUND CACHE PATH "path of SDK subfolder of itom root (build) directory")

message(
    STATUS "------------------- Version = ${itom_plugins_VERSION} -----------------------\n")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${ITOM_SDK_DIR}/cmake)
find_package(ITOM_SDK COMPONENTS dataobject itomCommonLib itomCommonQtLib itomWidgets REQUIRED)
include(ItomBuildMacros)

# PLUGIN BUILD OPTIONS
set(PLUGIN_BUILD_OPTION "default" CACHE STRING "If ITOM plugin build options: default, setup, test")
set_property(CACHE PLUGIN_BUILD_OPTION PROPERTY STRINGS default setup test)

#include itom_plugin_option macro
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/plugins_options.cmake)

# LibUSB
itom_plugin_option(PLUGIN_LIBUSB)
if(PLUGIN_LIBUSB)
    add_subdirectory(LibUSB)
endif(PLUGIN_LIBUSB)

# AerotechA3200
if(NOT BUILD_TARGET64)
    itom_plugin_option(PLUGIN_aerotechA3200)
    if(PLUGIN_aerotechA3200 )
        add_subdirectory(AerotechA3200)
    endif(PLUGIN_aerotechA3200)
endif(NOT BUILD_TARGET64)

# AerotechEnsemble
if(BUILD_QTVERSION STREQUAL "Qt5")
    itom_plugin_option(PLUGIN_aerotechEnsemble)
    if(PLUGIN_aerotechEnsemble)
        add_subdirectory(AerotechEnsemble)
    endif(PLUGIN_aerotechEnsemble)
endif(BUILD_QTVERSION STREQUAL "Qt5")

# AndorSDK3
if(BUILD_QTVERSION STREQUAL "Qt5")
    itom_plugin_option(PLUGIN_andorSDK3)
    if(PLUGIN_andorSDK3)
        add_subdirectory(AndorSDK3)
    endif(PLUGIN_andorSDK3)
endif(BUILD_QTVERSION STREQUAL "Qt5")

# AvantesAvaSpec
itom_plugin_option(PLUGIN_AvantesAvaSpec)
if(PLUGIN_AvantesAvaSpec AND PLUGIN_LIBUSB)
        add_subdirectory(AvantesAvaSpec)
endif(PLUGIN_AvantesAvaSpec AND PLUGIN_LIBUSB)

# AVTVimba
itom_plugin_option(PLUGIN_AVTVimba)
if(PLUGIN_AVTVimba)
    add_subdirectory(AVTVimba)
endif(PLUGIN_AVTVimba)

# BasicFilters
itom_plugin_option(PLUGIN_BasicFilters)
if(PLUGIN_BasicFilters)
    add_subdirectory(BasicFilters)
endif(PLUGIN_BasicFilters)

# BasicGPLFilters
itom_plugin_option(PLUGIN_BasicGPLFilters)
if(PLUGIN_BasicGPLFilters)
    add_subdirectory(BasicGPLFilters)
endif(PLUGIN_BasicGPLFilters)

# HidApi
itom_plugin_option(PLUGIN_HidApi)
if(PLUGIN_HidApi AND PLUGIN_LIBUSB)
    add_subdirectory(hidapi)
endif(PLUGIN_HidApi AND PLUGIN_LIBUSB)

# cmu1394
itom_plugin_option(PLUGIN_cmu1394)
if(PLUGIN_cmu1394)
    add_subdirectory(cmu1394)
endif(PLUGIN_cmu1394)

# CommonVisionBlox
itom_plugin_option(PLUGIN_CommonVisionBlox)
if(PLUGIN_CommonVisionBlox)
    add_subdirectory(CommonVisionBlox)
endif(PLUGIN_CommonVisionBlox)

# CyUSB
itom_plugin_option(PLUGIN_CyUSB)
if(PLUGIN_CyUSB)
    add_subdirectory(CyUSB)
endif(PLUGIN_CyUSB)

# dataobjectarithmetic
itom_plugin_option(PLUGIN_dataobjectarithmetic)
if(PLUGIN_dataobjectarithmetic)
    add_subdirectory(dataobjectarithmetic)
endif(PLUGIN_dataobjectarithmetic)

# DataObjectIO
itom_plugin_option(PLUGIN_DataObjectIO)
if(PLUGIN_DataObjectIO)
    add_subdirectory(DataObjectIO)
endif(PLUGIN_DataObjectIO)

# Digital Image Correlation - DIC
itom_plugin_option(PLUGIN_DIC)
if(PLUGIN_DIC)
    add_subdirectory(DIC)
endif(PLUGIN_DIC)

# dispWindow
itom_plugin_option(PLUGIN_dispWindow)
if(PLUGIN_dispWindow)
    add_subdirectory(dispWindow)
endif(PLUGIN_dispWindow)

# dslrRemote
itom_plugin_option(PLUGIN_DslrRemote)
if(PLUGIN_DslrRemote)
    add_subdirectory(DslrRemote)
endif(PLUGIN_DslrRemote)

# dslrRemote2
itom_plugin_option(PLUGIN_DslrRemote2)
if(PLUGIN_DslrRemote2)
    add_subdirectory(DslrRemote2)
endif(PLUGIN_DslrRemote2)

# DummyGrabber
itom_plugin_option(PLUGIN_DummyGrabber)
if(PLUGIN_DummyGrabber)
    add_subdirectory(DummyGrabber)
endif(PLUGIN_DummyGrabber)

# DummyMotor
itom_plugin_option(PLUGIN_DummyMotor)
if(PLUGIN_DummyMotor)
    add_subdirectory(DummyMotor)
endif(PLUGIN_DummyMotor)

# DemoAlgorithms
itom_plugin_option(PLUGIN_DemoAlgorithms)
if(PLUGIN_DemoAlgorithms)
    add_subdirectory(DemoAlgorithms)
endif(PLUGIN_DemoAlgorithms)

# FaulhaberMCS
itom_plugin_option(PLUGIN_FaulhaberMCS)
if(PLUGIN_FaulhaberMCS)
    add_subdirectory(FaulhaberMCS)
endif(PLUGIN_FaulhaberMCS)

# FFTW-Wrapper
itom_plugin_option(PLUGIN_FFTWFilters)
if(PLUGIN_FFTWFilters)
    add_subdirectory(FFTWfilters)
endif(PLUGIN_FFTWFilters)

# FileGrabber
itom_plugin_option(PLUGIN_FileGrabber)
if(PLUGIN_FileGrabber)
    add_subdirectory(FileGrabber)
endif(PLUGIN_FileGrabber)

# FittingFilters
itom_plugin_option(PLUGIN_FittingFilters)
if(PLUGIN_FittingFilters)
    add_subdirectory(FittingFilters)
endif(PLUGIN_FittingFilters)

# FireGrabber
itom_plugin_option(PLUGIN_FireGrabber)
if(PLUGIN_FireGrabber)
    add_subdirectory(FireGrabber)
endif(PLUGIN_FireGrabber)

# FirgelliLAC
itom_plugin_option(PLUGIN_FirgelliLAC)
if(PLUGIN_FirgelliLAC)
    add_subdirectory(FirgelliLAC)
endif(PLUGIN_FirgelliLAC)

# FringeProj
itom_plugin_option(PLUGIN_FringeProj)
if(PLUGIN_FringeProj)
    add_subdirectory(FringeProj)
endif(PLUGIN_FringeProj)

# GenICam
itom_plugin_option(PLUGIN_GenICam)
if(PLUGIN_GenICam)
    add_subdirectory(GenICam)
endif(PLUGIN_GenICam)

# glDisplay
itom_plugin_option(PLUGIN_GLDisplay)
if(PLUGIN_GLDisplay)
    add_subdirectory(glDisplay)
endif(PLUGIN_GLDisplay)

# GWInstekPSP
itom_plugin_option(PLUGIN_GWInstekPSP)
if(PLUGIN_GWInstekPSP)
    add_subdirectory(GWInstekPSP)
endif(PLUGIN_GWInstekPSP)

# HBM Spider8
itom_plugin_option(PLUGIN_HBMSpider8)
if(PLUGIN_HBMSpider8)
    add_subdirectory(HBMSpider8)
endif(PLUGIN_HBMSpider8)

# Holography
itom_plugin_option(PLUGIN_Holography)
if(PLUGIN_Holography)
    add_subdirectory(Holography)
endif(PLUGIN_Holography)

# IDSuEye
itom_plugin_option(PLUGIN_IDSuEye)
if(PLUGIN_IDSuEye)
    add_subdirectory(IDSuEye)
endif(PLUGIN_IDSuEye)

# LeicaMotorFocus
itom_plugin_option(PLUGIN_LeicaMotorFocus)
if(PLUGIN_LeicaMotorFocus)
    add_subdirectory(LeicaMotorFocus)
endif(PLUGIN_LeicaMotorFocus)

# libmodbus
itom_plugin_option(PLUGIN_LibModbus)
if(PLUGIN_LibModbus)
    add_subdirectory(libmodbus)
endif(PLUGIN_LibModbus)

# MeasurementComputing
itom_plugin_option(PLUGIN_MeasurementComputing)
if(PLUGIN_MeasurementComputing)
    add_subdirectory(MeasurementComputing)
endif(PLUGIN_MeasurementComputing)

# MSMediaFoundation
itom_plugin_option(PLUGIN_MSMediaFoundation)
if(PLUGIN_MSMediaFoundation)
    add_subdirectory(MSMediaFoundation)
endif(PLUGIN_MSMediaFoundation)

# NerianSceneScanPro
itom_plugin_option(PLUGIN_NerianSceneScanPro)
if(PLUGIN_NerianSceneScanPro)
    add_subdirectory(NerianSceneScanPro)
endif(PLUGIN_NerianSceneScanPro)

# NanotecStepMotor
itom_plugin_option(PLUGIN_NanotecStepMotor)
if(PLUGIN_NanotecStepMotor)
    add_subdirectory(NanotecStepMotor)
endif(PLUGIN_NanotecStepMotor)

# Newport2936
itom_plugin_option(PLUGIN_NEWPORT_2936)
if(PLUGIN_NEWPORT_2936)
    add_subdirectory(Newport2936)
endif(PLUGIN_NEWPORT_2936)

# Newport Conex LDS
itom_plugin_option(PLUGIN_NEWPORT_CONEXLDS)
if(PLUGIN_NEWPORT_CONEXLDS)
    add_subdirectory(NewportConexLDS)
endif(PLUGIN_NEWPORT_CONEXLDS)

# NewportSMC100
itom_plugin_option(PLUGIN_NEWPORT_SMC100)
if(PLUGIN_NEWPORT_SMC100)
    add_subdirectory(NewportSMC100)
endif(PLUGIN_NEWPORT_SMC100)

# niDAQmx
itom_plugin_option(PLUGIN_niDAQmx)
if(PLUGIN_niDAQmx)
    add_subdirectory(niDAQmx)
endif(PLUGIN_niDAQmx)

# NITWidySWIR
itom_plugin_option(PLUGIN_NITWidySWIR)
if(PLUGIN_NITWidySWIR)
    add_subdirectory(NITWidySWIR)
endif(PLUGIN_NITWidySWIR)

# OceanOpticsSpec
itom_plugin_option(PLUGIN_OceanOpticsSpec)
if(PLUGIN_OceanOpticsSpec AND PLUGIN_LIBUSB)
    add_subdirectory(OceanOpticsSpec)
endif(PLUGIN_OceanOpticsSpec AND PLUGIN_LIBUSB)

# OpenCVFilters
itom_plugin_option(PLUGIN_OpenCVFilters)
if(PLUGIN_OpenCVFilters)
    add_subdirectory(OpenCVFilters)
endif(PLUGIN_OpenCVFilters)

# OpenCVFilters(nonfree)
itom_plugin_option(PLUGIN_OpenCVFilters_Nonfree)
if(PLUGIN_OpenCVFilters_Nonfree)
    add_subdirectory(OpenCVFiltersNonFree)
endif(PLUGIN_OpenCVFilters_Nonfree)

# OpenCVGrabber
itom_plugin_option(PLUGIN_OpenCVGrabber)
if(PLUGIN_OpenCVGrabber)
    add_subdirectory(OpenCVGrabber)
endif(PLUGIN_OpenCVGrabber)

# OphirPowermeter
itom_plugin_option(PLUGIN_OphirPowermeter)
if(PLUGIN_OphirPowermeter)
    add_subdirectory(OphirPowermeter)
endif(PLUGIN_OphirPowermeter)

# PclTools
if(BUILD_WITH_PCL)
    itom_plugin_option(PLUGIN_PclTools)
    if(PLUGIN_PclTools)
        add_subdirectory(PclTools)
    endif(PLUGIN_PclTools)
endif(BUILD_WITH_PCL)

# PCOCamera
itom_plugin_option(PLUGIN_PCOCamera)
    if(PLUGIN_PCOCamera)
        add_subdirectory(PCOCamera)
endif(PLUGIN_PCOCamera)

# PCOPixelFly
itom_plugin_option(PLUGIN_PCOPixelFly)
    if(PLUGIN_PCOPixelFly)
    add_subdirectory(PCOPixelFly)
endif(PLUGIN_PCOPixelFly)

# PiezosystemJena_NV40_1
itom_plugin_option(PLUGIN_PiezosystemJena_NV40_1)
if(PLUGIN_PiezosystemJena_NV40_1)
  add_subdirectory(PiezosystemJena_NV40_1)
endif(PLUGIN_PiezosystemJena_NV40_1)

# PGRFlyCapture
itom_plugin_option(PLUGIN_PGRFlyCapture)
if(PLUGIN_PGRFlyCapture)
    add_subdirectory(PGRFlyCapture)
endif(PLUGIN_PGRFlyCapture)

# PIHexapodCtrl
itom_plugin_option(PLUGIN_PIHexapodCtrl)
if(PLUGIN_PIHexapodCtrl)
    add_subdirectory(PIHexapodCtrl)
endif(PLUGIN_PIHexapodCtrl)

# PIPiezoCtrl
itom_plugin_option(PLUGIN_PIPiezoCtrl)
if(PLUGIN_PIPiezoCtrl)
    add_subdirectory(PIPiezoCtrl)
endif(PLUGIN_PIPiezoCtrl)

# PI_GCS2
itom_plugin_option(PLUGIN_PI_GCS2)
if(PLUGIN_PI_GCS2)
    add_subdirectory(PI_GCS2)
endif(PLUGIN_PI_GCS2)

# PmdPico
itom_plugin_option(PLUGIN_PMD_PICO)
if(PLUGIN_PMD_PICO)
    add_subdirectory(PmdPico)
endif(PLUGIN_PMD_PICO)

# QuantumComposer
itom_plugin_option(PLUGIN_QuantumComposer)
if(PLUGIN_QuantumComposer)
    add_subdirectory(QuantumComposer)
endif(PLUGIN_QuantumComposer)

# rawImport
itom_plugin_option(PLUGIN_rawImport)
if(PLUGIN_rawImport)
    add_subdirectory(RawImport)
endif(PLUGIN_rawImport)

# Roughness
itom_plugin_option(PLUGIN_Roughness)
if(PLUGIN_Roughness)
    add_subdirectory(Roughness)
endif(PLUGIN_Roughness)

# SerialIO
itom_plugin_option(PLUGIN_SerialIO)
if(PLUGIN_SerialIO)
    add_subdirectory(SerialIO)
endif(PLUGIN_SerialIO)

# SuperlumBS
itom_plugin_option(PLUGIN_SuperlumBS)
if(PLUGIN_SuperlumBS)
    add_subdirectory(SuperlumBS)
endif(PLUGIN_SuperlumBS)

# SuperlumBL
itom_plugin_option(PLUGIN_SuperlumBL)
if(PLUGIN_SuperlumBL)
    add_subdirectory(SuperlumBL)
endif(PLUGIN_SuperlumBL)

# ST8SMC4USB
itom_plugin_option(PLUGIN_ST8SMC4USB)
if(PLUGIN_ST8SMC4USB)
        add_subdirectory(ST8SMC4USB)
endif(PLUGIN_ST8SMC4USB)

# ThorlabsBP benchtop piezo
itom_plugin_option(PLUGIN_ThorlabsBP)
if(PLUGIN_ThorlabsBP)
    add_subdirectory(ThorlabsBP)
endif(PLUGIN_ThorlabsBP)

# ThorlabsBDCServo benchtop DC Servo
itom_plugin_option(PLUGIN_ThorlabsBDCServo)
if(PLUGIN_ThorlabsBDCServo)
    add_subdirectory(ThorlabsBDCServo)
endif(PLUGIN_ThorlabsBDCServo)

# ThorlabsFF filter flipper
itom_plugin_option(PLUGIN_ThorlabsFF)
if(PLUGIN_ThorlabsFF)
    add_subdirectory(ThorlabsFF)
endif(PLUGIN_ThorlabsFF)

# ThorlabsTCubeTEC TCube TEC controller (temperature controller)
itom_plugin_option(PLUGIN_ThorlabsTCubeTEC)
if(PLUGIN_ThorlabsTCubeTEC)
    add_subdirectory(ThorlabsTCubeTEC)
endif()

# ThorlabsKCubeDCServo KCube DC Servo
itom_plugin_option(PLUGIN_ThorlabsKCubeDCServo)
if(PLUGIN_ThorlabsKCubeDCServo)
    add_subdirectory(ThorlabsKCubeDCServo)
endif(PLUGIN_ThorlabsKCubeDCServo)

# ThorlabsKCubeIM KCube inertial motor
itom_plugin_option(PLUGIN_ThorlabsKCubeIM)
if(PLUGIN_ThorlabsKCubeIM)
    add_subdirectory(ThorlabsKCubeIM)
endif(PLUGIN_ThorlabsKCubeIM)

# ThorlabsKCubePA KCube position aligner
itom_plugin_option(PLUGIN_ThorlabsKCubePA)
if(PLUGIN_ThorlabsKCubePA)
    add_subdirectory(ThorlabsKCubePA)
endif(PLUGIN_ThorlabsKCubePA)

# ThorlabsCCS Spectrometer
itom_plugin_option(PLUGIN_ThorlabsCCS)
if(PLUGIN_ThorlabsCCS)
    add_subdirectory(ThorlabsCCS)
endif(PLUGIN_ThorlabsCCS)

# ThorlabsISM stepper motor
itom_plugin_option(PLUGIN_ThorlabsISM)
if(PLUGIN_ThorlabsISM)
    add_subdirectory(ThorlabsISM)
endif(PLUGIN_ThorlabsISM)

# ThorlabsDCxCam cameras
itom_plugin_option(PLUGIN_ThorlabsDCxCam)
if(PLUGIN_ThorlabsDCxCam)
    add_subdirectory(ThorlabsDCxCam)
endif(PLUGIN_ThorlabsDCxCam)

# Thorlabs Power Meter
itom_plugin_option(PLUGIN_ThorlabsPowerMeter)
if(PLUGIN_ThorlabsPowerMeter)
    add_subdirectory(ThorlabsPowerMeter)
endif(PLUGIN_ThorlabsPowerMeter)

# UhlRegister
itom_plugin_option(PLUGIN_UhlRegister)
if(PLUGIN_UhlRegister)
    add_subdirectory(UhlRegister)
endif(PLUGIN_UhlRegister)

# UhlText
itom_plugin_option(PLUGIN_UhlText)
if(PLUGIN_UhlText)
    add_subdirectory(UhlText)
endif(PLUGIN_UhlText)

# V4L2
itom_plugin_option(PLUGIN_V4L2)
if(PLUGIN_V4L2)
    add_subdirectory(V4L2)
endif(PLUGIN_V4L2)

# Vistek
itom_plugin_option(PLUGIN_Vistek)
if(PLUGIN_Vistek)
    add_subdirectory(Vistek)
endif(PLUGIN_Vistek)

# VRMagic
itom_plugin_option(PLUGIN_VRMagic)
if(PLUGIN_VRMagic)
    add_subdirectory(VRMagic)
endif(PLUGIN_VRMagic)

# x3pio
itom_plugin_option(PLUGIN_x3pio)
if(PLUGIN_x3pio)
    add_subdirectory(x3pio)
endif(PLUGIN_x3pio)

# Xeneth
itom_plugin_option(PLUGIN_Xeneth)
if(PLUGIN_Xeneth)
    add_subdirectory(Xeneth)
endif(PLUGIN_Xeneth)

# XIMEA
itom_plugin_option(PLUGIN_XIMEA)
if(PLUGIN_XIMEA)
    add_subdirectory(Ximea)
endif(PLUGIN_XIMEA)
