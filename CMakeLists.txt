project(itom_plugins)

cmake_minimum_required(VERSION 2.8)

OPTION(BUILD_UNICODE "Build with unicode charset if set to ON, else multibyte charset." ON)
OPTION(BUILD_SHARED_LIBS "Build shared library." ON)
OPTION(BUILD_TARGET64 "Build for 64 bit target if set to ON or 32 bit if set to OFF." OFF)
SET(ITOM_DIR "" CACHE PATH "base path to itom") 

# AerotechEnsemble
IF (WIN32)
    OPTION(PLUGIN_aerotechEnsemble "Build with this plugin." ON)
    if (PLUGIN_aerotechEnsemble)
            ADD_SUBDIRECTORY(AerotechEnsemble)
    endif (PLUGIN_aerotechEnsemble)
ENDIF (WIN32)

# cmu1394
IF (WIN32)
    OPTION(PLUGIN_cmu1394 "Build with this plugin." ON)
    if (PLUGIN_cmu1394)
            ADD_SUBDIRECTORY(cmu1394)
    endif (PLUGIN_cmu1394)
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
IF (WIN32)
    OPTION(PLUGIN_FireGrabber "Build with this plugin." ON)
    if (PLUGIN_FireGrabber)
        ADD_SUBDIRECTORY(FireGrabber)
    endif (PLUGIN_FireGrabber)
ENDIF (WIN32)

# GWInstekPSP
OPTION(PLUGIN_GWInstekPSP "Build with this plugin." ON)
if (PLUGIN_GWInstekPSP)
    ADD_SUBDIRECTORY(GWInstekPSP)
endif (PLUGIN_GWInstekPSP)

# BasicFilters
OPTION(PLUGIN_BasicFilters "Build with this plugin." ON)
if (PLUGIN_BasicFilters)
    ADD_SUBDIRECTORY(BasicFilters)
endif (PLUGIN_BasicFilters)

# LeicaMotorFocus
OPTION(PLUGIN_LeicaMotorFocus "Build with this plugin." ON)
if (PLUGIN_LeicaMotorFocus)
    ADD_SUBDIRECTORY(LeicaMotorFocus)
endif (PLUGIN_LeicaMotorFocus)

# LibUSB
OPTION(PLUGIN_LIBUSB "Build with this plugin." ON)
if (PLUGIN_LIBUSB)
    ADD_SUBDIRECTORY(LibUSB)
endif (PLUGIN_LIBUSB)

IF(WIN32)
# MSMediaFoundation
OPTION(PLUGIN_MSMediaFoundation "Build with this plugin." OFF)
if (PLUGIN_MSMediaFoundation)
    ADD_SUBDIRECTORY(MSMediaFoundation)
endif (PLUGIN_MSMediaFoundation)
ENDIF(WIN32)

# OpenCVFilters
OPTION(PLUGIN_OpenCVFilters "Build with this plugin." ON)
if (PLUGIN_OpenCVFilters)
    ADD_SUBDIRECTORY(OpenCVFilters)
endif (PLUGIN_OpenCVFilters)

# OpenCVGrabber
OPTION(PLUGIN_OpenCVGrabber "Build with this plugin." ON)
if (PLUGIN_OpenCVGrabber)
    ADD_SUBDIRECTORY(OpenCVGrabber)
endif (PLUGIN_OpenCVGrabber)

# PCOPixelFly
IF (WIN32)
  OPTION(PLUGIN_PCOPixelFly "Build with this plugin." ON)
  if (PLUGIN_PCOPixelFly)
      ADD_SUBDIRECTORY(PCOPixelFly)
  endif (PLUGIN_PCOPixelFly)
ENDIF (WIN32)

# PGRFlyCapture
IF (WIN32)
    OPTION(PLUGIN_PGRFlyCapture "Build with this plugin." ON)
    if (PLUGIN_PGRFlyCapture)
            ADD_SUBDIRECTORY(PGRFlyCapture)
    endif (PLUGIN_PGRFlyCapture)
ENDIF (WIN32)

# PIPiezoCtrl
OPTION(PLUGIN_PIPiezoCtrl "Build with this plugin." ON)
if (PLUGIN_PIPiezoCtrl)
    ADD_SUBDIRECTORY(PIPiezoCtrl)
endif (PLUGIN_PIPiezoCtrl)

# QCam
OPTION(PLUGIN_QCam "Build with this plugin." OFF)
if (PLUGIN_QCam)
    ADD_SUBDIRECTORY(QCam)
endif (PLUGIN_QCam)

# SerialIO
OPTION(PLUGIN_SerialIO "Build with this plugin." ON)
if (PLUGIN_SerialIO)
    ADD_SUBDIRECTORY(SerialIO)
endif (PLUGIN_SerialIO)

# USBMotion3XIII
IF (WIN32)
    OPTION(PLUGIN_USBMotion3XIII "Build with this plugin." ON)
    if (PLUGIN_USBMotion3XIII)
        ADD_SUBDIRECTORY(USBMotion3XIII)
    endif (PLUGIN_USBMotion3XIII)
ENDIF (WIN32)

# Vistek
IF (WIN32)
    OPTION(PLUGIN_Vistek "Build with this plugin." ON)
    if (PLUGIN_Vistek)
        ADD_SUBDIRECTORY(Vistek)
    endif (PLUGIN_Vistek)
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
