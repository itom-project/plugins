/* ********************************************************************
    Plugin "AndorSDK3" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#ifndef AndorSDK3_H
#define AndorSDK3_H

#include "common/addInGrabber.h"

#include "atcore.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

//----------------------------------------------------------------------------------------------------------------------------------
class AndorSDK3 : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~AndorSDK3();
        AndorSDK3();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);
        void dockWidgetVisibilityChanged(bool visible);

        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class AndorSDK3Interface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        ito::RetVal checkError(const int &code);

        enum SyncParams {
            sPixelClock = 0x0001,
            sExposure = 0x0002,
            sBinning = 0x0004,
            sRoi = 0x0008,
            sGain = 0x0010,
            sOffset = 0x0020,
            sTriggerMode = 0x0040,
            sBppAndPreAmpGain = 0x0080,
            sElectronicShutteringMode = 0x0100,
            sFanSpeed = 0x0200,
            sCooling = 0x0400,
            sReadoutRate = 0x0800,
            sFrameRate = 0x1000,
            sAll = sPixelClock | sExposure | sBinning | sRoi | sGain | sOffset | sTriggerMode | sBppAndPreAmpGain | sElectronicShutteringMode | sFanSpeed | sCooling | sReadoutRate | sFrameRate};

        AT_H m_handle;
        int m_cameraIndex;
        ito::RetVal m_acquisitionRetVal;

        struct BufferStruct {
            BufferStruct() : buffer(NULL), alignedBuffer(NULL) {}
            AT_64 aoiWidth;
            AT_64 aoiHeight;
            AT_64 aoiStride;
            AT_U8 aoiBitsPerPixel;
            AT_U8 *buffer;
            AT_U8 *alignedBuffer;
            AT_64 bufferSize;
            bool imageAvailable;
        };

        struct PixelEncodingIdx
        {
            PixelEncodingIdx() : mono8(-1), mono12(-1), mono16(-1) {}
            int mono8;
            int mono12;
            int mono16;
        };

        struct TriggerModeIdx
        {
            TriggerModeIdx() : tInternal(-1), tSoftware(-1), tExternal(-1), tExternalStart(-1), tExternalExposure(-1) {}
            int tInternal;
            int tSoftware;
            int tExternal;
            int tExternalStart;
            int tExternalExposure;
        };

        struct ElectronicShutteringMode
        {
            ElectronicShutteringMode() : tShutterRolling(-1), tShutterGlobal(-1) {}
            int tShutterRolling;
            int tShutterGlobal;
        };

        struct BitDepthIdx
        {
            BitDepthIdx() : t11Bit(-1), t12Bit(-1), t16Bit(-1) {}
            int t11Bit;
            int t12Bit;
            int t16Bit;
        };

        struct FanSpeedIdx
        {
            FanSpeedIdx(): sOff(-1), sLow(-1), sOn(-1) {}
            int sOff;
            int sLow;
            int sOn;
        };

        struct PixelReadoutRateIdx
        {
            PixelReadoutRateIdx(): p100(-1), p200(-1), p280(-1), p550(-1) {}
            int p100;
            int p200;
            int p280;
            int p550;
        };

        struct AT_Size
        {
            AT_64 x;
            AT_64 y;
        };


        PixelEncodingIdx m_pixelEncodingIdx;
        TriggerModeIdx m_triggerModeIdx;
        ElectronicShutteringMode m_electronicShutteringMode;
        BitDepthIdx m_bitDepthIdx;
        FanSpeedIdx m_fanSpeedIdx;
        PixelReadoutRateIdx m_pixelReadoutRate;
        int m_bitDepth; //read-only, this indicates how many bits per pixel are transferred.

        BufferStruct m_buffer;
        bool m_camRestartNecessary;

        static int andorOpenedIndices[32];

        ito::RetVal synchronizeCameraSettings(int what = sAll);
        ito::RetVal loadEnumIndices();
        ito::RetVal loadSensorInfo();

        int m_hBin, m_vBin;
        bool m_softwareTriggerEnabled;
        AT_64 m_timestampFrequency; // in Hz
        AT_64 m_lastTimestamp;

    signals:

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // AndorSDK3_H
