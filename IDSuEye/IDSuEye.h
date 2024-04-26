/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
    Copyright (C) 2017, Institut für Technische Optik, Universität Stuttgart

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

#ifndef IDSUEYE_H
#define IDSUEYE_H

#include "common/addInGrabber.h"

#if linux
    #include "uEye.h"
#else
    #include "ueye.h"
#endif

#include <qsharedpointer.h>
#include <QTimerEvent>

#define BUFSIZE 4

//----------------------------------------------------------------------------------------------------------------------------------
class IDSuEye : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~IDSuEye();
        IDSuEye();

        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);
        void dockWidgetVisibilityChanged(bool visible);

        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class IDSInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        enum SyncParams {
            sPixelClock = 1,
            sExposure = 2,
            sBinning = 4,
            sRoi = 8,
            sGain = 16,
            sOffset = 32,
            sTriggerMode = 64,
            sBppAndColorMode = 128,
            sFrameTime = 256,
            sAll = sPixelClock | sExposure | sBinning | sRoi | sGain | sOffset | sTriggerMode | sBppAndColorMode | sFrameTime};

        struct MemoryStruct {
            int width;
            int height;
            int bitspixel;
            char *ppcImgMem;
            INT pid;
            bool imageAvailable;
            MemoryStruct() { width = 0; height = 0; bitspixel = 0; ppcImgMem = NULL, pid = -1; imageAvailable = 0; }
            ~MemoryStruct() { }
        };

        ito::RetVal checkError(const int &code, const char* prefix = "");

        ito::RetVal synchronizeCameraSettings(int what = sAll);
        ito::RetVal loadSensorInfo();
        ito::RetVal setMeanFrameRate();
        ito::RetVal setFrameRate(ito::float64 framerate);

        IS_POINT_2D m_monochromeBitDepthRange;

        HIDS m_camera;
        IS_RANGE_S32 m_blacklevelRange; //range for the offset
        SENSORINFO m_sensorInfo;

        QVector<INT> m_viSeqMemId;
        QVector<char*> m_vpcSeqImgMem;
        int m_NumberOfBuffers;
        int m_oldNumBuf;

        int m_bitspixel; //bits per pixel that needs to be allocated
        int m_imageAvailable;
        bool m_seqAvailable;
        MemoryStruct m_pLockedBuf;
        bool m_colouredOutput;
        ito::RetVal m_acquisitionRetVal;
        bool m_captureVideoActive;
        bool m_seqEventInit;

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

#endif // IDSUEYE_H
