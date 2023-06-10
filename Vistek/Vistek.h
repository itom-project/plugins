/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#ifndef VISTEK_H
#define VISTEK_H

#include "common/addInGrabber.h"

#include "SVGigE.h"
#include "VistekInterface.h"
#include "VistekContainer.h"

#include <qsharedpointer.h>
#include <QTimerEvent>
#include <qmutex.h>

class VistekFeatures
{
public:
    bool adjustExposureTime;
    bool adjustGain;
    bool adjustBinning;
    bool adjustOffset;
    bool has8bit;
    bool has10bit;
    bool has12bit;
    bool has16bit;
};

//----------------------------------------------------------------------------------------------------------------------------------
class Vistek : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        // Constructor and destructor
        Vistek(QObject *parent = 0);
        ~Vistek();
        // Other
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class VistekInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

        enum AcquisitionStatus { asNoImageAcquired = 0, asWaitingForTransfer = 1, asImageReady = 2, asTimeout = 3, asConnectionLost = 4, asOtherError = 5 };

        VistekFeatures m_features;

        struct AcquiredImage
        {
            int /*AcquisitionStatus*/ status;
            int sizex;
            int sizey;
            int dataID;
            int packetCount;
            double transferTime;
            double timestamp;
            bool frameCompleted;
            GVSP_PIXEL_TYPE pixelType;
            QVector<ito::uint8> buffer;
            QMutex mutex;
        };

        AcquiredImage m_acquiredImage;
        ito::RetVal m_acquisitionRetVal;

        // Variables that are filled during the data callback
        int TriggerViolationCount;
        double Timestamp, TimeSinceLastFrame, TransferTime;
        double MessageTimestampStartOfTransfer, MessageTimestampLastStartOfTransfer/*, MessageTimestampFrameCompleted, MessageTimestampEndOfExposure*/;
        StreamingChannel_handle m_streamingChannel;
        Event_handle m_eventID;

    private:
        ito::RetVal checkError(const char *prependStr, SVGigE_RETURN returnCode);

        VistekContainer *m_pVistekContainer;
        Camera_handle m_cam;
        double TimestampTickFrequency;
        BINNING_MODE m_binningMode;

        float m_gainIncrement;
        float m_exposureIncrement;
        int m_numBuf;

        // Utility functions to control the camera
        ito::RetVal initCamera(int CameraNumber);
        ito::RetVal startStreamAndRegisterCallbacks();
        ito::RetVal stopStreamAndDeleteCallbacks();

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

        void updateTimestamp();

    private slots:

        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
// Callback function for Vistek
SVGigE_RETURN __stdcall DataCallback(Image_handle data, void* context);
SVGigE_RETURN __stdcall MessageCallback(Event_handle eventID, void* context);

//----------------------------------------------------------------------------------------------------------------------------------
#endif // VISTEK_H
