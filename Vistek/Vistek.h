#ifndef VISTEK_H
#define VISTEK_H

#include "common/addInGrabber.h"
#include "dialogVistek.h"
#include "SVGigE.h"
#include "VistekInterface.h"
#include "VistekContainer.h"

#include <qsharedpointer.h>
#include <QTimerEvent>
#include <qmutex.h>

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

        enum AcquisitionStatus { asNoImageAcquired, asWaitingForTransfer, asImageReady, asTimeout };

        struct Features
        {
            bool adjustExposureTime;
            bool adjustGain;
            bool adjustBinning;
            bool has8bit;
            bool has10bit;
            bool has12bit;
            bool has16bit;
        };

        Features m_features;

        struct AcquiredImage
        {
            AcquisitionStatus status;
            int sizex;
            int sizey;
            int dataID;
            int packetCount;
            double transferTime;
            double timestamp;
            GVSP_PIXEL_TYPE pixelType;
            QVector<ito::uint8> buffer;
        };

        AcquiredImage m_acquiredImage;

        

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

        

        // Utility functions to control the camera
        ito::RetVal initCamera(int CameraNumber);
        ito::RetVal startStreamAndRegisterCallbacks();
        ito::RetVal stopStreamAndDeleteCallbacks();

    signals:
        void parametersChanged(QMap<QString, ito::Param> params);

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
        void GainOffsetExposurePropertiesChanged(double gain, double offset, double exposure);

        void dockWidgetVisibilityChanged(bool visible);

};


//----------------------------------------------------------------------------------------------------------------------------------
// Callback function for Vistek
SVGigE_RETURN __stdcall DataCallback(Image_handle data, void* context);
SVGigE_RETURN __stdcall MessageCallback(Event_handle eventID, void* context);

//----------------------------------------------------------------------------------------------------------------------------------
#endif // VISTEK_H
