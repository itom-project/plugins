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
class Vistek : public ito::AddInGrabber //, public VistekInterface
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

		// Variables that are filled during the data callback
		void * ImageData;
		int SizeX, SizeY, DataID, PacketCount, TriggerViolationCount;
		double Timestamp, TimeSinceLastFrame, TransferTime;
		double MessageTimestampStartOfTransfer, MessageTimestampLastStartOfTransfer, MessageTimestampFrameCompleted, MessageTimestampEndOfExposure;
		GVSP_PIXEL_TYPE PixelType;
		StreamingChannel_handle StreamingChannel;
		Event_handle EventID;
		bool FrameCompletedFlag;

    private:
		VistekContainer *m_pVistekContainer;
		Camera_handle Cam;
		int NumberOfCameras;
		int BufferCount;
		double TimestampTickFrequency;
		BINNING_MODE BinningMode;

		// Utility functions to control the camera
		ito::RetVal initCamera(int CameraNumber);
		ito::RetVal updateStreamingChannel();
		ito::RetVal registerCallbacks();

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

		void dataParametersChanged(int sizex, int sizey, int bpp);
		void gainPropertiesChanged(double gain);
		void exposurePropertiesChanged(double gain);
		void updateTimestamp();

    private slots:

};


//----------------------------------------------------------------------------------------------------------------------------------
// Callback function for Vistek
SVGigE_RETURN __stdcall DataCallback(Image_handle Data, void* Context);
SVGigE_RETURN __stdcall MessageCallback(Event_handle EventID, void* Context);

//----------------------------------------------------------------------------------------------------------------------------------
#endif // VISTEK_H
