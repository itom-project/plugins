#ifndef IDSUEYE_H
#define IDSUEYE_H

#include "common/addInGrabber.h"

#include "IDS/uEye.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

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
        int hasConfDialog(void) { return 0; }; //!< indicates that this plugin has got a configuration dialog

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
            sAll = sPixelClock | sExposure | sBinning | sRoi | sGain | sOffset | sTriggerMode | sBppAndColorMode };

        struct MemoryStruct {
            int width;
            int height;
            int bitspixel;
            char *ppcImgMem;
            INT pid;
            bool imageAvailable;
        };

        ito::RetVal checkError(const int &code);

        ito::RetVal synchronizeCameraSettings(int what = sAll);
        ito::RetVal loadSensorInfo();
        ito::RetVal setMinimumFrameRate();

        IS_POINT_2D m_monochromeBitDepthRange;

        HIDS m_camera;
        IS_RANGE_S32 m_blacklevelRange; //range for the offset
        SENSORINFO m_sensorInfo;
        int m_bitspixel; //bits per pixel that needs to be allocated
        MemoryStruct *m_pMemory;
        bool m_colouredOutput;
        ito::RetVal m_acquisitionRetVal;

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
