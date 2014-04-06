#ifndef PCOCAMERA_H
#define PCOCAMERA_H

#include "common/addInGrabber.h"
#include "dialogPCOCamera.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

#include "sc2_defs.h"
#include "PCO_err.h"
#define PCO_ERRT_H_CREATE_OBJECT
#include "sc2_SDKStructures.h"
#include "SC2_CamExport.h"

//----------------------------------------------------------------------------------------------------------------------------------
class PCOCameraInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this PCOCameraInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */
    PLUGIN_ITOM_API

    public:
        PCOCameraInterface();                    /*!< Constructor */
        ~PCOCameraInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of PCOCamera and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of PCOCamera, given by *addInInst */

};

//----------------------------------------------------------------------------------------------------------------------------------
class PCOCamera : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        ~PCOCamera();
        PCOCamera();

//        ito::RetVal checkData();
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);

    public:
        friend class PCOCameraInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_isgrabbing;
        HANDLE m_hCamera;

        HANDLE m_hEvent;
        WORD m_wActSeg;
        unsigned short m_recstate;

        WORD * m_wBuf;
        short m_curBuf;
        PCO_Description m_caminfo;

        ito::RetVal setExposure(double exposure);

        ito::RetVal checkError(int error);

    signals:

    public slots:
        /*ito::RetVal getParam(const char *name, QSharedPointer<char> val, QSharedPointer<int> len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getParam(const char *name, QSharedPointer<double> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const char *val, const int len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const double val, ItomSharedSemaphore *waitCond = NULL);*/

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        //ito::RetVal setVal(const char *dObj, const int length, ItomSharedSemaphore *waitCond);

        //void dataParametersChanged(int sizex, int sizey, int bpp);
        void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    private slots:
        ito::RetVal updateCamParams(void);


};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // PCOCamera_H
