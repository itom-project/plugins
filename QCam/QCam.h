#ifndef QCAM_H
#define QCAM_H

#include "common/addInGrabber.h"
#include "dialogQCam.h"

#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class	QCamInterface 
  *
  *\brief	Interface-Class for QCam-class
  *
  */
class QCamInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

    protected:

    public:
        QCamInterface();
        ~QCamInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class	QCam 
  *\brief	class to use firewire-Cameras with the QCam driver from QImaging
  *
  *
  *	\sa	AddInDataIO
  *	\author	
  *
  */
class QCam : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        QCam();
        ~QCam();

    public:
        friend class QCamInterface;
        const ito::RetVal showConfDialog(void);	/*!< Open the config nonmodal dialog to set camera parameters */

    protected:
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL);	/*! <Wait for acquired picture */
//        ito::RetVal checkData(void);	/*!< Check if objekt has to be reallocated */

    private:
        QCam_Handle m_camHandle;

    public slots:
        
		//! returns parameter of m_params with key name.
		ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        //! sets parameter of m_params with key name. 
		ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);
		
		//! Initialise board, load dll, allocate buffer
		ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //! Free buffer, delete board, unload dll
		ito::RetVal close(ItomSharedSemaphore *waitCond);

		//! Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //! Stop the camera to disable acquire-commands
		ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //! Softwaretrigger for the camera
		ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
		//! Calls retrieveData(NULL), than copy the picture to dObj of right type and size
        ito::RetVal getVal(void *dObj, ItomSharedSemaphore *waitCond);
		//! Deep copy the camera buffer to dObj. Object must be of right size and type. If liveData is running, a second deep-copy is performed to copy data to the grabber 
        ito::RetVal copyVal(void *dObj, ItomSharedSemaphore *waitCond);
        
        //! Retrieve new offset and new gain and give them to the camera dll
		void updateParameters(QMap<QString, ito::ParamBase> params);

    private slots:

};



//----------------------------------------------------------------------------------------------------------------------------------

#endif // QCAM_H
