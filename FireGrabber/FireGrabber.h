#ifndef FIREGRABBER_H
#define FIREGRABBER_H

#include "common/addInGrabber.h"
#include "dialogFireGrabber.h"

//#include "./FireGrab/lib/FGCamera.h"
#include <FGCamera.h>

#include "common/helperCommon.h"

#include <qsharedpointer.h>
#include <QTimerEvent>

#define BUFFERNUMBER 1 //Maximal Number of Buffers: 32

//----------------------------------------------------------------------------------------------------------------------------------

class FireGrabberInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)  /*!< this FireGrabberInterface implements the ito::AddInInterfaceBase-interface, which makes it available as plugin in itom */

    public:
        FireGrabberInterface();                    /*!< Constructor */
        ~FireGrabberInterface();                   /*!< Destructor */
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);   /*!< creates new instance of FireGrabber and returns this instance */

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);  /*!< closes any specific instance of FireGrabber, given by *addInInst */

};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class	FireGrabber 
  *\brief	class to use a the standard grabber from Allied Fire Grab Packet as an itom-Addin.
  *
  *
  *	\sa	AddInDataIO, FireGrabber
  *	\date	Jun.2012
  *	\author	Alexander Bielke
  * \warning	NA
  *
  */

class FireGrabber : public ito::AddInGrabber //, public FireGrabberInterface
{
    Q_OBJECT

    protected:
		//! Destructor
        ~FireGrabber();
		//! Constructor
        FireGrabber();
		ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */


    public:
        friend class FireGrabberInterface;

        const ito::RetVal showConfDialog(void);	//! Open the config nonmodal dialog to set camera parameters 
		int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
		
    private:
		CFGCamera  Camera;

		unsigned long  m_xSize, m_ySize;
        
        bool m_isgrabbing; /*!< Check if acquire was called */
        //bool saturation_on; /*!< Check if saturation is controlled manually */

		ito::RetVal AlliedChkError(int errornumber); /*!< Map Allied-Error-Number to ITOM-Errortype and Message */

        ito::RetVal adjustROI(int x0, int x1, int y0, int y1);

        static int m_numberOfInstances;

    public slots:
        //!< Get Camera-Parameter
		ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set Camera-Parameter
		ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
		ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
		ito::RetVal close(ItomSharedSemaphore *waitCond);

		//!< Start the camera to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the camera to disable acquire-commands
		ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the camera
		ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
		//!< Wait for acquired picture, copy the picture to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);

        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

		void updateParameters(QMap<QString, ito::ParamBase> params);

		void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);

		 
};


//----------------------------------------------------------------------------------------------------------------------------------

#endif // FireGrabber_H
