/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef AVTVIMBA_H
#define AVTVIMBA_H

#include "common/addInGrabber.h"
#include "opencv/cv.h"
#include <qsharedpointer.h>
#include "dialogAvtVimba.h"
#include <VimbaCPP/Include/VimbaCPP.h>

using namespace AVT::VmbAPI;

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabberInterface 
  *
  *\brief    Interface-Class for MyGrabber-Class
  *
  *    \sa    AddInDataIO, MyGrabber
  *
  */
class AvtVimbaInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        AvtVimbaInterface();
        ~AvtVimbaInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabber

  */
class AvtVimba : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~AvtVimba();
        //! Constructor
        AvtVimba();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        
    public:
        friend class AvtVimbaInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
		ito::RetVal checkError(VmbErrorType errCode);

		ito::RetVal getIntFeatureByName(const char *name, VmbInt64_t &value);
		ito::RetVal getEnumFeatureByName(const char *name, std::string &value);
		ito::RetVal getDblFeatureByName(const char *name, double &value);
		ito::RetVal SetDblFeature(const char *name, double &value);
		ito::RetVal SetIntFeature(const char *name, int &value);
		ito::RetVal SetEnumFeature(const char *name, const char *eValue);
		ito::RetVal getRange(const char *name, double &max, double &min);
		ito::RetVal getRange(const char *name, VmbInt64_t &max, VmbInt64_t &min);


        bool m_isgrabbing; /*!< Check if acquire was executed */
		ito::RetVal m_acquisitionStatus;
		CameraPtr m_camera;
		FramePtr m_frame;
        
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
        
        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // AVTVIMBA_H
