/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef THORLABSCCS_H
#define THORLABSCCS_H

#include "common/addInGrabber.h"
//#include "opencv/cv.h"
#include <qsharedpointer.h>
#include "dialogThorlabsCCS.h"

#include <visa.h>
#include "TLCCS.h"

#if defined(linux) || defined(__APPLE__) 
    //#include <unistd.h>
#else
    #include <windows.h>
#endif

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabberInterface 
  *
  *\brief    Interface-Class for MyGrabber-Class
  *
  *    \sa    AddInDataIO, MyGrabber
  *
  */
class ThorlabsCCSInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        ThorlabsCCSInterface();
        ~ThorlabsCCSInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    MyGrabber

  */
class ThorlabsCCS : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~ThorlabsCCS();
        //! Constructor
        ThorlabsCCS();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */

        virtual ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);
        
    public:
        friend class ThorlabsCCSInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */
        ito::RetVal m_grabbingRetVal;

        ViReal64 buffer[TLCCS_NUM_PIXELS];
        ViReal64 dummyBuffer[TLCCS_NUM_PIXELS];

        ito::RetVal checkError(ViStatus err);

        ViSession m_instrument;

        inline ito::RetVal secureGetStatus(ViInt32 &status)
        {
            ViStatus s = tlccs_getDeviceStatus(m_instrument,  &status);
            while(status == 1 && s == 0)
            {
                Sleep(1);
                s = tlccs_getDeviceStatus(m_instrument,  &status);
                //qDebug() << "Status: " << s;
            }

            return checkError(s);
        }
        
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

#endif // THORLABSCCS_H
