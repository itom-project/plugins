/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef MYGRABBER_H
#define MYGRABBER_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>
#include "dialogNewport2936.h"

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    Newport2936Interface 
  *
  *\brief    Interface-Class for Newport2936-Class
  *
  *    \sa    AddInDataIO, Newport2936
  *
  */
class Newport2936Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        Newport2936Interface();
        ~Newport2936Interface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    Newport2936

  */
class Newport2936 : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~Newport2936();
        //! Constructor
        Newport2936();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
		ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);
        
    public:
        friend class Newport2936Interface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

        
        char* bufferPtr; //this can be a pointer holding the image array from the camera. This buffer is then copied to the dataObject m_data (defined in AddInGrabber)

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */
		ito::DataObject m_data;
        int m_faileIdx;
		int devID;	//Device ID to communicate via USB Port

		enum SyncParams {
			bWavelength = 0x0001,
			bAttenuator = 0x0002,
            bFilterType = 0x0004,
			bPowerRange = 0x0008,
			bAutoRange = 0x0010,
			bPowerOffset = 0x0020,

			bAll = bWavelength | bAttenuator | bFilterType | bPowerRange | bAutoRange | bPowerOffset 
		};
		ito::RetVal synchronizeParams(int what = bAll);
        ito::RetVal charToInt(const char* str, int &val);
        ito::RetVal charToDouble(const char* str, double &val);

        
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
        ito::RetVal zeroDevice(int channel, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal zeroDeviceTo(double val, int channel,ItomSharedSemaphore *waitCond = NULL);

		ito::RetVal sendCommand(long DeviceID, const char* commandBuffer);
		ito::RetVal readResponse(long DeviceID, char* responseBuffer, const unsigned long& length);
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);
        

        ito::RetVal acquireAutograbbing(QSharedPointer<QList<double> > value, ItomSharedSemaphore *waitCond);
        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // MYGRABBER_H
