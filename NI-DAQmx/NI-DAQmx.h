/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef niDAQmx_H
#define niDAQmx_H

//#include "opencv/cv.h"
#include "common\addInInterface.h"
#include <qsharedpointer.h>
#include <qmap.h>
#include <DataObject\dataobj.h>

#include "dialogNI-DAQmx.h"
#include "NIDAQmx.h"
//#include "NIDAQmx-LibHeader.h"  // include NI-DAQmx Library functions       
#include "NI-PeripheralClasses.h" // Classes that encapsulate general stuff like channels and tasks

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    niDAQmxInterface 
  *
  *\brief    Interface-Class for niDAQmx-Class
  *
  *    \sa    AddInDataIO, niDAQmx
  *
  */
class niDAQmxInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        niDAQmxInterface();
        ~niDAQmxInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    niDAQmx

  */
class niDAQmx : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~niDAQmx();
        niDAQmx();
        
        
    public:
        friend class niDAQmxInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog

    private:
		ito::RetVal checkData(ito::DataObject *externalDataObject, int channels, int samples);

        bool m_isgrabbing; /*!< Check if acquire was executed */

		// These three bools are set true by the acquire method to indicate what kind of data is
		// received in the getVal method. The getVal method also resets the three bools to false
		bool m_aInIsAcquired;
		bool m_dInIsAcquired;
		bool m_cInIsAcquired;

		bool m_aOutIsAcquired;
		bool m_dOutIsAcquired;
		bool m_cOutIsAcquired;
		
		QMap<QString, niTask*> m_taskMap;
		niChannelList m_channels;
		ito::DataObject m_data;
		
		// Read-functions
		ito::RetVal readAnalog(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        ito::RetVal readDigital(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        ito::RetVal readCounter(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        
		// Write-functions
		ito::RetVal writeAnalog(const ito::DataObject *externalDataObject = NULL);
		ito::RetVal writeDigital(ito::DataObject *externalDataObject = NULL);
		ito::RetVal writeCounter(ito::DataObject *externalDataObject = NULL);
        
    public slots:
        //!< Get ADC-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set ADC-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //!< Start the ADC to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the ADC to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the ADC
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //!< Wait for acquired picture, copy the Values to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);
		//!< 
		ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond = NULL);
		//!< 
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        
        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

#endif // niDAQmx_H
