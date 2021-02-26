/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef LIVOX_H
#define LIVOX_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>
#include "dialogLivox.h"

#include "livox_sdk.h"
#include "livox_def.h"

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    LivoxInterface 
  *
  *\brief    Interface-Class for Livox-Class
  *
  *    \sa    AddInDataIO, Livox
  *
  */
class LivoxInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        LivoxInterface();
        ~LivoxInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    Livox

  */
class Livox : public ito::AddInGrabber
{
    Q_OBJECT

    protected:
        //! Destructor
        ~Livox();
        //! Constructor
        Livox();
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
		
		//ito::RetVal OnDeviceBroadcast(const BroadcastDeviceInfo * info);
		//void GetLidarData(uint8_t handle, LivoxEthPacket * data, uint32_t data_num, void * client_data);
		///** Callback function of stopping sampling. */
		//void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data);


    public:
        friend class LivoxInterface;
        const ito::RetVal showConfDialog(void);

        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
        
        char* bufferPtr; //this can be a pointer holding the image array from the camera. This buffer is then copied to the dataObject m_data (defined in AddInGrabber)

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */

		/* from samples/main.c*/
		typedef enum {
			kDeviceStateDisconnect = 0,
			kDeviceStateConnect = 1,
			kDeviceStateSampling = 2,
		} DeviceState;

		typedef struct {
			uint8_t handle;
			DeviceState device_state;
			DeviceInfo info;
		} DeviceItem;

		DeviceItem devices[kMaxLidarCount];

		uint32_t data_recv_count[kMaxLidarCount];
		int lidar_count;
		char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];
		LivoxSdkVersion _sdkversion;

		//int SetProgramOption(int argc, const char *argv[]);


        
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
#endif // LIVOX_H

