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

#include <stdio.h>
#include <stdlib.h>

#include <memory>
#include <vector>
#include <string>
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
		
    public:
        friend class LivoxInterface;
        const ito::RetVal showConfDialog(void);

        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
        
        char* bufferPtr; //this can be a pointer holding the image array from the camera. This buffer is then copied to the dataObject m_data (defined in AddInGrabber)

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */

        
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

typedef enum {
    kConnectStateOff = 0,
    kConnectStateOn = 1,
    kConnectStateConfig = 2,
    kConnectStateSampling = 3,
} LidarConnectState;

typedef enum {
    kConfigFan = 1,
    kConfigReturnMode = 2,
    kConfigCoordinate = 4,
    kConfigImuRate = 8
} LidarConfigCodeBit;

typedef enum {
    kCoordinateCartesian = 0,
    kCoordinateSpherical
} CoordinateType;

typedef struct {
    bool enable_fan;
    uint32_t return_mode;
    uint32_t coordinate;
    uint32_t imu_rate;
    volatile uint32_t set_bits;
    volatile uint32_t get_bits;
} UserConfig;

typedef struct {
    uint8_t handle;
    LidarConnectState connect_state;
    DeviceInfo info;
    UserConfig config;
} LidarDevice;

/**
 * LiDAR data source, data from dependent lidar.
 */
class LdsLidar : Livox {
public:

    //Vllt den QSP in die ItomLivox klasse und von dort auf die Lds class verweisen?!?!?!
    static QSharedPointer ldsLidarInstance; //try singleton using QtSahredPointer --> MSMediaFoundation-Plugin
    //*Singleton by Livox Sample --> not working correctly*/
	/*static LdsLidar& GetInstance() 
    {
		static LdsLidar lds_lidar;
		return lds_lidar;
	}*/
	bool DeInitLdsLidar(void);
    bool InitLdsLidar(std::vector<std::string>& broadcast_code_strs);

private:
	LdsLidar();
	//LdsLidar(const LdsLidar&) = delete;
	~LdsLidar();
	//LdsLidar& operator=(const LdsLidar&) = delete;
    QSharedPointer<LdsLidar> m_ldsLidar;


	static void GetLidarDataCb(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);
	static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
	static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);
	static void StartSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
	static void StopSampleCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
	static void DeviceInformationCb(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *clent_data);
	static void LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message);
	static void ControlFanCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
	static void SetPointCloudReturnModeCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
    static void SetCoordinateCb(livox_status status, uint8_t handle, uint8_t response, void *clent_data);
    static void SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,uint8_t response, void *clent_data);

	int AddBroadcastCodeToWhitelist(const char* broadcast_code);
	void AddLocalBroadcastCode(void);
	bool FindInWhitelist(const char* broadcast_code);

	void EnableAutoConnectMode(void) { auto_connect_mode_ = true; }
	void DisableAutoConnectMode(void) { auto_connect_mode_ = false; }
	bool IsAutoConnectMode(void) { return auto_connect_mode_; }

	bool auto_connect_mode_;
	uint32_t whitelist_count_;
	volatile bool is_initialized_;
	char broadcast_code_whitelist_[kMaxLidarCount][kBroadcastCodeSize];

	uint32_t lidar_count_;
	LidarDevice lidars_[kMaxLidarCount];

	uint32_t data_recveive_count_[kMaxLidarCount];
};

#endif // LIVOX_H

