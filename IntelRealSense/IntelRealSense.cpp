/* ********************************************************************
Plugin "IntelRealSense" for itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2019, Institut fuer Technische Optik (ITO),
Universitaet Stuttgart, Germany

This file is part of a plugin for the measurement software itom.

This itom-plugin is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */



/*-------------------------------------------------------------------------
NOTES & TODOs:
+   COlor
+   Check if simplification is possible
+   Stream all & select IR/Stereo/etc. -frame by Params->mode
+   Sync image acquisition:     
        rs2::frame_queue
        rs2::syncer
+   Post-processing filters?! ---> rs_processing.hpp
        rs2::threshold_filter
        rs2::units_transform
        decimation_filter
        temporal_filter
        disparity_transform
        zero_order_invalistion
        hole_filling_filter
        rates_printer
--------------------------------------------------------------------------- */
#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "IntelRealSense.h"
#include "pluginVersion.h"
#include "gitVersion.h"
#include <qstring.h>
#include <qstringlist.h>
#include <qsharedpointer.h>
#include <qplugin.h>
#include <qmessagebox.h>
#include "dockWidgetIntelRealSense.h"


//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
IntelRealSenseInterface::IntelRealSenseInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("IntelRealSense");

    m_description = QObject::tr("IntelRealSense");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This camera or grabber plugin can be used to control an INTEL RealSense camera device. It uses the IntelRealSense SDK with Connection over USB3.\n\
\n\
The device includes several single camera instances controlled separately with parameters. Also the stereo image processing is done by the camera's ASIC delivering a depth map (16bit)";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Apache License 2.0");
    m_aboutThis = QObject::tr(GITVERSION); 

	ito::Param paramVal = ito::Param("Sensor", ito::ParamBase::String, "", tr("Sensor-Device to be opened. Choose 'default' (=stereo)|'left'|'right'|'stereo'|'color'").toLatin1().data());
	ito::StringMeta meta(ito::StringMeta::String);
	meta.addItem("default");
	meta.addItem("left");
    meta.addItem("right");
	meta.addItem("stereo");
	meta.addItem("color");
	paramVal.setMeta(&meta, false);
	m_initParamsOpt.append(paramVal);

}

//----------------------------------------------------------------------------------------------------------------------------------
IntelRealSenseInterface::~IntelRealSenseInterface()
{
	m_initParamsMand.clear();
	m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSenseInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(IntelRealSense)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSenseInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(IntelRealSense)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(IntelRealSenseinterface, IntelRealSenseInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
IntelRealSense::IntelRealSense() : AddInGrabber(), m_isgrabbing(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "IntelRealSense", NULL);
    m_params.insert(paramVal.getName(), paramVal);
	
	paramVal= ito::Param("mode", ito::ParamBase::String | ito::ParamBase::In, "Sensor-Mode", NULL);
	m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("height of ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("format", ito::ParamBase::String | ito::ParamBase::Readonly, tr("Image format").toLatin1().data(), NULL);
	m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 8, 32, 8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integrationTime", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.01, tr("Integrationtime of CCD [0..1] (no unit)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("fps", ito::ParamBase::Int | ito::ParamBase::Readonly , tr("Frames per second").toLatin1().data(), NULL);
	m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("filter", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("Filtering of Stereo image [0=OFF; 1=ON]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    DockWidgetIntelRealSense *dw = new DockWidgetIntelRealSense(this);
    
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);   
}

//----------------------------------------------------------------------------------------------------------------------------------
IntelRealSense::~IntelRealSense()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //InitParam of selected Sensor ("default"|"color"|"stereo"|"left"|"right")
	QString selectedMode = paramsOpt->at(0).getVal<char *>();
    *m_pMode = selectedMode;

	// Device context - represents the current platform w/ respect to connected devices
	rs2::context ctx;
	// List of available & connected RealSense Devices
	rs2::device_list devlist = ctx.query_devices();
	// device handler for an INTEL device
	rs2::device dev;
	// Check if NO detected devices:
	if (devlist.size() == 0)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("No devices found, please connect a RealSense device").toLatin1().data());
	}
	if (!retValue.containsError())
	{
		dev = devlist.front();	// Choose first device from list
		std::string name = "UnknownDevice";	// DEVICE NAME info of selected device
		std::string sn = "NoSerial";	// Device serial number
		if (dev.supports(RS2_CAMERA_INFO_NAME)&& dev.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
			name = dev.get_info(RS2_CAMERA_INFO_NAME);
			sn = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
		}
		retValue += m_params["name"].setVal<char*>(&name[0]);
		m_identifier = QString(&sn[0]);
		setIdentifier(m_identifier);
	}
	// Sensor construct for depth/color/etc.-cameras (or IMU)
	rs2::sensor sensor;
	// List of sensors as part of a rs2::device container
	std::vector<rs2::sensor> sensorList = dev.query_sensors();
	// Mode or Sensor of Camera (Stereo / Color) ***e.g. Stereo includes Depth, Disparity, IRleft, -right, ... Stream-Modes
	std::string s_mode = "Unknown";
	// Selected stream by User
	//	Data types are represented by rs2_stream enumeration:
	//	* rs2_stream::RS2_STREAM_DEPTH
	//	* rs2_stream::RS2_STREAM_COLOR
	//	* rs2_stream::RS2_STREAM_INFRARED
	//rs2_stream s_streamType;
	s_streamType = new rs2_stream();

	if (!retValue.containsError())
	{
		// User Selection
		if (selectedMode == "stereo") 
		{
			sensor = sensorList[0];
			s_mode = sensor.get_info(RS2_CAMERA_INFO_NAME);
			*s_streamType = RS2_STREAM_DEPTH;
		}
		if (selectedMode == "left" || selectedMode == "right")
		{
			sensor = sensorList[0];
			s_mode = sensor.get_info(RS2_CAMERA_INFO_NAME);
			*s_streamType = RS2_STREAM_INFRARED;
		}
		if (selectedMode == "color") 
		{
			sensor = sensorList[1];
			s_mode = sensor.get_info(RS2_CAMERA_INFO_NAME);
			*s_streamType = RS2_STREAM_COLOR;
		}
        else
		{
            selectedMode = "default";
            sensor = sensorList[0];
			s_mode = sensor.get_info(RS2_CAMERA_INFO_NAME);
			*s_streamType = RS2_STREAM_DEPTH;
		}
        retValue += m_params["mode"].setVal<char*>(&m_pMode->toStdString()[0]);   ///(&s_mode[0]);

		// Stream profile list of a sensor (can be multiple streams)
		std::vector<rs2::stream_profile> stream_profile_list = sensor.get_stream_profiles();
		// Stream profile
		rs2::stream_profile s_profile;
		for (rs2::stream_profile stream_profile : stream_profile_list)
		{
			rs2_stream stream_data_type = stream_profile.stream_type();
			if (stream_data_type == *s_streamType)
			{
				//std::cout << "StreamDataType: " << stream_data_type << std::endl;
				//// As mentioned, a sensor can have multiple streams.
				//// In order to distinguish between streams with the same
				////  stream type we can use the following methods:
				//// 1) Each stream type can have multiple occurances. All streams, of the same type, provided from a single device have distinct indices:
				//int stream_index = stream_profile.stream_index();
				//// 2) Each stream has a user-friendly name. The stream's name is not promised to be unique, rather a human readable description of the stream
				//std::string stream_name = stream_profile.stream_name();
				//// 3) Each stream in the system, which derives from the same rs2::context, has a unique identifier
				////	This identifier is unique across all streams, regardless of the stream type.
				//int unique_stream_id = stream_profile.unique_id(); // The unique identifier can be used for comparing two streams
				////std::cout << std::setw(3) << profile_num << ": " << stream_data_type << " #" << stream_index;

				// A stream is an abstraction; Get additional data for the specific type of a stream through "Is" and "As"
				if (stream_profile.is<rs2::video_stream_profile>())
				{
					if (stream_profile.is_default())
					{
						s_profile = stream_profile;
                        
                        rs2::video_stream_profile video_stream_profile = stream_profile.as<rs2::video_stream_profile>();
						// Format of stream (enum)
							//	RS2_FORMAT_ANY, /**< When passed to enable stream, librealsense will try to provide best suited format */
							//	RS2_FORMAT_Z16, /**< 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value. */
							//	RS2_FORMAT_DISPARITY16, /**< 16-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth. */
							//	RS2_FORMAT_XYZ32F, /**< 32-bit floating point 3D coordinates. */
							//	RS2_FORMAT_YUYV, /**< 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422 but packed in a different order - https://en.wikipedia.org/wiki/YUV */
							//	RS2_FORMAT_RGB8, /**< 8-bit red, green and blue channels */
							//	RS2_FORMAT_BGR8, /**< 8-bit blue, green, and red channels -- suitable for OpenCV */
							//	RS2_FORMAT_RGBA8, /**< 8-bit red, green and blue channels + constant alpha channel equal to FF */
							//	RS2_FORMAT_BGRA8, /**< 8-bit blue, green, and red channels + constant alpha channel equal to FF */
							//	RS2_FORMAT_Y8, /**< 8-bit per-pixel grayscale image */
							//	RS2_FORMAT_DISTANCE, /**< 32-bit float-point depth distance value.  */
						rs2_format format = video_stream_profile.format();
						std::string f = rs2_format_to_string(format);
						int stream_width = video_stream_profile.width();
						int stream_height = video_stream_profile.height();
						int stream_bpp;
						switch (format)
						{
						case RS2_FORMAT_ANY: stream_bpp = 32;
							break;
						case RS2_FORMAT_Z16:	stream_bpp = 16;
							break;
						case RS2_FORMAT_DISPARITY16:	stream_bpp = 16;
							break;
						case RS2_FORMAT_XYZ32F:	stream_bpp = 32;
							break;
						case RS2_FORMAT_YUYV:	stream_bpp = 32;
							break;
						case RS2_FORMAT_RGB8:	stream_bpp = 24;
							break;
						case RS2_FORMAT_BGR8:	stream_bpp = 24;
							break;
						case RS2_FORMAT_RGBA8:	stream_bpp = 32;
							break;
						case RS2_FORMAT_BGRA8:	stream_bpp = 32;
							break;
						case RS2_FORMAT_Y8:	stream_bpp = 8;
							break;
						case RS2_FORMAT_Y16:	stream_bpp = 16;
							break;
						//case RS2_FORMAT_RAW10:	stream_bpp = 10;
						//	break;
						//case RS2_FORMAT_RAW16:	stream_bpp = 16;
						//	break;
						//case RS2_FORMAT_RAW8:	stream_bpp = 8;
						//	break;
						//case RS2_FORMAT_UYVY:	stream_bpp = 32;
						//	break;
						//case RS2_FORMAT_MOTION_RAW:	stream_bpp = 8;
						//	break;
						//case RS2_FORMAT_MOTION_XYZ32F:	stream_bpp = 32;
						//	break;
						//case RS2_FORMAT_GPIO_RAW:	stream_bpp = 8;
						//	break;
						//case RS2_FORMAT_6DOF:	stream_bpp = 8;
						//	break;
						case RS2_FORMAT_DISPARITY32:	stream_bpp = 32;
							break;
						//case RS2_FORMAT_Y10BPACK:	stream_bpp = 10;
						//	break;
						case RS2_FORMAT_DISTANCE:	stream_bpp = 32;
							break;
						//case RS2_FORMAT_MJPEG:	stream_bpp = 16;
						//	break;
						//case RS2_FORMAT_COUNT:	stream_bpp = 8;
						//	break;
						default:
							stream_bpp = 32;
						}
						// Frames per second from current stream profile
						double stream_fps = video_stream_profile.fps();

						//Set Params from selected sensor
						retValue += m_params["sizex"].setVal<int>(stream_width);
						retValue += m_params["sizey"].setVal<int>(stream_height);
						retValue += m_params["fps"].setVal<int>(stream_fps);
						retValue += m_params["format"].setVal<char*>(&f[0]);
						retValue += m_params["bpp"].setVal<int>(stream_bpp);
					}
				}
			}
		}
		
		////////////////////////////////////////////////////////////
		//// Get SENSOR OPTIONS ===> NOT PARAMETERS!!!
		//for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
		//{
		//	rs2_option option_type = static_cast<rs2_option>(i);
		//	std::cout << "  " << i << ": " << option_type << std::endl;
		//	if (sensor.supports(option_type))
		//	{
		//		// Get description of the option
		//		const char* description = sensor.get_option_description(option_type);
		//		std::cout << "Description:	" << description << "\n" << std::endl;
		//		float current_value = sensor.get_option(option_type);
		//		std::cout << "	Current Value:	" << current_value << "\n" << std::endl;
		//	}
		//}
	}

    if (!retValue.containsError())
    {        
        retValue += checkData();
    }
    
    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    setInitialized(true);
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    delete m_pFrame, m_pFrameset, m_pPipe, s_streamType, m_pMode;
    m_pFrame, m_pFrameset, m_pPipe, s_streamType , m_pMode  = NULL;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
    {
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
        *val = it.value();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }


    // TODO: Params!


    if (!retValue.containsError())
    {
        if (key == "mode")
        {
			*m_pMode = val->getVal<char*>();

            retValue += it->copyValueFrom( &(*val) );
        }
        else if (key == "format")
        {
            
            retValue += it->copyValueFrom( &(*val) );
        }
		else if (key == "sizex")
		{
			
			retValue += it->copyValueFrom(&(*val));
		}
		else if (key == "sizey")
		{
			retValue += it->copyValueFrom(&(*val));
		}
		else if (key == "bpp")
		{

			retValue += it->copyValueFrom(&(*val));
		}
        else if (key == "filter")
        {
            isFilter = val->getVal<int>();
            
            retValue += it->copyValueFrom(&(*val));
        }
        else
        {
            //all parameters that don't need further checks can simply be assigned
            retValue += it->copyValueFrom( &(*val) );
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted(); //increment a counter to see how many times startDevice has been called
    
    //todo:
    // if this function has been called for the first time (grabberStartedCount() == 1),

    if (!retValue.containsError())
    {
        // Configuration of what sensor/stream to use for grabbing
        rs2::config cfg;
        // Pipeline, encapsulating the actual device and sensors (all avail.)
        rs2::pipeline pipe;
        //Ptr to pipe
        *m_pPipe = pipe;

        rs2::pipeline_profile selection;

        if (*s_streamType)
        {
            //cfg.enable_stream(*s_streamType);	// enables the selected stream
            cfg.enable_all_streams();
            selection = m_pPipe->start(cfg);	// starts the pipeline streaming process based on cfg
        }
    }
    

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }
    
    if (!retValue.containsError())
    {
        // Stop Pipeline from streaming
        m_pPipe->stop();
    }
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
		m_isgrabbing = false;

		if (waitCond)
		{
			waitCond->returnValue = retValue;
			waitCond->release();
		}
    }
    else
    {
        m_isgrabbing = true;
    }
    
	if (!retValue.containsError())
	{
		rs2::frameset frames = m_pPipe->wait_for_frames();  //frameset containing frames from acquisition queue (-->wait for frames)
		rs2::frame frame = frames.first(*s_streamType);     //grabbed image frame OR error
		// Alternative:
        //rs2::frame frame = frames.first_or_default(*s_streamType); // grabbed image frame OR empty frame --> this is used in get_xxx_frame()

        *m_pFrameset = frames;  //Ptr to frameset
        *m_pFrame = frame;      //Ptr to frame

		if (frame)
		{
			rs2::video_frame vframe = frame.as<rs2::video_frame>(); //frame as/is video-frame

			int width = vframe.get_width();     //videoframe width in px
			int height = vframe.get_height();   //videoframe height in px
			retValue += m_params["sizex"].setVal<int>(width);
			retValue += m_params["sizey"].setVal<int>(height);
			
            //int bits = vf.get_bits_per_pixel();
			//int bytes = vframe.get_bytes_per_pixel();
            int bpp;
			rs2_format format = frame.get_profile().format();       //videoframe format (bpp)
			switch (format)
			{
			case RS2_FORMAT_ANY: bpp = 16;      //usually 32bit
				break;
			case RS2_FORMAT_Z16:	bpp = 16;       //USED BY DEPTH
				break;
			case RS2_FORMAT_DISPARITY16:	bpp = 16;
				break;
			case RS2_FORMAT_YUYV:	bpp = 32;
				break;
			case RS2_FORMAT_RGB8:	bpp = 24;
				break;
			case RS2_FORMAT_RGBA8:	bpp = 32;
				break;
			case RS2_FORMAT_Y8:	bpp = 8;        //USED BY IR1 & IR2
				break;
			case RS2_FORMAT_Y16:	bpp = 16;   
				break;
			case RS2_FORMAT_DISPARITY32:	bpp = 32;
				break;
			case RS2_FORMAT_DISTANCE:	bpp = 32;
				break;
			default:
				bpp = 16;
			}
			retValue += m_params["bpp"].setVal<int>(bpp);
		}
	}
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);
    
    int bufferWidth = m_params["sizex"].getVal<int>();    //img buffer width in px
    int bufferHeight = m_params["sizey"].getVal<int>();   //img buffer height in px
	//int desiredBpp = m_params["bpp"].getVal<int>();



    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        if (externalDataObject && hasListeners)
        {
            retValue += checkData(NULL); //update m_data
            retValue += checkData(externalDataObject); //update external object
        }
        else
        {
            retValue += checkData(externalDataObject); //update external object or m_data
        }
        
        if (!retValue.containsError())
        {
            //Polling image data to img buffer of selected stream - except IR: see below
            const void* bufferPtr = m_pFrame->get_data(); //pointer to the start of the frame data
            //frame.get_frame_number(); //möglicherweise wichtig zur sync von bildern
            //int f_px = m_pFrame->get_data_size(); //frame_size in px ????? the number of bytes in frame (laut docu)

            //Polling exclusively IR-left OR -right frames from the AllEnabledStream
            if (*m_pMode == "left") //m_pFrame->get_profile().stream_type() == RS2_STREAM_INFRARED && 
            {
                bufferPtr = m_pFrameset->get_infrared_frame(1).get_data();
            }
            if (*m_pMode == "right") //m_pFrame->get_profile().stream_type() == RS2_STREAM_INFRARED && 
            {
                bufferPtr = m_pFrameset->get_infrared_frame(2).get_data();
            }
            if ((*m_pMode == "stereo" || *m_pMode == "default") && isFilter)
            {
                //Filter the DEPTH-Image
                rs2::decimation_filter dec_filter;
                rs2::spatial_filter spat_filter;
                rs2::temporal_filter temp_filter;
                rs2::disparity_transform disparityToDepth(false);
                rs2::disparity_transform depth_to_disparity(true);
                //rs2::frame_queue original_data;
                //rs2::frame_queue filtered_data;
                
                rs2::frame filtered = m_pFrameset->get_depth_frame();
                filtered = dec_filter.process(filtered);
                filtered = spat_filter.process(filtered);
                filtered = temp_filter.process(filtered);
                filtered = disparityToDepth.process(filtered);

                bufferPtr = filtered.get_data();
            }

            if (m_data.getType() == ito::tUInt8)
            {
                if (copyExternal)
                {
                    retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*) bufferPtr, bufferWidth, bufferHeight);
                }
                if (!copyExternal || hasListeners)
                {
                    retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*) bufferPtr, bufferWidth, bufferHeight);
                }
            }
            else if (m_data.getType() == ito::tUInt16)
            {
                if (copyExternal)
                {
                    retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*) bufferPtr, bufferWidth, bufferHeight);
                }
                if (!copyExternal || hasListeners)
                {
                    retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*) bufferPtr, bufferWidth, bufferHeight);            
                }
            }
            //TODO: COLOR IMAGES!!!
            else if (m_data.getType() == ito::tRGBA32 && *m_pMode == "color")
            {
                int planes = dataObj->getDims();
                
                if (copyExternal)
                {
                    for (int i = 0; i < planes; ++i)
                    {
                        cv::Mat* externalMat = dataObj->getCvPlaneMat(i);
                        memcpy(externalMat->data, &bufferPtr, bufferWidth*bufferHeight*externalMat->elemSize());
                    }
                }
                if (!copyExternal || hasListeners)
                {
                    memcpy(&dataObj, bufferPtr, bufferWidth*bufferHeight*dataObj->elemSize()-1);
                }
                
                
                
                
                //int planes = m_data.getNumPlanes();
                //for (int i = 0; i < planes; ++i)
                //{
                //    const cv::Mat* internalMat = m_data.getCvPlaneMat(i);
                //    cv::Mat* externalMat = externalDataObject->getCvPlaneMat(i);

                //    if (externalMat->isContinuous())
                //    {
                //        memcpy(externalMat->ptr(0), internalMat->ptr(0), internalMat->cols * internalMat->rows * externalMat->elemSize());
                //    }
                //    else
                //    {
                //        for (int y = 0; y < internalMat->rows; y++)
                //        {
                //            memcpy(externalMat->ptr(y), internalMat->ptr(y), internalMat->cols * externalMat->elemSize());
                //        }
                //    }

                //}
                //cv::Range ranges[] = { cv::Range(0,bufferHeight), cv::Range(0,bufferWidth) };
                //
                //cv::Mat* tempImage = NULL;
                //memcpy(tempImage,bufferPtr,bufferWidth*bufferHeight*3);//  tempImage = cv::Mat(bufferPtr, ranges);
                ////cv::Mat tempImage = cv::Mat(bufferWidth,bufferHeight,0,&bufferPtr,0Ui32); ///not working
                //
                //cv::Mat out[] = { *(dataObj->getCvPlaneMat(0)) }; 
                //int fromTo[] = { 0,0,1,1,2,2 }; //{0,2,1,1,2,0}; //implicit BGR (camera) -> BGR (dataObject style) conversion

                //cv::mixChannels(tempImage, 1, out, 1, fromTo, 3);

            }

            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("copying image buffer not possible since unsupported type.").toLatin1().data());
            }
        }
        m_isgrabbing = false;
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSense::checkData(ito::DataObject *externalDataObject)
{
    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureType;
    int futureCh = (*m_pMode == "color") ? 3 : 1;


    int bpp = m_params["bpp"].getVal<int>();
    if (bpp <= 8)
    {
        futureType = ito::tUInt8;
    }
    else if (bpp <= 16)
    {
        futureType = ito::tUInt16;
    }
    else if (bpp == 24 && futureCh == 3)
    {
        futureType = ito::tRGBA32;
    }
    else if (bpp <= 32 && futureCh == 1)
    {
        futureType = ito::tInt32;
    }
    else
    {
        futureType = ito::tFloat64;
    }

    if (futureType == ito::tRGBA32 && (m_alphaChannel.cols != futureWidth || m_alphaChannel.rows != futureHeight))
    {
        m_alphaChannel = cv::Mat(futureHeight, futureWidth, CV_8UC1, cv::Scalar(255));
    }



    if (externalDataObject == NULL)
    {
        if (m_data.getDims() < 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight, futureWidth, futureType);

            if (futureType == ito::tRGBA32)
            {
                //copy alpha channel to 4th channel in m_data
                const int relations[] = { 0,3 };
                cv::mixChannels(&m_alphaChannel, 1, m_data.get_mdata()[0], 1, relations, 1);
            }
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0)
        {
            *externalDataObject = ito::DataObject(futureHeight, futureWidth, futureType);
        }
        else if (externalDataObject->calcNumMats() != 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more or less than 1 plane. It must be of right size and type or an uninitilized image.").toLatin1().data());
        }
        else if (externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or an uninitilized image.").toLatin1().data());
        }
        
        if (futureType == ito::tRGBA32)
        {
            //copy alpha channel to 4th channel in m_data
            const int relations[] = { 0,3 };
            cv::mixChannels(&m_alphaChannel, 1, (cv::Mat*)externalDataObject->get_mdata()[externalDataObject->seekMat(0)], 1, relations, 1);
        }
    }
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.
/*!
    This method returns a reference to the recently acquired image. Therefore this camera size must fit to the data structure of the 
    DataObject.
    
    This method returns a reference to the internal dataObject m_data of the camera where the currently acquired image data is copied to (either
    in the acquire method or in retrieve data). Please remember, that the reference may directly change if a new image is acquired.

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*). After the call, the dataObject is a reference to the internal m_data dataObject of the camera.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    
    \sa retrieveImage, copyVal
*/
ito::RetVal IntelRealSense::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    retValue += retrieveData();

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            (*dObj) = m_data; //copy reference to externally given object
        }
    }

    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a deep copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject. 
    
    The given dataObject must either have an empty size (then it is resized to the size and type of the camera image) or its size or adjusted region of
    interest must exactly fit to the size of the camera. Then, the acquired image is copied inside of the given region of interest (copy into a subpart of
    an image stack is possible then)

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    
    \sa retrieveImage, getVal
*/
ito::RetVal IntelRealSense::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    
    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
    }

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.
    }
    
    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}


//ito::RetVal IntelRealSense::syncParams(SyncParams what /*=sAll*/)
//{
//    ito::RetVal retVal(ito::retOk);
//    bool get_depth_frame;
//    if (m_params["getDepthData"].getVal<int>())
//    {
//        get_depth_frame = true;
//    }
//    if (what&sRoi)
//    {
//        int size_x, size_y;
//        int* test = m_params["roi"].getVal<int*>();
//        size_x = (*(test + 2)) - (*test);
//        size_y = (*(test + 3)) - (*(test + 1));
//        m_params["sizex"].setVal<int>(size_x);
//        m_params["sizey"].setVal<int>(size_y);
//    }
//    if (what&sRes)
//    {
//        int resol = m_params["resolution"].getVal<int>();
//        int fps = m_params["fps"].getVal<int>();
//        int new_fps{ 0 };
//        bool change_fps{ false };
//        if (get_depth_frame)
//        {
//            switch (resol)
//            {
//            case 0:
//                res[0] = 256;
//                res[1] = 144;
//                if (fps != 90)
//                {
//                    change_fps = true;
//                    new_fps = 90;
//                }
//                break;
//            case 1:
//                res[0] = 424;
//                res[1] = 240;
//                if (fps != 6 && fps != 15 && fps != 30 && fps != 60 && fps != 90)
//                {
//                    change_fps = true;
//                    new_fps = 30;
//                }
//                break;
//            case 2:
//                res[0] = 480;
//                res[1] = 270;
//                if (fps != 6 && fps != 15 && fps != 30 && fps != 60 && fps != 90)
//                {
//                    change_fps = true;
//                    new_fps = 30;
//                }
//                break;
//            case 3:
//                res[0] = 640;
//                res[1] = 360;
//                if (fps != 6 && fps != 15 && fps != 30 && fps != 60 && fps != 90)
//                {
//                    change_fps = true;
//                    new_fps = 30;
//                }
//                break;
//            case 4:
//                res[0] = 640;
//                res[1] = 480;
//                if (fps != 6 && fps != 15 && fps != 30 && fps != 60 && fps != 90)
//                {
//                    change_fps = true;
//                    new_fps = 30;
//                }
//                break;
//            case 5:
//                res[0] = 848;
//                res[1] = 100;
//                if (fps != 100)
//                {
//                    change_fps = true;
//                    new_fps = 100;
//                }
//                break;
//            case 6:
//                res[0] = 848;
//                res[1] = 480;
//                if (fps != 6 && fps != 15 && fps != 30 && fps != 60 && fps != 90)
//                {
//                    change_fps = true;
//                    new_fps = 30;
//                }
//                break;
//            case 7:
//                res[0] = 1280;
//                res[1] = 720;
//                if (fps != 6 && fps != 15 && fps != 30)
//                {
//                    change_fps = true;
//                    new_fps = 30;
//                }
//                break;
//                //case 8:
//                //	retVal += ito::RetVal(ito::retWarning, 0, tr("Chosen resolution not supported for depth-frame. Previous resolution is maintained.").toLatin1().data());
//                //	break;
//            default:
//                break;
//            }
//            if (change_fps)
//            {
//                retVal += ito::RetVal(ito::retWarning, 0, tr("Incompatible with current fps-value. Changing fps to %1.").arg(new_fps).toLatin1().data());
//                m_params["fps"].setVal<int>(new_fps);
//                //setVal fps to new_fps
//            }
//        }
//        else
//        {
//            int dummyInt = 0;//TODO
//        }
//    }
//    if (what&sDoDepth)
//    {
//        bool dodepth = m_params["getDepthData"].getVal<int>();
//        if (!dodepth)
//        {
//            retVal += ito::RetVal(ito::retError, 0, tr("Tried to acquire non-depth-data. This feature is not implemented yet!").toLatin1().data());
//        }
//    }
//    if (what&sFps)
//    {
//        int resol = m_params["resolution"].getVal<int>();
//        int fps = m_params["fps"].getVal<int>();
//        int new_resolution{ 0 };
//        bool change_resolution{ false };
//        if (get_depth_frame)
//        {
//            switch (fps)
//            {
//            case 6:
//            case 15:
//            case 30:
//                if ((resol == 5 || resol == 0))
//                {
//                    change_resolution = true;
//                    new_resolution = 7;
//                }
//                break;
//            case 25:
//                retVal += ito::RetVal(ito::retWarning, 0, tr("Chosen fps-rate not supported for depth-frame. Previous fps-rate is maintained.").toLatin1().data());
//            case 60:
//                if ((resol != 1) && (resol != 2) && (resol != 3) && (resol != 4) && (resol != 6))
//                {
//                    change_resolution = true;
//                    new_resolution = 6;
//                }
//            case 90:
//                if (resol == 5 || resol == 7)
//                {
//                    change_resolution = true;
//                    new_resolution = 6;
//                }
//            case 100:
//                if (resol != 5)
//                {
//                    change_resolution = true;
//                    new_resolution = 5;
//                }
//            default:
//                break;
//            }
//            if (change_resolution)
//            {
//                retVal += ito::RetVal(ito::retWarning, 0, tr("Incompatible with current resolution-value. Changing resolution to %1.").arg(new_resolution).toLatin1().data());
//                m_params["resolution"].setVal<int>(new_resolution);
//                //setVal fps to new_fps
//            }
//        }
//        else
//        {
//            int dummyInt = 0; //TODO
//        }
//    }
//    return retVal;
//}


////----------------------------------------------------------------------------------------------------------------------------------
//template<typename _Tp> inline ito::RetVal IntelRealSense::getParamInfo(_Tp & min, _Tp & max, _Tp &inc, _Tp & value, const char * name)
//{
//	ito::RetVal retVal = ito::RetVal(ito::retOk);
//	if (m_pParamsObj)
//	{
//		std::map<std::string, ParameterInfo> paramMap = m_pParamsObj->getAllParameters();
//		std::map<std::string, ParameterInfo>::iterator it = paramMap.find(name);
//
//
//		if (it != paramMap.end())
//		{
//			ParameterInfo::ParameterType t = it->second.getType();
//			try {
//				switch (t)
//				{
//				case visiontransfer::ParameterInfo::TYPE_INT:
//					inc = it->second.getInc<_Tp>();
//					min = it->second.getMin<_Tp>();
//					max = it->second.getMax<_Tp>();
//					value = it->second.getValue<_Tp>();
//					break;
//				case visiontransfer::ParameterInfo::TYPE_DOUBLE:
//					inc = 0.0;
//					min = it->second.getMin<_Tp>();
//					max = it->second.getMax<_Tp>();
//					value = it->second.getValue<_Tp>();
//					break;
//				case visiontransfer::ParameterInfo::TYPE_BOOL:
//					inc = 1;
//					min = 0;
//					max = 1;
//					bool val;
//					val = it->second.getValue<_Tp>();
//					value = val ? 1 : 0;
//					break;
//				default:
//					break;
//				}
//			}
//			catch (std::exception &ex)
//			{
//				retVal += ito::RetVal(ito::retError, 0, tr(QString("Error while reading parameter %1: %2").arg(name).arg(ex.what()).toLatin1().data()).toLatin1().data());
//			}
//
//
//		}
//		else
//		{
//			retVal += ito::RetVal(ito::retError, 0, tr(QString("%1 is not included in parameter map.").arg(name).toLatin1().data()).toLatin1().data());
//		}
//	}
//	else
//	{
//		retVal += ito::RetVal(ito::retError, 0, QString("parameter instance not callable").toLatin1().data());
//	}
//	return retVal;
//}




//----------------------------------------------------------------------------------------------------------------------------------
void IntelRealSense::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget *widget = getDockWidget()->widget();
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), widget, SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal IntelRealSense::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogIntelRealSense(this));
}