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
/*!
    \todo add necessary information about your plugin here.
*/
IntelRealSenseInterface::IntelRealSenseInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("IntelRealSense");

    m_description = QObject::tr("IntelRealSense");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This camera or grabber plugin can be used to control an INTEL RealSense camera device. It uses the IntelRealSense SDK with Connection over USB3.\n\
\n\
The device includes several single camera instances controlled separately with parameters. Also the stereo image processing is done by the cameras ASIC delivering depth map (16bit)";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Apache License 2.0");
    m_aboutThis = QObject::tr(GITVERSION); 

	ito::Param paramVal = ito::Param("Sensor", ito::ParamBase::String, "", tr("Sensor-Device to be opened. Choose 'default' (stereo)|'color'|'stereo'").toLatin1().data());
	ito::StringMeta meta(ito::StringMeta::String);
	meta.addItem("");
	meta.addItem("color");
	meta.addItem("stereo");
	//meta.addItem("left");
	//meta.addItem("right");
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
    NEW_PLUGININSTANCE(IntelRealSense) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal IntelRealSenseInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(IntelRealSense) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(IntelRealSenseinterface, IntelRealSenseInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
IntelRealSense::IntelRealSense() : AddInGrabber(), m_isgrabbing(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "IntelRealSense", NULL);
    m_params.insert(paramVal.getName(), paramVal);
	
	paramVal= ito::Param("mode", ito::ParamBase::String | ito::ParamBase::Readonly, "Sensor-Mode", NULL);
	m_params.insert(paramVal.getName(), paramVal);
	/*paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1279, 1279, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1023, 1023, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);*/
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
	paramVal = ito::Param("fps", ito::ParamBase::Double | ito::ParamBase::Readonly , tr("Frames per second").toLatin1().data(), NULL);
	m_params.insert(paramVal.getName(), paramVal);
    
    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
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
//! initialization of plugin
ito::RetVal IntelRealSense::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	//Initparam of selected Sensor (default=""|"color"|"stereo"|"left"|"right")
	QString selectedMode = paramsOpt->at(0).getVal<char*>();
	
	// Device context - represents the current platform w/ respect to connected devices
	rs2::context ctx;
	// List of available & connected RealSense Devices
	rs2::device_list devlist = ctx.query_devices();
	// device handler for an INTEL device
	rs2::device dev;
	// Check for NO detected devices:
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
	// Sensor construct for depth/color/etc.-cameras or IMU
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
		if (selectedMode.isEmpty())
		{
			sensor = sensorList[0];
			s_mode = sensor.get_info(RS2_CAMERA_INFO_NAME);
			*s_streamType = RS2_STREAM_ANY;
		}
		retValue += m_params["mode"].setVal<char*>(&s_mode[0]);

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
						case RS2_FORMAT_RAW10:	stream_bpp = 10;
							break;
						case RS2_FORMAT_RAW16:	stream_bpp = 16;
							break;
						case RS2_FORMAT_RAW8:	stream_bpp = 8;
							break;
						case RS2_FORMAT_UYVY:	stream_bpp = 32;
							break;
						case RS2_FORMAT_MOTION_RAW:	stream_bpp = 8;
							break;
						case RS2_FORMAT_MOTION_XYZ32F:	stream_bpp = 32;
							break;
						case RS2_FORMAT_GPIO_RAW:	stream_bpp = 8;
							break;
						case RS2_FORMAT_6DOF:	stream_bpp = 8;
							break;
						case RS2_FORMAT_DISPARITY32:	stream_bpp = 32;
							break;
						case RS2_FORMAT_Y10BPACK:	stream_bpp = 10;
							break;
						case RS2_FORMAT_DISTANCE:	stream_bpp = 32;
							break;
						case RS2_FORMAT_MJPEG:	stream_bpp = 16;
							break;
						case RS2_FORMAT_COUNT:	stream_bpp = 8;
							break;
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



		/////===============================================================================================
		////////	Functions for using the sensor to acquire frames
		//sensor.open(s_profile);
		//sensor.start([&](rs2::frame f) {});
		//sensor.stop();
		//sensor.close();
		//////////////////////////////////////////////////////////////////
		/////////////2nd Approach:	Config & Pipeline Streaming
		
		// Configuration of what sensor/stream to use for grabbing
		rs2::config cfg;
		// Pipeline, encapsulating the actual device and sensors (all avail.)
		pipe = rs2::pipeline();
		
		rs2::pipeline_profile selection;

		if (*s_streamType)
		{
			cfg.enable_stream(*s_streamType, -1);	// enables the selected stream
			selection = pipe.start(cfg);	// starts the pipeline streaming process based on cfg
			//auto data = selection.get_stream(s_streamType).as<rs2::video_stream_profile>();	// contains specific video_stream_profile parameters (width,height,fps,...)
			//int data_sizex = data.width();	// Videostream image size in X
			//int data_sizey = data.height();	// Videostream image size in Y
			//std::cout << "Pipeline Size: " << data_sizex << "x" << data_sizey << "px" << std::endl;
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
    
    //todo:
    // - disconnect the device if not yet done
    // - this funtion is considered to be the "inverse" of init.

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

    if (!retValue.containsError())
    {
        if (key == "mode")
        {
            //check the new value and if ok, assign it to the internal parameter
			
			//mode = val->getVal<char>();


            retValue += it->copyValueFrom( &(*val) );
        }
        else if (key == "format")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
        }
		else if (key == "sizex")
		{
			//check the new value and if ok, assign it to the internal parameter
			retValue += it->copyValueFrom(&(*val));
		}
		else if (key == "sizey")
		{
			//check the new value and if ok, assign it to the internal parameter
			retValue += it->copyValueFrom(&(*val));
		}
		else if (key == "bpp")
		{
			//check the new value and if ok, assign it to the internal parameter
			retValue += it->copyValueFrom(&(*val));
		}
        else
        {
            //all parameters that don't need further checks can simply be assigned
            //to the value in m_params (the rest is already checked above)
            retValue += it->copyValueFrom( &(*val) );
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
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
    // start the camera, allocate necessary buffers or do other work that is necessary
    // to prepare the camera for image acquisitions.
    
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
    
    //todo:
    // if the counter (obtained by grabberStartedCount()) drops to zero again, stop the camera, free all allocated
    // image buffers of the camera... (it is the opposite from all things that have been started, allocated... in startDevice)

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
		rs2::frameset frames = pipe.wait_for_frames();
		rs2::frame frame = frames.first(*s_streamType);	// grabbed image frame OR error
		//rs2::frame frame = frames.first_or_default(s_streamType); // grabbed image frame OR empty frame --> this is used in get_xxx_frame()

		if (frame)
		{
			int f_px = frame.get_data_size(); //frame_size in px ????? the number of bytes in frame (laut docu)
			auto f_data = frame.get_data(); //frame_data in ?????? the pointer to the start of the frame data
				//frame.get_frame_number();
			rs2::video_frame vframe = frame.as<rs2::video_frame>();
			//int bits = vf.get_bits_per_pixel();
			int bytes = vframe.get_bytes_per_pixel();
			int width = vframe.get_width();
			int height = vframe.get_height();
			retValue += m_params["sizex"].setVal<int>(width);
			retValue += m_params["sizey"].setVal<int>(height);
			int bpp;
			rs2_format format = frame.get_profile().format();
			switch (format)
			{
			case RS2_FORMAT_ANY: bpp = 32;
				break;
			case RS2_FORMAT_Z16:	bpp = 16;
				break;
			case RS2_FORMAT_DISPARITY16:	bpp = 16;
				break;
			case RS2_FORMAT_YUYV:	bpp = 32;
				break;
			case RS2_FORMAT_RGB8:	bpp = 24;
				break;
			case RS2_FORMAT_RGBA8:	bpp = 32;
				break;
			case RS2_FORMAT_Y8:	bpp = 8;
				break;
			case RS2_FORMAT_Y16:	bpp = 16;
				break;
			case RS2_FORMAT_DISPARITY32:	bpp = 32;
				break;
			case RS2_FORMAT_DISTANCE:	bpp = 32;
				break;
			default:
				bpp = 32;
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
    
    const int bufferWidth = m_params["sizex"].getVal<int>();
    const int bufferHeight = m_params["sizey"].getVal<int>();
	int desiredBpp = m_params["bpp"].getVal<int>();


    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
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
			else if (m_data.getType() == ito::tUInt32)
			{
				if (copyExternal)
				{
					retValue += externalDataObject->copyFromData2D<ito::uint32>((ito::uint32*) bufferPtr, bufferWidth, bufferHeight);
				}
				if (!copyExternal || hasListeners)
				{
					retValue += m_data.copyFromData2D<ito::uint32>((ito::uint32*) bufferPtr, bufferWidth, bufferHeight);
				}
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
// usually it is not necessary to implement the checkData method, since the default implementation from AddInGrabber is already
// sufficient.
//
// What is does:
// - it obtains the image size from sizex, sizey, bpp
// - it checks whether the rows, cols and type of m_data are unequal to the requested dimensions and type
// - if so, m_data is reallocated, else nothing is done
// - if an external data object is given (from copyVal), this object is checked in place of m_data
// - the external data object is only reallocated if it is empty, else its size or its region of interest must exactly
//    fit to the given size restrictions
//
// if you need to do further things, overload checkData and implement your version there
/*ito::RetVal IntelRealSense::checkData(ito::DataObject *externalDataObject)
{
    return ito::retOk;
}*/

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
//	ito::RetVal retVal(ito::retOk);
//	if (m_pParamsObj)
//	{
//		std::map<std::string, rs2_camera_info> test = m_pParamsObj->getAllParameters();
//		if (what & sMode)
//		{
//			int min, max, inc, val;
//
//			retVal += getParamInfo<int>(min, max, inc, val, "operation_mode");
//			//int val = m_pParamsObj->getOperationMode();
//
//
//			if (!retVal.containsError())
//			{
//				retVal += m_params["operationMode"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["operationMode"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//		}
//		if (what & sFormat)
//		{
//			int min, max, inc, val;
//
//			retVal += getParamInfo<int>(min, max, inc, val, "disparity_offset");
//
//			//int val = m_pParamsObj->getOperationMode();
//
//
//			if (!retVal.containsError())
//			{
//				retVal += m_params["disparityOffset"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["disparityOffset"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			//int val = m_pParamsObj->getDisparityOffset();
//		}
//		if (what & sStereoMatching)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "sgm_p1_edge");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["stereoMatchingP1Edge"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["stereoMatchingP1Edge"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			retVal += getParamInfo<int>(min, max, inc, val, "sgm_p1_no_edge");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["stereoMatchingP1NoEdge"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["stereoMatchingP1NoEdge"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			retVal += getParamInfo<int>(min, max, inc, val, "sgm_p2_edge");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["stereoMatchingP2Edge"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["stereoMatchingP2Edge"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			retVal += getParamInfo<int>(min, max, inc, val, "sgm_p1_no_edge");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["stereoMatchingP2NoEdge"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["stereoMatchingP2NoEdge"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			retVal += getParamInfo<int>(min, max, inc, val, "sgm_edge_sensitivity");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["stereoMatchingEdgeSensitivity"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["stereoMatchingEdgeSensitivity"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//
//
//			/*int val;
//			val = m_pParamsObj->getStereoMatchingP1Edge();
//			retVal += m_params["stereoMatchingP1Edge"].setVal<int>(val);
//			val = m_pParamsObj->getStereoMatchingP1NoEdge();
//			retVal += m_params["stereoMatchingP1NoEdge"].setVal<int>(val);
//			val = m_pParamsObj->getStereoMatchingP2Edge();
//			retVal += m_params["stereoMatchingP2Edge"].setVal<int>(val);
//			val = m_pParamsObj->getStereoMatchingP2NoEdge();
//			retVal += m_params["stereoMatchingP2NoEdge"].setVal<int>(val);
//			val = m_pParamsObj->getStereoMatchingEdgeSensitivity();
//			retVal += m_params["stereoMatchingEdgeSensitivity"].setVal<int>(val);*/
//		}
//		if (what &sMaskBorderPixels)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "mask_border_pixels_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["maskBorderPixels"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["maskBorderPixels"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			/*bool val = m_pParamsObj->getMaskBorderPixelsEnabled();
//			retVal += m_params["maskBorderPixels"].setVal<int>(val? 1 : 0);*/
//		}
//		if (what & sConsistencyCheck)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "consistency_check_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["consistencyCheck"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["consistencyCheck"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			retVal += getParamInfo<int>(min, max, inc, val, "consistency_check_sensitivity");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["consistencyCheckSensitivity"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["consistencyCheckSensitivity"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			/*bool val = m_pParamsObj->getConsistencyCheckEnabled();
//			retVal += m_params["consistencyCheck"].setVal<int>(val ? 1 : 0);
//			int i = m_pParamsObj->getConsistencyCheckSensitivity();
//			retVal += m_params["consistencyCheckSensitivity"].setVal<int>(i);*/
//		}
//		if (what & sUniquenessCheck)
//		{
//
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "uniqueness_check_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["uniquenessCheck"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["uniquenessCheck"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			retVal += getParamInfo<int>(min, max, inc, val, "uniqueness_check_sensitivity");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["uniquenessCheckSensitivity"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["uniquenessCheckSensitivity"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			/*bool val = m_pParamsObj->getUniquenessCheckEnabled();
//			retVal += m_params["uniquenessCheck"].setVal<int>(val ? 1 : 0);
//			int i = m_pParamsObj->getUniquenessCheckSensitivity();
//			retVal += m_params["uniquenessCheckSensitivity"].setVal<int>(i);*/
//
//		}
//		if (what & sTextureFilter)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "texture_filter_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["textureFilter"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["textureFilter"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			retVal += getParamInfo<int>(min, max, inc, val, "texture_filter_sensitivity");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["textureFilterSensitivity"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["textureFilterSensitivity"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*bool val = m_pParamsObj->getTextureFilterEnabled();
//			retVal += m_params["textureFilter"].setVal<int>(val ? 1 : 0);
//			int i = m_pParamsObj->getTextureFilterSensitivity();
//			retVal += m_params["textureFilterSensitivity"].setVal<int>(i);*/
//		}
//		if (what & sGapInterpolation)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "gap_interpolation_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["gapInterpolation"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["gapInterpolation"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*bool val = m_pParamsObj->getGapInterpolationEnabled();
//			retVal += m_params["gapInterpolation"].setVal<int>(val ? 1 : 0);*/
//		}
//		if (what & sNoiseReduction)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "noise_reduction_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["noiseReduction"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["noiseReduction"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*bool val = m_pParamsObj->getNoiseReductionEnabled();
//			retVal += m_params["noiseReduction"].setVal<int>(val ? 1 : 0);*/
//		}
//		if (what & sSpeckleFilterIterations)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "speckle_filter_iterations");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["speckleFilterIterations"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["speckleFilterIterations"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*int val = m_pParamsObj->getSpeckleFilterIterations();
//			retVal += m_params["speckleFilterIterations"].setVal<int>(val);*/
//		}
//		if (what & sAutoMode)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "auto_exposure_mode");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["exposureGainMode"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["exposureGainMode"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*int val = m_pParamsObj->getAutoMode();
//			retVal += m_params["exposureGainMode"].setVal<int>(val);*/
//		}
//		if (what & sAutoTargetIntensity)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "auto_target_intensity");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoTargetIntensity"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["autoTargetIntensity"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*double val = m_pParamsObj->getAutoTargetIntensity();
//			retVal += m_params["autoTargetIntensity"].setVal<double>(val);*/
//		}
//		if (what & sAutoIntensityDelta)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "auto_intensity_delta");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoIntensityDelta"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["autoIntensityDelta"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*double val = m_pParamsObj->getAutoIntensityDelta();
//			retVal += m_params["autoIntensityDelta"].setVal<double>(val);*/
//		}
//		if (what & sAutoTargetFrame)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "auto_target_frame");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoTargetFrame"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["autoTargetFrame"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*int val = m_pParamsObj->getAutoTargetFrame();
//			retVal += m_params["autoTargetFrame"].setVal<int>(val);*/
//		}
//		if (what &sAutoSkippedFrames)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "auto_skipped_frames");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoSkippedFrames"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["autoSkippedFrames"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*int val = m_pParamsObj->getAutoSkippedFrames();
//			retVal += m_params["autoSkippedFrames"].setVal<int>(val);*/
//		}
//		if (what & sAutoMaxExposureTime)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "auto_maximum_exposure_time");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoMaxExposureTime"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["autoMaxExposureTime"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*double val = m_pParamsObj->getAutoMaxExposureTime();
//			retVal += m_params["autoMaxExposureTime"].setVal<double>(val);*/
//		}
//		if (what & sAutoMaxGain)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "auto_maximum_gain");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoMaxGain"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["autoMaxGain"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*double val = m_pParamsObj->getAutoMaxGain();
//			retVal += m_params["autoMaxGain"].setVal<double>(val);*/
//		}
//		if (what & sManualExposureTime)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "manual_exposure_time");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["manualExposureTime"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["manualExposureTime"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*double val = m_pParamsObj->getManualExposureTime();
//			retVal += m_params["manualExposureTime"].setVal<double>(val);*/
//		}
//		if (what & sManualGain)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "manual_gain");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["manualGain"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["manualGain"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*double val = m_pParamsObj->getManualGain();
//			retVal += m_params["manualGain"].setVal<double>(val);*/
//		}
//		if (what & sAutoROI)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "auto_exposure_roi_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoROI"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["autoROI"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			int min1, max1, inc1, val1;
//			retVal += getParamInfo<int>(min1, max1, inc1, val1, "auto_exposure_roi_x");
//
//			int min2, max2, inc2, val2;
//			retVal += getParamInfo<int>(min2, max2, inc2, val2, "auto_exposure_roi_y");
//
//			int min3, max3, inc3, val3;
//			retVal += getParamInfo<int>(min3, max3, inc3, val3, "auto_exposure_roi_width");
//
//			int min4, max4, inc4, val4;
//			retVal += getParamInfo<int>(min4, max4, inc4, val4, "auto_exposure_roi_height");
//			if (!retVal.containsError())
//			{
//				int roi[4];
//				roi[0] = val1;
//				roi[1] = val2;
//				roi[2] = val3;
//				roi[3] = val4;
//
//				retVal += m_params["autoExposureGainControlROI"].setVal<int*>(roi, 4);
//				ito::RectMeta* meta = m_params["autoExposureGainControlROI"].getMetaT<ito::RectMeta>();
//				meta->setWidthRangeMeta(ito::RangeMeta(min3, max3, inc3));
//				meta->setHeightRangeMeta(ito::RangeMeta(min4, max4, inc4));
//			}
//
//			// m_pParamsObj->getAutoROI();
//		}
//		if (what & sMaxFrameTimeDifference)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "max_frame_time_difference_ms");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["maxFrameTimeDifference"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["maxFrameTimeDifference"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*int val = m_pParamsObj->getMaxFrameTimeDifference();
//			retVal += m_params["maxFrameTimeDifference"].setVal<int>(val);*/
//		}
//		if (what & sTrigger)
//		{
//			double min, max, inc, val;
//			retVal += getParamInfo<double>(min, max, inc, val, "trigger_frequency");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["triggerFrequency"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["triggerFrequency"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			int mini, maxi, vali, inci;
//			retVal += getParamInfo<int>(mini, maxi, inci, vali, "trigger_0_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["trigger0"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["trigger0"].getMetaT<ito::IntMeta>();
//				meta->setMin(mini);
//				meta->setMax(maxi);
//				meta->setStepSize(inci);
//			}
//			retVal += getParamInfo<int>(mini, maxi, inci, vali, "trigger_1_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["trigger1"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["trigger1"].getMetaT<ito::IntMeta>();
//				meta->setMin(mini);
//				meta->setMax(maxi);
//				meta->setStepSize(inci);
//			}
//			retVal += getParamInfo<double>(min, max, inc, val, "trigger_0_pulse_width");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["trigger0PulseWidth"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["trigger0PulseWidth"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			retVal += getParamInfo<double>(min, max, inc, val, "trigger_1_pulse_width");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["trigger1PulseWidth"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["trigger1PulseWidth"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			retVal += getParamInfo<double>(min, max, inc, val, "trigger_1_offset");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["trigger1Offset"].setVal<double>(val);
//				ito::DoubleMeta* meta = m_params["trigger1Offset"].getMetaT<ito::DoubleMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//
//			/*double val = m_pParamsObj->getTriggerFrequency();
//			retVal += m_params["triggerFrequency"].setVal<double>(val);
//			bool val1 = m_pParamsObj->getTrigger0Enabled();
//			retVal += m_params["trigger0"].setVal<int>(val1 ? 1 : 0);
//			bool val2 = m_pParamsObj->getTrigger1Enabled();
//			retVal += m_params["trigger1"].setVal<int>(val2 ? 1 : 0);
//			double val3 = m_pParamsObj->getTrigger0PulseWidth();
//			retVal += m_params["trigger0PulseWidth"].setVal<double>(val3);
//			double val4 = m_pParamsObj->getTrigger1PulseWidth();
//			retVal += m_params["trigger1PulseWidth"].setVal<double>(val4);
//			double val5 = m_pParamsObj->getTrigger1Offset();
//			retVal += m_params["trigger1Offset"].setVal<double>(val5);*/
//
//		}
//		if (what & sAutoRecalibration)
//		{
//			int min, max, inc, val;
//			retVal += getParamInfo<int>(min, max, inc, val, "auto_recalibration_enabled");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["autoRecalibration"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["autoRecalibration"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			retVal += getParamInfo<int>(min, max, inc, val, "auto_recalibration_permanent");
//			if (!retVal.containsError())
//			{
//				retVal += m_params["saveAutoRecalibration"].setVal<int>(val);
//				ito::IntMeta* meta = m_params["saveAutoRecalibration"].getMetaT<ito::IntMeta>();
//				meta->setMin(min);
//				meta->setMax(max);
//				meta->setStepSize(inc);
//			}
//			/*bool val = m_pParamsObj->getAutoRecalibrationEnabled();
//			retVal += m_params["autoRecalibration"].setVal<int>(val ? 1 : 0);
//			val = m_pParamsObj->getSaveAutoReclabration();
//			retVal += m_params["saveAutoRecalibration"].setVal<int>(val ? 1 : 0);*/
//
//		}
//		if (what & sImageFormat)
//		{
//			if (m_pImagePair && m_pParamsObj)
//			{
//				for (int i = 0; i < 2; ++i)// Changes are often only visible after the 2nd image
//				{
//					while (!m_pImageTransferObj->receiveImagePair(*m_pImagePair))
//					{
//						//wait till done
//					}
//				}
//				ImagePair::ImageFormat format1 = m_pImagePair->getPixelFormat(0);
//				ImagePair::ImageFormat format2 = m_pImagePair->getPixelFormat(1);
//				if (format1 != format2)
//				{
//					if (format1 == ImagePair::FORMAT_8_BIT_RGB || format2 == ImagePair::FORMAT_8_BIT_RGB)
//					{
//						retVal += ito::RetVal(ito::retError, 0, tr("camera delivers rgb image. This is not implemented yet").toLatin1().data());
//
//					}
//					else
//					{
//						m_params["bpp"].setVal(16);
//					}
//				}
//				else
//				{
//					if (format1 == ImagePair::FORMAT_8_BIT_MONO)
//					{
//						m_params["bpp"].setVal(8);
//					}
//					else
//					{
//						m_params["bpp"].setVal(16);
//					}
//
//				}
//
//				int width = m_pImagePair->getWidth();
//				int height = m_pImagePair->getHeight();
//				int *roi = m_params["roi"].getVal<int*>();
//				roi[0] = 0;
//				roi[1] = 0;
//				roi[2] = width;
//				roi[3] = height;
//				m_params["sizex"].setVal<int>(width);
//				m_params["sizey"].setVal<int>(height);
//			}
//			else
//			{
//				retVal += ito::RetVal(ito::retError, 0, tr("ImagePair instance not available").toLatin1().data());
//			}
//		}
//	}
//	else
//	{
//		retVal = ito::RetVal(ito::retError, 0, QString("parameter instance not callable").toLatin1().data());
//	}
//	return retVal;
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
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
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
//! method called to show the configuration dialog
/*!
    This method is called from the main thread from itom and should show the configuration dialog of the plugin.
    If the instance of the configuration dialog has been created, its slot 'parametersChanged' is connected to the signal 'parametersChanged'
    of the plugin. By invoking the slot sendParameterRequest of the plugin, the plugin's signal parametersChanged is immediately emitted with
    m_params as argument. Therefore the configuration dialog obtains the current set of parameters and can be adjusted to its values.
    
    The configuration dialog should emit reject() or accept() depending if the user wanted to close the dialog using the ok or cancel button.
    If ok has been clicked (accept()), this method calls applyParameters of the configuration dialog in order to force the dialog to send
    all changed parameters to the plugin. If the user clicks an apply button, the configuration dialog itsself must call applyParameters.
    
    If the configuration dialog is inherited from AbstractAddInConfigDialog, use the api-function apiShowConfigurationDialog that does all
    the things mentioned in this description.
    
    Remember that you need to implement hasConfDialog in your plugin and return 1 in order to signalize itom that the plugin
    has a configuration dialog.
    
    \sa hasConfDialog
*/
const ito::RetVal IntelRealSense::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogIntelRealSense(this));
}