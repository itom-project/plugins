/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "avtVimba.h"
#include "pluginVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qmessagebox.h>

#include <VimbaCPP/Include/EnumEntry.h>

#include "dockWidgetAvtVimba.h"

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
/*!
    \todo add necessary information about your plugin here.
*/

static char InitList[5] = {0, 0, 0, 0, 0};  /*!<A map with successfull initialized Cameras (max = 5) */
static char Initnum = 0;    /*!< Number of successfull initialized Cameras */ 

AvtVimbaInterface::AvtVimbaInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("AVTVimba");

    m_description = QObject::tr("todo");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This template can be used for implementing a new type of camera or grabber plugin \n\
\n\
Put a detailed description about what the plugin is doing, what is needed to get it started, limitations...";
    m_detaildescription = QObject::tr(docstring);

    m_author = "Authors of the plugin";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("The plugin's license string");
    m_aboutThis = QObject::tr(""); 

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
    ito::Param paramVal("CameraNumber", ito::ParamBase::Int, 0, 100, 0, "camera number");
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("GigEPacketSize", ito::ParamBase::Int, 0, new ito::IntMeta(0, 8228, 1), "ethernet packet size (GigE cameras only). If 0, the camera tries to startup with a value of 8228 bytes/sec (network card must have jumbo frames enabled). Else set it to 1500 bytes/sec, which is a safe setting for all GigE Ethernet network cards");
    m_initParamsOpt.append(paramVal);

	//paramVal=ito::Param("bpp", ito::ParamBase::Int, 8, 14, 8, "Bits per Pixel (Greyscale, 8, 10, 12, 14)");
    //m_initParamsOpt.append(paramVal);

	//paramVal=ito::Param("bpp", ito::ParamBase::Int, 8, 14, 8, "Bits per Pixel (Greyscale, 8, 10, 12, 14)");
    //m_initParamsOpt.append(paramVal);


	//paramVal = ito::Param("colorMode", ito::ParamBase::String, "Mono8", tr("color mode of camera (Mono8|Mono10|Mono12|Mono14, default: Mono8)").toLatin1().data());
 //   ito::StringMeta cm_meta(ito::StringMeta::String);
 //   cm_meta.addItem("Mono8");
 //   cm_meta.addItem("Mono10");
 //   cm_meta.addItem("Mono12");
 //   cm_meta.addItem("Mono14");
 //   paramVal.setMeta(&cm_meta, false);
 //   m_initParamsOpt.append(paramVal);

	/*paramVal = ito::Param("TriggerMode", ito::ParamBase::String, "Off", tr("Trigger Mode (Off|On, default Off").toLatin1().data());
    ito::StringMeta tm_meta(ito::StringMeta::String);
    tm_meta.addItem("Off");
    tm_meta.addItem("On");
    paramVal.setMeta(&tm_meta, false);
    m_initParamsOpt.append(paramVal);*/

	/*paramVal = ito::Param("TriggerSource", ito::ParamBase::String, "Freerun", tr("Trigger Source (Freerun|Software|Line1|Line2|FixedRate, default Freerun").toLatin1().data());
    ito::StringMeta ts_meta(ito::StringMeta::String);
    ts_meta.addItem("Freerun");
    ts_meta.addItem("Software");
	ts_meta.addItem("Line1");
	ts_meta.addItem("Line2");
	ts_meta.addItem("FixedRate");
    paramVal.setMeta(&ts_meta, false);
    m_initParamsOpt.append(paramVal);*/


}

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
/*!
    
*/
AvtVimbaInterface::~AvtVimbaInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimbaInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(AvtVimba) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimbaInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(AvtVimba) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(avtvimbainterface, AvtVimbaInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif




//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
/*!
    \todo add internal parameters of the plugin to the map m_params. It is allowed to append or remove entries from m_params
    in this constructor or later in the init method
*/
AvtVimba::AvtVimba() : 
	AddInGrabber(), 
	m_isgrabbing(false),  
	m_camera(CameraPtr())
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "AVTVimba", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1279, 1279, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1023, 1023, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("height of ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("CameraNumber", ito::ParamBase::Int | ito::ParamBase::In, 0, 10, 0, tr("Camera Number").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 14, 8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	//paramVal = ito::Param("shift_bits", ito::ParamBase::Int, 0, 4, 0, tr("Shiftbits in 8-bitmode only").toLatin1().data());
	//m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("intTime", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 60.0, 0.01, tr("Integrationtime of CCD [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("intTimeAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("auto-controlled integration time of CCD (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 33.0, 1.0, tr("Gain of AD in dB").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("gainAuto", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 1, tr("auto-controlled gain (on:1, off:0)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("StreamBytesPerSecond", ito::ParamBase::Int | ito::ParamBase::In, 1000000, 124000000, 124000000, tr("Bandwidth allocation for each camera. Must be adapted if multiple cameras are connected to the same ethernet adapter").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("PacketSize", ito::ParamBase::Int | ito::ParamBase::In, 500, 16384, 8228, tr("Bandwidth allocation for each camera. Must be adapted if multiple cameras are connected to the same ethernet adapter").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("DeviceTemperature", ito::ParamBase::Double | ito::ParamBase::Readonly | ito::ParamBase::In, 1.0, 100.0, 25.0, tr("Device Temperature").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("colorMode", ito::ParamBase::String, "Mono8", tr("color mode of camera (Mono8|Mono10|Mono12|Mono14, default: Mono8)").toLatin1().data());
    ito::StringMeta cm_meta(ito::StringMeta::String);
    cm_meta.addItem("Mono8");
    cm_meta.addItem("Mono10");
    cm_meta.addItem("Mono12");
    cm_meta.addItem("Mono14");
    paramVal.setMeta(&cm_meta, false);
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("TriggerMode", ito::ParamBase::String, "Off", tr("Trigger Mode (Off|On, default Off)").toLatin1().data());
    ito::StringMeta tm_meta(ito::StringMeta::String);
    tm_meta.addItem("Off");
    tm_meta.addItem("On");
    paramVal.setMeta(&tm_meta, false);
    m_params.insert(paramVal.getName(),paramVal);

	paramVal = ito::Param("TriggerSource", ito::ParamBase::String, "Freerun", tr("Trigger Source (Freerun|Software|Line1|Line2|FixedRate, default Freerun)").toLatin1().data());
    ito::StringMeta ts_meta(ito::StringMeta::String);
    ts_meta.addItem("Freerun");
    ts_meta.addItem("Software");
	ts_meta.addItem("Line1");
	ts_meta.addItem("Line2");
	ts_meta.addItem("FixedRate");
    paramVal.setMeta(&ts_meta, false);
    m_params.insert(paramVal.getName(),paramVal);

    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetAvtVimba *dw = new DockWidgetAvtVimba(this);
    
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);   
}

//----------------------------------------------------------------------------------------------------------------------------------
AvtVimba::~AvtVimba()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::checkError(VmbErrorType errCode, const char* prefix /*= NULL*/)
{
	if (errCode == VmbErrorSuccess)
	{
		return ito::retOk;
	}
    else
    {
        const char *p = (prefix ? prefix : "Vimba API error");
	    switch (errCode)
	    {
	    case VmbErrorApiNotStarted:
		    return ito::RetVal::format(ito::retError,errCode, "%s: Vimba API not started.", p);
	    case VmbErrorTimeout:
		    return ito::RetVal::format(ito::retError,errCode, "%s: Timeout while acquiring image", p);
	    case VmbErrorDeviceNotOpen:
		    return ito::RetVal::format(ito::retError,errCode, "%s: Device was not opened for usage", p);
	    default:
		    return ito::RetVal::format(ito::retError,errCode, "%s: Undefined error %i.", p, errCode);
	    }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
/*!
    \sa close
*/
ito::RetVal AvtVimba::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
	std::string name, serialNumber, DeviceID;
	FeaturePtr pFeature;
    VmbInt64_t enum_idx;
    std::string enum_name;

    int packetSize = paramsOpt->at(1).getVal<int>();

	/*CameraPtrVector cameras;
	FramePtr rpFrame;
	VmbUint32_t BufferSize, Width, Height;
	VmbUchar_t *pBuffer;

	VimbaSystem &sys = VimbaSystem::GetInstance();
	sys.Startup();
	sys.GetCameras(cameras);

	cameras[0]->Open(VmbAccessModeFull);
	cameras[0]->AcquireSingleImage(rpFrame, 2000);
	cameras[0]->Close();

	rpFrame->GetImage( pBuffer );

	FILE	*fp;
	fp = fopen("PictureMono8.pgm", "w");

	rpFrame->GetBufferSize(BufferSize);	
	rpFrame->GetWidth(Width);
	rpFrame->GetHeight(Height);

	fprintf(fp ,"P2\n# PictureMono8.pgm\n%d %d\n255\n", Width, Height);

	for(int i=0; i < (int)BufferSize; i++){
		fprintf(fp,"%d ", pBuffer[i]);
		if(((i+1) % 70)==0){
			fprintf(fp, "\n");
		}
	}

	fclose(fp);

	sys.Shutdown();*/

	unsigned int cameraNumber = static_cast<unsigned int>(paramsOpt->at(0).getVal<int>());

	m_params["CameraNumber"].setVal<int>(cameraNumber);


	VimbaSystem& sys = VimbaSystem::GetInstance();
	CameraPtrVector cameras;

	//startup API
	if (Initnum==0)
	{
		retValue += checkError(sys.Startup());
	}
	if (InitList[cameraNumber]<1)
	{

		//obtain all cameras
		if (!retValue.containsError())
		{
			retValue += checkError(sys.GetCameras(cameras));
		}

		if (!retValue.containsError())
		{
			if (cameraNumber >= 0 && cameraNumber < cameras.size())
			{
				retValue += checkError(cameras[cameraNumber]->Open(VmbAccessModeFull));

				if (!retValue.containsError())
				{
					++Initnum;
					++InitList[cameraNumber];
					m_camera = cameras[cameraNumber];
					m_camera->GetName(name);
					m_camera->GetSerialNumber(serialNumber);
					m_camera->GetInterfaceID(DeviceID);

                    AVT::VmbAPI::FeaturePtrVector v;
                    std::string blub;
                    m_camera->GetFeatures(v);
                    for (int i = 0; i < v.size(); ++i)
                    {
                        v[i]->GetName(blub);
                        std::cout << blub.data() << "\n" << std::endl;
                    }

                    retValue += checkError(m_camera->GetFeatureByName("StreamType" /*"TLType"*/, pFeature), "Vimba feature TLType");
                    if (!retValue.containsError())
                    {
                        std::string tltype;
                        pFeature->GetValue(tltype);
                        if (tltype == "GEV")
                        {
                            m_transportType = tGigE;
                        }
                        else if (tltype == "IIDC")
                        {
                            m_transportType = tFirewire;
                        }
                        else
                        {
                            retValue += ito::RetVal(ito::retError, 0, "unknown or unsupported transport type (GigE, Firewire...)");
                        }
                    }

                    

					QString identifier = QString::fromStdString(name) + " (" + QString::fromStdString(serialNumber) + ") @ " + QString::fromStdString(DeviceID);
					setIdentifier(identifier);

                }

                if (!retValue.containsError())
                {
                    if (m_transportType == tGigE)
                    {
                        ito::RetVal retGigE = checkError(m_camera->GetFeatureByName("ExposureMode", pFeature));
                        if (!retGigE.containsError())
					    {
						    retValue += setEnumFeature("ExposureMode","Timed");
					    }
					
                        retGigE = checkError(m_camera->GetFeatureByName("GVSPPacketSize", pFeature));
                        if (!retGigE.containsError())
                        {
                            if (packetSize == 0)
                            {
                                packetSize = 8228;
                            }
                            retValue += checkError(pFeature->SetValue(packetSize));
                        }
                    }
                    else if (m_transportType == tFirewire)
                    {
                    }
				}

                if (!retValue.containsError())
                {
                    retValue += checkError(m_camera->GetFeatureByName("PixelFormat", pFeature));
                    if (!retValue.containsError())
                    {
                        AVT::VmbAPI::EnumEntry f;

                        AVT::VmbAPI::EnumEntryVector pixelTypes;
                        pixelTypes.resize(1, f);
                        
                        pFeature->GetEntries(pixelTypes);
                        int minBpp = 16;
                        int maxBpp = 8;
                        for (int i = 0; i < pixelTypes.size(); ++i)
                        {
                            pixelTypes[i].GetName(enum_name);
                            pixelTypes[i].GetValue(enum_idx);
                            if (name == "Mono8")
                            {
                                minBpp = std::min(8, minBpp);
                                maxBpp = std::max(8, maxBpp);
                                m_bppEnum.bppMono8 = enum_idx;
                            }
                            else if (name == "Mono10")
                            {
                                minBpp = std::min(10, minBpp);
                                maxBpp = std::max(10, maxBpp);
                                m_bppEnum.bppMono10 = enum_idx;
                            }
                            else if (name == "Mono12")
                            {
                                minBpp = std::min(12, minBpp);
                                maxBpp = std::max(12, maxBpp);
                                m_bppEnum.bppMono12 = enum_idx;
                            }
                            else if (name == "Mono14")
                            {
                                minBpp = std::min(14, minBpp);
                                maxBpp = std::max(14, maxBpp);
                                m_bppEnum.bppMono14 = enum_idx;
                            }
                        }

                        if (minBpp > maxBpp)
                        {
                            retValue += ito::RetVal(ito::retError, 0, "error determining the possible bit depths. No monochrome bitdepths are available");
                        }
                        else
                        {
                            enum_idx = m_bppEnum.bppMono14;
                            if (enum_idx == -1) m_bppEnum.bppMono12;
                            if (enum_idx == -1) m_bppEnum.bppMono10;
                            if (enum_idx == -1) m_bppEnum.bppMono8;
                            retValue += setEnumFeature("PixelFormat", enum_idx);
                            m_params["bpp"].setMeta(new ito::IntMeta(minBpp, maxBpp, 2), true);
                        }
                    }
                }

                if (!retValue.containsError())
                {
                    retValue += sychronizeParameters(fAll);
                }
			}
			else
			{
				retValue += ito::RetVal::format(ito::retError,0,"cameraNumber %i is out of range [0,%i]", cameraNumber, cameras.size()-1);
			}
		}
	}
	else
	{
		retValue += ito::RetVal::format(ito::retError,0,"cameraNumber %i already opened", cameraNumber);
	}
	if (!retValue.containsError())
	{
		//// Set the GeV packet size to the highest possible value
  //      // (In this example we do not test whether this cam actually is a GigE cam)
  //      FeaturePtr pCommandFeature;
  //      if ( VmbErrorSuccess == m_camera->GetFeatureByName( "GVSPAdjustPacketSize", pCommandFeature ))
  //      {
  //          if ( VmbErrorSuccess == pCommandFeature->RunCommand() )
  //          {
  //              bool bIsCommandDone = false;
  //              do
  //              {
  //                  if ( VmbErrorSuccess != pCommandFeature->IsCommandDone( bIsCommandDone ))
  //                  {
  //                      break;
  //                  }
  //              } while ( false == bIsCommandDone );
  //          }
  //      }


		VmbInt64_t offset_x, offset_y, width, height, widthMax, heightMax,PackSize;
		double exptime,max,min;
		std::string trMode,trSource;

		retValue += getRange("ExposureTimeAbs",max,min);
		retValue += getIntFeatureByName("OffsetX", offset_x);
		retValue += getIntFeatureByName("OffsetY", offset_y);
		retValue += getIntFeatureByName("Width", width);
		retValue += getIntFeatureByName("Height", height);
		retValue += getIntFeatureByName("WidthMax", widthMax);
		retValue += getIntFeatureByName("HeightMax", heightMax);
		retValue += getDblFeatureByName("ExposureTimeAbs", exptime);
		retValue += getEnumFeatureByName("TriggerMode", trMode, enum_idx);
		retValue += getIntFeatureByName("GVSPPacketSize", PackSize);
		//retValue += getIntFeatureByName("GVSPPacketSize", PackSize);


		if (getEnumFeatureByName("TriggerSource", trSource, enum_idx)==0)
		{
		    m_params["TriggerSource"].setVal<char*>((char*)trSource.c_str());
		}
		

		if (!retValue.containsError())
		{
			m_params["x0"].setMeta(new ito::IntMeta(0, offset_x + width), true);
			m_params["x0"].setVal<int>(offset_x);

			m_params["y0"].setMeta(new ito::IntMeta(0, offset_y + height), true);
			m_params["y0"].setVal<int>(offset_y);

			m_params["x1"].setMeta(new ito::IntMeta(offset_x, offset_x + width), true);
			m_params["x1"].setVal<int>(offset_x + width);

			m_params["y1"].setMeta(new ito::IntMeta(offset_x, offset_x + width), true);
			m_params["y1"].setVal<int>(offset_y + height);

			m_params["sizex"].setMeta(new ito::IntMeta(0, widthMax), true);
			m_params["sizex"].setVal<int>(width);

			m_params["sizey"].setMeta(new ito::IntMeta(0, heightMax), true);
			m_params["sizey"].setVal<int>(height);

			m_params["intTime"].setMeta(new ito::DoubleMeta((min+1)/1000000, (max-1)/1000000,0.000001), true);
			m_params["intTime"].setVal<double>(exptime/100000);

			
			m_params["TriggerMode"].setVal<char*>((char*)trMode.c_str());

			m_params["PacketSize"].setVal<int>(PackSize);

			m_params["StreamBytesPerSecond"].setVal<int>(124000000);
			
			/*ito::Param paramVal("TriggerMode",ito::ParamBase::String | ito::ParamBase::In);
			paramVal = ito::Param("TriggerMode", ito::ParamBase::String | ito::ParamBase::In,trModec, tr("Trigger Mode (Off|On, default Off)").toLatin1().data());
			m_params.insert(paramVal.getName(),paramVal);*/

			//paramVal = ito::Param("TriggerSource",ito::ParamBase::String | ito::ParamBase::In);
			/*paramVal = ito::Param("TriggerSource", ito::ParamBase::String | ito::ParamBase::In,trSourcec, tr("Trigger Source").toLatin1().data());
			m_params.insert(paramVal.getName(),paramVal);*/

			//ito::StringMeta *sm = (ito::StringMeta*)(paramDbl.getMeta());
			//m_params["TriggerMode"].setVal<char*>(trMode);
		}

		//int pixelFormat = paramsOpt->at(1).getVal<int>();
		//m_params["bpp"].setVal<int>(pixelFormat);

		
		//retValue += getEnumFeatureByName("PixelFormat", pixelFormat);

		//if (!retValue.containsError())
		//{
		//	char* colorMode = paramsOpt->at(1).getVal<char*>();
		//	if (strcmp(colorMode, "Mono8")==0)
		//	{
		//		m_params["bpp"].setVal<int>(8);
		//	}
		//	else if (strcmp(colorMode, "Mono10")==0)
		//	{
		//		m_params["bpp"].setVal<int>(10);
		//	}
		//	else if (strcmp(colorMode, "Mono12")==0)
		//	{
		//		m_params["bpp"].setVal<int>(12);
		//	}
		//	else if (strcmp(colorMode, "Mono14")==0)
		//	{
		//		m_params["bpp"].setVal<int>(14);
		//	}
		//	//TODO
		//	else 
		//	{
		//		retValue += ito::RetVal::format(ito::retError,0,"PixelFormat %s currently not supported", colorMode);
		//	}

		//	char* triggerMode = paramsOpt->at(2).getVal<char*>();
		//	if (strcmp(triggerMode, "Off")==0 || strcmp(triggerMode,"On")==0)
		//	{
		//		//QVector<QSharedPointer<ito::ParamBase> > values;
		//		m_params.insert("triggerMode",triggerMode);
		//		//values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("triggerMode", ito::ParamBase::String, triggerMode)));
		//		//m_params["triggerMode"].setVal<char*>(8);
		//	}
		//	else
		//	{
		//		retValue += ito::RetVal::format(ito::retError,0,"Invalid TriggerMode", colorMode);
		//	}	

		//	char* triggerSource = paramsOpt->at(3).getVal<char*>();
		//	if (strcmp(triggerSource, "Freerun")==0 || strcmp(triggerSource,"Software")==0 || strcmp(triggerSource,"Line1")==0 || strcmp(triggerSource,"Line2")==0 || strcmp(triggerSource,"FixedRate")==0)
		//	{
		//		//QVector<QSharedPointer<ito::ParamBase> > values;
		//		m_params.insert("triggerSource",triggerSource);
		//	}
		//	else
		//	{
		//		retValue += ito::RetVal::format(ito::retError,0,"Invalid TriggerSource", colorMode);
		//	}


		//}

			/*FeaturePtr triggerSourceFeature;
		retValue += checkError(m_camera->GetFeatureByName("TriggerSource", triggerSourceFeature));
		retValue += checkError(triggerSourceFeature->SetValue("Freerun"));
		getEnumFeatureByName("TriggerSource", pixelFormat);*/

		//int i=1;

		//TODO
	}

    //steps todo:
    // - get all initialization parameters
    // - try to detect your device
    // - establish a connection to the device
    // - synchronize the current parameters of the device with the current values of parameters inserted in m_params
    // - if an identifier string of the device is available, set it via setIdentifier("yourIdentifier")
    // - call checkData() in order to reconfigure the temporary image buffer m_data (or other structures) depending on the current size, image type...
    // - call emit parametersChanged(m_params) in order to propagate the current set of parameters in m_params to connected dock widgets...
    // - call setInitialized(true) to confirm the end of the initialization (even if it failed)
    
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

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
/*!
    \sa init
*/
ito::RetVal AvtVimba::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    //todo:
    // - disconnect the device if not yet done
    // - this funtion is considered to be the "inverse" of init.

	int nr = m_params["CameraNumber"].getVal<int>();
	

	if (m_camera.get())
	{
		retValue += checkError(m_camera->Close());
		m_camera = CameraPtr();
		InitList[nr] = 0;
		--Initnum;
	}
	if (Initnum<1)
	{
		VimbaSystem& sys = VimbaSystem::GetInstance();
	

		//shutdown API
		retValue += checkError(sys.Shutdown());
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getIntFeatureByName(const char *name, VmbInt64_t &value)
{
	ito::RetVal retValue;
	FeaturePtr pFeature;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
	{
		retValue += checkError(pFeature->GetValue(value));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal  AvtVimba::getIntFeatureByName(const char *name, VmbInt64_t &value, VmbInt64_t &max, VmbInt64_t &min, VmbInt64_t &inc)
{
    ito::RetVal retValue;
	FeaturePtr pFeature;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
	{
		retValue += checkError(pFeature->GetValue(value));
        retValue += checkError(pFeature->GetRange(min, max));
        retValue += checkError(pFeature->GetIncrement(inc));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getDblFeatureByName(const char *name, double &value)
{
	ito::RetVal retValue;
	FeaturePtr pFeature;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
	{
		retValue += checkError(pFeature->GetValue(value));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getEnumFeatureByName(const char *name, std::string &value, VmbInt64_t &idx)
{
	ito::RetVal retValue;
	FeaturePtr pFeature;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
	{
		retValue += checkError(pFeature->GetValue(value));
        retValue += checkError(pFeature->GetValue(idx));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::SetDblFeature(const char *name, double &fValue)
{
	FeaturePtr pFeature;
	ito::RetVal retValue;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
		{
			retValue += checkError(pFeature->SetValue(fValue));
			if (!retValue.containsError())
			{
				//std::cout << "Feature " << name << " set to " << fValue << std::endl;
			}	
		}
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::SetIntFeature(const char *name, int &iValue)
{
	FeaturePtr pFeature;
	ito::RetVal retValue;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
		{
			retValue += checkError(pFeature->SetValue(iValue));
			if (!retValue.containsError())
			{
				//std::cout << "Feature " << name << " set to " << iValue << std::endl;
			}	
		}
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::setEnumFeature(const char *name, const char *eValue)
{
	FeaturePtr pFeature;
	ito::RetVal retValue;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
		{
			retValue += checkError(pFeature->SetValue(eValue));
		}
	
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::setEnumFeature(const char *name, VmbInt64_t value)
{
    FeaturePtr pFeature;
	ito::RetVal retValue;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
		{
			retValue += checkError(pFeature->SetValue(value));
		}
	
	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getRange(const char *name, VmbInt64_t &max, VmbInt64_t &min, VmbInt64_t &inc)
{
	ito::RetVal retValue;
	FeaturePtr pFeature;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
	{
		retValue += checkError(pFeature->GetRange(min,max));
        retValue += checkError(pFeature->GetIncrement(inc));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getRange(const char *name, double &max, double &min)
{
	ito::RetVal retValue;
	FeaturePtr pFeature;

	retValue += checkError(m_camera->GetFeatureByName(name, pFeature));

	if (!retValue.containsError())
	{
		retValue += checkError(pFeature->GetRange(min,max));
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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


		 if (key == "intTime")
        {
			std::string tmpString="ExposureTimeAbs";
            //check the new value and if ok, assign it to the internal parameter
			//retValue += it->copyValueFrom( &(*val) );
			double tmpDouble;
			getDblFeatureByName("ExposureTimeAbs",tmpDouble);
			tmpDouble=tmpDouble/1000000;
			retValue+= it->setVal<double>(tmpDouble);
		 }
		 if (key == "DeviceTemperature")
        {
			std::string tmpString="DeviceTemperature";
            //check the new value and if ok, assign it to the internal parameter
			//retValue += it->copyValueFrom( &(*val) );
			double tmpDouble;
			getDblFeatureByName("DeviceTemperature",tmpDouble);
			retValue+= it->setVal<double>(tmpDouble);
			
        }
		 else if (key == "StreamBytesPerSecond")
		 {
			 std::string tmpString="StreamBytesPerSecond";
            //check the new value and if ok, assign it to the internal parameter
			//retValue += it->copyValueFrom( &(*val) );
			VmbInt64_t tmpInt;
			getIntFeatureByName("StreamBytesPerSecond",tmpInt);
			retValue+= it->setVal<int>(tmpInt);
		 }
		 else if (key == "PacketSize")
		 {
			 std::string tmpString="GVSPPacketSize";
            //check the new value and if ok, assign it to the internal parameter
			//retValue += it->copyValueFrom( &(*val) );
			VmbInt64_t tmpInt;
			getIntFeatureByName("GVSPPacketSize",tmpInt);
			retValue+= it->setVal<int>(tmpInt);
		 }

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
ito::RetVal AvtVimba::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
	FeaturePtr pFeature;
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
        if (key == "intTime")
        {
			std::string tmpString="ExposureTimeAbs";
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
			double tmpDouble = it->getVal<double>()*1000000;
			retValue += SetDblFeature("ExposureTimeAbs",tmpDouble);
			
        }
		else if (key == "colorMode")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
			char* tmpChar = it->getVal<char*>();
			if (strcmp(tmpChar, "Mono8")==0)
			{
				retValue += setEnumFeature("PixelFormat",tmpChar);
				m_params["bpp"].setVal<int>(8);
			}
			else if (strcmp(tmpChar, "Mono10")==0)
			{
				retValue += setEnumFeature("PixelFormat",tmpChar);
				m_params["bpp"].setVal<int>(10);
			}
			else if (strcmp(tmpChar, "Mono12")==0)
			{
				retValue += setEnumFeature("PixelFormat",tmpChar);
				m_params["bpp"].setVal<int>(12);
			}
			else if (strcmp(tmpChar, "Mono14")==0)
			{
				retValue += setEnumFeature("PixelFormat",tmpChar);
				m_params["bpp"].setVal<int>(14);
			}
			else
			{
				retValue += ito::RetVal::format(ito::retError,0,"PixelFormat %s currently not supported", tmpChar);
			}
			//int tmpInt = it->getVal<int>();
			//if(tmpInt == 8)
			//{
			//	SetEnumFeature("PixelFormat","Mono8");
			//}
			//else if (tmpInt==10)
			//{
			//	SetEnumFeature("PixelFormat","Mono10");
			//}
			//else if (tmpInt==12)
			//{
			//	SetEnumFeature("PixelFormat","Mono12");
			//}
			//else if (tmpInt==14)
			//{
			//	SetEnumFeature("PixelFormat","Mono14");
			//}
			//else
			//{
			//	retValue += ito::RetVal::format(ito::retError,0,"PixelFormat %i currently not supported", tmpInt);
			//}
        }
				else if (key == "TriggerMode")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
			char* tmpChar = it->getVal<char*>();
			if (strcmp(tmpChar, "Off")==0 || strcmp(tmpChar,"On")==0)
			{
				retValue += setEnumFeature("TriggerMode",tmpChar);
			}
			else
			{
				retValue += ito::RetVal::format(ito::retError,0,"TriggerMode %s invalid", tmpChar);
			}
		}
				else if (key == "TriggerSource")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
			char* tmpChar = it->getVal<char*>();
			if (strcmp(tmpChar, "Freerun")==0 || strcmp(tmpChar, "Software")==0 || strcmp(tmpChar, "Line1")==0 || strcmp(tmpChar, "Line2")==0 || strcmp(tmpChar, "FixedRate")==0)
			{
				retValue += setEnumFeature("TriggerSource",tmpChar);
			}
			else
			{
				retValue += ito::RetVal::format(ito::retError,0,"TriggerSouce %s invalid", tmpChar);
			}
		}
		else if (key == "StreamBytesPerSecond")
		{
			retValue += it->copyValueFrom( &(*val) );
			int tmpInt = it->getVal<int>();
			retValue += SetIntFeature("StreamBytesPerSecond",tmpInt);
		}
		else if (key == "PacketSize")
		{
			retValue += it->copyValueFrom( &(*val) );
			int tmpInt = it->getVal<int>();
			retValue += SetIntFeature("GVSPPacketSize",tmpInt);
		}
        else if (key == "demoKey2")
        {
            //check the new value and if ok, assign it to the internal parameter
            retValue += it->copyValueFrom( &(*val) );
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
ito::RetVal AvtVimba::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted(); //increment a counter to see how many times startDevice has been called
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
    
	//FramePtr rpFrame;
	//VmbUint32_t BufferSize, Width, Height;
	//m_camera->AcquireSingleImage(rpFrame, 2000);
	////cameras[0]->Close();

	///*rpFrame->GetImage( pBuffer );

	//FILE	*fp;
	//fp = fopen("PictureMono8.pgm", "w");*/

	//rpFrame->GetBufferSize(BufferSize);	
	//rpFrame->GetWidth(Width);
	//VmbFrameStatusType status;
	//rpFrame->GetReceiveStatus(status);
	//rpFrame->GetHeight(Height);

	///*fprintf(fp ,"P2\n# PictureMono8.pgm\n%d %d\n255\n", Width, Height);

	//for(int i=0; i < (int)BufferSize; i++){
	//	fprintf(fp,"%d ", pBuffer[i]);
	//	if(((i+1) % 70)==0){
	//		fprintf(fp, "\n");
	//	}
	//}

	//fclose(fp);*/

    m_acquisitionStatus = checkError(m_camera->AcquireSingleImage(m_frame, 10000));

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::retrieveData(ito::DataObject *externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);

	retValue += m_acquisitionStatus; //e.g. timeout in acquisition (from acquire)

	if (!retValue.containsError())
	{

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
				VmbUint32_t frameImgSize, bufferHeight, bufferWidth;
				VmbUchar_t *bufferPtr;
				m_frame->GetImageSize(frameImgSize);
				m_frame->GetImage(bufferPtr);
				m_frame->GetHeight(bufferHeight);
				m_frame->GetWidth(bufferWidth);

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
				else
				{
					retValue += ito::RetVal(ito::retError, 1002, tr("copying image buffer not possible since unsupported type.").toLatin1().data());
				}
			}
		}
	}

	m_isgrabbing = false;

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
/*ito::RetVal MyGrabber::checkData(ito::DataObject *externalDataObject)
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
ito::RetVal AvtVimba::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal AvtVimba::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
/*!
    Overwrite this method if the plugin has a dock widget. If so, you can connect the parametersChanged signal of the plugin
    with the dock widget once its becomes visible such that no resources are used if the dock widget is not visible. Right after
    a re-connection emit parametersChanged(m_params) in order to send the current status of all plugin parameters to the dock widget.
*/
void AvtVimba::dockWidgetVisibilityChanged(bool visible)
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
const ito::RetVal AvtVimba::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogAvtVimba(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal AvtVimba::sychronizeParameters(int features)
{
    ito::RetVal retval, ret_;
    VmbInt64_t enumIdx;
    std::string enumVal;

    if (features & AvtVimba::fBpp)
    {
        ret_ += getEnumFeatureByName("PixelFormat", enumVal, enumIdx);
        if (!ret_.containsError())
        {
            if (enumIdx == m_bppEnum.bppMono8)
            {
                m_params["bpp"].setVal<int>(8);
            }
            if (enumIdx == m_bppEnum.bppMono10)
            {
                m_params["bpp"].setVal<int>(10);
            }
            if (enumIdx == m_bppEnum.bppMono12)
            {
                m_params["bpp"].setVal<int>(12);
            }
            if (enumIdx == m_bppEnum.bppMono14)
            {
                m_params["bpp"].setVal<int>(14);
            }
        }

        retval += ret_;
    }

    if (features & fSize)
    {
        VmbInt64_t x0, x0_min, x0_max, x0_inc;
        VmbInt64_t y0, y0_min, y0_max, y0_inc;
        VmbInt64_t w, w_min, w_max, w_inc;
        VmbInt64_t h, h_min, h_max, h_inc;
        VmbInt64_t width_max, height_max;

        ret_ = getIntFeatureByName("OffsetX", x0, x0_max, x0_min, x0_inc);
        ret_ += getIntFeatureByName("OffsetY", y0, y0_max, y0_min, y0_inc);
		ret_ += getIntFeatureByName("Width", w, w_max, w_min, w_inc);
		ret_ += getIntFeatureByName("Height", h, h_max, h_min, h_inc);
		ret_ += getIntFeatureByName("WidthMax", width_max);
		ret_ += getIntFeatureByName("HeightMax", height_max);

        if (!ret_.containsError())
        {
            m_params["x0"].setMeta(new ito::IntMeta(x0_min, x0 + w, x0_inc), true);
	        m_params["x0"].setVal<int>(x0);

            m_params["y0"].setMeta(new ito::IntMeta(y0_min, y0 + h, y0_inc), true);
	        m_params["y0"].setVal<int>(y0);

			m_params["x1"].setMeta(new ito::IntMeta(x0 + w_min - 1, width_max - 1, w_inc), true);
			m_params["x1"].setVal<int>(x0 + w);

			m_params["y1"].setMeta(new ito::IntMeta(y0 + h_min - 1, height_max - 1, h_inc), true);
			m_params["y1"].setVal<int>(y0 + h);

			m_params["sizex"].setMeta(new ito::IntMeta(w_min, width_max, w_inc), true);
			m_params["sizex"].setVal<int>(w);

			m_params["sizey"].setMeta(new ito::IntMeta(h_min, height_max, h_inc), true);
			m_params["sizey"].setVal<int>(h);
        }

        retval += ret_;
    }

    if (features & fBinning)
    {
        retval += ret_;
    }

    return retval;
}