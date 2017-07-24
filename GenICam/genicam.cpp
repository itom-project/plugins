/* ********************************************************************
    Plugin "OpenCV-Grabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#include "genicam.h"
#include "pluginVersion.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include <qfileinfo.h>
#include <qdir.h>
#include <qprocess.h>
#include <PFNC.h>

#include "dockWidgetGenicam.h"

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamInterface::getAddInInst(ito::AddInBase **addInInst)
{
	NEW_PLUGININSTANCE(GenICamClass)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamInterface::closeThisInst(ito::AddInBase **addInInst)
{
	REMOVE_PLUGININSTANCE(GenICamClass)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
GenICamInterface::GenICamInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("GenICam");

    m_description = QObject::tr("Camera control of devices that support the GenICam standard");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This plugins is a wrapper for the GenICam 3 interface and is able to control cameras, that \n\
support the GenICam standard. The connection is realized by a transport layer library of the specific camera\n\
vendor. This library has the file suffix .cti and must be in the same binary representation (32/64bit) than itom and this plugin.\n\
\n\
Usually, the locations of cti-files are contained in the environment variable GENICAM_GENTL64_PATH or GENICAM_GENTL32_PATH. \n\
Indicate the path to your desired transport layer in order to access a camera. If the parameter GenTLProducer is kept empty, \n\
a list of all detected cti-files is printed to the command line. Every transport layer can support one or multiple interfaces (USB3, GigE, Firewire...).\n\
Indicate the right interface or leave 'interface' empty, in order to get a list of all supported interfaces by the selected transport layer.\n\
\n\
In order to keep this plugin compatible to other camera plugins, the additional parameters 'integration_time', 'roi', 'sizex', 'sizey', \n\
and 'bpp' are added to the plugin are kept synchronized with 'ExposureTime', 'Width', 'Height', 'OffsetX', 'OffsetY' or 'PixelFormat'. \n\
\n\
Up to now the following pixel formats are supported: Mono8, Mono10, Mono12, Mono14, Mono16 and Mono12Packed. \n\
\n\
This plugin has been tested with the following cameras: \n\
\n\
* Allied Vision, Manta (Firewire) \n\
* Ximea (USB3) \n\
* Vistek, exo174MU3 (USB3)";
    m_detaildescription = QObject::tr(docstring);

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");     
    
    //m_callInitInNewThread = false; //camera must be opened in main-thread

	QString description = tr("Indicate either a string containing the vendor and model name separated with a semicolon (e.g. 'XIMEA GmbH.;xiApi') or \
the path to a cti file (GenICam GenTL transport layer) with the file suffix .cti of the respective camera driver. If nothing is indicated, \
a list of all auto-detected vendors and models is returned.");
	ito::Param paramVal = ito::Param("GenTLProducer", ito::ParamBase::String, NULL, "C:\\XIMEA\\GenTL Producer\\x64\\ximea.gentlX64.cti", description.toLatin1().data());
    m_initParamsOpt.append(paramVal);

	paramVal = ito::Param("interface", ito::ParamBase::String, NULL, "auto", tr("interface to be opened (e.g. IIDC, U3V, USB3, USB, Ethernet...). Open with an empty string to get a list of all possible interfaces for the chosen transport layer. Default: 'auto' opens the first supported interface of the chosen transport layer.").toLatin1().data());
	m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("deviceID", ito::ParamBase::String, NULL, "", tr("name of the device to be opened. Leave empty to open first detected device of given transport layer and interface.").toLatin1().data());
    m_initParamsOpt.append(paramVal);

	paramVal = ito::Param("streamIndex", ito::ParamBase::Int, 0, std::numeric_limits<int>::max(), 0, tr("index of data stream to be opened (default: 0).").toLatin1().data());
	m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
GenICamInterface::~GenICamInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(genicaminterface, GenICamInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
GenICamClass::GenICamClass() : AddInGrabber(),
	m_hasTriggerSource(false)
{
	ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::In, "GenICam", NULL);
    m_params.insert(paramVal.getName(), paramVal);

	paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 2048, \
        new ito::IntMeta(1, 2048, 1, "ImageFormatControl"), \
        tr("width of ROI").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 2048, \
        new ito::IntMeta(1, 2048, 1, "ImageFormatControl"), \
        tr("height of ROI").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, \
        new ito::IntMeta(8, 24, 2, "ImageFormatControl"), \
        tr("bitdepth in bits per pixel").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

    ito::DoubleMeta *dm = new ito::DoubleMeta(0.0000001, std::numeric_limits<double>::max(), 0.0, "AcquisitionControl");
    dm->setDisplayPrecision(4);
    dm->setRepresentation(ito::ParamMeta::Linear);
    dm->setUnit("s");
	paramVal = ito::Param("integration_time", ito::ParamBase::Double | ito::ParamBase::In, 0.5, \
        dm, tr("integration time in seconds (this parameter is mapped to the GenICam standard parameter 'ExposureTime' (in ms))").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

	int roi[] = { 0, 0, 2048, 2048 };
	paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height)").toLatin1().data());
	ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[2] - 1), ito::RangeMeta(roi[1], roi[3] - 1), "ImageFormatControl"); //RangeMeta includes the last value, therefore -1
	paramVal.setMeta(rm, true);
	m_params.insert(paramVal.getName(), paramVal);

    dm = new ito::DoubleMeta(0.000010, 3600.0, 0.0, "AcquisitionControl");
    dm->setDisplayPrecision(1);
    dm->setUnit("s");
	paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::In, 10.0, dm, tr("Timeout for acquisition in seconds.").toLatin1().data());
	m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetGenicam *dw = new DockWidgetGenicam(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
GenICamClass::~GenICamClass()
{
   m_params.clear();
}


//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal GenICamClass::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
	ParamMapIterator it;

    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retValue == ito::retOk)
    {
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
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
/*!
    \detail This method copies the value of val to to the m_params-parameter and sets the corresponding camera parameters.

    \param [in] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal GenICamClass::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
	ParamMapIterator it;
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
	bool restartNecessary = false;
	int started = grabberStartedCount();

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    ito::RetVal retValue = apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        //here the parameter val is checked if it can be casted to *it and if the
        //possible meta information requirements of *it are met.
		retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retValue.containsError())
    {
		if (key == "timeout")
		{
			m_stream->setTimeoutSec(val->getVal<double>());
			it->copyValueFrom(&(*val));
		}
		else if (key == "integration_time")
		{
			QSharedPointer<ito::ParamBase> val(new ito::ParamBase("ExposureTime", ito::ParamBase::Double, val->getVal<double>() * 1.e6));
			retValue += setParam(val, NULL);
		}
		else if (key == "bpp")
		{
			QVector<int> bitdepths;
			QStringList supportedFormatsStr;
			QVector<PfncFormat> imageFormats = m_device->supportedImageFormats(&bitdepths, &supportedFormatsStr);
			bool found = false;
			ito::RetVal ret_;
			QStringList triedFormats;

			for (int i = 0; i < bitdepths.size(); ++i)
			{
				if (bitdepths[i] == val->getVal<int>())
				{
					triedFormats << supportedFormatsStr[i];
					QSharedPointer<ito::ParamBase> val(new ito::ParamBase("PixelFormat", ito::ParamBase::String, supportedFormatsStr[i].toLatin1().data()));
					ret_ = setParam(val, NULL);
					if (ret_ == ito::retOk)
					{
						found = true;
						break;
					}
				}
			}

			if (!found)
			{
				if (triedFormats.size() > 0)
				{
					retValue += ito::RetVal::format(ito::retError, 0, "Unsupported bitdepth (either due to camera or due to plugin). Tried the following PixelFormats: %s.", triedFormats.join(";").toLatin1().data());
				}
				else
				{
					retValue += ito::RetVal::format(ito::retError, 0, "Unsupported bitdepth (either due to camera or due to plugin)");
				}
			}
		}
		else if (key == "roi")
		{
			if (hasIndex)
			{
				switch (index)
				{
				case 0: //x
				{
					QSharedPointer<ito::ParamBase> val(new ito::ParamBase("OffsetX", ito::ParamBase::Int, val->getVal<int>()));
					retValue += setParam(val, NULL);
				}
					break;
				case 1: //y
				{
					QSharedPointer<ito::ParamBase> val(new ito::ParamBase("OffsetY", ito::ParamBase::Int, val->getVal<int>()));
					retValue += setParam(val, NULL);
				}
					break;
				case 2: //width
				{
					QSharedPointer<ito::ParamBase> val(new ito::ParamBase("Width", ito::ParamBase::Int, val->getVal<int>()));
					retValue += setParam(val, NULL);
				}
					break;
				case 3: //height
				{
					QSharedPointer<ito::ParamBase> val(new ito::ParamBase("Height", ito::ParamBase::Int, val->getVal<int>()));
					retValue += setParam(val, NULL);
				}
					break;
				}
			}
			else
			{
				if (grabberStartedCount() > 0)
				{
					//not yet stopped
					setGrabberStarted(1);
					retValue += stopDevice(NULL);
					restartNecessary = true;
				}

				const int* old_roi = it->getVal<const int*>();
				const int* roi = val->getVal<const int*>();

				if (old_roi[0] >= roi[0])
				{
					//offset is decreased, do it first, then width
					QSharedPointer<ito::ParamBase> val1(new ito::ParamBase("OffsetX", ito::ParamBase::Int, roi[0]));
					QSharedPointer<ito::ParamBase> val2(new ito::ParamBase("Width", ito::ParamBase::Int, roi[2]));
					retValue += setParam(val1, NULL);
					retValue += setParam(val2, NULL);
				}
				else
				{
					//offset is increased, decrease width at first, then increase offset
					QSharedPointer<ito::ParamBase> val1(new ito::ParamBase("Width", ito::ParamBase::Int, roi[2]));
					QSharedPointer<ito::ParamBase> val2(new ito::ParamBase("OffsetX", ito::ParamBase::Int, roi[0]));
					retValue += setParam(val1, NULL);
					retValue += setParam(val2, NULL);
				}

				if (old_roi[1] >= roi[1])
				{
					//offset is decreased, do it first, then width
					QSharedPointer<ito::ParamBase> val1(new ito::ParamBase("OffsetY", ito::ParamBase::Int, roi[1]));
					QSharedPointer<ito::ParamBase> val2(new ito::ParamBase("Height", ito::ParamBase::Int, roi[3]));
					retValue += setParam(val1, NULL);
					retValue += setParam(val2, NULL);
				}
				else
				{
					//offset is increased, decrease width at first, then increase offset
					QSharedPointer<ito::ParamBase> val1(new ito::ParamBase("Height", ito::ParamBase::Int, roi[3]));
					QSharedPointer<ito::ParamBase> val2(new ito::ParamBase("OffsetY", ito::ParamBase::Int, roi[1]));
					retValue += setParam(val1, NULL);
					retValue += setParam(val2, NULL);
				}
			}
		}
		else if (m_device->isDeviceParam(it))
		{
			if (key == "PixelFormat")
			{
				//check if format is among the supported formats
				QStringList supportedFormatsStr;
				m_device->supportedImageFormats(NULL, &supportedFormatsStr);
				if (!supportedFormatsStr.contains(val->getVal<const char*>(), Qt::CaseInsensitive))
				{
					retValue += ito::RetVal::format(ito::retError, 0, "This plugin currently only supports the following pixel formats: %s", supportedFormatsStr.join("; ").toLatin1().data());
				}
				else
				{
					//redirect set param to device
					retValue += m_device->setDeviceParam(val, it);
					retValue += m_device->syncImageParameters(m_params);
					restartNecessary = m_stream->setPayloadSize(m_device->getPayloadSize());
					retValue += checkData();
				}
			}
			else if (key == "BinningHorizontal" || key == "BinningVertical" \
				|| key == "DecimationHorizontal" || key == "DecimationVertical")
			{
				//redirect set param to device
				retValue += m_device->setDeviceParam(val, it);
				retValue += m_device->syncImageParameters(m_params);
				restartNecessary = m_stream->setPayloadSize(m_device->getPayloadSize());
				retValue += checkData();
			}
			else if (key == "OffsetX" || key == "OffsetY" || key == "Width" || key == "Height")
			{
				if (grabberStartedCount() > 0)
				{
					//not yet stopped
					setGrabberStarted(1);
					retValue += stopDevice(NULL);
					restartNecessary = true;
				}

				//redirect set param to device
				retValue += m_device->setDeviceParam(val, it);
				retValue += m_device->syncImageParameters(m_params);
				m_stream->setPayloadSize(m_device->getPayloadSize());
				retValue += checkData();
			}
			else
			{
				//redirect set param to device
				retValue += m_device->setDeviceParam(val, it);
			}
		}
		else
		{
			it->copyValueFrom(&(*val));
		}

		if (restartNecessary && started > 0)
		{
			if (grabberStartedCount() > 0)
			{
				//not yet stopped
				setGrabberStarted(1);
				retValue += stopDevice(NULL);
			}
			retValue += startDevice(NULL);
			setGrabberStarted(started);
		}
    }

    if (!retValue.containsError())
    {
        //retValue += checkCameraAbilities();
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
void GenICamClass::parameterChangedTimerFired()
{
	//this method is called if the m_parameterChangedTimer emits the timout signal.
	//This is always the case, if the GenTLDevice::onParameterChanged method
	//is fired for the 'last' time. Whenever an parameter of the camera is changed,
	//its change callback method is called. However, the change of one parameter
	//can lead to changes of other parameters. Therefore, the timer is always reset
	//such that this method is only called after a slight delay of the last called
	//onParameterChanged callback.
	//std::cout << "fired\n" << std::endl;
    bool changed = false;

	/*update dependent parameters*/
	if (m_params.contains("ExposureTime"))
	{
        double newVal = m_params["ExposureTime"].getVal<double>() * 1.e-6;
        ito::Param &p = m_params["integration_time"];
        if (!qFuzzyCompare(newVal, p.getVal<double>()))
        {
            p.setVal<double>(newVal);
            changed = true;
        }
	}
	
	if (m_params.contains("OffsetX") && m_params.contains("Width"))
	{
		int* roi = m_params["roi"].getVal<int*>();
		roi[0] = m_params["OffsetX"].getVal<int>();
		roi[2] = m_params["Width"].getVal<int>();
	}
	
	if (m_params.contains("OffsetY") && m_params.contains("Height"))
	{
		int* roi = m_params["roi"].getVal<int*>();
		roi[1] = m_params["OffsetY"].getVal<int>();
		roi[3] = m_params["Height"].getVal<int>();
	}

	cacheAcquisitionParameters();

    if (changed)
    {
        emit parametersChanged(m_params);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void GenICamClass::cacheAcquisitionParameters()
{
	if (m_hasTriggerSource)
	{
		m_acquisitionCache.triggerMode = (strcmp("On", m_params["TriggerMode"].getVal<const char*>()) == 0) ? true : false;
		m_acquisitionCache.triggerSource = m_params["TriggerSource"].getVal<const char*>();
	}
	else
	{
		m_acquisitionCache.triggerMode = false;
		m_acquisitionCache.triggerSource = "Software";
		
	}

	if (m_params.contains("AcquisitionMode"))
	{
		QByteArray mode = m_params["AcquisitionMode"].getVal<const char*>();
		if (mode == "Continuous")
		{
			m_acquisitionCache.mode = AcquisitionCache::Continuous;
		}
		else if (mode == "SingleFrame")
		{
			m_acquisitionCache.mode = AcquisitionCache::SingleFrame;
		}
		else if (mode == "MultiFrame")
		{
			m_acquisitionCache.mode = AcquisitionCache::MultiFrame;
		}
		else
		{
			m_acquisitionCache.mode = AcquisitionCache::Other;
		}
	}
	else
	{
		m_acquisitionCache.mode = AcquisitionCache::Continuous;
	}
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamClass::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    //exceptionally, this init dialog is executed in the main thread of itom (m_callInitInNewThread == false in GenICamInterface)
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

	QByteArray genTlProducerFile = paramsOpt->at(0).getVal<const char*>();
	QByteArray interfaceType = paramsOpt->at(1).getVal<const char*>();
    QByteArray deviceID = paramsOpt->at(2).getVal<const char*>();
	int streamIndex = paramsOpt->at(3).getVal<int>();

	if (genTlProducerFile.endsWith(".cti"))
	{
		retValue += searchGenTLProducer(genTlProducerFile.data(), "", "");
	}
	else
	{
		QList<QByteArray> splits = genTlProducerFile.split(';');
		if (splits.size() > 1)
		{
			retValue += searchGenTLProducer("", splits[0], splits[1]);
		}
		else
		{
			retValue += searchGenTLProducer("", splits[0], "");
		}
	}
    
    if (!retValue.containsError())
    {
		m_interface = m_system->getInterface(interfaceType, retValue);
    }

    if (!retValue.containsError())
    {
        m_device = m_interface->getDevice(deviceID, true, retValue);
		
    }

	if (!retValue.containsError())
	{
		retValue += m_device->connectToGenApi(0);
	}

	if (!retValue.containsError())
	{
		m_stream = m_device->getDataStream(streamIndex, true, retValue);
	}

	if (!retValue.containsError())
	{
		m_device->setCallbackParameterChangedReceiver(this);
		retValue += m_device->syncImageParameters(m_params);
		m_stream->setTimeoutSec(m_params["timeout"].getVal<double>());
		m_stream->setPayloadSize(m_device->getPayloadSize());
		retValue += m_device->createParamsFromDevice(m_params);

		//synchronize some default parameters
		//1. integration_time <-> ExposureTime
		if (m_params.contains("ExposureTime"))
		{
			ParamMapIterator it1 = m_params.find("ExposureTime");
			ParamMapIterator it2 = m_params.find("integration_time");
			ito::DoubleMeta *dm = it2->getMetaT<ito::DoubleMeta>();
			dm->setMin(it1->getMin() * 1.e-6);
			dm->setMax(it1->getMax() * 1.e-6);
			it2->setVal<double>(it1->getVal<double>() * 1.e-6);
		}
	}

	if (!retValue.containsError())
	{
		m_hasTriggerSource = (m_params.contains("TriggerSource") && m_params.contains("TriggerMode"));
		setIdentifier(m_device->deviceName());

		retValue += checkData();
		cacheAcquisitionParameters();
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
ito::RetVal GenICamClass::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
	m_stream.clear();
	m_device.clear();
	m_interface.clear();
	m_system.clear();
    

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamClass::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted();

	if (m_stream.isNull() == false)
	{
		if (grabberStartedCount() == 1)
		{
			//first time to be started
			retValue += m_stream->allocateAndAnnounceBuffers(1, 0);
			if (!retValue.containsError())
			{
				retValue += m_stream->startAcquisition();
			}
			if (!retValue.containsError())
			{
				retValue += m_device->invokeCommandNode("AcquisitionStart", ito::retWarning);
			}
		}
	}
	else
	{
		retValue += ito::RetVal(ito::retError, 0, "Device could not be started since data stream is not available.");
	}

	if (retValue.containsError())
	{
		decGrabberStarted();
	}
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamClass::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("the grabber already had zero users.").toLatin1().data());
        setGrabberStarted(0);
    }

	if (grabberStartedCount() == 0)
	{
		retValue += m_stream->stopAcquisition();
		retValue += m_device->invokeCommandNode("AcquisitionStop", ito::retWarning);
		retValue += m_stream->revokeAllBuffers();
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamClass::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
	ItomSharedSemaphoreLocker locker(waitCond);
	ito::RetVal retValue(ito::retOk);
	bool RetCode = false;

	if (grabberStartedCount() <= 0)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("Acquisition failed since device has not been started.").toLatin1().data());
	}
	else
	{
		retValue += m_stream->queueOneBufferForAcquisition();
	}

	if (m_acquisitionCache.mode == AcquisitionCache::SingleFrame)
	{
		retValue += m_device->invokeCommandNode("AcquisitionStart", ito::retWarning);
	}
	else if (m_acquisitionCache.mode == AcquisitionCache::MultiFrame)
	{
		retValue += ito::RetVal(ito::retError, 0, tr("AcquisitionMode 'multiFrame' not yet supported").toLatin1().data());
	}

	if (!retValue.containsError())
	{
		if (m_hasTriggerSource && \
			m_acquisitionCache.triggerMode == true && \
			m_acquisitionCache.triggerSource == "Software")
		{
			retValue += m_device->invokeCommandNode("TriggerSoftware", ito::retError);
		}
	}

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }

	if (!retValue.containsError())
	{
		GenTL::BUFFER_HANDLE buffer;
		m_acquisitionRetVal = m_stream->checkForNewBuffer(buffer);

		if (!m_acquisitionRetVal.containsError())
		{
			m_acquisitionRetVal += m_stream->copyBufferToDataObject(buffer, m_data);
			m_newImageAvailable = true;
		}

		m_stream->unlockBuffer(buffer);
	}

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamClass::retrieveData(ito::DataObject *externalDataObject)
{
	//todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
	//enhance it and adjust it for your needs
	ito::RetVal retValue(ito::retOk);

	ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

	bool hasListeners = (m_autoGrabbingListeners.size() > 0);
	bool copyExternal = (externalDataObject != NULL);

	retValue += m_acquisitionRetVal; //e.g. timeout in acquisition (from acquire)

	if (!retValue.containsError())
	{

		if (m_newImageAvailable == false)
		{
			retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
		}
		else
		{
			//step 1: update size of externalDataObject. The internal one m_data is already checked after any parameter change (in synchronizeParameters method)
			if (externalDataObject)
			{
				retValue += checkData(externalDataObject);
			}

			if (!retValue.containsError() && copyExternal)
			{
				retValue += externalDataObject->deepCopyPartial(m_data);
			}

			m_newImageAvailable = false;
		}
	}

	return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GenICamClass::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    retValue += retrieveData();

    if(!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if(dObj)
        {
            (*dObj) = m_data;
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
    This method copies the recently grabbed camera frame to the given DataObject. Therefore this camera size must fit to the data structure of the 
    DataObject.

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is deep copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal GenICamClass::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }

    if(!retValue.containsError())
    {
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
    }

    if(!retValue.containsError())
    {
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
ito::RetVal GenICamClass::searchGenTLProducer(const QString &producer, const QString &vendor, const QString &model)
{
    ito::RetVal retval;
    QFileInfo ctiFile;
    QStringList ctiFiles;

    //get GENICAM_GENTL32/64_PATH environment variable
    QStringList environment = QProcess::systemEnvironment();
    QList<QDir> searchPathes;

#ifdef _WIN64
    QString envName = "GENICAM_GENTL64_PATH";
#else
    QString envName = "GENICAM_GENTL32_PATH";
#endif

    foreach(const QString &str, environment)
    {
        if (str.startsWith(envName, Qt::CaseInsensitive))
        {
            QStringList strSplit = str.split("=");
            QDir tempDir;

            if (strSplit.length() > 1)
            {
#if linux
                strSplit = strSplit[1].trimmed().split(":");
#else
                strSplit = strSplit[1].trimmed().split(";");
#endif

                foreach(const QString &str2, strSplit)
                {
                    tempDir = str2;
                    if (tempDir.exists())
                    {
                        searchPathes.append(tempDir);
                    }
                }
            }
        }
    }

    //search for one or multiple possible cti files
    if (producer != "")
    {
        if (producer.endsWith(".cti"))
        {
            ctiFile = producer;

            if (ctiFile.exists())
            {
                ctiFiles << ctiFile.canonicalFilePath();
            }
            else
            {
				retval += ito::RetVal(ito::retError, 0, tr("The transport layer file '%1' does not exist.").arg(producer).toLatin1().data());
            }
        }
        else
        {
            retval += ito::RetVal(ito::retError, 0, tr("Producer must be empty or indicate a filename that ends with '.cti'").toLatin1().data());
        }
    }
    else //search for all cti files in searchPathes
    {
        QStringList filters;
        filters << "*.cti";
        QStringList files;

        foreach(const QDir &searchPath, searchPathes)
        {
            files = searchPath.entryList(filters);
            
            foreach (const QString &file, files)
            {
                ctiFiles << searchPath.absoluteFilePath(file);
            }
        }
    }

    if (!retval.containsError() && ctiFiles.size() == 0) //no file found
    {
        retval += ito::RetVal(ito::retError, 0, tr("No valid *.cti file could be found in any search directory").toLatin1().data());
    }
    else if (!retval.containsError())
    {
        QSharedPointer<GenTLSystem> system;
        QByteArray info;
        ito::RetVal tempRetVal;
		QStringList filteredCtiFiles;
		QByteArray infoVendor, infoModel;
		int i = 0;

		if (producer == "" && vendor == "" && model == "")
		{
			std::cout << "Detected GenICam transport layers:\n-------------------------------------\n" << std::endl;
		}

		foreach(const QString &file, ctiFiles)
		{
			ito::RetVal libRetVal, tempRet;
			system = GenTLOrganizer::instance()->getSystem(file, libRetVal);

			if (producer != "")
			{
				retval += libRetVal; //if the user decided to open a specific producer cti file and if this cti file could not be loaded, the error message should be propagated to the user.
			}

			if (!libRetVal.containsError())
			{
				tempRet = ito::retOk;
				infoVendor = system->getStringInfo(GenTL::TL_INFO_VENDOR, tempRet);
				infoModel = system->getStringInfo(GenTL::TL_INFO_MODEL, tempRet);

				if (!tempRet.containsError())
				{
					if (producer == "" && vendor == "" && model == "")
					{
						i++;
						QByteArray infoId = system->getStringInfo(GenTL::TL_INFO_ID, tempRet);
						QByteArray infoVersion = system->getStringInfo(GenTL::TL_INFO_VERSION, tempRet);
						QByteArray infoTlType = system->getStringInfo(GenTL::TL_INFO_TLTYPE, tempRet);

						std::cout << i << ". Vendor: " << infoVendor.data() << ", Model: " << infoModel.data() << "\n    " << std::endl;
						std::cout << "    ID: " << infoId.data() << "; Version: " << infoVersion.data() << "; TLType: " << infoTlType.data() << "\n    ";
						std::cout << "    CTI-File: " << file.toLatin1().data() << "\n" << std::endl;
					}

					if (vendor != "" && model != "")
					{
						if (vendor.compare(infoVendor, Qt::CaseInsensitive) == 0 && \
							model.compare(infoModel, Qt::CaseInsensitive) == 0)
						{
							filteredCtiFiles << file;
						}
					}
					else if (vendor != "")
					{
						if (vendor.compare(infoVendor, Qt::CaseInsensitive) == 0)
						{
							filteredCtiFiles << file;
						}
					}
					else if (model != "")
					{
						if (model.compare(infoModel, Qt::CaseInsensitive) == 0)
						{
							filteredCtiFiles << file;
						}
					}
					else
					{
						filteredCtiFiles << file;
					}
				}
				else
				{
					filteredCtiFiles << file;
				}
			}
			else
			{
				if (producer == "" && vendor == "" && model == "")
				{
					i++;
					std::cout << i << ". Unloadable cti-file: " << file.toLatin1().data() << "\n" << std::endl;
				}

				tempRetVal += libRetVal;
			}
		}

		if (filteredCtiFiles.size() == 0)
		{
			//nothing found
			retval += tempRetVal;

			if (!retval.containsError())
			{
				retval = ito::RetVal(ito::retError, 0, "no valid *.cti transport layer file could be found or opened. Open a GenICam instance with empty vendor / transport layer and interface for a list of all detected transport layers.");
			}
		}
		else if (filteredCtiFiles.size() == 1)
		{
			m_system = GenTLOrganizer::instance()->getSystem(filteredCtiFiles[0], retval);
			retval += m_system->openSystem();
		}
		else if (producer != "" || model != "" || vendor != "")
		{
			m_system = GenTLOrganizer::instance()->getSystem(filteredCtiFiles[0], retval);
			retval += m_system->openSystem();
			retval += ito::RetVal::format(ito::retWarning, 0, "more than one possible *.cti transport layer file is available. The first '%s' has been opened.", filteredCtiFiles[0].toLatin1().data());
		}
		else
		{
			retval += ito::RetVal(ito::retError, 0, "no camera opened since no producer, vendor or model indicated.");
		}
    }

    return retval;
}