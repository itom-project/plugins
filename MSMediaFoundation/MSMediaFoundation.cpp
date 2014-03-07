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

#include "MSMediaFoundation.h"
#include "pluginVersion.h"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>



Q_DECLARE_METATYPE(ito::DataObject)

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundationInterface::getAddInInst(ito::AddInBase **addInInst)
{
    MSMediaFoundation* newInst = new MSMediaFoundation();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundationInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
      delete ((MSMediaFoundation *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundationInterface::MSMediaFoundationInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("MSMediaFoundation");

    m_description = QObject::tr("MSMediaFoundation");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"";
    m_detaildescription = QObject::tr(docstring);

    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");     
    
    m_callInitInNewThread = false; //camera must be opened in main-thread

    ito::Param paramVal = ito::Param("cameraNumber", ito::ParamBase::Int, 0, 16, 0, tr("consecutive number of the connected camera (starting with 0, default)").toLatin1().data());
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param("colorMode", ito::ParamBase::String, "auto", tr("color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)").toLatin1().data());
    ito::StringMeta meta(ito::StringMeta::String);
    meta.addItem("auto");
    meta.addItem("color");
    meta.addItem("red");
    meta.addItem("green");
    meta.addItem("blue");
    meta.addItem("gray");
    paramVal.setMeta(&meta, false);
    m_initParamsOpt.append(paramVal);

    //paramVal = ito::Param("Init-Dialog", ito::ParamBase::Int, 0, 1, 0, tr("If true, a camera selection dialog is opened during startup").toLatin1().data());
    //m_initParamsOpt.append(paramVal);

   return;
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundationInterface::~MSMediaFoundationInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(msmediafoundationinterface, MSMediaFoundationInterface)
#endif

//----------------------------------------------------------------------------------------------------------------------------------

void StopEvent(int deviceID, void *userData)
{
    VideoInput *VI = &VideoInput::getInstance();
 
    VI->closeDevice(deviceID);
}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundation::MSMediaFoundation() : AddInGrabber(), m_isgrabbing(false), m_pVI(NULL), m_deviceID(0), m_camStatusChecked(false)
{
    ito::Param paramVal("name", ito::ParamBase::String, "MSMediaFoundation", NULL);
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

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 24, 8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double | ito::ParamBase::In, 0.000010, 10.0, 0.01, tr("Integrationtime of CCD [s], does not exist for all cameras").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("brightness", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("brightness [0..1], does not exist for all cameras").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("contrast", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("contrast [0..1], does not exist for all cameras").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("saturation", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 1.0, tr("saturation [0..1], does not exist for all cameras").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("hue", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("hue [0..1], does not exist for all cameras").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.0, tr("Gain [0..1], does not exist for all cameras").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
   
    /*paramVal = ito::Param("channel", ito::ParamBase::Int, 0, 3, 0, tr("selected color channel (all available (0, default), R (1), G (2), B (3)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("colorConversion", ito::ParamBase::Int, 0, 1, 1, tr("no conversion (0), RGB->Grayscale (1, default). If the camera image only has one channel or channel>0, this parameter is ignored").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);*/

    paramVal = ito::Param("colorMode", ito::ParamBase::String, "auto", tr("color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)").toLatin1().data());
    ito::StringMeta meta(ito::StringMeta::String);
    meta.addItem("auto");
    meta.addItem("color");
    meta.addItem("red");
    meta.addItem("green");
    meta.addItem("blue");
    meta.addItem("gray");
    paramVal.setMeta(&meta, false);
    m_params.insert(paramVal.getName(), paramVal);

}

//----------------------------------------------------------------------------------------------------------------------------------
MSMediaFoundation::~MSMediaFoundation()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// Funktion to set and update data informations
ito::RetVal MSMediaFoundation::checkCameraAbilities()
{
    bool camRetVal = false;
    m_camStatusChecked = false;
    ito::RetVal retValue;

    //acquire test image in order to get knownledge about camera's abilities
    //camRetVal = m_pCam->grab();
    //camRetVal = m_pCam->retrieve(m_pDataMatBuffer);

    m_imgChannels = 3;
    m_imgCols = m_pVI->getWidth(m_deviceID);
    m_imgRows = m_pVI->getHeight(m_deviceID);
    m_imgBpp = 8;

    static_cast<ito::IntMeta*>( m_params["sizex"].getMeta() )->setMax( m_imgCols );
    static_cast<ito::IntMeta*>( m_params["sizey"].getMeta() )->setMax( m_imgRows );
    m_params["sizex"].setVal<int>(m_imgCols);
    m_params["sizey"].setVal<int>(m_imgRows);

    static_cast<ito::IntMeta*>( m_params["x0"].getMeta() )->setMax( m_imgCols-1 );
    static_cast<ito::IntMeta*>( m_params["y0"].getMeta() )->setMax( m_imgRows-1 );
    m_params["x0"].setVal<int>(0);
    m_params["y0"].setVal<int>(0);


    //m_params["bpp"].setMin(8);
    //m_params["bpp"].setMax(elemSize1*8);
    m_params["bpp"].setMeta( new ito::IntMeta(8, m_imgBpp), true);
    m_params["bpp"].setVal<int>(m_imgBpp);

    if(m_imgBpp < 8 || m_imgBpp > 32) 
    {
        retValue += ito::RetVal(ito::retError, 0, tr("unknown bpp").toLatin1().data());
    }

    m_camStatusChecked = true;

    return retValue;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \details This method copies the complete tparam of the corresponding parameter to val

    \param [in,out] val  is a input of type::tparam containing name, value and further informations
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal MSMediaFoundation::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

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
ito::RetVal MSMediaFoundation::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    QMap<QString,ito::Param>::iterator it;
    QString key;
    bool hasIndex;
    int index;
    QString suffix;

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
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "colorMode")
        {
            const char *mode = val->getVal<char*>();

            if (m_imgChannels == 1)
            {
                if (QString::compare(mode,"auto") != 0 || QString::compare(mode,"gray") != 0)
                {
                    retValue += ito::RetVal(ito::retError,0,"The connected grayscale camera cannot be operated in any colored colorMode");
                }
            }
            else if (m_imgChannels == 3 && m_imgBpp > 8)
            {
                retValue += ito::RetVal(ito::retError,0,"The connected color camera cannot output an color image since the bit depth is > 8");
            }

            if (!retValue.containsError())
            {
                switch( mode[0] )
                {
                case 'a':
                    m_colorMode = modeAuto;
                    break;
                case 'c':
                    m_colorMode = modeColor;
                    break;
                case 'g':
                    if (mode[2] == 'a')
                        m_colorMode = modeGray;
                    else
                        m_colorMode = modeGreen;
                    break;
                case 'r':
                    m_colorMode = modeRed;
                    break;
                case 'b':
                    m_colorMode = modeBlue;
                    break;
                }
            }
        }

        if (!retValue.containsError())
        {
            //here you can add specific sub-checks for every keyword and finally put the value into (*it).
            retValue += it->copyValueFrom( &(*val) );
        }

        if (key == "x0" || key == "x1")
        {
            m_params["sizex"].setVal<int>(1+ m_params["x1"].getVal<int>() - m_params["x0"].getVal<int>());
        }
        else if (key == "y0" || key == "y1")
        {
            m_params["sizey"].setVal<int>(1+ m_params["y1"].getVal<int>() - m_params["y0"].getVal<int>());
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
ito::RetVal MSMediaFoundation::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    //exceptionally, this init dialog is executed in the main thread of itom (m_callInitInNewThread == false in OpenCVGrabberInterface)
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool ret;

    m_deviceID = paramsOpt->at(0).getVal<int>();

    m_pVI = &VideoInput::getInstance();
    int numDevices = m_pVI->listDevices();

    if (m_deviceID >= numDevices)
    {
        retValue += ito::RetVal::format(ito::retError,0,tr("invalid cameraNumber. Only %i cameras found").toLatin1().data(), numDevices);
    }
    else
    {
        if (!m_pVI->setupDevice(m_deviceID, 640, 480, 60))
        {
            retValue += ito::RetVal::format(ito::retError,0,tr("Camera (%i) could not be opened").toLatin1().data(), numDevices);
        }
        else
        {
            if (m_pVI->isFrameNew(m_deviceID))
            {
                m_pVI->setEmergencyStopEvent(m_deviceID, NULL, StopEvent);
                m_camParams = m_pVI->getParameters(m_deviceID);
            }
            else
            {
                retValue += ito::RetVal::format(ito::retError,0,tr("No frame could be aquired from device %i").toLatin1().data(), numDevices);
            }
        }
    }


    if(!retValue.containsError())
    {
        m_params["sizex"].setVal<int>( m_pVI->getWidth(m_deviceID) );
        m_params["sizey"].setVal<int>( m_pVI->getHeight(m_deviceID) );
        m_pDataMatBuffer = cv::Mat(m_pVI->getHeight(m_deviceID), m_pVI->getWidth(m_deviceID), CV_8UC3);

        if(m_camParams.Brightness.Available == false)    //Brightness of the data (only for cameras).
        {
            m_params.remove("brightness");
        }
        if(m_camParams.Contrast.Available == false) //Contrast of the data (only for cameras).
        {
            m_params.remove("contrast");
        } 
        if(m_camParams.Hue.Available == false) //Hue of the data (only for cameras).
        {
            m_params.remove("hue");
        }
        if(m_camParams.Saturation.Available == false) //Saturation of the data (only for cameras).
        {
            m_params.remove("saturation");
        }
        if(m_camParams.Gain.Available == false) //Gain of the data (only for cameras).
        {
            m_params.remove("gain");
        }
        if(m_camParams.Exposure.Available == false) //Exposure of the data (only for cameras).
        {
            m_params.remove("integration_time");
        }
    }


    if(!retValue.containsError())
    {
        retValue += checkCameraAbilities();
    }

    if(!retValue.containsError())
    {
        QSharedPointer<ito::ParamBase> colorMode( new ito::ParamBase("colorMode", ito::ParamBase::String, paramsOpt->at(1).getVal<char*>()) );
        retValue += setParam( colorMode, NULL );
        
        retValue += checkData();
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
ito::RetVal MSMediaFoundation::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    if (m_timerID > 0)
    { 
        killTimer(m_timerID);
        m_timerID=0;
    }

    retValue += stopDevice(NULL);

    m_pVI->closeDevice(m_deviceID);

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted();
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted();

    if(grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("the grabber already had zero users.").toLatin1().data());
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
ito::RetVal MSMediaFoundation::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    bool RetCode = false;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Tried to acquire without starting device").toLatin1().data());
    }
    else if(m_camStatusChecked == false)
    {
        retValue += ito::RetVal(ito::retError,0,tr("Cannot acquire image since camera status is unverified").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
        m_timeout = false;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }

    if (!retValue.containsError())
    {
        int i = 0;
        while (1)
        {
            if (m_pVI->isFrameNew(m_deviceID))
            {
                m_pVI->getPixels(m_deviceID, (unsigned char *)m_pDataMatBuffer.data); 
                break;
            }

            if (++i > 30)
            {
                m_timeout = true;
                break;
            }
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    bool RetCode = false;
    cv::Mat *internalMat = NULL;

    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();
    bool resizeRequired = (x0 > 0 || y0 > 0);

    ito::DataObject *dataObj = &m_data;
    if(externalDataObject)
    {
        dataObj = externalDataObject;
    }

    bool hasListeners = false;
    if(m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else if (m_timeout == true)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Timeout while acquiring image").toLatin1().data());
    }
    else
    {
        

        if(!retValue.containsError())
        {
            //int desiredChannel = m_params["channel"].getVal<int>();
            //if(desiredChannel > 0 && m_imgChannels == 1)
            //{
            //    desiredChannel = 0; //no r,g,b channel in camera image available (grayscale camera)
            //}

            //int colorConversion = m_params["colorConversion"].getVal<int>();
            //if(colorConversion == 1 /*rgb2gray*/ && (m_imgChannels == 1 || desiredChannel > 0)) 
            //{
            //    colorConversion = 0; //grayscale camera image or selected channel -> no conversion necessary
            //}

            int desiredBpp = m_params["bpp"].getVal<int>();
            cv::Mat tempImage;

            if(m_imgCols != curxsize || m_imgRows != curysize)
            {
                resizeRequired = true;
            }

            if(m_imgBpp != 8 && m_imgBpp != 16)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: bpp other than 8 or 16 not allowed.").toLatin1().data());
            }
            else if(m_imgChannels != 1 && m_imgChannels != 3)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: channels sizes other than 1 or 3 not allowed.").toLatin1().data());
            }
            else if((desiredBpp != 8 && desiredBpp != 16))
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Error: desired bpp must be 8 or 16 bit.").toLatin1().data());
            }
            else
            {   
                //step 1. check ROI
                if(resizeRequired == false)
                {
                    tempImage = m_pDataMatBuffer;
                }
                else
                {
                    cv::Range ranges[] = { cv::Range(y0,y0+curysize), cv::Range(x0,x0+curxsize) };
                    tempImage = cv::Mat(m_pDataMatBuffer, ranges);
                }

                //step 2. check whether 3 channel color should be transformed to 1 channel grayscale
                if(m_imgChannels == 3 && m_colorMode == modeGray)
                {
                    cv::cvtColor(tempImage, tempImage, CV_BGR2GRAY, 0); //camera provides BGR images in OpenCV
                }

                //step 3: create m_data (if not yet available)
                if(externalDataObject && hasListeners)
                {
                    retValue += checkData(NULL); //update m_data
                    retValue += checkData(externalDataObject); //update external object
                }
                else
                {
                    retValue += checkData(externalDataObject); //update external object or m_data
                }

                if(!retValue.containsError())
                {

                    //step 4: check whether tempImage must be converted to other type
                    if(desiredBpp != m_imgBpp)
                    {
                        if(desiredBpp == 8)
                        {
                            tempImage.convertTo( tempImage, CV_8U );
                        }
                        else if(desiredBpp == 16)
                        {
                            tempImage.convertTo( tempImage, CV_16U );
                        }
                        else
                        {
                            retValue += ito::RetVal(ito::retError, 0, tr("Error while converting data format. Unsupported format.").toLatin1().data());
                        }
                    }
                
                    
                }
                     
                if(!retValue.containsError())
                {
                    if(tempImage.channels() == 1)
                    {
                        internalMat = (cv::Mat*)(dataObj->get_mdata()[0]);
                        tempImage.copyTo( *(internalMat) );

                        if(externalDataObject && hasListeners)
                        {
                            internalMat = (cv::Mat*)(m_data.get_mdata()[0]);
                            tempImage.copyTo( *(internalMat) );
                        }
                    }
                    else if(tempImage.channels() == 3 && (m_colorMode == modeAuto || m_colorMode == modeColor))
                    {
                        cv::Mat out[] = { *(cv::Mat*)(dataObj->get_mdata()[0]) }; //{ *(cv::Mat*)(dataObj->get_mdata()[0]) , *(cv::Mat*)(dataObj->get_mdata()[1]) , *(cv::Mat*)(dataObj->get_mdata()[2]) };
                        int fromTo[] = {0,0,1,1,2,2}; //{0,2,1,1,2,0}; //implicit BGR (camera) -> BGR (dataObject style) conversion
                        
                        //qDebug() << "tempImage.channels():" << tempImage.channels() << " elem1size:" << tempImage.elemSize1() << " elemSize:" << tempImage.elemSize() << "[" << tempImage.rows << "x" << tempImage.cols << "] depth:" << tempImage.depth();
                        //qDebug() << "out.channels():" << out[0].channels() << " elem1size:" << out[0].elemSize1() << " elemSize:" << out[0].elemSize() << "[" << out[0].rows << "x" << out[0].cols << "] depth:" << out[0].depth();

                        cv::mixChannels( &tempImage, 1, out, 1, fromTo, 3 );

                        if(externalDataObject && hasListeners)
                        {
                            cv::Mat out[] = { *(cv::Mat*)(dataObj->get_mdata()[0]) }; //{ *(cv::Mat*)(m_data.get_mdata()[0]) , *(cv::Mat*)(m_data.get_mdata()[1]) , *(cv::Mat*)(m_data.get_mdata()[2]) };
                            cv::mixChannels( &tempImage, 1, out, 1, fromTo, 3 );
                        }
                    }
                    else if(tempImage.channels() == 3) //R,G,B selection
                    {
                        cv::Mat out[] = { *(cv::Mat*)(dataObj->get_mdata()[0]) };
                        int fromTo[] = {0,0};
                        switch(m_colorMode)
                        {
                        case modeRed: fromTo[0] = 2; break; //red
                        case modeGreen: fromTo[0] = 1; break; //green
                        default /*3*/: fromTo[0] = 0; break; //blue
                        }
                        cv::mixChannels( &tempImage, 1, out, 1, fromTo, 1 );

                        if(externalDataObject && hasListeners)
                        {
                        cv::Mat out[] = { *(cv::Mat*)(m_data.get_mdata()[0]) };
                        cv::mixChannels( &tempImage, 1, out, 1, fromTo, 1 );
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal(ito::retError,0,tr("unknown color, conversion... combination in retrieveImage").toLatin1().data());
                    }
                }
            }
            
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::checkData(ito::DataObject *externalDataObject)
{
    if(!m_camStatusChecked)
    {
        return ito::RetVal(ito::retError,0,tr("current camera status is undefined").toLatin1().data());
    }

    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureChannels;
    int futureType;

    const char *colorMode = m_params["colorMode"].getVal<char*>();
    if (m_imgChannels == 1 && (m_colorMode == modeGray || m_colorMode == modeAuto))
    {
        futureChannels = 1;
    }
    else if (m_imgChannels == 3 && (m_colorMode == modeColor || m_colorMode == modeAuto))
    {
        futureChannels = 3;
    }
    else
    {
        futureChannels = 1;
    }

    int bpp = m_params["bpp"].getVal<int>();
    if(bpp <= 8 && futureChannels == 1)
    {
        futureType = ito::tUInt8;
    }
    else if(bpp <= 16 && futureChannels == 1)
    {
        futureType = ito::tUInt16;
    }
    else if(bpp <= 32 && futureChannels == 1)
    {
        futureType = ito::tInt32;
    }
    else  if(futureChannels == 1)
    {
        futureType = ito::tFloat64;
    }
    else if(futureChannels == 3 && bpp <= 8)
    {
        futureType = ito::tRGBA32;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, tr("A camera with a bitdepth > 8 cannot be operated in color mode.").toLatin1().data());            
    }

    if (futureType == ito::tRGBA32 && (m_alphaChannel.cols != futureWidth || m_alphaChannel.rows != futureHeight))
    {
        m_alphaChannel = cv::Mat(futureHeight, futureWidth, CV_8UC1, cv::Scalar(255));
    }

    if(!externalDataObject)
    {
        if(m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)futureHeight || m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight,futureWidth,futureType);   

            if (futureType == ito::tRGBA32)
            {
                //copy alpha channel to 4th channel in m_data
                const int relations[] = {0,3};
                cv::mixChannels( &m_alphaChannel, 1, (cv::Mat*)m_data.get_mdata()[0], 1, relations, 1);
            }

        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if(externalDataObject->getDims() == 0) //empty external dataObject
        {
            *externalDataObject = ito::DataObject(futureHeight,futureWidth,futureType);
        }
        else if(externalDataObject->calcNumMats () > 1)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object has more than 1 plane. It must be of right size and type or a uninitilized image.").toLatin1().data());            
        }
        else if(externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight || externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth || externalDataObject->getType() != futureType)
        {
            return ito::RetVal(ito::retError, 0, tr("Error during check data, external dataObject invalid. Object must be of right size and type or a uninitilized image.").toLatin1().data());
        }

        if (futureType == ito::tRGBA32)
        {
            //copy alpha channel to 4th channel in m_data
            const int relations[] = {0,3};
            cv::mixChannels( &m_alphaChannel, 1, (cv::Mat*)externalDataObject->get_mdata()[externalDataObject->seekMat(0)], 1, relations, 1);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal MSMediaFoundation::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal MSMediaFoundation::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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


