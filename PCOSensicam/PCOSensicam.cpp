/* ********************************************************************
    Plugin "PCOSensicam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2015, Institut für Technische Optik (ITO),
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

#include "PCOSensicam.h"
#include "pluginVersion.h"
#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetPCOSensicam.h"

#include "common/helperCommon.h"

#define PCO_ERRT_H_CREATE_OBJECT
#include "PCO_errt.h"
#undef PCO_ERRT_H_CREATE_OBJECT

#include <QElapsedTimer>

//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class PCOSensicamInterface
    \brief Small interface class for class PCOSensicam. This class contains basic information about PCOSensicam as is able to
        create one or more new instances of PCOSensicam.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! creates new instance of PCOSensicam and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created PCOSensicam-instance is stored in *addInInst
    \return retOk
    \sa PCOSensicam
*/
ito::RetVal PCOSensicamInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(PCOSensicam)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! deletes instance of PCOSensicam. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa PCOSensicam
*/
ito::RetVal PCOSensicamInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(PCOSensicam)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for interface
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: PCOSensicam) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.
*/
PCOSensicamInterface::PCOSensicamInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("PCOSensicam");

    m_description = QObject::tr("DLL for PCO-Sensicam cameras");
    
    char docstring[] = \
"The PCOSensicam is a plugin to access PCO sensicam, dicam pro and hsfc pro cameras. \n\
\n\
For compiling this plugin, set the CMake variable **PCO_SENSICAM_SDK_DIR** to the base directory of the pco.sensicam.sdk. \n\
The SDK from PCO can be downloaded from http://www.pco.de (pco Software-Development-Toolkit (SDK)). \n\
Download the SDK and install it at any location. Additionally you need to install the drivers for operating your framegrabber board.";

    m_detaildescription = QObject::tr(docstring);
    
    m_author = "M. Gronle, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("LGPL / copyright of the external DLLs belongs to PCO");
    m_aboutThis = QObject::tr("N.A.");      
    
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
    m_initParamsOpt.append( ito::Param("board_id", ito::ParamBase::Int, 0, MAXBOARD, 0, "board number that should be connected"));
    
    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
PCOSensicamInterface::~PCOSensicamInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class PCOSensicamInterface with the name PCOSensicamInterface as plugin for the Qt-System (see Qt-DOC)
Q_EXPORT_PLUGIN2(PCOSensicamInterface, PCOSensicamInterface)

//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class PCOSensicam
    \brief Class for the PCOSensicam. The PCOSensicam is able to create noisy images or simulate a typical WLI or confocal image signal.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the 
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of DialogPCOSensicam, calls the method setVals of DialogPCOSensicam, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa DialogPCOSensicam
*/
const ito::RetVal PCOSensicam::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogPCOSensicam(this, m_caminfo));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for PCOSensicam
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the PCOSensicam's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this PCOSensicam-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
PCOSensicam::PCOSensicam() : 
    AddInGrabber(),
    m_isgrabbing(false),
    m_hCamera(NULL),
    m_hEvent(NULL),
	m_isstarted(false)
{
    //qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");
    //qRegisterMetaType<ito::DataObject>("ito::DataObject");

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "PCOSensicam", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.001, 1000.0, 0.01, tr("Integrationtime of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
	paramVal = ito::Param("delay_time", ito::ParamBase::Double, 0.0, 1000.0, 0.0, tr("Delay time of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    // is not used anywhere...
    //paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.05, 10.0, 0.1, tr("Time between two frames").toLatin1().data());
    //m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1.0, 1.0, tr("Virtual gain").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1.0, 0.0, tr("Virtual offset (not available here)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 101, 101, tr("Binning of different pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 4048, 4048};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray | ito::ParamBase::In, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, 4048), ito::RangeMeta(0, 4048));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 4047, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 4047, 4047, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 16, 16, 16, tr("bits per pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 0, tr("trigger: software (0, default), external rising edge (1), external falling edge (2)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);


    //now create dock widget for this plugin
    DockWidgetPCOSensicam *dw = new DockWidgetPCOSensicam(this);

    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);

    checkData();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
PCOSensicam::~PCOSensicam()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! adds the PCO error to ito::RetVal and translates the hex error to an error text.
ito::RetVal PCOSensicam::checkError(int error)
{
    ito::RetVal retVal;

    if (error != PCO_NOERROR)
    {
        char buffer[512];
        buffer[511] = '\0';
        PCO_GetErrorText(error, buffer, 512);
        if (error & PCO_ERROR_IS_WARNING)
        {
            retVal += ito::RetVal(ito::retWarning,0,buffer);
        }
        else
        {
            retVal += ito::RetVal(ito::retError,0,buffer);
        }
    }
    return retVal;
}


//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of PCOSensicam.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal PCOSensicam::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;
    int ret = 0;

    int board_id = paramsOpt->at(0).getVal<int>();

    retVal += checkError( INITBOARD(board_id, &m_hCamera) );

    if (!retVal.containsError())
    {
        //open a connection to the camera
        //and set the camera to idle mode
        retVal += checkError( SETUP_CAMERA(m_hCamera) );
    }
    
    if(!retVal.containsError())
    {
        m_caminfo.wSize =sizeof(SC_Camera_Description);
        retVal += checkError(GET_CAMERA_DESC(m_hCamera, &m_caminfo));
    }

    if (!retVal.containsError())
    {
        m_params["name"].setVal<char*>(CAMTYPE_NAMES[m_caminfo.wCameraTypeDESC]);
        setIdentifier(QString("%1 (%2)").arg(CAMTYPE_NAMES[m_caminfo.wCameraTypeDESC]).arg( getID() ));
    }

    if (!retVal.containsError())
    {
		retVal += sychronizeParameters();
    }

    if(!retVal.containsError())
    {
        int bpp = m_caminfo.wDynResDESC;
        m_params["bpp"].setVal<int>(bpp);
        m_params["bpp"].setMeta( new ito::IntMeta(bpp,bpp), true);
    }

    if(!retVal.containsError())
    {
        checkData(); //check if image must be reallocated
    }

    if(waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();  
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! close method which is called before that this instance is deleted by the PCOSensicamInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal PCOSensicam::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    char errbuffer[400]={0};
    int ret = 0;
    ito::RetVal retVal = stopCamera();

    retVal += checkError(CLOSEBOARD(&m_hCamera));

    if(m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID = 0;
    }

    if(waitCond)
    {
        waitCond->returnValue = ito::retOk;
        waitCond->release();

        return waitCond->returnValue;
    }
    else
    {
        return ito::retOk;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
//! returns parameter of m_params with key name.
/*!
    This method copies the string of the corresponding parameter to val with a maximum length of maxLen.

    \param [in] name is the key name of the parameter
    \param [in,out] val is a shared-pointer of type char*.
    \param [in] maxLen is the maximum length which is allowed for copying to val
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal PCOSensicam::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retVal += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retVal == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retVal += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retVal.containsError())
    {
        /*if (key == "temperatures")
        {
            short ccdtemp, camtemp, powtemp;
            retVal += checkError(PCO_GetTemperature(m_hCamera, &ccdtemp, &camtemp, &powtemp));
            if (!retVal.containsError())
            {
                double temps[] = {(double)ccdtemp/10.0, (double)camtemp, (double)powtemp};
                it->setVal<double*>(temps,3);                
            }            
        }*/
        //finally, save the desired value in the argument val (this is a shared pointer!)
        *val = it.value();
    }

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! sets parameter of m_params with key name.
/*!
    This method copies the given value  to the m_params-parameter.

    \param [in] name is the key name of the parameter
    \param [in] val is the double value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal PCOSensicam::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retVal += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retVal.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retVal += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retVal.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retVal += apiValidateParam(*it, *val, false, true);
    }  

    if (!retVal.containsError())
    {
        /*if (key == "binning")
        {
            WORD newbinY = val->getVal<int>() % 100;
            WORD newbinX = (val->getVal<int>() - newbinY) / 100;
            WORD oldbinY = it->getVal<int>() % 100;
            WORD oldbinX = (it->getVal<int>() - oldbinY) / 100;

            if (newbinX < 1 || newbinY < 1)
            {
                retVal += ito::RetVal(ito::retError, 0, tr("binning in X and Y must be >= 1 (>= 101 in total)").toLatin1().data());
            }
            else if (newbinX > m_caminfo.wMaxBinHorzDESC) //linear or binary
            {
                retVal += ito::RetVal(ito::retError, 0, tr("binning in X must be in range [1,%i]").arg(m_caminfo.wMaxBinHorzDESC).toLatin1().data());
            }
            else if (newbinY > m_caminfo.wMaxBinVertDESC) //linear or binary
            {
                retVal += ito::RetVal(ito::retError, 0, tr("binning in Y must be in range [1,%i]").arg(m_caminfo.wMaxBinVertDESC).toLatin1().data());
            }
            else if (m_caminfo.wBinHorzSteppingDESC == 0) //binary (check steps)
            {
                WORD count = 1;
                while (count < newbinX)
                {
                    count <<= 1; //*=2
                }

                if (count != newbinX)
                {
                    retVal += ito::RetVal(ito::retError, 0, tr("binning in X must be in binary steps [1,2,4,8 ...]").toLatin1().data());
                }
            }
            else if (m_caminfo.wBinVertSteppingDESC == 0) //binary (check steps)
            {
                WORD count = 1;
                while (count < newbinY)
                {
                    count <<= 1; //*=2
                }

                if (count != newbinY)
                {
                    retVal += ito::RetVal(ito::retError, 0, tr("binning in Y must be in binary steps [1,2,4,8 ...]").toLatin1().data());
                }
            }

            if (!retVal.containsError())
            {
                if (grabberStartedCount() > 0)
                {
                    retVal += stopCamera();
                }

                WORD wRoiX0, wRoiY0, wRoiX1, wRoiY1;
                retVal += checkError(PCO_GetROI(m_hCamera, &wRoiX0, &wRoiY0, &wRoiX1, &wRoiY1));

                retVal += checkError(PCO_SetBinning(m_hCamera,newbinX, newbinY));

                if(!retVal.containsError())
                {
                    it->setVal<int>(newbinX*100+newbinY);

                    float factorX = (float)newbinX / (float)oldbinX;
                    float factorY = (float)newbinY / (float)oldbinY;

                    WORD newSizeX = float(1 + wRoiX1 - wRoiX0) / factorX;
                    WORD newSizeY = float(1 + wRoiY1 - wRoiY0) / factorY;
                    newSizeX -= m_caminfo.wRoiHorStepsDESC > 1 ? (newSizeX % m_caminfo.wRoiHorStepsDESC) : 0;
                    newSizeY -= m_caminfo.wRoiVertStepsDESC > 1 ? (newSizeY % m_caminfo.wRoiVertStepsDESC) : 0;
#ifndef PCO_SDK_OLD
                    newSizeX = std::max(newSizeX, WORD(m_caminfo.wMinSizeHorzDESC));
                    newSizeY = std::max(newSizeY, WORD(m_caminfo.wMinSizeVertDESC));
#endif

                    //adapt ROI to new binning, this also affects sizex, sizey
                    wRoiX0 = 1 + float(wRoiX0 - 1) / factorX;
                    wRoiX0 -= m_caminfo.wRoiHorStepsDESC > 1 ? ((wRoiX0-1) % m_caminfo.wRoiHorStepsDESC) : 0; //get back to given discrete step size
                    wRoiX0 = std::max((WORD)1, wRoiX0);

                    wRoiY0 = 1 + float(wRoiY0 - 1) / factorY;
                    wRoiY0 -= m_caminfo.wRoiVertStepsDESC > 1 ? ((wRoiY0-1) % m_caminfo.wRoiVertStepsDESC) : 0; //get back to given discrete step size
                    wRoiY0 = std::max((WORD)1, wRoiY0);

                    wRoiX1 = std::min(wRoiX0 + newSizeX - 1, m_caminfo.wMaxHorzResStdDESC / newbinX);
                    wRoiY1 = std::min(wRoiY0 + newSizeY - 1, m_caminfo.wMaxVertResStdDESC / newbinX);

                    retVal += checkError(PCO_SetROI(m_hCamera, wRoiX0, wRoiY0, wRoiX1, wRoiY1));

                    retVal += sychronizeParameters();
                }

                if (grabberStartedCount() > 0)
                {
                    retVal += startCamera();
                }
            }
                    
        }*/
        //if (key == "x0" || key == "x1" || key == "y0" || key == "y1")
        //{
        //    if (grabberStartedCount() > 0)
        //    {
        //        retVal += stopCamera();
        //    }

        //    // Adapted parameters and send out depending parameter
        //    WORD wRoiX0 = (key == "x0") ? val->getVal<int>() : m_params["x0"].getVal<int>();
        //    WORD wRoiY0 = (key == "y0") ? val->getVal<int>() : m_params["y0"].getVal<int>(); 
        //    WORD wRoiX1 = (key == "x1") ? val->getVal<int>() : m_params["x1"].getVal<int>(); 
        //    WORD wRoiY1 = (key == "y1") ? val->getVal<int>() : m_params["y1"].getVal<int>();

        //    //retVal += checkError(PCO_SetROI(m_hCamera, wRoiX0 + 1, wRoiY0 + 1, wRoiX1 + 1, wRoiY1 + 1)); //rois are 1/1-based
        //    retVal += sychronizeParameters(); //here the current roi is checked and x0,x1,y0,y1,sizex and sizey are adapted!
        //                
        //    if (grabberStartedCount() > 0)
        //    {
        //        retVal += startCamera(); 
        //    }
        //}

        if (key == "integration_time" || key == "delay_time" || key == "trigger")
        {
            if (!retVal.containsError())
            {
                it->copyValueFrom( &(*val));
            }   

			if (!retVal.containsError())
			{
				if (m_isstarted)
				{
					retVal += stopCamera();
					retVal += startCamera();
				}
			}
        }
    }

    if (!retVal.containsError())
    {
        emit parametersChanged(m_params);
    }

    retVal += checkData(); //check if image must be reallocated

    if (waitCond) 
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
// function checkdata moved into addInGrabber.cpp -> standard for all cameras / ADDA


//----------------------------------------------------------------------------------------------------------------------------------
//! With startDevice this camera is initialized.
/*!
    In the PCOSensicam, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal PCOSensicam::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal = ito::retOk;

    checkData(); //this will be reallocated in this method.

    incGrabberStarted();

    if(grabberStartedCount() == 1)
    {
        retVal += startCamera();
    }

    if(waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    return retVal;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    In this PCOSensicam, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal PCOSensicam::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal = ito::retOk;

    decGrabberStarted();
    if(grabberStartedCount() == 0)
    {
        retVal += stopCamera();        
    }
    else if(grabberStartedCount() < 0)
    {
        retVal += ito::RetVal(ito::retError, 1001, tr("StopDevice of PCOSensicam can not be executed, since camera has not been started.").toLatin1().data());
        setGrabberStarted(0);
    }


    if(waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOSensicam::stopCamera()
{
    ito::RetVal retVal;
	
	if (!m_isstarted)
	{
		retVal = checkError( STOP_COC(m_hCamera, 0) );

		retVal += checkError( REMOVE_ALL_BUFFERS_FROM_LIST(m_hCamera) );

		if (!retVal.containsError())
		{
			for (short sBufNr = 0; sBufNr < PCO_NUMBER_BUFFERS; ++sBufNr)
			{
				PCOBuffer *buffer = &(m_buffers[sBufNr]);
				retVal += checkError(FREE_BUFFER(m_hCamera, buffer->bufNr));
				buffer->bufNr = -1; //request new buffer
				buffer->bufData = NULL;
			}
		}
	}

	m_isstarted = false;

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOSensicam::startCamera()
{
    ito::RetVal retVal;

	if (m_isstarted)
	{
		retVal += stopCamera();
	}

	//simple status information
	int sta,temp1,temp2;
	int x;
	int roixmin, roixmax, roiymin, roiymax;
	int hbin, vbin;
	cam_param campar;
	int mode, submode, trig;
	char times[256];
	memset(times, 0, 256 * sizeof(char));

	retVal += checkError(GET_STATUS(m_hCamera,&sta,&temp1,&temp2));
	//printf("GET_STATUS returned 0x%x %d %d\n",sta,temp1,temp2);

	retVal += checkError(GET_CAM_PARAM(m_hCamera,&campar));
	//printf("GET_CAM_PARAM failed with error 0x%x\n",err);

	int exposure_ms = m_params["integration_time"].getVal<double>() * 1e3;
	int delay_ms = m_params["delay_time"].getVal<double>() * 1e3;
	int trigger = m_params["trigger"].getVal<int>();

	//camera parameters
	switch(campar.cam_typ)
	{
    default:
		retVal += ito::RetVal::format(ito::retError, 0, "Invalid camera typ %d found",campar.cam_typ);
     break;

    case FASTEXP:
		printf("Fast Shutter camera found\n");
		mode=M_FAST;
		submode=NORMALFAST;
		trig=trigger;

		/*int d,t;
		GET_STATUS(hdriver,&d,&t,&t);
		if((((d&0x0E00)>>9)>0x01)&&((d&0x00C0)==0x00C0))
		printf(", DOUBLE modes avaiable\n");
		else
		printf("\n");*/

		//1ms exposure time
		sprintf(times,"0,1000000,-1,-1"); 
     break;

    case LONGEXP:
		printf("Long Exposure camera found\n");
		mode=M_LONG;
		submode=NORMALLONG; //VIDEO; //NORMALLONG;
		trig=trigger;
		sprintf(times,"%i,%i,-1,-1",delay_ms,exposure_ms); 
     break;

    case OEM:
		printf("Long Exposure OEM camera found\n");
		mode=M_LONG;
		submode=NORMALLONG;
		trig=trigger;
		sprintf(times,"%i,%i,-1,-1",delay_ms,exposure_ms);  
     break;

    case LONGEXPQE:
		printf("Long Exposure QE camera found\n");
		mode=M_LONG;
		submode=NORMALLONG; //VIDEO; //NORMALLONG;
		trig=trigger;
		//10ms exposure time
		sprintf(times,"0,10,-1,-1"); 
     break;

    case FASTEXPQE:
		printf("Fast Exposure QE camera found\n");
		mode=M_LONG;
		submode=NORMALLONG; //VIDEO; //NORMALLONG;
		trig=trigger;
		//10ms exposure time
		sprintf(times,"0,10,-1,-1"); 
     break;

    case DICAM:
		printf("DicamPro camera found\n");
		mode=M_DICAM;
		submode=DPSINGLE;
		trig=trigger;
		//phosphor decay 2, mcpgain 990,0 delay,10ms exposure time
		sprintf(times,"2,990,0,1,0,0,10,0,-1,-1"); 
	}

	//binning
	int binning = m_params["binning"].getVal<int>();
	vbin = binning % 100;
	hbin = (binning - vbin) / 100;

	//roi
	const int *roi = m_params["roi"].getVal<int*>();
	int hstep = m_caminfo.wRoiHorStepsDESC;
	int vstep = m_caminfo.wRoiVertStepsDESC;

	if (hstep > 0)
	{
		roixmin = 1 + roi[0] / hstep;
		roixmax = std::ceil((float)(roi[0] + roi[2]) / (float)(hstep));
	}
	else
	{
		roixmin = 1;
		roixmax = roi[0] + roi[2];
	}

	if (vstep > 0)
	{
		roiymin = 1 + roi[1] / vstep;
		roiymax = std::ceil((float)(roi[1] + roi[3]) / (float)(vstep));
	}
	else
	{
		roiymin = 1;
		roiymax = roi[1] + roi[3];
	}

	////get camera typ 
	//switch(campar.cam_ccd)
	//{
	//default:
	//	retVal += ito::RetVal(ito::retError, 0, "Invalid camera CCD-typ found");
	//break;

	//case CCD74:
	//	printf("CCD74 640x480 black&white found\n");
	//	roixmin=roiymin=1;
	//	roixmax=20;
	//	roiymax=15;
	//	hbin=vbin=1;
	//break;

	//case CCD74C:
	//	printf("CCD74 640x480 color found\n");
	//	roixmin=roiymin=1;
	//	roixmax=20;
	//	roiymax=15;
	//	hbin=vbin=1;
	//break;

	//case CCD85:
	//	printf("CCD85 1280x1024 black&white found\n");
	//	roixmin=roiymin=1;
	//	roixmax=40;
	//	roiymax=32;
	//	hbin=vbin=1;
	//break;

	//case CCD85C:
	//	printf("CCD85 1280x1024 color found\n");
	//	roixmin=roiymin=1;
	//	roixmax=40;
	//	roiymax=32;
	//	hbin=vbin=1;
	//break;

	//case CCD285QE:
	//	printf("CCD285 1376x1040 black&white found\n");
	//	roixmin=roiymin=1;
	//	roixmax=43;
	//	roiymax=33;
	//	hbin=vbin=1;
	//break;

	//case CCD285QEF:
	//	printf("CCD285 1376x1040 black&white with fast-mode found\n");
	//	roixmin=roiymin=1;
	//	roixmax=43;
	//	roiymax=33;
	//	hbin=vbin=1;
	//break;

	//case CCD285QED:
	//	printf("CCD285 1376x1040 black&white with double-mode found\n");
	//	roixmin=roiymin=1;
	//	roixmax=43;
	//	roiymax=33;
	//	hbin=vbin=1;
	//break;

	//case CCDTIEM285:
	//	printf("CCDTIEM285 1024x1002 black&white with fast-mode found\n");
	//	roixmin=roiymin=1;
	//	roixmax=32;
	//	roiymax=32;
	//	hbin=vbin=1;
	//break;
 //  }

   if (!retVal.containsError())
   {
		//test and set camera values

		//normal mode, auto triggger binning 1x1 max roi
		//test if SET_COC gets right values 
		int len=sizeof(times);
		x = mode+(submode<<16);

		int err = TEST_COC(m_hCamera,&x,&trig,&roixmin,&roixmax,&roiymin,&roiymax,&hbin,&vbin,times,&len);

		if(err != PCO_NOERROR)
		{
			if((err&0xF000FFFF)==PCO_WARNING_SDKDLL_COC_VALCHANGE)
			{
				printf("\nTEST_COC changed some values\n");
				printf("\nSET_COC(0x%x,%d,%d,%d,%d,%d,%d,%d,\"%s\"\n",
					x,trig,roixmin,roixmax,roiymin,roiymax,hbin,vbin,times);
			}
			else if((err&0xF000FFFF)==PCO_WARNING_SDKDLL_COC_STR_SHORT)
			{
				retVal += ito::RetVal::format(ito::retError, err, "TEST_COC buffer for string not large enough need %d bytes",len);
			}
			else
			{
				retVal += ito::RetVal::format(ito::retError, err, "TEST_COC failed with error 0x%x",err);
			}
		}
   }

	if (!retVal.containsError())
	{
		//set camera values
		retVal += checkError(SET_COC(m_hCamera,x,trig,roixmin,roixmax,roiymin,roiymax,hbin,vbin,times));
	}

    int ccdxsize, ccdysize, width, height, bitpix;
    retVal += checkError(GETSIZES(m_hCamera, &ccdxsize, &ccdysize, &width, &height, &bitpix));
        
    if(!retVal.containsError())
    {
        retVal += sychronizeParameters();

        if(!retVal.containsError())
        {
            int imgsize = width*height*((bitpix+7)/8);

            for (short sBufNr = 0; sBufNr < PCO_NUMBER_BUFFERS; ++sBufNr)
            {
                PCOBuffer *buffer = &(m_buffers[sBufNr]);
                buffer->bufNr = -1; //request new buffer
                buffer->bufData = NULL;
                buffer->bufQueued = false;
                buffer->bufEvent = NULL;
                buffer->bufError = false;
                retVal += checkError(ALLOCATE_BUFFER(m_hCamera, &(buffer->bufNr), &imgsize));
                retVal += checkError(MAP_BUFFER(m_hCamera, buffer->bufNr, imgsize, 0, &(buffer->bufData)));
                retVal += checkError(SETBUFFER_EVENT(m_hCamera, buffer->bufNr, &(buffer->bufEvent)));
            }
        }
    }

    if (!retVal.containsError())
    {
        retVal += checkError(RUN_COC(m_hCamera,0)); //run with continuous mode
    }

	if (!retVal.containsError())
	{
		m_isstarted = true;
	}

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOSensicam::sychronizeParameters()
{
	int mode, trig, roix1, roix2, roiy1, roiy2, hbin, vbin;
	int ccdxsize, ccdysize, actxsize, actysize, bit_pix;
	char table[64];
	memset(table, 0, 64 * sizeof(char));
	ParamMapIterator it;

    ito::RetVal retVal = checkError(GET_COC_SETTING(m_hCamera,  &mode, &trig, &roix1, &roix2, &roiy1, &roiy2, &hbin, &vbin, table, 64));
	retVal += checkError(GETSIZES(m_hCamera, &ccdxsize, &ccdysize, &actxsize, &actysize, &bit_pix));
	int hstep = m_caminfo.wRoiHorStepsDESC;
	int vstep = m_caminfo.wRoiVertStepsDESC;

	if (!retVal.containsError())
	{
		it = m_params.find("sizex");
		it->setMeta(new ito::IntMeta(1, ccdxsize), true);
		it->setVal<int>(actxsize);

		it = m_params.find("sizey");
		it->setMeta(new ito::IntMeta(1, ccdysize), true);
		it->setVal<int>(actysize);

		it = m_params.find("x0");
		if (hstep > 0)
		{
			it->setMeta(new ito::IntMeta(0, std::min(roix2*hstep, ccdxsize) - 1, hstep), true);
		}
		else
		{
			it->setMeta(new ito::IntMeta(0, 0, 1), true);
			it->setFlags(ito::ParamBase::Readonly);
		}
		it->setVal<int>(std::min((roix1 - 1) * hstep, ccdxsize - 1));

		it = m_params.find("x1");
		if (hstep > 0)
		{
			it->setMeta(new ito::IntMeta(std::min((roix1 - 1) * hstep, ccdxsize - 1) + hstep, ccdxsize-1, hstep), true);
		}
		else
		{
			it->setMeta(new ito::IntMeta(ccdxsize-1, ccdxsize-1, 1), true);
			it->setFlags(ito::ParamBase::Readonly);
		}
		it->setVal<int>(std::min(roix2 * hstep - 1, ccdxsize - 1));

		it = m_params.find("y0");
		if (hstep > 0)
		{
			it->setMeta(new ito::IntMeta(0, std::min(roiy2*vstep, ccdysize)-1, vstep), true);
		}
		else
		{
			it->setMeta(new ito::IntMeta(0, 0, 1), true);
			it->setFlags(ito::ParamBase::Readonly);
		}
		it->setVal<int>(std::min((roiy1 - 1) * vstep, ccdysize - 1));

		it = m_params.find("y1");
		if (vstep > 0)
		{
			it->setMeta(new ito::IntMeta(std::min((roiy1 - 1) * vstep, ccdysize - 1) + vstep, ccdysize-1, vstep), true);
		}
		else
		{
			it->setMeta(new ito::IntMeta(ccdysize-1, ccdysize-1, 1), true);
			it->setFlags(ito::ParamBase::Readonly);
		}
		it->setVal<int>(std::min(roiy2 * vstep - 1, ccdysize - 1));
			
		it = m_params.find("roi");
		if (vstep > 0)
		{
			ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, ccdxsize, hstep), ito::RangeMeta(0, ccdysize, hstep));
			it->setMeta(rm, true);
		}
		else
		{
			ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, ccdxsize, ccdxsize), ito::RangeMeta(0, ccdysize, ccdysize));
			it->setMeta(rm, true);
			it->setFlags(ito::ParamBase::Readonly);
		}

		int roi[4];
		roi[0] = std::min((roix1 - 1) * hstep, ccdxsize - 1);
		roi[1] = std::min((roiy1 - 1) * vstep, ccdysize - 1);
		roi[2] = actxsize;
		roi[3] = actysize;
		it->setVal<int*>(roi, 4);
	}
	
	
	// = checkError(PCO_ArmCamera(m_hCamera));
	/*
    WORD roiX0, roiY0, roiX1, roiY1;
    WORD sizeX, sizeY, sizeXMax, sizeYMax;
    WORD binX, binY;
    retVal += checkError(PCO_GetROI(m_hCamera, &roiX0, &roiY0, &roiX1, &roiY1)); //roi starts with 1/1
    retVal += checkError(PCO_GetSizes(m_hCamera, &sizeX, &sizeY, &sizeXMax, &sizeYMax));
    retVal += checkError(PCO_GetBinning(m_hCamera, &binX, &binY));

    //maximum sizex and sizey are m_caminfo.wMaxHorzResStdDESC/binX and m_caminfo.wMaxVertResStdDESC/binY
        
    if(!retVal.containsError())
    {
        m_params["x0"].setVal<int>(roiX0 - 1);
        m_params["y0"].setVal<int>(roiY0 - 1);
        m_params["x1"].setVal<int>(roiX1 - 1);
        m_params["y1"].setVal<int>(roiY1 - 1);

        ito::IntMeta *im;

        //x0
        im = static_cast<ito::IntMeta*>( m_params["x0"].getMeta() );
        im->setMax(roiX1 - 1);
        im->setStepSize(std::max(m_caminfo.wRoiHorStepsDESC,(WORD)1));

        //x1
        im = static_cast<ito::IntMeta*>( m_params["x1"].getMeta() );
#ifdef PCO_SDK_OLD
        im->setMin(roiX0 + 1);
#else
        im->setMin(roiX0 + m_caminfo.wMinSizeHorzDESC - 2);
#endif
        im->setMax(m_caminfo.wMaxHorzResStdDESC/binX - 1);
        im->setStepSize(std::max(m_caminfo.wRoiHorStepsDESC, (WORD)1));

        //y0
        im = static_cast<ito::IntMeta*>( m_params["y0"].getMeta() );
        im->setMax(roiY1 - 1);
        im->setStepSize(std::max(m_caminfo.wRoiVertStepsDESC,(WORD)1));

        //y1
        im = static_cast<ito::IntMeta*>( m_params["y1"].getMeta() );
#ifdef PCO_SDK_OLD
        im->setMin(roiY0 + 1);
#else
        im->setMin(roiY0 + m_caminfo.wMinSizeVertDESC - 2);
#endif
        im->setMax(m_caminfo.wMaxVertResStdDESC/binY - 1);
        im->setStepSize(std::max(m_caminfo.wRoiVertStepsDESC,(WORD)1));

        m_params["sizex"].setVal<int>(sizeX);
        m_params["sizey"].setVal<int>(sizeY);
        static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(m_caminfo.wMaxHorzResStdDESC/binX);
        static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(m_caminfo.wMaxVertResStdDESC/binY);
    }
	*/
    return retVal;
}

         
//----------------------------------------------------------------------------------------------------------------------------------
//! Call this method to trigger a new image.
/*!
    By this method a new image is trigger by the camera, that means the acquisition of the image starts in the moment, this method is called.
    The new image is then stored either in internal camera memory or in internal memory of this class.

    \note This method is similar to VideoCapture::grab() of openCV

    \param [in] trigger may describe the trigger parameter (unused here)
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera has not been started or an older image lies in memory which has not be fetched by getVal, yet.
    \sa getVal
*/
ito::RetVal PCOSensicam::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal = ito::retOk;
    int ret = 0;
    WORD intrigger = 0;

    int xsize = m_params["sizex"].getVal<int>();
    int ysize = m_params["sizey"].getVal<int>();
	int bpp = m_params["bpp"].getVal<int>();
	int delay_exp_ms = 4000 + (m_params["integration_time"].getVal<double>() + m_params["delay_time"].getVal<double>()) * 2e3; //give double the theoretical time for the timeout
	int imgsize = xsize*ysize*((bpp+7)/8);

    if(grabberStartedCount() <= 0 || !m_isstarted)
    {
        retVal += ito::RetVal(ito::retError, 1002, tr("Acquire of PCOSensicam can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        m_acquisitionRetVal = ito::retOk;
        if(!retVal.containsError())
        {
			//add buffer to list in order to start acquisition
			retVal += checkError(ADD_BUFFER_TO_LIST(m_hCamera,m_buffers[0].bufNr,imgsize,0,0));
        }

        if(retVal.containsError()) // || !intrigger)
        {
            retVal += ito::retError;
            m_isgrabbing = false;
        }
        else
        {
            m_isgrabbing = true;
        }
    }

    if(waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

	if (!retVal.containsError())
	{
		DWORD stat = WaitForSingleObject(m_buffers[0].bufEvent , delay_exp_ms);
		if(stat == WAIT_OBJECT_0)
		{
			int bufstat;
			m_acquisitionRetVal += checkError(GETBUFFER_STATUS(m_hCamera, m_buffers[0].bufNr, 0, &bufstat, sizeof(bufstat)));

			if (!m_acquisitionRetVal.containsError())
			{
				switch (bpp)
				{
				case 8:
					m_data.copyFromData2D<ito::uint8>((ito::uint8*)(m_buffers[0].bufData), xsize, ysize);
					break;
				case 12:
				case 16:
					m_data.copyFromData2D<ito::uint16>((ito::uint16*)(m_buffers[0].bufData), xsize, ysize);
					break;
				}
			}

			  /*if(err!=PCO_NOERROR)
			   printf("\nError ADD_BUFFER_TO_LIST bufnr[%d] 0x%x\n",x,err);
			  printf("pic %d buffer %d done status 0x%x\r",i+1,x,bufstat);
			  if((count<=20)||((bufstat&0xFFFF)!=0x0072))
			   printf("\n");
			  ResetEvent(buffer_event[x]);
			  err=ADD_BUFFER_TO_LIST(hdriver,bufnr[x],size,0,0);
			  if(err!=PCO_NOERROR)
			   printf("\nError ADD_BUFFER_TO_LIST bufnr[%d] 0x%x\n",x,err);
			  i++;*/
		}
		else if (stat == WAIT_TIMEOUT)
		{
			m_acquisitionRetVal += ito::RetVal(ito::retError, 0, "timeout while waiting for acquired image");
			REMOVE_BUFFER_FROM_LIST(m_hCamera, m_buffers[0].bufNr); //if the buffer is still in list remove it (no error check here)
		}

		ResetEvent(m_buffers[0].bufEvent);
      
    }

    /*m_acquisitionRetVal = checkError(WAIT_FOR_IMAGE(m_hCamera, 5000));
    if (!m_acquisitionRetVal.containsError())
    {
        m_acquisitionRetVal += checkError(READ_IMAGE_12BIT(m_hCamera, 0x0000, m_data.getSize(1), m_data.getSize(0), (unsigned short*)(m_data.rowPtr(0,0))));
    }*/

	/*QElapsedTimer timer;
    timer.start();
    bool waitingSuccessful = false;
    WORD *wBuf = NULL;
    HANDLE Event = NULL;
    short bufNr = 255; //invalid

    HANDLE handles[PCO_NUMBER_BUFFERS];
    short bufNumbers[PCO_NUMBER_BUFFERS];
    DWORD ncount = 0;

    for (short i = 0; i < PCO_NUMBER_BUFFERS; ++i)
    {
        //list all buffers that could potentially fire an event with a really new image now
        if (m_buffers[i].bufQueued && !m_buffers[i].bufError)
        {
            bufNumbers[ncount] = i;
            handles[ncount++] = m_buffers[i].bufEvent;
        }
    }*/

    /*while (timer.elapsed() < timeOutMS)
    {
        ret = WaitForMultipleObjects(ncount, handles, false, 500);

        if (ret >= WAIT_OBJECT_0 && ret < (WAIT_OBJECT_0 + PCO_NUMBER_BUFFERS)) //waitForSingleObject is done
        {
            bufNr = bufNumbers[ret - WAIT_OBJECT_0];
            retVal += checkError(PCO_GetBuffer(m_hCamera, bufNr, &wBuf, &Event));

            for (short i = 0; i < PCO_NUMBER_BUFFERS; ++i)
            {
                if (m_buffers[i].bufNr == bufNr)
                {
                    m_buffers[i].bufQueued = false;
                    m_buffers[i].bufError = false;
                }
            }
            waitingSuccessful = true;
            break;
        }
        else if (ret != WAIT_TIMEOUT)
        {
            retVal += ito::RetVal(ito::retError, 1002, tr("Error waiting for image acquisition (%1).").arg(ret).toLatin1().data());
            break;
        }
        else //timeout -> is ok
        {
            setAlive();
        }
    }*/


    return retVal + m_acquisitionRetVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOSensicam::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retVal = m_acquisitionRetVal;
    int ret = 0;

    bool hasListeners = false;
    bool copyExternal = false;
    if(m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }
    if(externalDataObject != NULL)
    {
        copyExternal = true;
		retVal += checkData(externalDataObject);
    }

	if (!m_isgrabbing)
	{
		retVal += ito::RetVal(ito::retError, 0, "no image has been acquired");
	}


	if (!retVal.containsError())
    {      
        if (externalDataObject)
		{
			switch (m_params["bpp"].getVal<int>())
            {
                case 8:
                    retVal += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                case 16:
                case 12:
                    retVal += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)m_data.rowPtr(0,0), m_params["sizex"].getVal<int>(), m_params["sizey"].getVal<int>());
                    break;
                default:
                    retVal += ito::RetVal(ito::retError, 0, tr("Wrong picture type").toLatin1().data());
                    break;
            }
		}

        m_isgrabbing = false;
    }

    /*if (bufNr < 255) //try to requeue buffer
    {
        for (short i = 0; i < PCO_NUMBER_BUFFERS; ++i)
        {
            if (!m_buffers[i].bufQueued)
            {
                retVal += checkError(PCO_AddBufferEx(m_hCamera, 0, 0, m_buffers[i].bufNr, curxsize, curysize, m_caminfo.wDynResDESC));
                m_buffers[i].bufQueued = true;
            }
        }
    }

    if (retVal.containsError()) //maybe all buffers are pending, no free buffers, try to requeue them though
    {
        int icount = 0;
        PCO_GetPendingBuffer(m_hCamera, &icount);

        if (icount == PCO_NUMBER_BUFFERS)
        {
            //queue all images
            for (short sBufNr = 0; sBufNr < PCO_NUMBER_BUFFERS; ++sBufNr)
            {
                PCOBuffer *buffer = &(m_buffers[sBufNr]);
                if (buffer->bufNr >= 0)
                {
                    retVal += checkError(PCO_AddBufferEx(m_hCamera, 0, 0, buffer->bufNr, curxsize, curysize, m_caminfo.wDynResDESC));
                    buffer->bufQueued = true;
                    buffer->bufError = false;
                }
                
            }
        }
    }*/

    return retVal;
}


//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a shallow copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject-handle

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is shallow-copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal PCOSensicam::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    ito::RetVal retVal(ito::retOk);

    retVal += retrieveData();

    if(!retVal.containsError())
    {
        if(dObj == NULL)
        {
            retVal += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toLatin1().data());
        }
        else
        {
            retVal += sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.

            (*dObj) = this->m_data;
        }
    }

    if (waitCond) 
    {
        waitCond->returnValue=retVal;
        waitCond->release();
    }

    return retVal;
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
ito::RetVal PCOSensicam::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if(!dObj)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    else
    {
        retVal += checkData(dObj);  
    }

    if(!retVal.containsError())
    {
        retVal += retrieveData(dObj);  
    }

    if(!retVal.containsError())
    {
        sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.
    }

    if (waitCond) 
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
void PCOSensicam::dockWidgetVisibilityChanged(bool visible)
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