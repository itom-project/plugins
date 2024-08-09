/* ********************************************************************
    Plugin "PCOSensicam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut für Technische Optik (ITO),
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
#include "gitVersion.h"
#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
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

/*    char docstring[] = \
"The PCOSensicam is a plugin to access PCO sensicam, dicam pro and hsfc pro cameras. \n\
\n\
For compiling this plugin, set the CMake variable **PCO_SENSICAM_SDK_DIR** to the base directory of the pco.sensicam.sdk. \n\
The SDK from PCO can be downloaded from http://www.pco.de (pco Software-Development-Toolkit (SDK)). \n\
Download the SDK and install it at any location. Additionally you need to install the drivers for operating your framegrabber board.";
    m_detaildescription = QObject::tr(docstring);*/
    m_detaildescription = QObject::tr("The PCOSensicam is a plugin to access PCO sensicam, dicam pro and hsfc pro cameras. \n\
\n\
For compiling this plugin, set the CMake variable **PCO_SENSICAM_SDK_DIR** to the base directory of the pco.sensicam.sdk. \n\
The SDK from PCO can be downloaded from http://www.pco.de (pco Software-Development-Toolkit (SDK)). \n\
Download the SDK and install it at any location. Additionally you need to install the drivers for operating your framegrabber board.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsMand.clear();
    m_initParamsOpt.clear();
    m_initParamsOpt.append(ito::Param("board_id", ito::ParamBase::Int, 0, MAXBOARD, 0, "board number that should be connected"));

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
// this macro registers the class PCOSensicamInterface with the name PCOSensicamInterface as plugin for the Qt-System (see Qt-DOC)


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
    m_isstarted(false)
{
    //qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");
    //qRegisterMetaType<ito::DataObject>("ito::DataObject");

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "PCOSensicam", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.001, 1000.0, 0.01, tr("Integrationtime of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("delay_time", ito::ParamBase::Double, 0.0, 1000.0, 0.0, tr("Delay time between trigger signal and start of image acquisition in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("gain (not available here, see gain mode)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1.0, 0.0, tr("offset (not available here)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 101, 101, tr("Binning of different pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("fast_mode", ito::ParamBase::Int, 0, 1, 0, tr("long exposure camera types can enable a fast acquisition mode. The step size and range of integration_time and delay_time are then changed.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gain_mode", ito::ParamBase::Int, 0, 1, 0, tr("0: normal analog gain, 1: extended analog gain, 3: low light mode (only for sensicam qe standard and sensicam qe double shutter)").toLatin1().data());
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

    paramVal = ito::Param("ccd_temperature", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, -30, 65, -12, tr("current temperature of CCD").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("electronic_temperature", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, -30, 65, -12, tr("current temperature of electronics").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 12, 12, 12, tr("bits per pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("trigger", ito::ParamBase::Int | ito::ParamBase::In, 0, 2, 0, tr("trigger: software (0, default), external rising edge (1), external falling edge (2)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("cam_type", ito::ParamBase::Int | ito::ParamBase::In | ito::ParamBase::Readonly, 0, 10, 0, tr("PCO internal type number of camera").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetPCOSensicam *dw = new DockWidgetPCOSensicam(this);

        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }

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

    retVal += checkError(INITBOARD(board_id, &m_hCamera));

    if (!retVal.containsError())
    {
        //open a connection to the camera
        //and set the camera to idle mode
        retVal += checkError(SETUP_CAMERA(m_hCamera));
    }

    if (!retVal.containsError())
    {
        m_caminfo.wSize =sizeof(SC_Camera_Description);
        retVal += checkError(GET_CAMERA_DESC(m_hCamera, &m_caminfo));
    }

    if (!retVal.containsError())
    {
        cam_param campar;
        retVal += checkError(GET_CAM_PARAM(m_hCamera,&campar));

        if (!retVal.containsError())
        {
            ParamMapIterator it;
            m_params["cam_type"].setVal<int>(campar.cam_typ);

            switch (campar.cam_typ)
            {
            case NOCAM:
            case DICAM:
            case TEST:
                retVal += ito::RetVal::format(ito::retError, 0, "unsupported camera type: %s", CAMTYPE_NAMES[campar.cam_typ]);
                break;

            case FASTEXP: //"Fast Exposure"
            case FASTEXPQE: //"Fast Exposure QE"
                m_params["fast_mode"].setMeta(new ito::IntMeta(1,1,1), true);
                m_params["fast_mode"].setFlags(ito::ParamBase::Readonly);
                it = m_params.find("integration_time");
                it->setMeta(new ito::DoubleMeta(0.0, 1.0e-3, 100e-9), true); //0 - 1ms in steps of 100ns
                it->setVal<double>(1e-3); //default: 1ms
                it = m_params.find("delay_time");
                it->setMeta(new ito::DoubleMeta(0.0, 1.0e-3, 100e-9), true); //0 - 1e6 in steps of 100ns
                it->setVal<double>(0.0); //no delay
                break;
            case LONGEXPQE: //"Long Exposure QE"
                it = m_params.find("gain_mode");
                it->setMeta(new ito::IntMeta(0,3,1), true);
                it->setVal<int>(0);
            case OEM:
            case LONGEXP: //"Long Exposure"
            case LONGEXPI: //"Long Exposure special"
                m_params["fast_mode"].setMeta(new ito::IntMeta(0,1,1), true);
                m_params["fast_mode"].setFlags(0);
                it = m_params.find("integration_time");
                it->setMeta(new ito::DoubleMeta(1.0e-3, 1000.0, 1.0e-3), true); //1 - 1000.0s in steps of 1ms
                it->setVal<double>(10e-3); //default: 10ms
                it = m_params.find("delay_time");
                it->setMeta(new ito::DoubleMeta(0.0, 1000.0, 1.0e-3), true); //0 - 1000.0s in steps of 1ms
                it->setVal<double>(0.0); //no delay
                break;
            default:
                retVal += ito::RetVal::format(ito::retError, 0, "unsupported and unknown camera type");
                break;

            }
        }
    }

    if (!retVal.containsError())
    {
        m_params["name"].setVal<const char*>(CAMTYPE_NAMES[m_caminfo.wCameraTypeDESC]);
        setIdentifier(QString("%1 (%2)").arg(CAMTYPE_NAMES[m_caminfo.wCameraTypeDESC]).arg(getID()));
    }

    if (!retVal.containsError())
    {
        retVal += synchronizeParameters();
    }

    if (!retVal.containsError())
    {
        int bpp = m_caminfo.wDynResDESC;
        m_params["bpp"].setVal<int>(bpp);
        m_params["bpp"].setMeta(new ito::IntMeta(bpp,bpp), true);
    }

    if (!retVal.containsError())
    {
        checkData(); //check if image must be reallocated
    }

    if (waitCond)
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

    if (m_timerID > 0)
    {
        killTimer(m_timerID);
        m_timerID = 0;
    }

    if (waitCond)
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

    if (retVal == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retVal += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retVal.containsError())
    {
        if (key == "ccd_temperature")
        {
            cam_values camValues;
            retVal += checkError(GET_CAM_VALUES(m_hCamera,&camValues));
            if (!retVal.containsError())
            {
                it->setVal<int>(camValues.ccdtemp);
            }
        }
        else if (key == "electronic_temperature")
        {
            cam_values camValues;
            retVal += checkError(GET_CAM_VALUES(m_hCamera,&camValues));
            if (!retVal.containsError())
            {
                it->setVal<int>(camValues.eletemp);
            }
        }

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
    retVal += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retVal.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retVal += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retVal.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retVal += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if (!retVal.containsError())
    {
        if (key == "x0" || key == "y0" || key == "x1" || key == "y1")
        {
            it->copyValueFrom(&(*val));
            int rois[] = {m_params["x0"].getVal<int>(), m_params["y0"].getVal<int>(), m_params["x1"].getVal<int>() - m_params["x0"].getVal<int>() + 1, m_params["y1"].getVal<int>() - m_params["y0"].getVal<int>() + 1};
            m_params["roi"].setVal<int*>(rois, 4);
            m_params["sizex"].setVal<int>(rois[2]);
            m_params["sizey"].setVal<int>(rois[3]);

            retVal += checkData(); //check if image must be reallocated

            if (m_isstarted)
            {
                retVal += stopCamera();
                retVal += startCamera();
            }
        }
        else if (key == "roi")
        {
            const int *roi = val->getVal<int*>();
            it->copyValueFrom(&(*val));
            m_params["x0"].setVal<int>(roi[0]);
            m_params["x1"].setVal<int>(roi[2] + roi[0] - 1);
            m_params["y0"].setVal<int>(roi[1]);
            m_params["y1"].setVal<int>(roi[3] + roi[1] - 1);
            m_params["sizex"].setVal<int>(roi[2]);
            m_params["sizey"].setVal<int>(roi[3]);

            retVal += checkData(); //check if image must be reallocated

            if (m_isstarted)
            {
                retVal += stopCamera();
                retVal += startCamera();
            }
        }
        else if (key == "binning")
        {
            int binV = val->getVal<int>() % 100;
            int binH = (val->getVal<int>() - binV) / 100;
            if (binV != 1 && binV != 2 && binV != 4 && binV != 8)
            {
                retVal += ito::RetVal(ito::retError, 0, "the vertical binning values must be (1,2,4,8), hence, binning = 101, 102, 104, 108, 201, ...");
            }
            else if (binH != 1 && binH != 2 && binH != 4 && binH != 8)
            {
                retVal += ito::RetVal(ito::retError, 0, "the horizontal binning values must be (1,2,4,8), hence, binning = 101, 201, 401, 801, ...");
            }
            else
            {
                COCValues coc = m_cocValues;
                coc.vbin = binV;
                coc.hbin = binH;
                int ret = test_coc2(coc);
                if (ret == PCO_NOERROR)
                {
                    it->copyValueFrom(&(*val));

                    retVal += checkData(); //check if image must be reallocated

                    if (m_isstarted)
                    {
                        retVal += stopCamera();
                        retVal += startCamera();
                    }
                }
                else if ((ret & 0xF000FFFF) == PCO_WARNING_SDKDLL_COC_VALCHANGE)
                {
                    retVal += ito::RetVal::format(ito::retError, 0, "invalid binning values. Camera proposes hbin: %i and vbin: %i", coc.hbin, coc.vbin);
                }
                else
                {
                    retVal += checkError(ret);
                }
            }
        }
        else if (key == "trigger")
        {
            COCValues coc = m_cocValues;
            coc.trig = val->getVal<int>();
            int ret = test_coc2(coc);
            if (ret == PCO_NOERROR || (((ret & 0xF000FFFF) == PCO_WARNING_SDKDLL_COC_VALCHANGE) && (coc.trig == val->getVal<int>())))
            {
                it->copyValueFrom(&(*val));

                if (m_isstarted)
                {
                    retVal += stopCamera();
                    retVal += startCamera();
                }
            }
            else if ((ret & 0xF000FFFF) == PCO_WARNING_SDKDLL_COC_VALCHANGE)
            {
                retVal += ito::RetVal::format(ito::retError, 0, "invalid trigger value. Camera proposes %i", coc.trig);
            }
            else
            {
                retVal += checkError(ret);
            }
        }
        else if (key == "fast_mode")
        {
            cam_param campar;
            retVal += checkError(GET_CAM_PARAM(m_hCamera,&campar));

            if (!retVal.containsError())
            {
                ParamMapIterator it;

                switch (campar.cam_typ)
                {
                    case FASTEXP: //"Fast Exposure"
                    case FASTEXPQE: //"Fast Exposure QE"
                        //readonly, nothing to do, only fast mode available
                        break;
                    case OEM:
                    case LONGEXP: //"Long Exposure"
                    case LONGEXPI: //"Long Exposure special"
                    case LONGEXPQE: //"Long Exposure QE"
                    {
                        if (val->getVal<int>() == 0)
                        {
                            it = m_params.find("integration_time");
                            it->setMeta(new ito::DoubleMeta(1.0e-3, 1000.0, 1.0e-3), true); //1 - 1e6 in steps of 1ms
                            it->setVal<double>(qBound(1.0e-3, it->getVal<double>(), 1000.0));
                            it = m_params.find("delay_time");
                            it->setMeta(new ito::DoubleMeta(0.0, 1000.0, 1.0e-3), true); //0 - 1e6 in steps of 1ms
                            it->setVal<double>(qBound(0.0, it->getVal<double>(), 1000.0));
                        }
                        else
                        {
                            if (campar.cam_typ == LONGEXPQE) //QE_FAST
                            {
                                it = m_params.find("integration_time");
                                it->setMeta(new ito::DoubleMeta(500.0e-9, 10e-3, 100e-9), true); //500ns to 10ms in steps to 100ns
                                it->setVal<double>(qBound(500.0e-9, it->getVal<double>(), 10e-3));
                                it = m_params.find("delay_time");
                                it->setMeta(new ito::DoubleMeta(0.0, 50e-3, 100e-9), true); //0 - 50ms in steps of 100ns
                                it->setVal<double>(qBound(0.0, it->getVal<double>(), 50e-3)); //no delay
                            }
                            else //EM_FAST
                            {
                                it = m_params.find("integration_time");
                                it->setMeta(new ito::DoubleMeta(75e-6, 15e-3, 75e-6), true); //75 - 15ms in steps of 75mus
                                it->setVal<double>(qBound(75e-6, it->getVal<double>(), 15e-3));
                                it = m_params.find("delay_time");
                                it->setMeta(new ito::DoubleMeta(0.0, 15e-3, 75e-6), true); //0 - 15ms in steps of 75mus
                                it->setVal<double>(qBound(0.0, it->getVal<double>(), 15e-3));
                            }
                        }
                    }
                    break;
                }
            }

            if (!retVal.containsError())
            {
                it->copyValueFrom(&(*val));

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
        else if (key == "gain_mode")
        {
            if (val->getVal<int>() == 2)
            {
                retVal += ito::RetVal(ito::retError, 0, "gain_mode = 2 is invalid");
            }
            else if (val->getVal<int>() == 3)
            {
                //check that camera type is long exposure, qe
                cam_param campar;
                retVal += checkError(GET_CAM_PARAM(m_hCamera,&campar));
                if (campar.cam_typ != LONGEXPQE)
                {
                    retVal += ito::RetVal(ito::retError, 0, "gain_mode = 3 (low light mode) is only valid for sensicam qe, sensicam qe standard or sensicam qe double shutter.");
                }
            }

            if (!retVal.containsError())
            {
                COCValues cocValues = m_cocValues;
                cocValues.mode &= (0x00FF00FF); //keep type and submode, delete gain

                cocValues.mode |= ((val->getVal<int>() & 0xFF) << 8);
                int desiredMode = cocValues.mode;
                int err = test_coc2(cocValues);
                if (err == PCO_NOERROR)
                {
                    it->copyValueFrom(&(*val));
                }
                else if (cocValues.mode != desiredMode)
                {
                    retVal += ito::RetVal(ito::retError, 0, "camera rejected the desired gain mode. Maybe not supported by this camera.");
                }
                else
                {
                    it->copyValueFrom(&(*val));
                }
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
        else if (key == "integration_time" || key == "delay_time")
        {
            if (!retVal.containsError())
            {
                it->copyValueFrom(&(*val));
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

    emit parametersChanged(m_params);

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
    In the PCOSensicam, this method does nothing. In general, the hardware camera should be initialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successful, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal PCOSensicam::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal = ito::retOk;

    checkData(); //this will be reallocated in this method.

    incGrabberStarted();

    if (grabberStartedCount() == 1)
    {
        retVal += startCamera();
    }

    if (waitCond)
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
    if (grabberStartedCount() == 0)
    {
        retVal += stopCamera();
    }
    else if (grabberStartedCount() < 0)
    {
        retVal += ito::RetVal(ito::retError, 1001, tr("StopDevice of PCOSensicam can not be executed, since camera has not been started.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
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
        retVal = checkError(STOP_COC(m_hCamera, 0));

        retVal += checkError(REMOVE_ALL_BUFFERS_FROM_LIST(m_hCamera));

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
    COCValues cocValues = m_cocValues;
    cam_values camValues;
    cam_param campar;
    int type, submode;
    int gain;
    cocValues.table.fill('\0', 256);

    retVal += checkError(GET_CAM_VALUES(m_hCamera,&camValues));
    if (!retVal.containsError())
    {
        m_params["electronic_temperature"].setVal<int>(camValues.eletemp);
        m_params["ccd_temperature"].setVal<int>(camValues.ccdtemp);
    }
    //printf("GET_STATUS returned 0x%x %d %d\n",sta,temp1,temp2);

    retVal += checkError(GET_CAM_PARAM(m_hCamera,&campar));
    //printf("GET_CAM_PARAM failed with error 0x%x\n",err);

    double exposure_ms = m_params["integration_time"].getVal<double>() * 1e3;
    double delay_ms = m_params["delay_time"].getVal<double>() * 1e3;
    int trigger = m_params["trigger"].getVal<int>();
    bool fast = m_params["fast_mode"].getVal<int>() > 0 ? true : false;
    int gain_mode = m_params["gain_mode"].getVal<int>();

    //camera parameters
    switch(campar.cam_typ)
    {
    case FASTEXP: //"Fast Exposure" (sensicam fast shutter or sensicam double shutter - double mode not supported)
        type = M_FAST;
        gain = gain_mode; //(0: normal analog gain, 1: extended analog gain)
        submode=NORMALFAST; //CYCLE is not necessary here, DOUBLE and DOUBLEL are not supported yet.
        cocValues.trig = trigger;
        sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms * 1.0e6),qRound(exposure_ms * 1e6));  //must be in ns

        /*int d,t;
        GET_STATUS(hdriver,&d,&t,&t);
        if ((((d&0x0E00)>>9)>0x01)&&((d&0x00C0)==0x00C0))
        printf(", DOUBLE modes available\n");
        else
        printf("\n");*/
     break;

     case FASTEXPQE: //"Fast Exposure QE"
        type = M_FAST;
        gain = gain_mode; //(0: normal analog gain, 1: extended analog gain)
        submode=NORMALLONG;  //CYCLE is not necessary here
        cocValues.trig = trigger;
        sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms * 1.0e6),qRound(exposure_ms * 1e6));  //must be in ns
    break;

    case LONGEXP: //"Long Exposure" for 'sensicam long exposure', 'sensicam em' and 'sensicam uv'
        type = M_LONG;
        gain = gain_mode; //(0: normal analog gain, 1: extended analog gain)
        submode = fast ? EM_FAST : NORMALLONG; //EM_FAST is also supported;
        cocValues.trig = trigger;
        if (fast)
        {
            sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms * 1.0e3),qRound(exposure_ms * 1.0e3));  //must be mu-secs
        }
        else
        {
            sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms),qRound(exposure_ms));
        }
    break;

    case LONGEXPQE: //"Long Exposure QE" for all 'sensicam qe'
        type = M_LONG;
        gain = gain_mode; //(0: normal analog gain, 1: extended analog gain, 3: Low Light Mode: only sensicam qe, sensicam qe standard and sensicam qe double shutter)
        submode = fast ? QE_FAST : NORMALLONG; //QE_FAST is also supported, QE_DOUBLE not supported
        cocValues.trig = trigger;
        //10ms exposure time
        if (fast)
        {
            sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms * 1.0e6),qRound(exposure_ms * 1e6));  //must be in ns
        }
        else
        {
            sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms),qRound(exposure_ms));
        }
    break;

    case OEM: //Long Exposure OEM
        type = M_LONG;
        gain = gain_mode;
        submode=NORMALLONG;
        cocValues.trig = trigger;
        if (fast)
        {
            sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms * 1.0e3),qRound(exposure_ms * 1e3));  //must maybe be in mu-secs
        }
        else
        {
            sprintf(cocValues.table.data(),"%i,%i,-1,-1",qRound(delay_ms),qRound(exposure_ms));
        }
    break;

    default:
        retVal += ito::RetVal::format(ito::retError, 0, "Invalid camera typ %d found",campar.cam_typ);
    break;
    }

    //binning
    int binning = m_params["binning"].getVal<int>();
    cocValues.vbin = binning % 100;
    cocValues.hbin = (binning - cocValues.vbin) / 100;

    //roi
    const int *roi = m_params["roi"].getVal<int*>();
    int hstep = m_caminfo.wRoiHorStepsDESC;
    int vstep = m_caminfo.wRoiVertStepsDESC;

    if (hstep > 0)
    {
        cocValues.roix1 = 1 + roi[0] / hstep;
        cocValues.roix2 = std::ceil((float)(roi[0] + roi[2]) / (float)(hstep));
    }
    else
    {
        cocValues.roix1 = 1;
        cocValues.roix2 = roi[0] + roi[2];
    }

    if (vstep > 0)
    {
        cocValues.roiy1 = 1 + roi[1] / vstep;
        cocValues.roiy2 = std::ceil((float)(roi[1] + roi[3]) / (float)(vstep));
    }
    else
    {
        cocValues.roiy1 = 1;
        cocValues.roiy2 = roi[1] + roi[3];
    }

   if (!retVal.containsError())
   {
        //test and set camera values

        //normal mode, auto trigger binning 1x1 max roi
        //test if SET_COC gets right values
        cocValues.mode = ((type & 0xFF) | ((gain & 0xFF) << 8) | ((submode & 0xFF)<<16));
        int err = test_coc2(cocValues);

        if (err != PCO_NOERROR)
        {
            if ((err&0xF000FFFF)==PCO_WARNING_SDKDLL_COC_VALCHANGE)
            {
                std::cout << "\nTEST_COC changed some values\n" << std::endl;
                printf("\nSET_COC(0x%x,%d,%d,%d,%d,%d,%d,%d,\"%s\"\n",
                    cocValues.mode,cocValues.trig,cocValues.roix1,cocValues.roix2,cocValues.roiy1,cocValues.roiy2,cocValues.hbin,cocValues.vbin,cocValues.table.data());
            }
            else if ((err&0xF000FFFF)==PCO_WARNING_SDKDLL_COC_STR_SHORT)
            {
                retVal += ito::RetVal::format(ito::retError, err, "TEST_COC buffer for string not large enough need %d bytes",cocValues.table.size());
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
        retVal += checkError(SET_COC(m_hCamera,cocValues.mode,cocValues.trig,cocValues.roix1,cocValues.roix2,cocValues.roiy1, \
            cocValues.roiy2,cocValues.hbin,cocValues.vbin,cocValues.table.data()));

        if (!retVal.containsError())
        {
            m_cocValues = cocValues;
        }
        else
        {
            synchronizeParameters();
        }
    }

    int ccdxsize, ccdysize, width, height, bitpix;
    retVal += checkError(GETSIZES(m_hCamera, &ccdxsize, &ccdysize, &width, &height, &bitpix));

    if (!retVal.containsError())
    {
        retVal += synchronizeParameters();

        if (!retVal.containsError())
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
int PCOSensicam::test_coc2(COCValues &cocValues)
{
    while (true)
    {
        int len = cocValues.table.size();
        int ret = TEST_COC(m_hCamera,  &cocValues.mode, &cocValues.trig, &cocValues.roix1, \
                                        &cocValues.roix2, &cocValues.roiy1, &cocValues.roiy2, \
                                        &cocValues.hbin, &cocValues.vbin, cocValues.table.data(), &len);

        if ((ret == PCO_NOERROR) || ((ret & 0xF000FFFF) == PCO_WARNING_SDKDLL_COC_VALCHANGE))
        {
            cocValues.table.resize(len);
            return ret;
        }
        else if (ret == PCO_WARNING_SDKDLL_COC_STR_SHORT)
        {
            if (cocValues.table.size() < 2048)
            {
                cocValues.table.resize(2*std::max(32, (int)cocValues.table.size()));
            }
            else
            {
                return PCO_ERROR_PROGRAMMER;
            }
        }
    }

    return PCO_ERROR_PROGRAMMER;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOSensicam::synchronizeParameters()
{
    int ccdxsize, ccdysize, actxsize, actysize, bit_pix;
    ParamMapIterator it;

    m_cocValues.table.fill('\0', 64);
    ito::RetVal retVal = checkError(GET_COC_SETTING(m_hCamera,  &m_cocValues.mode, &m_cocValues.trig, &m_cocValues.roix1, \
                                                    &m_cocValues.roix2, &m_cocValues.roiy1, &m_cocValues.roiy2, \
                                                    &m_cocValues.hbin, &m_cocValues.vbin, m_cocValues.table.data(), m_cocValues.table.size()));

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
            it->setMeta(new ito::IntMeta(0, std::min(m_cocValues.roix2*hstep, ccdxsize) - 1, hstep), true);
        }
        else
        {
            it->setMeta(new ito::IntMeta(0, 0, 1), true);
            it->setFlags(ito::ParamBase::Readonly);
        }
        it->setVal<int>(std::min((m_cocValues.roix1 - 1) * hstep, ccdxsize - 1));

        it = m_params.find("x1");
        if (hstep > 0)
        {
            it->setMeta(new ito::IntMeta(std::min((m_cocValues.roix1 - 1) * hstep, ccdxsize - 1) + hstep, ccdxsize-1, hstep), true);
        }
        else
        {
            it->setMeta(new ito::IntMeta(ccdxsize-1, ccdxsize-1, 1), true);
            it->setFlags(ito::ParamBase::Readonly);
        }
        it->setVal<int>(std::min(m_cocValues.roix2 * hstep - 1, ccdxsize - 1));

        it = m_params.find("y0");
        if (hstep > 0)
        {
            it->setMeta(new ito::IntMeta(0, std::min(m_cocValues.roiy2*vstep, ccdysize)-1, vstep), true);
        }
        else
        {
            it->setMeta(new ito::IntMeta(0, 0, 1), true);
            it->setFlags(ito::ParamBase::Readonly);
        }
        it->setVal<int>(std::min((m_cocValues.roiy1 - 1) * vstep, ccdysize - 1));

        it = m_params.find("y1");
        if (vstep > 0)
        {
            it->setMeta(new ito::IntMeta(std::min((m_cocValues.roiy1 - 1) * vstep, ccdysize - 1) + vstep, ccdysize-1, vstep), true);
        }
        else
        {
            it->setMeta(new ito::IntMeta(ccdysize-1, ccdysize-1, 1), true);
            it->setFlags(ito::ParamBase::Readonly);
        }
        it->setVal<int>(std::min(m_cocValues.roiy2 * vstep - 1, ccdysize - 1));

        it = m_params.find("roi");
        if (vstep > 0 && hstep > 0)
        {
            ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, ccdxsize, hstep, hstep, ccdxsize, hstep), ito::RangeMeta(0, ccdysize, vstep, vstep, ccdysize, vstep));
            it->setMeta(rm, true);
        }
        else
        {
            ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(0, ccdxsize, ccdxsize), ito::RangeMeta(0, ccdysize, ccdysize));
            it->setMeta(rm, true);
            it->setFlags(ito::ParamBase::Readonly);
        }

        int roi[4];
        roi[0] = std::min((m_cocValues.roix1 - 1) * hstep, ccdxsize - 1);
        roi[1] = std::min((m_cocValues.roiy1 - 1) * vstep, ccdysize - 1);
        roi[2] = actxsize;
        roi[3] = actysize;
        it->setVal<int*>(roi, 4);
    }

    cam_values camValues;
    retVal += checkError(GET_CAM_VALUES(m_hCamera,&camValues));
    if (!retVal.containsError())
    {
        m_params["ccd_temperature"].setVal<int>(camValues.ccdtemp);
        m_params["electronic_temperature"].setVal<int>(camValues.eletemp);
    }

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

    if (grabberStartedCount() <= 0 || !m_isstarted)
    {
        retVal += ito::RetVal(ito::retError, 1002, tr("Acquire of PCOSensicam can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        m_acquisitionRetVal = ito::retOk;
        if (!retVal.containsError())
        {
            //add buffer to list in order to start acquisition
            retVal += checkError(ADD_BUFFER_TO_LIST(m_hCamera,m_buffers[0].bufNr,imgsize,0,0));
        }

        if (retVal.containsError()) // || !intrigger)
        {
            retVal += ito::retError;
            m_isgrabbing = false;
        }
        else
        {
            m_isgrabbing = true;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    if (!retVal.containsError())
    {
        DWORD stat = WaitForSingleObject(m_buffers[0].bufEvent , delay_exp_ms);
        if (stat == WAIT_OBJECT_0)
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

              /*if (err!=PCO_NOERROR)
               printf("\nError ADD_BUFFER_TO_LIST bufnr[%d] 0x%x\n",x,err);
              printf("pic %d buffer %d done status 0x%x\r",i+1,x,bufstat);
              if ((count<=20)||((bufstat&0xFFFF)!=0x0072))
               printf("\n");
              ResetEvent(buffer_event[x]);
              err=ADD_BUFFER_TO_LIST(hdriver,bufnr[x],size,0,0);
              if (err!=PCO_NOERROR)
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

    return retVal + m_acquisitionRetVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal PCOSensicam::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retVal = m_acquisitionRetVal;
    int ret = 0;

    bool hasListeners = false;
    bool copyExternal = false;
    if (m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }
    if (externalDataObject != NULL)
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

    if (!retVal.containsError())
    {
        if (dObj == NULL)
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

    if (!dObj)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    else
    {
        retVal += checkData(dObj);
    }

    if (!retVal.containsError())
    {
        retVal += retrieveData(dObj);
    }

    if (!retVal.containsError())
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
