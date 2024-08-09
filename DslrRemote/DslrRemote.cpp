/* ********************************************************************
    Plugin "DummyGrabber" for itom software
    URL: http://lccv.ufal.br/
    Copyright (C) 2016, Universidade Federal de Alagoas (UFAL), Brazil

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

#include "DslrRemote.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declarations of PI constant
#include "math.h"

#ifndef WIN32
    #include <unistd.h>
#endif
#include <QRandomGenerator>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>
#include <qcoreapplication.h>
#include <qdatetime.h>
#include <qdir.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
//#include "dockWidgetDslrRemote.h"

#include "pluginVersion.h"
#include "gitVersion.h"
#include "common/helperCommon.h"

#ifdef WIN32
#include <windows.h>
#endif


//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \class DslrRemoteInterface
    \brief Small interface class for class DslrRemote. This class contains basic information about DslrRemote as is able to
        create one or more new instances of DslrRemote.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! creates new instance of DslrRemote and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created DslrRemote-instance is stored in *addInInst
    \return retOk
    \sa DslrRemote
*/
ito::RetVal DslrRemoteInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DslrRemote)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! deletes instance of DslrRemote. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa DslrRemote
*/
ito::RetVal DslrRemoteInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DslrRemote)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for interface
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: DslrRemote) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.
*/
DslrRemoteInterface::DslrRemoteInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("DslrRemote");

//for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The DslrRemote is a virtual camera which emulates a camera with white noise. \n\
\n\
The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). \
The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera \
image is controlled using the parameter 'roi' if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line camera whose height is equal to 1. \n\
\n\
This plugin can also be used as template for other grabber.";*/

    m_description = QObject::tr("A virtual white noise grabber");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"The DummyGrabber is a virtual camera which emulates a camera with white noise. \n\
\n\
The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). \
The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera \
image is controlled using the parameter 'roi' if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line camera whose height is equal to 1. \n\
\n\
This plugin can also be used as template for other grabber.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);
    m_initParamsMand.clear();

    ito::Param param("maxXSize", ito::ParamBase::Int, 640, new ito::IntMeta(4, 4096, 4), tr("Width of virtual sensor chip").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("maxYSize", ito::ParamBase::Int, 480, new ito::IntMeta(1, 4096, 1), tr("Height of virtual sensor chip, please set this value to 1 (line camera) or a value dividable by 4 for a 2D camera.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(8, 30, 2), tr("Bits per Pixel, usually 8-16bit grayvalues").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------//! destructor
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
DslrRemoteInterface::~DslrRemoteInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------// this macro registers the class DummyGrabberInterface with the name DummyGrabberinterface as plugin for the Qt-System (see Qt-DOC)


//-------------------------------------------------------------------------------------------------------------------------------
void DslrRemote::error_func(GPContext *context, const char *format, va_list args, void *data)
{
    qDebug() << "Context error\n";
    fprintf(stderr, "Contexterror \n");
//    vfprintf(stderr, format, args);
    fprintf(stderr, "\n");
}

//-------------------------------------------------------------------------------------------------------------------------------
void DslrRemote::message_func(GPContext *context, const char *format, va_list args, void *data)
{
    qDebug() << "Warning\n";
//    vfprintf(stdout, format, args);
    fprintf(stdout, "\n");
}

//-------------------------------------------------------------------------------------------------------------------------------
/*!
    \class DslrRemote
    \brief Class for the DslrRemote. The DslrRemote is able to create noisy images or simulate a typical WLI or confocal image signal.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of dialogDummyGrabber, calls the method setVals of dialogDummyGrabber, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogDslrRemote
*/
const ito::RetVal DslrRemote::showConfDialog(void)
{
//    return apiShowConfigurationDialog(this, new DialogDslrRemote(this));
    return ito::RetVal(ito::retWarning, 0, tr("not yet implemented").toLatin1().data());
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for DslrRemote
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the DslrRemote's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this DslrRemote-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
DslrRemote::DslrRemote() :
    AddInGrabber(),
    m_isgrabbing(false),
    m_totalBinning(1),
    m_lineCamera(false),
    m_camera(NULL),
    m_context(NULL),
    m_camFile(NULL)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "DslrRemote", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_time", ito::ParamBase::Double, 0.0, 60.0, 0.0, tr("Minimum time between the start of two consecutive acquisitions [s], default: 0.0.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 60.0, 0.0, tr("Minimum integration time for an acquisition [s], default: 0.0 (as fast as possible).").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("Virtual gain").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Virtual offset").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("Binning of different pixel, binning = x-factor * 100 + y-factor").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 4, 4096, 4096, tr("size in x (cols) [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 4096, tr("size in y (rows) [px]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 2048, 2048};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[0] + roi[2] - 1), ito::RangeMeta(roi[1], roi[1] + roi[3] - 1));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(8, 30, 2), tr("bitdepth of images").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
DslrRemote::~DslrRemote()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of DslrRemote.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal DslrRemote::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retVal;

#ifdef WIN32
    QString p = QCoreApplication::applicationDirPath() + "/plugins/DslrRemote/";
    QByteArray newpath = "CAMLIBS=" + p.toLatin1(); //set libDir at the beginning of the path-variable
    _putenv(newpath.data());
    newpath = "IOLIBS=" + p.toLatin1(); //set libDir at the beginning of the path-variable
    _putenv(newpath.data());
#endif

    gp_camera_new(&m_camera);
    m_context = gp_context_new();

    // set callbacks for camera messages
    gp_context_set_error_func(m_context, (GPContextErrorFunc)(DslrRemote::error_func), NULL);
    gp_context_set_message_func(m_context, (GPContextMessageFunc)(DslrRemote::message_func), NULL);

    //This call will autodetect cameras, take the first one from the list and use it
/*
    printf("Camera init. Can take more than 10 seconds depending on the "
        "memory card's contents (remove card from camera to speed up).\n");
*/
    int ret = gp_camera_init(m_camera, m_context);
    if (ret < GP_OK)
    {
        fprintf(stderr, "No camera auto detected.\n");
        gp_camera_free(m_camera);
        retVal += ito::RetVal(ito::retWarning, 0, tr("No camera auto detected\n").toLatin1().data());
    }

    int sizeX = paramsOpt->at(0).getVal<int>();     // first optional parameter, corresponding to the grabber width
    int sizeY = paramsOpt->at(1).getVal<int>();     // second optional parameter, corresponding to the grabber height
    if (sizeY > 1 && sizeY % 4 != 0)
    {
        retVal += ito::RetVal(ito::retError, 0, "maxYSize must be 1 or dividable by 4");
    }
    else
    {
        int bpp = paramsOpt->at(2).getVal<int>();       // third optional parameter, corresponding to the grabber bit depth per pixel
        m_params["bpp"].setVal<int>(bpp);

        m_params["sizex"].setVal<int>(sizeX);
        m_params["sizex"].setMeta(new ito::IntMeta(4, sizeX, 4), true);

        m_params["sizey"].setVal<int>(sizeY);
        if (sizeY == 1)
        {
            m_lineCamera = true;
            m_params["sizey"].setMeta(new ito::IntMeta(1, 1, 1), true);
        }
        else
        {
            m_lineCamera = false;
            m_params["sizey"].setMeta(new ito::IntMeta(4, sizeY, 4), true);
        }

        int roi[] = {0, 0, sizeX, sizeY};
        m_params["roi"].setVal<int*>(roi, 4);
        if (sizeY == 1)
        {
            m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, sizeX - 1, 4, 4, sizeX, 4), ito::RangeMeta(0, 0, 1)), true);
        }
        else
        {
            m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, sizeX - 1, 4, 4, sizeX, 4), ito::RangeMeta(0, sizeY - 1, 4,  4, sizeY, 4)), true);
        }
    }

    if (!retVal.containsError())
    {
        checkData(); //check if image must be reallocated

        emit parametersChanged(m_params);
    }

    setIdentifier(QString::number(getID()));

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    setInitialized(true); //init method has been finished (independent on retval)
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------//! close method which is called before that this instance is deleted by the DummyGrabberInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal DslrRemote::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    if (m_camera && m_context)
    {
//        gp_camera_unref(m_camera);
        gp_camera_exit(m_camera, m_context);
        m_camera = NULL;
    }
/*
    if (m_context)
    {
        gp_context_unref(m_context);
        m_context = NULL;
    }
*/
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
    This method copies val of the corresponding parameter value.

    \param [in,out] val is a shared-pointer of ito::Param.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal DslrRemote::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if (retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if (!retValue.containsError())
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
//! sets parameter of m_params with key name.
/*!
    This method copies the given value  to the m_params-parameter.

    \param [in] val is the ito::ParamBase value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal DslrRemote::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    int running = 0; // Used to check if grabber was running bevor

    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

    retValue += ito::parseParamName(val->getName(), key, hasIndex, index, suffix);

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        //first check parameters that influence the size or data type of m_data
        if (key == "roi" || key == "binning" || key == "bpp")
        {
            if (!retValue.containsError())
            {
                if (grabberStartedCount() > 0)
                {
                    running = grabberStartedCount();
                    setGrabberStarted(1);
                    retValue += stopDevice(NULL);
                }
            }

            if (key == "bpp")
            {
                retValue += it->copyValueFrom(&(*val));
            }
            else if (key == "roi")
            {
                if (!hasIndex)
                {
                    retValue += it->copyValueFrom(&(*val));
                    m_params["sizex"].setVal<int>(it->getVal<int*>()[2]);
                    m_params["sizey"].setVal<int>(it->getVal<int*>()[3]);
                }
                else
                {
                    it->getVal<int*>()[index] = val->getVal<int>();
                    m_params["sizex"].setVal<int>(it->getVal<int*>()[2]);
                    m_params["sizey"].setVal<int>(it->getVal<int*>()[3]);
                }
            }
            else if (key == "binning")
            {
                int oldval = it->getVal<int>();

                int ival = val->getVal<int>();
                int newY = ival % 100;
                int newX = (ival - newY) / 100;

                if (m_lineCamera && (newY != 1))
                {
                    retValue += ito::RetVal(ito::retError, 0, "the vertical binning for a line camera must be 1");
                }
                else if ((newX != 1 && newX != 2 && newX != 4) || (newY != 1 && newY != 2 && newY != 4))
                {
                    retValue += ito::RetVal(ito::retError, 0, "horizontal and vertical binning must be 1, 2 or 4 (hence vertical * 100 + horizontal)");
                }
                else
                {
                    m_totalBinning = newX * newY;

                    retValue += it->copyValueFrom(&(*val));

                    if (oldval != ival)
                    {
                        int oldY = oldval % 100;
                        int oldX = (oldval - oldY) / 100;
                        float factorX = (float)oldX / (float)newX;
                        float factorY = (float)oldY / (float)newY;

                        int width = m_params["sizex"].getVal<int>() * factorX;
                        int height = m_params["sizey"].getVal<int>() * factorY;

                        int maxWidth = m_params["sizex"].getMax();
                        int maxHeight = m_params["sizey"].getMax();

                        m_params["sizex"].setVal<int>(width);
                        m_params["sizex"].setMeta(new ito::IntMeta(4/newX, maxWidth * factorX, 4/newX), true);

                        m_params["sizey"].setVal<int>(height);
                        m_params["sizey"].setMeta(new ito::IntMeta(4/newY, maxHeight * factorY, 4/newY), true);

                        int sizeX = m_params["roi"].getVal<int*>()[2] * factorX;
                        int sizeY = m_params["roi"].getVal<int*>()[3] * factorY;
                        int offsetX = m_params["roi"].getVal<int*>()[0] * factorX;
                        int offsetY = m_params["roi"].getVal<int*>()[1] * factorY;
                        int roi[] = {offsetX, offsetY, sizeX, sizeY};
                        m_params["roi"].setVal<int*>(roi, 4);
                        m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, width - 1,4/newX,4/newX,maxWidth * factorX,4/newX), ito::RangeMeta(0, height - 1,4/newY,4/newY,maxHeight * factorY,4/newY)), true);
                    }
                }
            }

            retValue += checkData(); //check if image must be reallocated

            if (running)
            {
                retValue += startDevice(NULL);
                setGrabberStarted(running);
            }
        }
        else
        {
            retValue += it->copyValueFrom(&(*val));
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
//! With startDevice this camera is initialized.
/*!
    In the DslrRemote, this method does nothing. In general, the hardware camera should be initialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successful, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal DslrRemote::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    checkData(); //this will be reallocated in this method.

    incGrabberStarted();

    if (grabberStartedCount() == 1)
    {
        m_startOfLastAcquisition = 0;
        m_isgrabbing = false;
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    In this DslrRemote, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal DslrRemote::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("stopDevice of DslrRemote can not be executed, since camera has not been started.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------
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
ito::RetVal DslrRemote::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    double frame_time = m_params["frame_time"].getVal<double>();
    double integration_time = m_params["integration_time"].getVal<double>();
    int bpp = m_params["bpp"].getVal<double>();
    float gain = m_params["gain"].getVal<double>();
    float offset = m_params["offset"].getVal<double>();
    int gpret = 0;

    // take a shot
    if ((gpret = gp_camera_capture(m_camera, GP_CAPTURE_IMAGE, &m_cameraFilePath, m_context)) != GP_OK)
    {
        retValue += ito::RetVal(ito::retError, gpret, tr("libpghoto error acquiring image").toLatin1().data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    if (bpp < 8 || bpp >= 32)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("wrong bit depth").toLatin1().data());
    }
    else if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of DslrRemote can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;
//        m_isgrabbing = false;
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as a shallow copy.
/*!
    This method copies the recently grabbed camera frame to the given DataObject-handle

    \note This method is similar to VideoCapture::retrieve() of openCV

    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to ito::DataObject*) where the acquired image is shallow-copied to.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError is camera has not been started or no image has been acquired by the method acquire.
    \sa DataObject, acquire
*/
ito::RetVal DslrRemote::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    ito::RetVal retValue(ito::retOk);

    retValue += retrieveData();

    if (!retValue.containsError())
    {
        if (dObj == NULL)
        {
            retValue += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toLatin1().data());
        }
        else
        {
            retValue += sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.

            (*dObj) = this->m_data;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue=retValue;
        waitCond->release();
    }

    return retValue;
}

//------------------------------------------------------------------------------------------------------------------------------
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
ito::RetVal DslrRemote::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    else
    {
        retValue += checkData(dObj);
    }

    if (!retValue.containsError())
    {
        retValue += retrieveData(dObj);
    }

    if (!retValue.containsError())
    {
        sendDataToListeners(0); //don't wait for live image, since user should get the image as fast as possible.
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DslrRemote::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    int gpret = 0;
    void *gpdata = NULL;
    CameraEventType evtype;

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("image could not be obtained since no image has been acquired.").toLatin1().data());
    }
    else
    {
        while (1)
        {
            gpret = gp_camera_wait_for_event(m_camera, m_waittime, &evtype, &gpdata, m_context);
            if (evtype == GP_EVENT_TIMEOUT)
            {
                m_isgrabbing = false;
                return retValue += ito::RetVal(ito::retError, 0, tr("time out while waiting for acquire picture to terminate").toLatin1().data());
            }
            else if (evtype == GP_EVENT_CAPTURE_COMPLETE)
            {
                break;
            }
            else if (evtype != GP_EVENT_UNKNOWN)
            {
                m_isgrabbing = false;
                return retValue += ito::RetVal(ito::retError, 0, tr("received unexpected event from camera").toLatin1().data());
            }
            else if (evtype = GP_EVENT_UNKNOWN)
                break;
        }

        CameraFileInfo finfo;
        uint64_t fsize;
        QString tmpPath = QDir::tempPath();
        QDateTime refTime1970(QDate(1970, 1, 1), QTime(0, 0, 0));
        QRandomGenerator qrand(refTime1970.secsTo(QDateTime::currentDateTime()));
        QString tmpFileName;

        if (tmpPath.lastIndexOf("/") < tmpPath.length() - 1
            && tmpPath.lastIndexOf("\\") < tmpPath.length() - 1)
        {
             tmpFileName = tmpPath + "/" + QString::number(qrand.generate());
        }
        else
        {
            tmpFileName = tmpPath + QString::number(qrand.generate());
        }
        gpret = gp_camera_file_get_info(m_camera, m_cameraFilePath.folder, m_cameraFilePath.name, &finfo, m_context);
        char *buf = (char*)malloc(finfo.file.size);
//        m_data = ito::DataObject(finfo.file.height, finfo.file.width, ito::tUInt32);
        gpret = gp_camera_file_read(m_camera, m_cameraFilePath.folder, m_cameraFilePath.name,
                            GP_FILE_TYPE_NORMAL,
                            0,
                             buf, &fsize,
                             m_context);
        CameraFile *camFile = NULL;
        gp_file_new(&camFile);
        gp_file_set_name(camFile, tmpFileName.toLatin1().data());
        gp_camera_file_get(m_camera, m_cameraFilePath.folder, m_cameraFilePath.name,
                           GP_FILE_TYPE_NORMAL, camFile, m_context);
        gp_file_save(camFile, tmpFileName.toLatin1().data());
        gp_file_free(camFile);

        if (finfo.file.type == "image/x-raw")
        {
            QVector<ito::ParamBase> filterParamsMand(0);
            QVector<ito::ParamBase> filterParamsOpt(0);
            QVector<ito::ParamBase> filterParamsOut(0);

            filterParamsOpt[0].setVal<int>(0);
            retValue += apiFilterParamBase("loadRawImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
            filterParamsMand[0].setVal<char*>((char*)&m_data);
            filterParamsMand[1].setVal<char*>(tmpFileName.toLatin1().data());
            if (!retValue.containsWarningOrError())
            {
                retValue += apiFilterCall("loadRawImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
            }
        }
        else
        {
            QVector<ito::ParamBase> filterParamsMand(0);
            QVector<ito::ParamBase> filterParamsOpt(0);
            QVector<ito::ParamBase> filterParamsOut(0);

            retValue += apiFilterParamBase("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
            filterParamsMand[0].setVal<char*>((char*)&m_data);
            filterParamsMand[1].setVal<char*>(tmpFileName.toLatin1().data());
            if (!retValue.containsWarningOrError())
            {
                retValue += apiFilterCall("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
            }
        }
        remove(tmpFileName.toLatin1().data());


        if (externalDataObject)
        {
            m_data.deepCopyPartial(*externalDataObject);
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//-------------------------------------------------------------------------------------------------------------------------------
void DslrRemote::dockWidgetVisibilityChanged(bool visible)
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

//-------------------------------------------------------------------------------------------------------------------------------
