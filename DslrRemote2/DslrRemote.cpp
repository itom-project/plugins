/* ********************************************************************
Plugin "DslrRemote2" for itom software
URL: http://lccv.ufal.br/
Copyright (C) 2017, Universidade Federal de Alagoas (UFAL), Brazil

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

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
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
//! constructor for interace
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
    setObjectName("DslrRemote2");

//for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The DslrRemote is a library to remote control some DSLR cameras. \n\
\n\
!!! Important !!!\nTo use this plugin under windows OS you probably will need to install libusb devcice driver.\
Check http://www.libusb.org/wiki/windows_backend and http://zadig.akeo.ie/ for instructions.\n\";*/

	m_description = QObject::tr("A virtual white noise grabber");
//    m_detaildescription = QObject::tr(docstring);
	m_detaildescription = QObject::tr(
"Library to remote control some DSLR cameras. \n\
\n\
!!! Important !!!\nTo use this plugin under windows OS you probably will need to install libusb devcice driver.\
Check http://www.libusb.org/wiki/windows_backend and http://zadig.akeo.ie/ for instructions.\n\
");

    m_author = "Universidade Federal de Alagoas (UFAL)";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = CREATEVERSION(1,4,0);
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = tr("!!! Important !!!\nTo use this plugin under windows OS you probably will need to install libusb devcice driver.\
Check http://www.libusb.org/wiki/windows_backend and http://zadig.akeo.ie/ for instructions.");

    m_initParamsMand.clear();

    ito::Param param("maxXSize", ito::ParamBase::Int, 640, new ito::IntMeta(4, 4096, 4), tr("Width of virtual sensor chip").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("maxYSize", ito::ParamBase::Int, 480, new ito::IntMeta(1, 4096, 1), tr("Height of virtual sensor chip, please set this value to 1 (line camera) or a value dividable by 4 for a 2D camera.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(8, 30, 2), tr("Bits per Pixel, usually 8-16bit grayvalues").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
DslrRemoteInterface::~DslrRemoteInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------
/*
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
*/
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
    m_ptp_cam(NULL),
    m_ptp_usb(NULL),
    m_ptp_params(NULL),
    m_ptp_dev(NULL),
    m_ptp_portnum(0),
    m_waittime(3000),
    m_lastImgNum(-1)
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

    paramVal = ito::Param("prop", ito::ParamBase::String, "", tr("Property, if none given list is returned").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("oper", ito::ParamBase::String, "", tr("Operation, if none given list is returned").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //register exec functions
    QVector<ito::Param> pMand = QVector<ito::Param>()
        << ito::Param("filename", ito::ParamBase::String, "", tr("Name of file to download").toLatin1().data())
        << ito::Param("data", ito::ParamBase::DObjPtr | ito::ParamBase::In | ito::ParamBase::Out, NULL, "data of downloaded file");
    QVector<ito::Param> pOpt = QVector<ito::Param>();
    QVector<ito::Param> pOut = QVector<ito::Param>();
    registerExecFunc("getFile", pMand, pOpt, pOut, tr("download a file"));

    pMand = QVector<ito::Param>();
    pOpt = QVector<ito::Param>();
    pOut = QVector<ito::Param>() << ito::Param("filelist", ito::ParamBase::String | ito::ParamBase::Out, "", tr("List of files on device").toLatin1().data());
    registerExecFunc("listFiles", pMand, pOpt, pOut, tr("list all files stored on device"));
/*
    //now create dock widget for this plugin
    DockWidgetDslrRemote *dw = new DockWidgetDslrRemote(this);
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
*/
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
    int bpp = 16;
    QMap<int, QString> devList;
    char *tmpVal = NULL;
    int sizeX = 1, sizeY = 1;
    int roi[] = {0, 0, sizeX, sizeY};
    m_ptp_cam = new PtpCam();
    retVal += m_ptp_cam->list_devices(0, devList);
    // currently simply use first found device
    if (devList.size() > 0)
        m_ptp_portnum = devList.firstKey();
    else
    {
        retVal += ito::RetVal(ito::retError, 0, "camera list is empty");
        goto end;
    }

    bpp = paramsOpt->at(2).getVal<int>();       // third optional parameter, corresponding to the grabber bit depth per pixel
    m_params["bpp"].setVal<int>(bpp);

    // try getting image size, property 0x5003
    retVal += m_ptp_cam->getset_property(m_ptp_portnum, 20483, &tmpVal, 0);
    if (tmpVal)
    {
        char *tok = strtok(tmpVal, "x");
        sizeX = atoi(tok);
        tok = strtok(NULL, "x");
        sizeY = atoi(tok);
    }

    m_params["sizex"].setVal<int>(sizeX);
    m_params["sizex"].setMeta(new ito::IntMeta(4, sizeX, 4), true);

    m_params["sizey"].setVal<int>(sizeY);
    if (sizeY == 1)
    {
        m_params["sizey"].setMeta(new ito::IntMeta(1, 1, 1), true);
    }
    else
    {
        m_params["sizey"].setMeta(new ito::IntMeta(4, sizeY, 4), true);
    }

    roi[2] = sizeX; roi[3] = sizeY;
    m_params["roi"].setVal<int*>(roi, 4);
    if (sizeY == 1)
    {
        m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, sizeX - 1, 4, 4, sizeX, 4), ito::RangeMeta(0, 0, 1)), true);
    }
    else
    {
        m_params["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, sizeX - 1, 4, 4, sizeX, 4), ito::RangeMeta(0, sizeY - 1, 4,  4, sizeY, 4)), true);
    }

    if (!retVal.containsError())
    {
        checkData(); //check if image must be reallocated

        emit parametersChanged(m_params);
    }

    setIdentifier(QString::number(getID()));

    setInitialized(true); //init method has been finished (independent on retval)
end:
    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

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

    if (m_ptp_cam)
    {
        m_ptp_cam->close_camera(m_ptp_usb, m_ptp_params, m_ptp_dev);
        delete m_ptp_cam;
    }

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
        //first check parameters that influence the size or data type of m_data
        if (key == "prop")
        {
            if (suffix == "")
            {
                QVector<QString> properties;
                QString propStr;
                retValue += m_ptp_cam->list_properties(m_ptp_portnum, 0, properties);
                for (int np = 0; np < properties.length(); np++)
                {
                    propStr.append(properties[np]);
                    propStr.append(" ");
                    *val = ito::Param("Properties", ito::Param::String, propStr.length(), propStr.toLatin1().data(), "");
                }
            }
            else
            {
                char *tmpVal = NULL;
                retValue += m_ptp_cam->getset_property(m_ptp_portnum, suffix.toInt(NULL, 16), &tmpVal, 0);
                *val = ito::Param("prop", ito::ParamBase::String, tmpVal, "");
                // tmpVal is borrowed memory, need to free it here
                free(tmpVal);
            }
        }
        else if (key == "oper")
        {
            if (suffix == "")
            {
                QVector<QString> operations;
                QString operStr;
                retValue += m_ptp_cam->list_operations(m_ptp_portnum, 0, operations);
                for (int np = 0; np < operations.length(); np++)
                {
                    operStr.append(operations[np]);
                    operStr.append(" ");
                }
                *val = ito::Param("Operations", ito::Param::String, operStr.length(), operStr.toLatin1().data(), "");
            }
            else
            {

            }
        }
        else
        {
            *val = it.value();
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
/*
            else if (key == "binning")
            {
                int oldval = it->getVal<int>();

                int ival = val->getVal<int>();
                int newY = ival % 100;
                int newX = (ival - newY) / 100;

                if ((newX != 1 && newX != 2 && newX != 4) || (newY != 1 && newY != 2 && newY != 4))
                {
                    retValue += ito::RetVal(ito::retError, 0, "horizontal and vertical binning must be 1, 2 or 4 (hence vertical * 100 + horizontal)");
                }
                else
                {
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
*/

            retValue += checkData(); //check if image must be reallocated

            if (running)
            {
                retValue += startDevice(NULL);
                setGrabberStarted(running);
            }
        }
        else if (key == "prop")
        {
            if (suffix != "")
            {
                char *tmpVal = val->getVal<char*>();
                retValue += m_ptp_cam->getset_property(m_ptp_portnum, suffix.toInt(NULL, 16), &tmpVal, 0);
            }
        }
        else if (key == "oper")
        {
            if (suffix != "")
            {

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
    In the DslrRemote, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
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
    retValue += m_ptp_cam->capture_image(m_ptp_portnum, 0);
    if (retValue.containsError())
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
ito::RetVal DslrRemote::getFileFromCam(ito::DataObject *data, const uint32_t fhandle, const uint32_t ftype)
{
    QDateTime refTime1970(QDate(1970, 1, 1), QTime(0, 0, 0));
    ito::RetVal retval;
    QString tmpPath = QDir::tempPath();
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

    retval += m_ptp_cam->get_file(m_ptp_portnum, 0, fhandle, tmpFileName.toLatin1().data(), 1);

    if (ftype == 14337)
    {
        QVector<ito::ParamBase> filterParamsMand(0);
        QVector<ito::ParamBase> filterParamsOpt(0);
        QVector<ito::ParamBase> filterParamsOut(0);

        retval += apiFilterParamBase("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
        filterParamsMand[0].setVal<void*>((void*)data);
        filterParamsMand[1].setVal<char*>(tmpFileName.toLatin1().data());
        if (!retval.containsWarningOrError())
        {
            retval += apiFilterCall("loadAnyImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
        }
    }
    else if (ftype == 12288)
    {
        QVector<ito::ParamBase> filterParamsMand(0);
        QVector<ito::ParamBase> filterParamsOpt(0);
        QVector<ito::ParamBase> filterParamsOut(0);

        retval += apiFilterParamBase("loadRawImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
        filterParamsOpt[0].setVal<int>(0);
        filterParamsMand[1].setVal<void*>((void*)data);
        filterParamsMand[0].setVal<char*>(tmpFileName.toLatin1().data());
        if (!retval.containsWarningOrError())
        {
            retval += apiFilterCall("loadRawImage", &filterParamsMand, &filterParamsOpt, &filterParamsOut);
        }
    }
    remove(tmpFileName.toLatin1().data());

    return retval;
}

//-------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DslrRemote::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    int gpret = 0;

//    if (m_isgrabbing == false)
    if (0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("image could not be obtained since no image has been acquired.").toLatin1().data());
    }
    else
    {
/*
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
*/

        uint32_t fhandle;
        int ftype;
        retValue += m_ptp_cam->get_last_file_handle(m_ptp_portnum, 0, m_lastImgNum, fhandle, ftype, this);
        m_lastImgNum++;
        retValue += getFileFromCam(&m_data, fhandle, ftype);

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
ito::RetVal DslrRemote::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ito::ParamBase *param1 = NULL;
    ito::ParamBase *param2 = NULL;
    ito::ParamBase *param3 = NULL;

    if (funcName == "listFiles")
    {
//        param1 = ito::getParamByName(&(*paramsMand), "filelist", &retValue);

        if (!retValue.containsError())
        {
            QVector<QString> FileList;
            QString filelststr;
            retValue += m_ptp_cam->list_files(m_ptp_portnum, 0, FileList, this);
            for (int nf = 0; nf < FileList.length(); nf++)
            {
                filelststr.append(FileList[nf]);
                filelststr.append(" ");
            }
            (*paramsOut)[0].setVal<void*>(filelststr.toLatin1().data());
        }
    }
    else if (funcName == "getFile")
    {
        param1 = ito::getParamByName(&(*paramsMand), "filename", &retValue);
        param2 = ito::getParamByName(&(*paramsMand), "data", &retValue);

        // first try if we got a handle passed
        uint32_t fhandle = atoi((char*)param1->getVal<void*>());
        if (fhandle != 0)
        {
            uint32_t ftype;
            retValue += getFileFromCam((ito::DataObject*)param2->getVal<void*>(), fhandle, ftype);
        }
        else
        {
            uint32_t ftype;
            retValue += m_ptp_cam->get_filehandlebyname(m_ptp_portnum, 0, (char*)param1->getVal<void*>(), fhandle, ftype, this);
            retValue += getFileFromCam((ito::DataObject*)param2->getVal<void*>(), fhandle, ftype);
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
