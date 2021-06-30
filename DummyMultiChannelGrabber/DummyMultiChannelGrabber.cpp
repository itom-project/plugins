/* ********************************************************************
    Plugin "DummyMultiChannelGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#include "DummyMultiChannelGrabber.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#ifndef WIN32
#include <unistd.h>
#endif
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qmetaobject.h>

#include <qdockwidget.h>
#include <qpushbutton.h>
#include <qmetaobject.h>
#include "dockWidgetDummyMultiChannelGrabber.h"

#include "pluginVersion.h"
#include "gitVersion.h"
#include "common/helperCommon.h"
#include "common/paramMeta.h"

#ifdef WIN32
#include <windows.h>
#endif

//----------------------------------------------------------------------------------------------------------------------------------
/** @func   fastrand
*   @brief  function for pseudo random values
*
*   This function delivers the noise for the image.
*/
template<typename _Tp> inline _Tp fastrand(cv::RNG &rng, _Tp maxval, float offset, float gain)
{
    return cv::saturate_cast<_Tp>(offset * maxval + gain * (((ito::uint32)rng.next()) & maxval));
}

//----------------------------------------------------------------------------------------------------------------------------------
/** @func   fastrand
*   @brief  function for pseudo random values
*
*   This function delivers the noise for the image.
*/
template<typename _Tp> inline _Tp fastrand_mean(cv::RNG &rng, _Tp maxval, ito::uint8 numMeans, float offset, float gain)
{
    ito::uint32 val = 0;

    for (ito::uint8 i = 0; i < numMeans; ++i)
    {
        val += ((ito::uint32)rng.next()) & maxval;
    }

    return cv::saturate_cast<_Tp>(offset * maxval + (gain / (float)numMeans) * val);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \class DummyMultiChannelGrabberInterface
    \brief Small interface class for class DummyMultiChannelGrabber. This class contains basic information about DummyMultiChannelGrabber as is able to
        create one or more new instances of DummyMultiChannelGrabber.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! creates new instance of DummyMultiChannelGrabber and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created DummyMultiChannelGrabber-instance is stored in *addInInst
    \return retOk
    \sa DummyMultiChannelGrabber
*/
ito::RetVal DummyMultiChannelGrabberInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DummyMultiChannelGrabber)
        return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! deletes instance of DummyMultiChannelGrabber. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa DummyMultiChannelGrabber
*/
ito::RetVal DummyMultiChannelGrabberInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DummyMultiChannelGrabber)
        return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for interace
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: DummyMultiChannelGrabber) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.
*/
DummyMultiChannelGrabberInterface::DummyMultiChannelGrabberInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("DummyMultiChannelGrabber");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The DummyMultiChannelGrabber is a virtual camera which emulates a camera with white noise. \n\
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
        "The DummyMultiChannelGrabber is a virtual camera which emulates a camera with white noise. \n\
\n\
The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). \
The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera \
image is controlled using the parameter 'roi' if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line camera whose height is equal to 1. \n\
\n\
This plugin can also be used as template for other grabber.");

    m_author = "C. Kohler, W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = CREATEVERSION(1, 4, 0);
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = tr(GITVERSION);

    m_initParamsMand.clear();

    ito::Param param("maxXSize", ito::ParamBase::Int, 640, new ito::IntMeta(4, 4096, 4), tr("Width of virtual sensor chip").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("maxYSize", ito::ParamBase::Int, 480, new ito::IntMeta(1, 4096, 1), tr("Height of virtual sensor chip, please set this value to 1 (line camera) or a value dividable by 4 for a 2D camera.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("pixelFormat", ito::ParamBase::String, "mono8", tr("Bits per Pixel, usually mono8, mono10, mono12, mono16 or rgb32").toLatin1().data());
    ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String, "mono8");
    m->addItem("mono10");
    m->addItem("mono12");
    m->addItem("mono16");
    m->addItem("rgb32");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);

    param = ito::Param("numberOfChannels", ito::ParamBase::Int, 2, new ito::IntMeta(2, 30, 1), tr("Number of channels").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
DummyMultiChannelGrabberInterface::~DummyMultiChannelGrabberInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class DummyMultiChannelGrabberInterface with the name DummyMultiChannelGrabberinterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class DummyMultiChannelGrabber
    \brief Class for the DummyMultiChannelGrabber. The DummyMultiChannelGrabber is able to create noisy images or simulate a typical WLI or confocal image signal.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of dialogDummyMultiChannelGrabber, calls the method setVals of dialogDummyMultiChannelGrabber, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogDummyMultiChannelGrabber
*/
const ito::RetVal DummyMultiChannelGrabber::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogDummyMultiChannelGrabber(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for DummyMultiChannelGrabber
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the DummyMultiChannelGrabber's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this DummyMultiChannelGrabber-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
DummyMultiChannelGrabber::DummyMultiChannelGrabber() :
    AddInMultiChannelGrabber("testtest"),
    m_isgrabbing(false),
    m_totalBinning(1),
    m_lineCamera(false)
{
    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetDummyMultiChannelGrabber *dw = new DockWidgetDummyMultiChannelGrabber(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
DummyMultiChannelGrabber::~DummyMultiChannelGrabber()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of DummyMultiChannelGrabber.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal DummyMultiChannelGrabber::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;

    int sizeX = paramsOpt->at(0).getVal<int>();     // first optional parameter, corresponding to the grabber width
    int sizeY = paramsOpt->at(1).getVal<int>();     // second optional parameter, corresponding to the grabber heigth
    int numChannel = paramsOpt->at(3).getVal<int>(); //third optional parameter, corresponding to the number of channels
    if (sizeY > 1 && sizeY % 4 != 0)
    {
        retVal += ito::RetVal(ito::retError, 0, "maxYSize must be 1 or dividable by 4");
    }
    else
    {


        QMap<QString, ito::Param> standardParam;
        ito::Param paramVal;
        int roi[] = { 0, 0, 2048, 2048 };
        paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
        ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[0] + roi[2] - 1), ito::RangeMeta(roi[1], roi[1] + roi[3] - 1), "ImageFormatControl");
        paramVal.setMeta(rm, true);
        standardParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 4, 4096, 4096, tr("size in x (cols) [px]").toLatin1().data());
        paramVal.getMetaT<ito::IntMeta>()->setCategory("ImageFormatControl");
        standardParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 4096, tr("size in y (rows) [px]").toLatin1().data());
        paramVal.getMetaT<ito::IntMeta>()->setCategory("ImageFormatControl");
        standardParam.insert(paramVal.getName(), paramVal);

        ito::StringMeta *m = new ito::StringMeta(ito::StringMeta::String, "mono8");
        m->addItem("mono10");
        m->addItem("mono12");
        m->addItem("mono16");
        paramVal = ito::Param("pixelFormat", ito::ParamBase::String, paramsOpt->at(2).getVal<char*>(), tr("bitdepth of images: mono8, mono10, mono12, mono16, rgb32").toLatin1().data());
        paramVal.setMeta(m, true);
        standardParam.insert(paramVal.getName(), paramVal);


        QMap<QString, ChannelContainer> channelMap;
        for (int i = 0; i < numChannel; i++)
        {
            channelMap.insert(QString("channel_%1").arg(i), ChannelContainer(standardParam["roi"], standardParam["pixelFormat"], standardParam["sizex"], standardParam["sizey"]));
        }
        
        channelMap["channel_1"].m_channelParam.insert("channelSpecificParam", ito::Param("channelSpecificParam", ito::ParamBase::Int, 0, 1, 1, tr("this is a channel specific parameter").toLatin1().data()));


        // global params
        QMap<QString, ito::Param> globalParam;
        globalParam.insert("globalParam", ito::Param("globalParam", ito::ParamBase::Int, 0, 1, 1, tr("this is a global parameter").toLatin1().data()));
        
        
        
        ito::DoubleMeta *dm;
        paramVal = ito::Param("frame_time", ito::ParamBase::Double, 0.0, 60.0, 0.0, tr("Minimum time between the start of two consecutive acquisitions [s], default: 0.0.").toLatin1().data());
        dm = paramVal.getMetaT<ito::DoubleMeta>();
        dm->setCategory("AcquisitionControl");
        dm->setUnit("s");
        dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 60.0, 0.0, tr("Minimum integration time for an acquisition [s], default: 0.0 (as fast as possible).").toLatin1().data());
        dm = paramVal.getMetaT<ito::DoubleMeta>();
        dm->setCategory("AcquisitionControl");
        dm->setUnit("s");
        dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("Virtual gain").toLatin1().data());
        dm = paramVal.getMetaT<ito::DoubleMeta>();
        dm->setCategory("AcquisitionControl");
        dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Virtual offset").toLatin1().data());
        dm = paramVal.getMetaT<ito::DoubleMeta>();
        dm->setCategory("AcquisitionControl");
        dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("Binning of different pixel, binning = x-factor * 100 + y-factor").toLatin1().data());
        paramVal.getMetaT<ito::IntMeta>()->setCategory("ImageFormatControl");
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("demoRegexpString", ito::ParamBase::String, "", tr("matches strings without whitespaces").toLatin1().data());
        paramVal.setMeta(new ito::StringMeta(ito::StringMeta::RegExp, "^\\S+$", "DemoParameters"), true);
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("demoWildcardString", ito::ParamBase::String, "test.bmp", tr("dummy filename of a bmp file, pattern: *.bmp").toLatin1().data());
        paramVal.setMeta(new ito::StringMeta(ito::StringMeta::Wildcard, "*.bmp", "DemoParameters"), true);
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("demoEnumString", ito::ParamBase::String, "mode 1", tr("enumeration string (mode 1, mode 2, mode 3)").toLatin1().data());
        ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "mode 1", "DemoParameters");
        sm->addItem("mode 2");
        sm->addItem("mode 3");
        paramVal.setMeta(sm, true);
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("demoArbitraryString", ito::ParamBase::String, "any string", tr("any string allowed").toLatin1().data());
        sm = new ito::StringMeta(ito::StringMeta::String);
        sm->setCategory("DemoParameters");
        paramVal.setMeta(sm, true);
        globalParam.insert(paramVal.getName(), paramVal);

        paramVal = ito::Param("channelSpecificParameter", ito::ParamBase::Int, 101, 404, 101, tr("channelSpecificParameter only available at Channel_0").toLatin1().data());
        paramVal.getMetaT<ito::IntMeta>()->setCategory("DemoParameters");
        globalParam.insert(paramVal.getName(), paramVal);


        initializeDefaultConfiguration(channelMap, globalParam);

        if (sizeY == 1)
        {
            m_lineCamera = true;
        }
        else
        {
            m_lineCamera = false;
        }
    }

    if (!retVal.containsError())
    {
        retVal += checkData(); //check if image must be reallocated

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

//----------------------------------------------------------------------------------------------------------------------------------
//! close method which is called before that this instance is deleted by the DummyMultiChannelGrabberInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal DummyMultiChannelGrabber::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

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
ito::RetVal DummyMultiChannelGrabber::getParameter(QSharedPointer<ito::Param> val, const ParamMapIterator& it, const QString& suffix, const QString& key, int index, bool hasIndex, bool &ok)
{
    return ito::retOk;
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
ito::RetVal DummyMultiChannelGrabber::setParameter(QSharedPointer<ito::ParamBase> val, const ParamMapIterator& it, const QString& suffix, const QString& key, int index, bool hasIndex, bool &ok, QStringList &pendingUpdate)
{
    ito::RetVal retValue;
    int running = 0; // Used to check if grabber was running bevor

    //first check parameters that influence the size or data type of m_channels
    if (key == "binning")
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
        if(!retValue.containsError())
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
                    int width, height, maxWidth, maxHeight, sizex, sizey, offsetx, offsety;
                    QMap<QString, ChannelContainer>::iterator i;
                    for (i = m_channels.begin(); i != m_channels.end(); ++i)
                    {
                        width = (i.value().m_channelParam["roi"].getVal<int*>()[1] - i.value().m_channelParam["roi"].getVal<int*>()[0])*factorX;
                        height = (i.value().m_channelParam["roi"].getVal<int*>()[3] - i.value().m_channelParam["roi"].getVal<int*>()[2]) * factorY;

                        maxWidth = static_cast<ito::RectMeta*>(i.value().m_channelParam["roi"].getMeta())->getWidthRangeMeta().getSizeMax();
                        maxHeight = static_cast<ito::RectMeta*>(i.value().m_channelParam["roi"].getMeta())->getHeightRangeMeta().getSizeMax();

                        int sizeX = i.value().m_channelParam["roi"].getVal<int*>()[2] * factorX;
                        int sizeY = i.value().m_channelParam["roi"].getVal<int*>()[3] * factorY;
                        int offsetX = i.value().m_channelParam["roi"].getVal<int*>()[0] * factorX;
                        int offsetY = i.value().m_channelParam["roi"].getVal<int*>()[1] * factorY;
                        int roi[] = { offsetX, offsetY, sizeX, sizeY };
                        i.value().m_channelParam["roi"].setVal<int*>(roi, 4);
                        i.value().m_channelParam["roi"].setMeta(new ito::RectMeta(ito::RangeMeta(0, width - 1, 4 / newX, 4 / newX, maxWidth * factorX, 4 / newX), ito::RangeMeta(0, height - 1, 4 / newY, 4 / newY, maxHeight * factorY, 4 / newY)), true);
                    }
                }
            }
            pendingUpdate << "binning" << "roi";
        }
        retValue += checkData(); //check if image must be reallocated

        if (running)
        {
            retValue += startDevice(NULL);
            setGrabberStarted(running);
        }
        ok = true;
    }
    if (key == "roi") 
    {
        m_isgrabbing = false; //we need to trigger again since the roi changed
        ok = false; //we want to further process the parameter by setParam
    }
    else
    {
        ok = false; // set ok to false to let setParam process the parameter
    }


    if (!retValue.containsError())
    {
        emit parametersChanged(m_params);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! With startDevice this camera is initialized.
/*!
    In the DummyMultiChannelGrabber, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal DummyMultiChannelGrabber::startDevice(ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    In this DummyMultiChannelGrabber, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal DummyMultiChannelGrabber::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("stopDevice of DummyMultiChannelGrabber can not be executed, since camera has not been started.").toLatin1().data());
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
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
ito::RetVal DummyMultiChannelGrabber::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    double frame_time = m_params["frame_time"].getVal<double>();
    double integration_time = m_params["integration_time"].getVal<double>();
    int bpp = pixelFormatStringToBpp(m_params["pixelFormat"].getVal<char*>());
    float gain = m_params["gain"].getVal<double>();
    float offset = m_params["offset"].getVal<double>();



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
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of DummyMultiChannelGrabber can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        m_isgrabbing = true;

        if (frame_time > 0.0)
        {
            double diff = (cv::getTickCount() - m_startOfLastAcquisition) / cv::getTickFrequency();

            if (diff < frame_time)
            {
                Sleep((frame_time - diff) * 1000.0);
            }
        }

        m_startOfLastAcquisition = cv::getTickCount();
        //ito::uint32 seed = m_startOfLastAcquisition % std::numeric_limits<ito::uint32>::max();
        cv::RNG &rng = cv::theRNG();

        if (m_totalBinning == 1)
        {
            if (bpp < 9)
            {
                ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
                ito::uint8 *linePtr;                
                foreach(ChannelContainer container, m_channels)
                {
                    ito::DataObject &channelObj = container.data;
                    for (int m = 0; m < channelObj.getSize(0); ++m)
                    {
                        linePtr = (ito::uint8*)channelObj.rowPtr(0, m);
                        for (int n = 0; n < channelObj.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand<ito::uint8>(rng, maxInt, offset, gain);
                        }
                    }
                }
            }
            else if (bpp < 17)
            {
                ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
                ito::uint16 *linePtr;
                foreach(ChannelContainer container, m_channels)
                {
                    ito::DataObject &channelObj = container.data;
                    for (int m = 0; m < channelObj.getSize(0); ++m)
                    {
                        linePtr = (ito::uint16*)channelObj.rowPtr(0, m);
                        for (int n = 0; n < channelObj.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand<ito::uint16>(rng, maxInt, offset, gain);
                        }
                    }
                }
            }
            else if (bpp < 32)
            {
                ito::int32 maxInt = cv::saturate_cast<ito::int32>(cv::pow(2.0, bpp) - 1);
                ito::int32 *linePtr;
                foreach(ChannelContainer container, m_channels)
                {
                    ito::DataObject &channelObj = container.data;
                    for (int m = 0; m < channelObj.getSize(0); ++m)
                    {
                        linePtr = (ito::int32*)channelObj.rowPtr(0, m);
                        for (int n = 0; n < channelObj.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand<ito::int32>(rng, maxInt, offset, gain);
                        }
                    }
                }
            }
        }
        else
        {
            if (bpp < 9)
            {
                ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
                ito::uint8 *linePtr;
                foreach(ChannelContainer container, m_channels)
                {
                    ito::DataObject &channelObj = container.data;
                    for (int m = 0; m < channelObj.getSize(0); ++m)
                    {
                        linePtr = (ito::uint8*)channelObj.rowPtr(0, m);
                        for (int n = 0; n < channelObj.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand_mean<ito::uint8>(rng, maxInt, m_totalBinning, offset, gain);
                        }
                    }
                }
            }
            else if (bpp < 17)
            {
                ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
                ito::uint16 *linePtr;
                foreach(ChannelContainer container, m_channels)
                {
                    ito::DataObject &channelObj = container.data;
                    for (int m = 0; m < channelObj.getSize(0); ++m)
                    {
                        linePtr = (ito::uint16*)channelObj.rowPtr(0, m);
                        for (int n = 0; n < channelObj.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand_mean<ito::uint16>(rng, maxInt, m_totalBinning, offset, gain);
                        }
                    }
                }
            }
            else if (bpp < 32)
            {
                ito::int32 maxInt = cv::saturate_cast<ito::int32>(cv::pow(2.0, bpp) - 1);
                ito::int32 *linePtr;
                foreach(ChannelContainer container, m_channels)
                {
                    ito::DataObject &channelObj = container.data;
                    for (int m = 0; m < channelObj.getSize(0); ++m)
                    {
                        linePtr = (ito::int32*)channelObj.rowPtr(0, m);
                        for (int n = 0; n < channelObj.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand_mean<ito::int32>(rng, maxInt, m_totalBinning, offset, gain);
                        }
                    }
                }
            }
        }

        if (integration_time > 0.0)
        {
            double diff = (cv::getTickCount() - m_startOfLastAcquisition) / cv::getTickFrequency();

            if (diff < integration_time)
            {
                Sleep((integration_time - diff) * 1000.0);
            }
        }
    }

    return retValue;
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
ito::RetVal DummyMultiChannelGrabber::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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

            (*dObj) = this->m_channels[m_params["defaultChannel"].getVal<char*>()].data;
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
ito::RetVal DummyMultiChannelGrabber::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DummyMultiChannelGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("image could not be obtained since no image has been acquired.").toLatin1().data());
    }
    else
    {
        if (externalDataObject)
        {
            m_channels[m_params["defaultChannel"].getVal<char*>()].data.deepCopyPartial(*externalDataObject);
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DummyMultiChannelGrabber::dockWidgetVisibilityChanged(bool visible)
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

