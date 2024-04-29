/* ********************************************************************
    Plugin "DummyMultiChannelGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
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

#include "dummyMultiChannelGrabber.h"

#ifndef WIN32
#include <unistd.h>
#endif

#include <QtCore/QtPlugin>

#include "dockWidgetDummyMultiChannelGrabber.h"

#include "gitVersion.h"
#include "pluginVersion.h"

#ifdef WIN32
#include <windows.h>
#endif


/*!
    \class DummyMultiChannelGrabber
    \brief Class for the DummyMultiChannelGrabber. The DummyMultiChannelGrabber is able to create
   noisy images or simulate a typical WLI or confocal image signal.

    Usually every method in this class can be executed in an own thread. Only the constructor,
   destructor, showConfDialog will be executed by the main (GUI) thread.
*/

//-------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is
//! usually called by the addIn-Manager.
/*!
    creates new instance of dialogDummyMultiChannelGrabber, calls the method setVals of
   dialogDummyMultiChannelGrabber, starts the execution loop and if the dialog is closed, reads the
   new parameter set and deletes the dialog.

    \return retOk
    \sa dialogDummyMultiChannelGrabber
*/
const ito::RetVal DummyMultiChannelGrabber::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogDummyMultiChannelGrabber(this));
}

//-------------------------------------------------------------------------------------
//! constructor for DummyMultiChannelGrabber
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or
   setParam, is built. Additionally the optional docking widget for the DummyMultiChannelGrabber's
   toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this DummyMultiChannelGrabber-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
DummyMultiChannelGrabber::DummyMultiChannelGrabber() :
    AddInMultiChannelGrabber("DummyMultiChannelGrabber")
{
    if (hasGuiSupport())
    {
        // now create dock widget for this plugin
        DockWidgetDummyMultiChannelGrabber* dw = new DockWidgetDummyMultiChannelGrabber(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable |
            QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<const char*>()), features, areas, dw);
    }
}

//-------------------------------------------------------------------------------------
//! destructor
/*!
    \sa ~AddInBase
*/
DummyMultiChannelGrabber::~DummyMultiChannelGrabber()
{
}

//-------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of
//! DummyMultiChannelGrabber.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy
   these given parameters to the internal m_params-vector. Notice that this method is called after
   that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk
*/
ito::RetVal DummyMultiChannelGrabber::init(
    QVector<ito::ParamBase>* /*paramsMand*/,
    QVector<ito::ParamBase>* paramsOpt,
    ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal;

    // get all arguments
    int sensorWidth = paramsOpt->at(0).getVal<int>();
    int sensorHeight = paramsOpt->at(1).getVal<int>();

    // A multi grabber camera plugin consists of >= 1 channels. Every channel
    // can have individual parameters, where a basic set of parameters are mandatory
    // (like sizex, sizey, pixelFormat...).
    // The channel parameters have a corresponding parameter in the m_params map
    // of the overall plugin. This item in m_params is like a proxy to the underlying
    // channel parameters, where the parameter 'channelSelector' defines which of
    // the more than one channels is used as current proxy.
    ChannelContainerMap channels;

    // create the channel parameters, that are the same for all channels
    ito::Param paramSizeX("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 4096, sensorWidth, tr("sensor width of the channel").toLatin1().data());
    ito::Param paramSizeY("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 4096, sensorHeight, tr("sensor height of the channel").toLatin1().data());
    int roi[] = { 0, 0, sensorWidth, sensorHeight };
    ito::Param paramRoi("roi", ito::ParamBase::IntArray, 4, roi, tr("current region of interest of the channel (x, y, width, height)").toLatin1().data());

    // initialize the first channel
    ito::Param pixelFormat1("pixelFormat", ito::ParamBase::String, paramsOpt->at(2).getVal<const char*>(), tr("pixel format of the 1st channel").toLatin1().data());
    auto pixelFormat1Meta = new ito::StringMeta(ito::StringMeta::String, "ImageFormatControl");
    pixelFormat1Meta->addItem("mono8");
    pixelFormat1Meta->addItem("mono10");
    pixelFormat1Meta->addItem("mono12");
    pixelFormat1Meta->addItem("mono16");
    pixelFormat1.setMeta(pixelFormat1Meta, true);

    ito::Param pixelFormat2("pixelFormat", ito::ParamBase::String, paramsOpt->at(3).getVal<const char*>(), tr("pixel format of the 2nd channel").toLatin1().data());
    auto pixelFormat2Meta = new ito::StringMeta(ito::StringMeta::String, "ImageFormatControl");
    pixelFormat2Meta->addItem("float32");
    pixelFormat2Meta->addItem("float64");
    pixelFormat2.setMeta(pixelFormat2Meta, true);

    ito::Param pixelFormat3("pixelFormat", ito::ParamBase::String, paramsOpt->at(4).getVal<const char*>(), tr("pixel format of the 3rd channel").toLatin1().data());
    auto pixelFormat3Meta = new ito::StringMeta(ito::StringMeta::String, "ImageFormatControl");
    pixelFormat3Meta->addItem("rgba8");
    pixelFormat3Meta->addItem("rgb8");
    pixelFormat3.setMeta(pixelFormat3Meta, true);

    ChannelContainer channel1(paramRoi, pixelFormat1, paramSizeX, paramSizeY);
    channel1.m_channelParams["valueDescription"].setVal<const char*>("intensity");

    // every channel can also further additional parameters.
    // Rules:
    /* 1. If 1 or more channels have a parameter with a specific name, no
    *     global parameter with this name must exist.
    *  2. Not every channel must have the same parameter, with the same name,
    *     however in initChannelsAndGlobalParameters it is verified, that
    *     any channel parameter is available in all channels with the same type flags.
    *     If a channel does not have a parameter, it is created (with default arguments)
    *     and the "not-available" flag.
    *  3. Channel parameters of the same name must have the same type flag
    *     (checked in initChannelsAndGlobalParamters).
    */
    channel1.addChannelParam(ito::Param(
        "gammaCorrection",
        ito::ParamBase::Double, 0.0, 1.0, 1.0, "gamma correction value (channelColour and channelMono only)"));

    // add channel1 to the preliminary channel map. The channel name is 'channelMono' as an example.
    channels["channelMono"] = channel1;

    ChannelContainer channel2(paramRoi, pixelFormat2, paramSizeX, paramSizeY);
    channel2.m_channelParams["valueDescription"].setVal<const char*>("topography");
    double axisScales[] = {0.05, 0.05};
    channel2.m_channelParams["axisScales"].setVal<double*>(axisScales, 2);
    ito::ByteArray axisUnits[] = { "mm", "mm" };
    channel2.m_channelParams["axisUnits"].setVal<ito::ByteArray*>(axisUnits, 2);

    // add channel2 to the preliminary channel map. The channel name is 'channelTopo' as an example.
    channels["channelTopo"] = channel2;

    ChannelContainer channel3(paramRoi, pixelFormat3, paramSizeX, paramSizeY);
    channel3.m_channelParams["valueDescription"].setVal<const char*>("color");

    channel3.addChannelParam(ito::Param(
        "gammaCorrection",
        ito::ParamBase::Double, 0.0, 1.0, 0.8, "gamma correction value (channelColour and channelMono only)"));

    // add channel3 to the preliminary channel map. The channel name is 'channelColour' as an example.
    channels["channelColour"] = channel3;

    // global parameters:
    /* Other than in other plugins, global parameters, that are the same for all
    channels will be added to a temporary list of Param and then given to initChannelsAndGlobalParameters.

    Internally, after some checks, they will be added to m_params, like in any other plugin.

    Due to the base class "addInMultiChannelGrabber", three default global parameters are
    created and initialized: defaultChannel, channelSelector, availableChannels.
    */

    QList<ito::Param> globalParams;

    globalParams << ito::Param(
            "globalParam",
            ito::ParamBase::Int,
            0,
            1,
            1,
            tr("this is a global parameter").toLatin1().data());

    ito::Param exposureTime(
        "exposureTime",
        ito::ParamBase::Double,
        1.e-6,
        0.5,
        10.e-3,
        tr("the exposure time for all channels. This is the time that is used to acquire one frame.").toLatin1().data());
    exposureTime.getMetaT<ito::DoubleMeta>()->setRepresentation(ito::ParamMeta::Logarithmic);
    globalParams << exposureTime;

    auto paramVal = ito::Param(
        "demoRegexpString",
        ito::ParamBase::String,
        "",
        tr("matches strings without whitespaces").toLatin1().data());
    paramVal.setMeta(
        new ito::StringMeta(ito::StringMeta::RegExp, "^\\S+$", "DemoParameters"), true);
    globalParams << paramVal;

    paramVal = ito::Param(
        "demoWildcardString",
        ito::ParamBase::String,
        "test.bmp",
        tr("dummy filename of a bmp file, pattern: *.bmp").toLatin1().data());
    paramVal.setMeta(
        new ito::StringMeta(ito::StringMeta::Wildcard, "*.bmp", "DemoParameters"), true);
    globalParams << paramVal;

    paramVal = ito::Param(
        "demoEnumString",
        ito::ParamBase::String,
        "mode 1",
        tr("enumeration string (mode 1, mode 2, mode 3)").toLatin1().data());
    auto sm = new ito::StringMeta(ito::StringMeta::String, "mode 1", "DemoParameters");
    sm->addItem("mode 2");
    sm->addItem("mode 3");
    paramVal.setMeta(sm, true);
    globalParams << paramVal;

    paramVal = ito::Param(
        "demoArbitraryString",
        ito::ParamBase::String,
        "any string",
        tr("any string allowed").toLatin1().data());
    sm = new ito::StringMeta(ito::StringMeta::String);
    sm->setCategory("DemoParameters");
    paramVal.setMeta(sm, true);
    globalParams << paramVal;

    retVal += initChannelsAndGlobalParameters(channels, "channelMono", globalParams);

    if (!retVal.containsError())
    {
        // check if image must be reallocated
        retVal += checkDataFromAllChannels();

        emit parametersChanged(m_params);
    }

    setIdentifier(tr("Camera %1").arg(getID()));

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    setInitialized(true); // init method has been finished (independent on retval)
    return retVal;
}

//-------------------------------------------------------------------------------------
//! close method which is called before that this instance is deleted by the
//! DummyMultiChannelGrabberInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk \sa ItomSharedSemaphore
*/
ito::RetVal DummyMultiChannelGrabber::close(ItomSharedSemaphore* waitCond)
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

//-------------------------------------------------------------------------------------
//! returns parameter of m_params with key name.
/*!
    This method copies val of the corresponding parameter value.

    \param [in,out] val is a shared-pointer of ito::Param.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk in case that everything is ok, else retError \sa ito::tParam,
   ItomSharedSemaphore
*/
ito::RetVal DummyMultiChannelGrabber::getParameter(
    QSharedPointer<ito::Param> val,
    const ParamMapIterator& it,
    const QString& key,
    const QString& suffix,
    int index,
    bool hasIndex,
    bool& ok)
{
    // just as a demo. If a specific parameter requires more actions than
    // just returning its value, can be handled in this method. Else set ok to false,
    // such that the returned parameter is the corresponding value in m_params and this
    // is handled by AddInMultiChannelGrabber::getParam.

    if (key == "demoArbitraryString")
    {
        // do anything special, for instance request a current value from a hardware
        // device, update the internal parameter and return this updated value.
        *val = it.value();
        ok = true;
    }
    else
    {
        // let the calling method return the current value in m_params.
        ok = false;
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------
//! sets parameter of m_params with key name.
/*!
    This method copies the given value  to the m_params-parameter.

    \param [in] val is the ito::ParamBase value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk in case that everything is ok, else retError \sa ito::tParam,
   ItomSharedSemaphore
*/
ito::RetVal DummyMultiChannelGrabber::setParameter(
    QSharedPointer<ito::ParamBase>& val,
    const ParamMapIterator& it,
    const QString& suffix,
    const QString& key,
    int index,
    bool hasIndex,
    bool& ok,
    QStringList& pendingUpdate)
{
    ito::RetVal retValue;
    int running = 0; // Used to check if grabber was running bevor

    // first check parameters that influence the size or data type of m_channels
    //if (key == "binning")
    //{
    //    if (!retValue.containsError())
    //    {
    //        if (grabberStartedCount() > 0)
    //        {
    //            running = grabberStartedCount();
    //            setGrabberStarted(1);
    //            retValue += stopDevice(nullptr);
    //        }
    //    }

    //    if (!retValue.containsError())
    //    {
    //        int oldval = it->getVal<int>();

    //        int ival = val->getVal<int>();
    //        int newY = ival % 100;
    //        int newX = (ival - newY) / 100;

    //        if (m_lineCamera && (newY != 1))
    //        {
    //            retValue += ito::RetVal(
    //                ito::retError, 0, "the vertical binning for a line camera must be 1");
    //        }
    //        else if ((newX != 1 && newX != 2 && newX != 4) || (newY != 1 && newY != 2 && newY != 4))
    //        {
    //            retValue += ito::RetVal(
    //                ito::retError,
    //                0,
    //                "horizontal and vertical binning must be 1, 2 or 4 (hence vertical * 100 + "
    //                "horizontal)");
    //        }
    //        else
    //        {
    //            m_totalBinning = newX * newY;

    //            retValue += it->copyValueFrom(&(*val));

    //            if (oldval != ival)
    //            {
    //                int oldY = oldval % 100;
    //                int oldX = (oldval - oldY) / 100;
    //                float factorX = (float)oldX / (float)newX;
    //                float factorY = (float)oldY / (float)newY;
    //                int width, height, maxWidth, maxHeight, sizeX, sizeY, offsetX, offsetY;
    //                QMap<QString, ChannelContainer>::iterator i;
    //                for (i = m_channels.begin(); i != m_channels.end();
    //                     ++i) // we need to adapt the roi for each channel
    //                {
    //                    width = (i.value().m_channelParams["roi"].getVal<int*>()[1] -
    //                             i.value().m_channelParams["roi"].getVal<int*>()[0]) *
    //                        factorX;
    //                    height = (i.value().m_channelParams["roi"].getVal<int*>()[3] -
    //                              i.value().m_channelParams["roi"].getVal<int*>()[2]) *
    //                        factorY;

    //                    maxWidth =
    //                        static_cast<ito::RectMeta*>(i.value().m_channelParams["roi"].getMeta())
    //                            ->getWidthRangeMeta()
    //                            .getSizeMax();
    //                    maxHeight =
    //                        static_cast<ito::RectMeta*>(i.value().m_channelParams["roi"].getMeta())
    //                            ->getHeightRangeMeta()
    //                            .getSizeMax();

    //                    sizeX = i.value().m_channelParams["roi"].getVal<int*>()[2] * factorX;
    //                    sizeY = i.value().m_channelParams["roi"].getVal<int*>()[3] * factorY;
    //                    offsetX = i.value().m_channelParams["roi"].getVal<int*>()[0] * factorX;
    //                    offsetY = i.value().m_channelParams["roi"].getVal<int*>()[1] * factorY;
    //                    int roi[] = {offsetX, offsetY, sizeX, sizeY};
    //                    i.value().m_channelParams["roi"].setVal<int*>(roi, 4);
    //                    i.value().m_channelParams["roi"].setMeta(
    //                        new ito::RectMeta(
    //                            ito::RangeMeta(
    //                                0, width - 1, 4 / newX, 4 / newX, maxWidth * factorX, 4 / newX),
    //                            ito::RangeMeta(
    //                                0,
    //                                height - 1,
    //                                4 / newY,
    //                                4 / newY,
    //                                maxHeight * factorY,
    //                                4 / newY)),
    //                        true);
    //                    if (i.key() == m_params["defaultChannel"].getVal<const char*>())
    //                    {
    //                        m_params["roi"].setVal<int*>(roi, 4);
    //                        m_params["roi"].setMeta(
    //                            new ito::RectMeta(
    //                                ito::RangeMeta(
    //                                    0,
    //                                    width - 1,
    //                                    4 / newX,
    //                                    4 / newX,
    //                                    maxWidth * factorX,
    //                                    4 / newX),
    //                                ito::RangeMeta(
    //                                    0,
    //                                    height - 1,
    //                                    4 / newY,
    //                                    4 / newY,
    //                                    maxHeight * factorY,
    //                                    4 / newY)),
    //                            true);
    //                    }
    //                }
    //                pendingUpdate << "roi"
    //                              << "binning"; // add roi to update list to trigger a update of
    //                                            // sizey and sizey
    //            }
    //        }
    //    }
    //    if (running)
    //    {
    //        retValue += startDevice(NULL);
    //        setGrabberStarted(running);
    //    }
    //    ok = true;
    //}

    //if (key == "roi")
    //{
    //    m_isgrabbing = false; // we need to trigger again since the roi changed
    //    ok = false; // we want to further process the parameter by setParam to set the size etc.
    //}

    //if (key == "integration_time")
    //{
    //    bool timerIsRunning = m_freerunTimer.isActive();
    //    m_freerunTimer.stop();
    //    m_freerunTimer.setInterval(
    //        int((val->getVal<double>() + m_params["frame_time"].getVal<double>()) * 1000.0));
    //    if (timerIsRunning)
    //    {
    //        m_freerunTimer.start();
    //    }
    //    ok = false;
    //}

    //if (key == "frame_time")
    //{
    //    bool timerIsRunning = m_freerunTimer.isActive();
    //    m_freerunTimer.stop();
    //    m_freerunTimer.setInterval(
    //        int((val->getVal<double>() + m_params["integration_time"].getVal<double>()) * 1000.0));
    //    if (timerIsRunning)
    //    {
    //        m_freerunTimer.start();
    //    }
    //    ok = false;
    //}
    //else
    //{
    //    ok = false; // set ok to false to let setParam process the parameter
    //
    //}

    ok = false; // set ok to false to let setParam process the parameter

    sendDataToListeners(0);

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal DummyMultiChannelGrabber::retrieveData(const QStringList& channels /*= QStringList()*/)
{
    ito::RetVal retVal;

    return retVal;
}

//-------------------------------------------------------------------------------------
QRect roiParamToRect(const ito::ParamBase& roiParam)
{
    const int* roi = roiParam.getVal<int*>();
    // int roi[] = { 0, 0, sensorWidth, sensorHeight };

    QRect r(roi[0], roi[1], roi[2], roi[3]);
    return r;
}

//-------------------------------------------------------------------------------------
//! With startDevice this camera is initialized.
/*!
    In the DummyMultiChannelGrabber, this method does nothing. In general, the hardware camera
   should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk if starting was successfull, retWarning if startDevice has been calling
   at least twice.
*/
ito::RetVal DummyMultiChannelGrabber::startDevice(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retValue;

    incGrabberStarted();

    if (grabberStartedCount() == 1)
    {
        // the grabber is started for the first time
        retValue += checkData();

        if (retValue == ito::retOk)
        {
            // configure emulator for mono image
            QByteArray pixelFormatMono = m_channels["channelMono"].m_channelParams["pixelFormat"].getVal<const char*>();
            QRect roiMono = roiParamToRect(m_channels["channelMono"].m_channelParams["roi"]);
            m_camEmulator.configureImageMono(roiMono, pixelFormatMono.mid(4 /*mono*/).toInt());

            // configure emulator for topography image (here: float)
            QByteArray pixelFormatTopo = m_channels["channelTopo"].m_channelParams["pixelFormat"].getVal<const char*>();
            QRect roiTopo = roiParamToRect(m_channels["channelTopo"].m_channelParams["roi"]);
            m_camEmulator.configureImageTopography(roiTopo, pixelFormatTopo.mid(5 /*float*/).toInt() == 32);

            // configure emulator for colour image
            QByteArray pixelFormatColour = m_channels["channelColour"].m_channelParams["pixelFormat"].getVal<const char*>();
            QRect roiColour = roiParamToRect(m_channels["channelColour"].m_channelParams["roi"]);
            m_camEmulator.configureImageColor(roiColour, pixelFormatColour == "rgba32");
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
//! With stopDevice the camera device is stopped (opposite to startDevice)
/*!
    In this DummyMultiChannelGrabber, this method does nothing. In general, the hardware camera
   should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk if everything is ok, retError if camera wasn't started before \sa
   startDevice
*/
ito::RetVal DummyMultiChannelGrabber::stopDevice(ItomSharedSemaphore* waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(
            ito::retWarning,
            1001,
            tr("stopDevice of DummyMultiChannelGrabber can not be executed, since camera has not "
               "been started.")
                .toLatin1()
                .data());
        setGrabberStarted(0);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}


//-------------------------------------------------------------------------------------
//! Call this method to trigger a new image.
/*!
    By this method a new image is trigger by the camera, that means the acquisition of the image
   starts in the moment, this method is called. The new image is then stored either in internal
   camera memory or in internal memory of this class.

    \note This method is similar to VideoCapture::grab() of openCV

    \param [in] trigger may describe the trigger parameter (unused here)
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been
   terminated \return retOk if everything is ok, retError if camera has not been started or an older
   image lies in memory which has not be fetched by getVal, yet. \sa getVal
*/
ito::RetVal DummyMultiChannelGrabber::acquire(const int /*trigger*/, ItomSharedSemaphore* waitCond)
{
    ito::RetVal retValue = ito::retOk;
    ItomSharedSemaphoreLocker locker(waitCond);

    double exposureTimeS = m_params["exposureTime"].getVal<double>();

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    // the emulator
    if (m_camEmulator.grabImages(true, true, true, exposureTimeS * 1000.0))
    {
        m_camEmulator.imageMono().deepCopyPartial(m_channels["channelMono"].m_data);
        m_camEmulator.imageTopography().deepCopyPartial(m_channels["channelTopo"].m_data);
        m_camEmulator.imageColor().deepCopyPartial(m_channels["channelColour"].m_data);
    }
    else
    {
        // todo
    }


    return retValue;
}

////-------------------------------------------------------------------------------------
////! Returns the grabbed camera frame as a shallow copy.
///*!
//    This method copies the recently grabbed camera frame to the given DataObject-handle
//
//    \note This method is similar to VideoCapture::retrieve() of openCV
//
//    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to
//   ito::DataObject*) where the acquired image is shallow-copied to. \param [in] waitCond is the
//   semaphore (default: NULL), which is released if this method has been terminated \return retOk if
//   everything is ok, retError is camera has not been started or no image has been acquired by the
//   method acquire. \sa DataObject, acquire
//*/
//ito::RetVal DummyMultiChannelGrabber::getVal(void* vpdObj, ItomSharedSemaphore* waitCond)
//{
//    ItomSharedSemaphoreLocker locker(waitCond);
//    ito::RetVal retValue(ito::retOk);
//    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(vpdObj);
//
//    if (!dObj)
//    {
//        retValue += ito::RetVal(
//            ito::retError, 0, tr("Empty dataObject handle retrieved from caller").toLatin1().data());
//    }
//
//    if (!retValue.containsError())
//    {
//        retValue += retrieveData();
//    }
//
//    if (!retValue.containsError())
//    {
//        // don't wait for live image, since user should get the image as fast as possible.
//        sendDataToListeners(0);
//
//        (*dObj) = m_channels[m_params["defaultChannel"].getVal<const char*>()].m_data;
//    }
//
//    if (waitCond)
//    {
//        waitCond->returnValue = retValue;
//        waitCond->release();
//    }
//
//    return retValue;
//}
//
////-------------------------------------------------------------------------------------
////! Returns the grabbed camera frame as a deep copy.
///*!
//    This method copies the recently grabbed camera frame to the given DataObject. Therefore this
//   camera size must fit to the data structure of the DataObject.
//
//    \note This method is similar to VideoCapture::retrieve() of openCV
//
//    \param [in,out] vpdObj is the pointer to a given dataObject (this pointer should be cast to
//   ito::DataObject*) where the acquired image is deep copied to. \param [in] waitCond is the
//   semaphore (default: NULL), which is released if this method has been terminated \return retOk if
//   everything is ok, retError is camera has not been started or no image has been acquired by the
//   method acquire. \sa DataObject, acquire
//*/
//ito::RetVal DummyMultiChannelGrabber::copyVal(void* vpdObj, ItomSharedSemaphore* waitCond)
//{
//    ItomSharedSemaphoreLocker locker(waitCond);
//    ito::RetVal retValue(ito::retOk);
//    ito::DataObject* dObj = reinterpret_cast<ito::DataObject*>(vpdObj);
//
//    if (!dObj)
//    {
//        retValue += ito::RetVal(
//            ito::retError, 0, tr("Empty dataObject handle retrieved from caller").toLatin1().data());
//    }
//    else
//    {
//        retValue += checkData(dObj);
//    }
//
//    if (!retValue.containsError())
//    {
//        retValue += retrieveData(dObj);
//    }
//
//    if (!retValue.containsError())
//    {
//         don't wait for live image, since user should get the image as fast as possible.
//        sendDataToListeners(0);
//    }
//
//    if (waitCond)
//    {
//        waitCond->returnValue = retValue;
//        waitCond->release();
//    }
//
//    return retValue;
//}


//-------------------------------------------------------------------------------------
ito::Rgba32 hsv2rgb(float hue, float saturation, float intensity, ito::uint8 alpha)
{
    double      hh, p, q, t, ff;
    long        i;
    ito::Rgba32 out;

    out.a = alpha;

    if (saturation <= 0.0)
    {
        // < is bogus, just shuts up warnings
        out.r = intensity * 255;
        out.g = out.b = out.r;
        return out;
    }

    hh = hue * 360;

    if (hh >= 360.0)
    {
        hh = 0.0;
    }

    hh /= 60.0;

    i = (long)hh;
    ff = hh - i;
    p = intensity * (1.0 - saturation);
    q = intensity * (1.0 - (saturation * ff));
    t = intensity * (1.0 - (saturation * (1.0 - ff)));

    switch (i) {
    case 0:
        out.r = intensity;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = intensity;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = intensity;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = intensity;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = intensity;
        break;
    case 5:
    default:
        out.r = intensity;
        out.g = p;
        out.b = q;
        break;
    }
    return out;
}

//-------------------------------------------------------------------------------------
void DummyMultiChannelGrabber::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        QWidget* widget = getDockWidget()->widget();
        if (visible)
        {
            connect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(
                this,
                SIGNAL(parametersChanged(QMap<QString, ito::Param>)),
                widget,
                SLOT(parametersChanged(QMap<QString, ito::Param>)));
        }
    }
}
