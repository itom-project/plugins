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

#include "DummyMultiChannelGrabber.h"

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

//-------------------------------------------------------------------------------------
/** @func   fastrand
 *   @brief  function for pseudo random values
 *
 *   This function delivers the noise for the image.
 */
template <typename _Tp> inline _Tp fastrand(cv::RNG& rng, _Tp maxval, float offset, float gain)
{
    return cv::saturate_cast<_Tp>(offset * maxval + gain * (((ito::uint32)rng.next()) & maxval));
}

//-------------------------------------------------------------------------------------
/** @func   fastrand
 *   @brief  function for pseudo random values
 *
 *   This function delivers the noise for the image.
 */
template <typename _Tp>
inline _Tp fastrand_mean(cv::RNG& rng, _Tp maxval, ito::uint8 numMeans, float offset, float gain)
{
    ito::uint32 val = 0;

    for (ito::uint8 i = 0; i < numMeans; ++i)
    {
        val += ((ito::uint32)rng.next()) & maxval;
    }

    return cv::saturate_cast<_Tp>(offset * maxval + (gain / (float)numMeans) * val);
}

//-------------------------------------------------------------------------------------
/** @func   gaussFunc
 *   @brief  function for 2d Gaussian function
 *
 *   This function delivers a 2d dataObject with a Gaussian function
 */
template <typename _Tp> ito::RetVal gaussFunc(cv::RNG& rng, ito::DataObject dObj, float amplitude)
{
    int width = dObj.getSize(1);
    int height = dObj.getSize(0);
    _Tp* rowPtr;
    float xval, yval;
    int planeID = dObj.seekMat(0);

    float yRandOffset = rng.uniform(0.f, 20.f);
    float xRandOffset = rng.uniform(0.f, 20.f);
    float aRandOfset = rng.uniform(0.f, 20.f);

    float sigmaX = width * rng.uniform(0.09f, 0.11f);
    float sigmaY = height * rng.uniform(0.09f, 0.11f);

    for (int y = 0; y < height; y++)
    {
        rowPtr = dObj.rowPtr<_Tp>(planeID, y);
        yval = ((y - height / 2 + yRandOffset) * ((float)y - height / 2 + yRandOffset)) /
            (2.0f * sigmaY * sigmaY);

        for (int x = 0; x < width; x++)
        {
            xval = ((x - width / 2 + xRandOffset) * ((float)x - width / 2 + xRandOffset)) /
                (2.0f * sigmaX * sigmaX);
            rowPtr[x] = (float)(amplitude - aRandOfset) * exp(-(xval + yval));
        }
    }

    return ito::retOk;
}

//-------------------------------------------------------------------------------------
/*!
    \class DummyMultiChannelGrabberInterface
    \brief Small interface class for class DummyMultiChannelGrabber. This class contains basic
   information about DummyMultiChannelGrabber as is able to create one or more new instances of
   DummyMultiChannelGrabber.
*/

//-------------------------------------------------------------------------------------
//! creates new instance of DummyMultiChannelGrabber and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created
   DummyMultiChannelGrabber-instance is stored in *addInInst \return retOk \sa
   DummyMultiChannelGrabber
*/
ito::RetVal DummyMultiChannelGrabberInterface::getAddInInst(ito::AddInBase** addInInst)
{
    NEW_PLUGININSTANCE(DummyMultiChannelGrabber)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
//! deletes instance of DummyMultiChannelGrabber. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa DummyMultiChannelGrabber
*/
ito::RetVal DummyMultiChannelGrabberInterface::closeThisInst(ito::AddInBase** addInInst)
{
    REMOVE_PLUGININSTANCE(DummyMultiChannelGrabber)
    return ito::retOk;
}

//-------------------------------------------------------------------------------------
//! constructor for interace
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real
   plugin (here: DummyMultiChannelGrabber) should or must be initialized (e.g. by a Python call)
   with mandatory or optional parameters, please initialize both vectors m_initParamsMand and
   m_initParamsOpt within this constructor.
*/
DummyMultiChannelGrabberInterface::DummyMultiChannelGrabberInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("DummyMultiChannelGrabber");

    // for the docstring, please don't set any spaces at the beginning of the line.
    /*    char docstring[] = \
    "The DummyMultiChannelGrabber is a virtual camera which emulates a camera with white noise. \n\
    \n\
    The camera is initialized with a maximum width and height of the simulated camera chip (both
    need to be a multiple
    of 4). \
    The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per
    pixel). The real size of
    the camera \
    image is controlled using the parameter 'roi' if the sizes stay within the limits given by the
    size of the camera
    chip.\n\
    \n\
    You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line
    camera whose height is
    equal to 1. \n\
    \n\
    This plugin can also be used as template for other grabber.";*/

    m_description = QObject::tr("A virtual white noise grabber");
    //    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
        "The DummyMultiChannelGrabber is a virtual camera which emulates a camera with multiple channels with white noise. \n\
\n\
The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). \
The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera \
image is controlled using the parameter 'roi' if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line camera whose height is equal to 1. \n\
\n\
This plugin can also be used as template for other grabbers.");

    m_author = "R. Hahn, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = CREATEVERSION(1, 4, 0);
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = tr(GITVERSION);

    m_initParamsMand.clear();

    ito::Param param(
        "sensorWidth",
        ito::ParamBase::Int,
        640,
        new ito::IntMeta(4, 4096, 4),
        tr("Width of sensor chip. In this demo, the width of the sensor is the same for "
            "all channels (could be different, if implemented).").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param(
        "sensorHeight",
        ito::ParamBase::Int,
        480,
        new ito::IntMeta(1, 4096, 1),
        tr("Height of sensor chip. In this demo, the height is the same for all "
            "channels. However, it could also be different, if implemented.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param(
        "pixelFormatChannel1",
        ito::ParamBase::String,
        "mono8",
        tr("Pixel format for the 1st channel (here: a grayscale intensity image).").toLatin1().data());
    ito::StringMeta* m = new ito::StringMeta(ito::StringMeta::String, "mono8");
    m->addItem("mono10");
    m->addItem("mono12");
    m->addItem("mono16");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);

    param = ito::Param(
        "pixelFormatChannel2",
        ito::ParamBase::String,
        "float32",
        tr("Pixel format for the 2nd channel (here: a float32 or float64 disparity image).").toLatin1().data());
    m = new ito::StringMeta(ito::StringMeta::String, "float32");
    m->addItem("float64");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);

    param = ito::Param(
        "pixelFormatChannel3",
        ito::ParamBase::String,
        "rgba8",
        tr("Pixel format for the 3nd channel (here: color image with or without alpha channel).").toLatin1().data());
    m = new ito::StringMeta(ito::StringMeta::String, "rgba8");
    m->addItem("rgb8");
    param.setMeta(m, true);
    m_initParamsOpt.append(param);

    param = ito::Param(
        "imageType",
        ito::ParamBase::String | ito::ParamBase::In,
        "noise",
        tr("Available dummy image types: noise (default), gaussianSpot, gaussianSpotArray")
            .toLatin1()
            .data());
    ito::StringMeta sm(ito::StringMeta::String, "noise");
    sm.addItem("gaussianSpot");
    sm.addItem("gaussianSpotArray");
    param.setMeta(&sm, false);

    m_initParamsOpt.append(param);
}

//-------------------------------------------------------------------------------------
//! destructor
/*!

*/
DummyMultiChannelGrabberInterface::~DummyMultiChannelGrabberInterface()
{
}

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
    AddInMultiChannelGrabber("DummyMultiChannelGrabber"), m_isgrabbing(false),
    m_startOfLastAcquisition(0), m_freerunTimer(this),
    m_imageType(imgTypeNoise)
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
    connect(&m_freerunTimer, &QTimer::timeout, this, &DummyMultiChannelGrabber::generateImageData);
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
    ito::Param paramSizeX("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 4096, 1280, tr("sensor width of the channel").toLatin1().data());
    ito::Param paramSizeY("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 16, 4096, 1024, tr("sensor height of the channel").toLatin1().data());
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
    channel1.m_channelParams["valueDescription"].setVal<ito::ByteArray>("intensity");

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
    channel2.m_channelParams["valueDescription"].setVal<ito::ByteArray>("topography");
    double axisScales[] = {0.05, 0.05};
    channel2.m_channelParams["axisScales"].setVal<double*>(axisScales, 2);
    ito::ByteArray axisUnits[] = { "mm", "mm" };
    channel2.m_channelParams["axisUnits"].setVal<ito::ByteArray*>(axisUnits, 2);

    // add channel2 to the preliminary channel map. The channel name is 'channelTopo' as an example.
    channels["channelTopo"] = channel2;

    ChannelContainer channel3(paramRoi, pixelFormat3, paramSizeX, paramSizeY);
    channel3.m_channelParams["valueDescription"].setVal<ito::ByteArray>("color");

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
        retVal += checkData(); // check if image must be reallocated

        emit parametersChanged(m_params);
    }

    setIdentifier(QString::number(getID()));

    // get type of dummy image
    QString type = paramsOpt->at(4).getVal<const char*>();

    if (type == "noise")
    {
        m_imageType = imgTypeNoise;
    }
    else if (type == "gaussianSpot")
    {
        m_imageType = imgTypeGaussianSpot;
    }
    else if (type == "gaussianSpotArray")
    {
        m_imageType = imgTypeGaussianSpotArray;
    }

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
    if (key == "binning")
    {
        if (!retValue.containsError())
        {
            if (grabberStartedCount() > 0)
            {
                running = grabberStartedCount();
                setGrabberStarted(1);
                retValue += stopDevice(nullptr);
            }
        }

        if (!retValue.containsError())
        {
            int oldval = it->getVal<int>();

            int ival = val->getVal<int>();
            int newY = ival % 100;
            int newX = (ival - newY) / 100;

            if (m_lineCamera && (newY != 1))
            {
                retValue += ito::RetVal(
                    ito::retError, 0, "the vertical binning for a line camera must be 1");
            }
            else if ((newX != 1 && newX != 2 && newX != 4) || (newY != 1 && newY != 2 && newY != 4))
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    "horizontal and vertical binning must be 1, 2 or 4 (hence vertical * 100 + "
                    "horizontal)");
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
                    int width, height, maxWidth, maxHeight, sizeX, sizeY, offsetX, offsetY;
                    QMap<QString, ChannelContainer>::iterator i;
                    for (i = m_channels.begin(); i != m_channels.end();
                         ++i) // we need to adapt the roi for each channel
                    {
                        width = (i.value().m_channelParams["roi"].getVal<int*>()[1] -
                                 i.value().m_channelParams["roi"].getVal<int*>()[0]) *
                            factorX;
                        height = (i.value().m_channelParams["roi"].getVal<int*>()[3] -
                                  i.value().m_channelParams["roi"].getVal<int*>()[2]) *
                            factorY;

                        maxWidth =
                            static_cast<ito::RectMeta*>(i.value().m_channelParams["roi"].getMeta())
                                ->getWidthRangeMeta()
                                .getSizeMax();
                        maxHeight =
                            static_cast<ito::RectMeta*>(i.value().m_channelParams["roi"].getMeta())
                                ->getHeightRangeMeta()
                                .getSizeMax();

                        sizeX = i.value().m_channelParams["roi"].getVal<int*>()[2] * factorX;
                        sizeY = i.value().m_channelParams["roi"].getVal<int*>()[3] * factorY;
                        offsetX = i.value().m_channelParams["roi"].getVal<int*>()[0] * factorX;
                        offsetY = i.value().m_channelParams["roi"].getVal<int*>()[1] * factorY;
                        int roi[] = {offsetX, offsetY, sizeX, sizeY};
                        i.value().m_channelParams["roi"].setVal<int*>(roi, 4);
                        i.value().m_channelParams["roi"].setMeta(
                            new ito::RectMeta(
                                ito::RangeMeta(
                                    0, width - 1, 4 / newX, 4 / newX, maxWidth * factorX, 4 / newX),
                                ito::RangeMeta(
                                    0,
                                    height - 1,
                                    4 / newY,
                                    4 / newY,
                                    maxHeight * factorY,
                                    4 / newY)),
                            true);
                        if (i.key() == m_params["defaultChannel"].getVal<const char*>())
                        {
                            m_params["roi"].setVal<int*>(roi, 4);
                            m_params["roi"].setMeta(
                                new ito::RectMeta(
                                    ito::RangeMeta(
                                        0,
                                        width - 1,
                                        4 / newX,
                                        4 / newX,
                                        maxWidth * factorX,
                                        4 / newX),
                                    ito::RangeMeta(
                                        0,
                                        height - 1,
                                        4 / newY,
                                        4 / newY,
                                        maxHeight * factorY,
                                        4 / newY)),
                                true);
                        }
                    }
                    pendingUpdate << "roi"
                                  << "binning"; // add roi to update list to trigger a update of
                                                // sizey and sizey
                }
            }
        }
        if (running)
        {
            retValue += startDevice(NULL);
            setGrabberStarted(running);
        }
        ok = true;
    }

    if (key == "roi")
    {
        m_isgrabbing = false; // we need to trigger again since the roi changed
        ok = false; // we want to further process the parameter by setParam to set the size etc.
    }

    if (key == "integration_time")
    {
        bool timerIsRunning = m_freerunTimer.isActive();
        m_freerunTimer.stop();
        m_freerunTimer.setInterval(
            int((val->getVal<double>() + m_params["frame_time"].getVal<double>()) * 1000.0));
        if (timerIsRunning)
        {
            m_freerunTimer.start();
        }
        ok = false;
    }

    if (key == "frame_time")
    {
        bool timerIsRunning = m_freerunTimer.isActive();
        m_freerunTimer.stop();
        m_freerunTimer.setInterval(
            int((val->getVal<double>() + m_params["integration_time"].getVal<double>()) * 1000.0));
        if (timerIsRunning)
        {
            m_freerunTimer.start();
        }
        ok = false;
    }
    else
    {
        ok = false; // set ok to false to let setParam process the parameter
    }

    sendDataToListeners(0);

    return retValue;
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
    ito::RetVal retValue = ito::retOk;

    checkData(); // this will be reallocated in this method.

    incGrabberStarted();

    if (grabberStartedCount() == 1)
    {
        m_startOfLastAcquisition = 0;
        m_isgrabbing = false;
    }
    if (strcmp(m_params["triggerMode"].getVal<const char*>(), "freerun") == 0)
    {
        m_freerunTimer.start(
            int((m_params["frame_time"].getVal<double>() +
                 m_params["integration_time"].getVal<double>()) *
                1000.0));
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
    else
    {
        m_freerunTimer.stop();
    }
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal DummyMultiChannelGrabber::generateImageData()
{
    ito::RetVal retValue = ito::retOk;
    double frame_time = m_params["frame_time"].getVal<double>();
    double integration_time = m_params["integration_time"].getVal<double>();
    float gain = m_params["gain"].getVal<double>();
    float offset = m_params["offset"].getVal<double>();
    int min, max = 0;
    bool ok = false;
    AbstractAddInGrabber::minMaxBoundariesFromIntegerPixelFormat(
        m_params["pixelFormat"].getVal<const char*>(), min, max, ok);
    if (!ok)
    {
        retValue += ito::RetVal(
            ito::retError, 0, tr("pixel format is not a integer format").toLatin1().data());
    }
    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(
            ito::retError,
            1002,
            tr("Can not acquire image, since camera has not been "
               "started.")
                .toLatin1()
                .data());
    }
    if (!retValue.containsError())
    {
        m_isgrabbing = true;
        if (strcmp(m_params["triggerMode"].getVal<const char*>(), "software") == 0 &&
            (frame_time > 0.0))
        {
            double diff = (cv::getTickCount() - m_startOfLastAcquisition) / cv::getTickFrequency();

            if (diff < frame_time)
            {
                Sleep((frame_time - diff) * 1000.0);
            }
        }

        m_startOfLastAcquisition = cv::getTickCount();
        // ito::uint32 seed = m_startOfLastAcquisition % std::numeric_limits<ito::uint32>::max();
        cv::RNG& rng = cv::theRNG();

        if (m_imageType == imgTypeNoise)
        {
            if (m_totalBinning == 1)
            {
                if (max < 256)
                {
                    ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(max);
                    ito::uint8* linePtr;
                    foreach (ChannelContainer container, m_channels)
                    {
                        ito::DataObject& channelObj = container.m_data;

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
                else if (max < 65536)
                {
                    ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(max);
                    ito::uint16* linePtr;
                    foreach (ChannelContainer container, m_channels)
                    {
                        ito::DataObject& channelObj = container.m_data;

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
            }
            else
            {
                if (max < 256)
                {
                    ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(max);
                    ito::uint8* linePtr;
                    foreach (ChannelContainer container, m_channels)
                    {
                        ito::DataObject& channelObj = container.m_data;

                        for (int m = 0; m < channelObj.getSize(0); ++m)
                        {
                            linePtr = (ito::uint8*)channelObj.rowPtr(0, m);

                            for (int n = 0; n < channelObj.getSize(1); ++n)
                            {
                                *linePtr++ = fastrand_mean<ito::uint8>(
                                    rng, maxInt, m_totalBinning, offset, gain);
                            }
                        }
                    }
                }
                else if (max < 65536)
                {
                    ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(max);
                    ito::uint16* linePtr;
                    foreach (ChannelContainer container, m_channels)
                    {
                        ito::DataObject& channelObj = container.m_data;

                        for (int m = 0; m < channelObj.getSize(0); ++m)
                        {
                            linePtr = (ito::uint16*)channelObj.rowPtr(0, m);

                            for (int n = 0; n < channelObj.getSize(1); ++n)
                            {
                                *linePtr++ = fastrand_mean<ito::uint16>(
                                    rng, maxInt, m_totalBinning, offset, gain);
                            }
                        }
                    }
                }
            }
        }
        else if (m_imageType == imgTypeGaussianSpot) // create dummy Gaussian image
        {
            if (max < 256)
            {
                ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, 8) - 1);
                foreach (ChannelContainer container, m_channels)
                {
                    gaussFunc<ito::uint8>(rng, container.m_data, amplitude);
                }
            }
            else if (max < 65536)
            {
                ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, 16) - 1);
                foreach (ChannelContainer container, m_channels)
                {
                    gaussFunc<ito::uint16>(rng, container.m_data, amplitude);
                }
            }
            else if (max < 2147483647)
            {
                ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, 32) - 1);
                foreach (ChannelContainer container, m_channels)
                {                    gaussFunc<ito::uint32>(rng, container.m_data, amplitude);
                }
            }
        }
        else if (m_imageType == imgTypeGaussianSpotArray)
        {
            ito::DataObject droi;

            int width =
                this->m_channels[m_params["defaultChannel"].getVal<char*>()].m_data.getSize(1);
            int height =
                this->m_channels[m_params["defaultChannel"].getVal<char*>()].m_data.getSize(0);

            int roiwidth = (int)width / 2;
            int roiheight = (int)height / 2;

            int roi[4][4] = {
                {-0, -roiheight, -roiwidth, 0},
                {-0, -roiheight, 0, -roiwidth},
                {-roiheight, 0, -roiwidth, 0},
                {-roiheight, 0, 0, -roiwidth}};

            for (int cnt = 0; cnt < 4; cnt++)
            {
                foreach (ChannelContainer container, m_channels)
                {
                    droi = container.m_data;
                    droi = droi.adjustROI(roi[cnt][0], roi[cnt][1], roi[cnt][2], roi[cnt][3]);

                    if (max < 256)
                    {
                        ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, 8) - 1);
                        gaussFunc<ito::uint8>(rng, droi, amplitude);
                    }
                    else if (max < 65536)
                    {
                        ito::uint16 amplitude =
                            cv::saturate_cast<ito::uint16>(cv::pow(2.0, 16) - 1);
                        gaussFunc<ito::uint16>(rng, droi, amplitude);
                    }
                    else if (max < 2147483647)
                    {
                        ito::uint32 amplitude =
                            cv::saturate_cast<ito::uint32>(cv::pow(2.0, 32) - 1);
                        gaussFunc<ito::uint32>(rng, droi, amplitude);
                    }
                }
            }
        }

        if ((strcmp(m_params["triggerMode"].getVal<const char*>(), "software") == 0) &&
            (integration_time > 0.0))
        {
            double diff = (cv::getTickCount() - m_startOfLastAcquisition) / cv::getTickFrequency();

            if (diff < integration_time)
            {
                Sleep((integration_time - diff) * 1000.0);
            }
        }
    }
    ////pack all channel images to a QMap
    QSharedPointer<QMap<QString, ito::DataObject>> returnMap(new QMap<QString, ito::DataObject>);
    QMap<QString, ChannelContainer>::iterator it = m_channels.begin();
    while (it != m_channels.end())
    {
        (*returnMap)[it.key()] = it.value().m_data;
        ++it;
    }
    emit newData(returnMap);
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

    if (strcmp(m_params["triggerMode"].getVal<const char*>(), "software") == 0)
    {
        retValue += generateImageData();
    }
    else
    {
        retValue += ito::RetVal(
            ito::retWarning,
            0,
            tr("The trigger mode of the camera is set to freerun therefore calling acquire is "
               "useless.")
                .toLatin1()
                .data());
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
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
ito::RetVal DummyMultiChannelGrabber::getValByMap(
    QSharedPointer<QMap<QString, ito::DataObject*>> dataObjMap)
{
    ito::RetVal retValue(ito::retOk);

    retValue += retrieveData();

    if (!retValue.containsError())
    {
        if (dataObjMap == NULL)
        {
            retValue += ito::RetVal(
                ito::retError,
                1004,
                tr("QMap<QString, ito::DataObject*> of getVal is NULL").toLatin1().data());
        }
        else
        {
            retValue += sendDataToListeners(0); // don't wait for live image, since user should get
                                                // the image as fast as possible.
            QMap<QString, ito::DataObject*>::iterator it = (*dataObjMap).begin();
            while (it != (*dataObjMap).end())
            {
                *(it.value()) = this->m_channels[it.key()].m_data;
                ++it;
            }
        }
    }
    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal DummyMultiChannelGrabber::retrieveData(
    QSharedPointer<QMap<QString, ito::DataObject*>> dataObjMap)
{
    ito::RetVal retValue(ito::retOk);

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(
            ito::retError,
            1002,
            tr("image could not be obtained since no image has been acquired.").toLatin1().data());
    }
    else
    {
        if (dataObjMap)
        {
            QMap<QString, ito::DataObject*>::const_iterator it = (*dataObjMap).constBegin();
            while (it != (*dataObjMap).constEnd())
            {
                m_channels[it.key()].m_data.deepCopyPartial(*it.value());
                ++it;
            }
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//-------------------------------------------------------------------------------------
ito::RetVal DummyMultiChannelGrabber::copyValByMap(
    QSharedPointer<QMap<QString, ito::DataObject*>> dataObjMap)
{
    ito::RetVal retValue(ito::retOk);

    retValue += checkData(*dataObjMap);
    retValue += retrieveData(dataObjMap);

    if (!retValue.containsError())
    {
        if (dataObjMap == NULL)
        {
            retValue += ito::RetVal(
                ito::retError,
                1004,
                tr("QMap<QString, ito::DataObject*> of getVal is NULL").toLatin1().data());
        }
        else
        {
            retValue += sendDataToListeners(0); // don't wait for live image, since user should get
                                                // the image as fast as possible.
            QMap<QString, ito::DataObject*>::iterator it = (*dataObjMap).begin();
            while (it != (*dataObjMap).end())
            {
                *(it.value()) = this->m_channels[it.key()].m_data;
                ++it;
            }
        }
    }
    return retValue;
}



//-------------------------------------------------------------------------------------
ito::RetVal DummyMultiChannelGrabber::retrieveData(ito::DataObject* externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    if (m_isgrabbing == false)
    {
        retValue += ito::RetVal(
            ito::retError,
            1002,
            tr("image could not be obtained since no image has been acquired.").toLatin1().data());
    }
    else
    {
        if (externalDataObject)
        {
            auto internalImage = getCurrentDefaultChannel().m_data;
            internalImage.deepCopyPartial(*externalDataObject);
        }

        m_isgrabbing = false;
    }

    return retValue;
}

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
void DummyMultiChannelGrabber::fillColorImage(ito::DataObject& img, const cv::Point2f& centerPixel, const float radius, bool hasAlpha) const
{
    Q_ASSERT_X(img.getType() == ito::tRGBA32, "fillNextColorImage", "img must be of type rgba32");

    int height = img.getSize()[0];
    int width = img.getSize()[1];
    float hue, saturation, intensity, alpha;
    float dx, dy, r_square;
    float sigma_square = radius * radius;

    alpha = 1.0;
    intensity = 1.0;

    for (int i = 0; i < height; ++i)
    {
        ito::Rgba32* ptr = img.rowPtr<ito::Rgba32>(0, i);

        for (int j = 0; j < width; ++j)
        {
            dx = j - centerPixel.x;
            dy = i - centerPixel.y;
            r_square = dx * dx + dy * dy;

            hue = cv::fastAtan2(dy, dx) / 360.0;
            saturation = cv::exp(-0.5 * r_square / sigma_square);

            if (hasAlpha)
            {
                alpha = saturation;
            }

            ptr[j] = hsv2rgb(hue, saturation, intensity, alpha * 255);
        }
    }
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
