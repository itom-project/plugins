/* ********************************************************************
    Plugin "DummyGrabber" for itom software
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

#include "DummyGrabber.h"

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
#include "dockWidgetDummyGrabber.h"

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
/** @func   gaussFunc
*   @brief  function for 2d Gaussian function
*
*   This function delivers a 2d dataObject with a Gaussian function
*/
template<typename _Tp> ito::RetVal gaussFunc(cv::RNG &rng, ito::DataObject dObj, float amplitude)
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
        yval = ((y - height / 2 + yRandOffset) * ((float)y - height / 2 + yRandOffset)) / (2.0f * sigmaY * sigmaY);

        for (int x = 0; x < width; x++)
        {
            xval = ((x - width / 2 + xRandOffset) * ((float)x - width / 2 + xRandOffset)) / (2.0f * sigmaX * sigmaX);
            rowPtr[x] = (float)(amplitude - aRandOfset) * exp(-(xval + yval));
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \class DummyGrabberInterface
    \brief Small interface class for class DummyGrabber. This class contains basic information about DummyGrabber as is able to
        create one or more new instances of DummyGrabber.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! creates new instance of DummyGrabber and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created DummyGrabber-instance is stored in *addInInst
    \return retOk
    \sa DummyGrabber
*/
ito::RetVal DummyGrabberInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(DummyGrabber)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! deletes instance of DummyGrabber. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa DummyGrabber
*/
ito::RetVal DummyGrabberInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(DummyGrabber)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for interace
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: DummyGrabber) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.
*/
DummyGrabberInterface::DummyGrabberInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("DummyGrabber");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The DummyGrabber is a virtual camera which emulates a camera with white noise. \n\
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
You can choose between different image types (noise, GaussianSpot). \
The value range is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). \
The gaussianSpot has some random noise for the position and amplitude to move around a bit. The real size of the camera \
image is controlled using the parameter 'roi' if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
You can initialize this camera either as a 2D sensor with a width and height >= 4 or as line camera whose height is equal to 1. \n\
\n\
This plugin can also be used as template for other grabber.");

    m_author = "C. Kohler, W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = CREATEVERSION(1,4,0);
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = tr(GITVERSION);

    m_initParamsMand.clear();

    ito::Param param("maxXSize", ito::ParamBase::Int, 640, new ito::IntMeta(4, 4096, 4), tr("Width of virtual sensor chip").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("maxYSize", ito::ParamBase::Int, 480, new ito::IntMeta(1, 4096, 1), tr("Height of virtual sensor chip, please set this value to 1 (line camera) or a value dividable by 4 for a 2D camera.").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(8, 30, 2), tr("Bits per Pixel, usually 8-16bit grayvalues").toLatin1().data());
    m_initParamsOpt.append(param);

    param = ito::Param("imageType", ito::ParamBase::String | ito::ParamBase::In, "noise", tr("Available dummy image types: noise (default), gaussianSpot").toLatin1().data());
    ito::StringMeta sm(ito::StringMeta::String, "noise");
    sm.addItem("gaussianSpot");
    sm.addItem("gaussianSpotArray");
    param.setMeta(&sm, false);
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
DummyGrabberInterface::~DummyGrabberInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class DummyGrabberInterface with the name DummyGrabberinterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class DummyGrabber
    \brief Class for the DummyGrabber. The DummyGrabber is able to create noisy images or simulate a typical WLI or confocal image signal.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of dialogDummyGrabber, calls the method setVals of dialogDummyGrabber, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogDummyGrabber
*/
const ito::RetVal DummyGrabber::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogDummyGrabber(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for DummyGrabber
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the DummyGrabber's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this DummyGrabber-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
DummyGrabber::DummyGrabber() :
    AddInGrabber(),
    m_isgrabbing(false),
    m_totalBinning(1),
    m_lineCamera(false),
    m_imageType(imgTypeNoise)
{
    ito::DoubleMeta *dm;

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "DummyGrabber", "GrabberName");
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::String, "General"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("frame_time", ito::ParamBase::Double, 0.0, 60.0, 0.0, tr("Minimum time between the start of two consecutive acquisitions [s], default: 0.0.").toLatin1().data());
    dm = paramVal.getMetaT<ito::DoubleMeta>();
    dm->setCategory("AcquisitionControl");
    dm->setUnit("s");
    dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.0, 60.0, 0.0, tr("Minimum integration time for an acquisition [s], default: 0.0 (as fast as possible).").toLatin1().data());
    dm = paramVal.getMetaT<ito::DoubleMeta>();
    dm->setCategory("AcquisitionControl");
    dm->setUnit("s");
    dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("Virtual gain").toLatin1().data());
    dm = paramVal.getMetaT<ito::DoubleMeta>();
    dm->setCategory("AcquisitionControl");
    dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.0, tr("Virtual offset").toLatin1().data());
    dm = paramVal.getMetaT<ito::DoubleMeta>();
    dm->setCategory("AcquisitionControl");
    dm->setRepresentation(ito::ParamMeta::Linear); //show a linear slider in generic paramEditorWidget...
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("Binning of different pixel, binning = x-factor * 100 + y-factor").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 4, 4096, 4096, tr("size in x (cols) [px]").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 4096, 4096, tr("size in y (rows) [px]").toLatin1().data());
    paramVal.getMetaT<ito::IntMeta>()->setCategory("ImageFormatControl");
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 2048, 2048};
    paramVal = ito::Param("roi", ito::ParamBase::IntArray, 4, roi, tr("ROI (x,y,width,height) [this replaces the values x0,x1,y0,y1]").toLatin1().data());
    ito::RectMeta *rm = new ito::RectMeta(ito::RangeMeta(roi[0], roi[0] + roi[2] - 1), ito::RangeMeta(roi[1], roi[1] + roi[3] - 1), "ImageFormatControl");
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(8, 30, 2, "ImageFormatControl"), tr("bitdepth of images").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("demoRegexpString", ito::ParamBase::String, "", tr("matches strings without whitespaces").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::RegExp, "^\\S+$", "DemoParameters"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("demoWildcardString", ito::ParamBase::String, "test.bmp", tr("dummy filename of a bmp file, pattern: *.bmp").toLatin1().data());
    paramVal.setMeta(new ito::StringMeta(ito::StringMeta::Wildcard, "*.bmp", "DemoParameters"), true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("demoEnumString", ito::ParamBase::String, "mode 1", tr("enumeration string (mode 1, mode 2, mode 3)").toLatin1().data());
    ito::StringMeta *sm = new ito::StringMeta(ito::StringMeta::String, "mode 1", "DemoParameters");
    sm->addItem("mode 2");
    sm->addItem("mode 3");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("demoArbitraryString", ito::ParamBase::String, "any string", tr("any string allowed").toLatin1().data());
    sm = new ito::StringMeta(ito::StringMeta::String);
    sm->setCategory("DemoParameters");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);


    paramVal = ito::Param(
        "demoEnumStringList",
        ito::ParamBase::StringList,
        nullptr,
        tr("one or two options allowed.").toLatin1().data());

    ito::ByteArray strList[] = {ito::ByteArray("option1"), ito::ByteArray("option3")};
    paramVal.setVal<ito::ByteArray*>(strList, 2);

    sm = new ito::StringListMeta(ito::StringListMeta::String, 1, 2, 1, "DemoParameters");
    sm->addItem("option1");
    sm->addItem("option2");
    sm->addItem("option3");
    paramVal.setMeta(sm, true);
    m_params.insert(paramVal.getName(), paramVal);


    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetDummyGrabber *dw = new DockWidgetDummyGrabber(this);
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
DummyGrabber::~DummyGrabber()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of DummyGrabber.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal DummyGrabber::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retVal;

    int sizeX = paramsOpt->at(0).getVal<int>();     // first optional parameter, corresponding to the grabber width
    int sizeY = paramsOpt->at(1).getVal<int>();     // second optional parameter, corresponding to the grabber heigth

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

    // get type of dummy image
    QString type = paramsOpt->at(3).getVal<const char*>();

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
//! close method which is called before that this instance is deleted by the DummyGrabberInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal DummyGrabber::close(ItomSharedSemaphore *waitCond)
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
ito::RetVal DummyGrabber::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
ito::RetVal DummyGrabber::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
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
    In the DummyGrabber, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal DummyGrabber::startDevice(ItomSharedSemaphore *waitCond)
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
    In this DummyGrabber, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal DummyGrabber::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("stopDevice of DummyGrabber can not be executed, since camera has not been started.").toLatin1().data());
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
ito::RetVal DummyGrabber::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    double frame_time = m_params["frame_time"].getVal<double>();
    double integration_time = m_params["integration_time"].getVal<double>();
    int bpp = m_params["bpp"].getVal<double>();
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
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of DummyGrabber can not be executed, since camera has not been started.").toLatin1().data());
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


        if (m_imageType == imgTypeNoise)
        {

            if (m_totalBinning == 1)
            {
                if (bpp < 9)
                {
                    ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp)-1);
                    ito::uint8 *linePtr;

                    for (int m = 0; m < m_data.getSize(0); ++m)
                    {
                        linePtr = m_data.rowPtr<ito::uint8>(0, m);

                        for (int n = 0; n < m_data.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand<ito::uint8>(rng, maxInt, offset, gain);
                        }
                    }
                }
                else if (bpp < 17)
                {
                    ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp)-1);
                    ito::uint16 *linePtr;

                    for (int m = 0; m < m_data.getSize(0); ++m)
                    {
                        linePtr = m_data.rowPtr<ito::uint16>(0, m);

                        for (int n = 0; n < m_data.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand<ito::uint16>(rng, maxInt, offset, gain);
                        }
                    }
                }
                else if (bpp < 32)
                {
                    ito::int32 maxInt = cv::saturate_cast<ito::int32>(cv::pow(2.0, bpp)-1);
                    ito::int32 *linePtr;

                    for (int m = 0; m < m_data.getSize(0); ++m)
                    {
                        linePtr = m_data.rowPtr<ito::int32>(0, m);

                        for (int n = 0; n < m_data.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand<ito::int32>(rng, maxInt, offset, gain);
                        }
                    }
                }
            }
            else
            {
                if (bpp < 9)
                {
                    ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp)-1);
                    ito::uint8 *linePtr;

                    for (int m = 0; m < m_data.getSize(0); ++m)
                    {
                        linePtr = m_data.rowPtr<ito::uint8>(0, m);

                        for (int n = 0; n < m_data.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand_mean<ito::uint8>(rng, maxInt, m_totalBinning, offset, gain);
                        }
                    }
                }
                else if (bpp < 17)
                {
                    ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp)-1);
                    ito::uint16 *linePtr;

                    for (int m = 0; m < m_data.getSize(0); ++m)
                    {
                        linePtr = m_data.rowPtr<ito::uint16>(0, m);

                        for (int n = 0; n < m_data.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand_mean<ito::uint16>(rng, maxInt,m_totalBinning, offset, gain);
                        }
                    }
                }
                else if (bpp < 32)
                {
                    ito::int32 maxInt = cv::saturate_cast<ito::int32>(cv::pow(2.0, bpp)-1);
                    ito::int32 *linePtr;

                    for (int m = 0; m < m_data.getSize(0); ++m)
                    {
                        linePtr = m_data.rowPtr<ito::int32>(0, m);

                        for (int n = 0; n < m_data.getSize(1); ++n)
                        {
                            *linePtr++ = fastrand_mean<ito::int32>(rng, maxInt, m_totalBinning, offset, gain);
                        }
                    }
                }
            }

        }
        else if(m_imageType == imgTypeGaussianSpot) //create dummy Gaussian image
        {

            cv::RNG& rng = cv::theRNG();

            if (bpp < 9)
            {
                ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
                gaussFunc<ito::uint8>(rng, m_data, amplitude);
            }
            else if (bpp < 17)
            {
                ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
                gaussFunc<ito::uint16>(rng, m_data, amplitude);
            }
            else if (bpp < 32)
            {
                ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, bpp) - 1);
                gaussFunc<ito::uint32>(rng, m_data, amplitude);
            }
        }
        else if (m_imageType == imgTypeGaussianSpotArray) //create dummy Gaussian image
        {
            ito::DataObject droi;
            cv::RNG& rng = cv::theRNG();

            int width = m_data.getSize(1);
            int height = m_data.getSize(0);

            int roiwidth = (int)width / 2;
            int roiheight = (int)height / 2;

            int roi[4][4] = {   { -0, -roiheight, -roiwidth, 0 },
                                { -0, -roiheight, 0, -roiwidth },
                                { -roiheight, 0, -roiwidth, 0 },
                                { -roiheight, 0, 0, -roiwidth } };

            for (int cnt = 0; cnt < 4; cnt++)
            {
                droi = m_data;
                droi = droi.adjustROI(roi[cnt][0], roi[cnt][1], roi[cnt][2], roi[cnt][3]);

                if (bpp < 9)
                {
                    ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
                    gaussFunc<ito::uint8>(rng, droi, amplitude);
                }
                else if (bpp < 17)
                {
                    ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
                    gaussFunc<ito::uint16>(rng, droi, amplitude);
                }
                else if (bpp < 32)
                {
                    ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, bpp) - 1);
                    gaussFunc<ito::uint32>(rng, droi, amplitude);
                }
            }




            //// take 1 rechts oben
            //droi = droi.adjustROI(- 0, - roiheight, -roiwidth, 0); // rechts unten

            //if (bpp < 9)
            //{
            //    ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint8>(rng, droi, amplitude);
            //}
            //else if (bpp < 17)
            //{
            //    ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint16>(rng, droi, amplitude);
            //}
            //else if (bpp < 32)
            //{
            //    ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint32>(rng, droi, amplitude);
            //}

            //// take 2
            //droi = m_data;
            //droi = droi.adjustROI(-0, -roiheight, 0, - roiwidth); // links unten

            //if (bpp < 9)
            //{
            //    ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint8>(rng, droi, amplitude);
            //}
            //else if (bpp < 17)
            //{
            //    ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint16>(rng, droi, amplitude);
            //}
            //else if (bpp < 32)
            //{
            //    ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint32>(rng, droi, amplitude);
            //}

            //// take 3
            //droi = m_data;
            //droi = droi.adjustROI(- roiheight, 0, - roiwidth, 0); // rechts oben

            //if (bpp < 9)
            //{
            //    ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint8>(rng, droi, amplitude);
            //}
            //else if (bpp < 17)
            //{
            //    ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint16>(rng, droi, amplitude);
            //}
            //else if (bpp < 32)
            //{
            //    ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint32>(rng, droi, amplitude);
            //}

            //// take 4
            //droi = m_data;
            //droi = droi.adjustROI(-roiheight, 0, 0, -roiwidth); // links oben

            //if (bpp < 9)
            //{
            //    ito::uint8 amplitude = cv::saturate_cast<ito::uint8>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint8>(rng, droi, amplitude);
            //}
            //else if (bpp < 17)
            //{
            //    ito::uint16 amplitude = cv::saturate_cast<ito::uint16>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint16>(rng, droi, amplitude);
            //}
            //else if (bpp < 32)
            //{
            //    ito::uint32 amplitude = cv::saturate_cast<ito::uint32>(cv::pow(2.0, bpp) - 1);
            //    gaussFunc<ito::uint32>(rng, droi, amplitude);
            //}




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
ito::RetVal DummyGrabber::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal DummyGrabber::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal DummyGrabber::retrieveData(ito::DataObject *externalDataObject)
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
            m_data.deepCopyPartial(*externalDataObject);
        }

        m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DummyGrabber::dockWidgetVisibilityChanged(bool visible)
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
