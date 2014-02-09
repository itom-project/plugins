/* ********************************************************************
    Plugin "DummyGrabber" for itom software
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

#include "DummyGrabber.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#if (defined linux) //| (defined CMAKE)
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
#include "common/helperCommon.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

Q_DECLARE_METATYPE(ito::DataObject)

//----------------------------------------------------------------------------------------------------------------------------------
/** @func   fastrand
*   @brief  function for pseudo random values
*
*   This function delivers the noise for the image.
*/
template<typename _Tp> inline _Tp fastrand(ito::uint32 &seed, _Tp maxval)
{
    seed <<= 1;
    if ((ito::int32)seed <= 0)
        seed ^= 0x1d872b41;
    return (_Tp)(seed & maxval);
}

//----------------------------------------------------------------------------------------------------------------------------------
/** @func   waitForImage
*   @brief  This function waits until the grab is done.
*
*   During the wait a real camera should fill the buffer. In this simulation it is done manually
*/
int SimulatedCam::waitForImage(int /*timeoutMS*/)
{
    static ito::uint32 seed = 0;
    int maxElemsBpp = (1 << m_bpp) - 1;
    int64 start = cv::getCPUTickCount();

    if (!this->m_started)
    {
        return SimulatedCam::missingStart;
    }

    if (!this->m_grabbed)
    {
        return SimulatedCam::missingAcquire;
    }

    if (m_myInternalBuffer == NULL)
    {
        return SimulatedCam::missingBuffer;
    }

    if (m_bpp < 9)
    {
        ito::uint8 maxInt = cv::saturate_cast<ito::uint8>(maxElemsBpp);
        ito::uint8* ptrBuf = (ito::uint8*)m_myInternalBuffer;
        for (int n = 0; n < m_xsize * m_ysize; n++)
        {

            *ptrBuf++ = fastrand<ito::uint8>(seed, maxInt);
        }
    }
    else if (m_bpp < 17)
    {
        ito::uint16 maxInt = cv::saturate_cast<ito::uint16>(maxElemsBpp);
        ito::uint16* ptrBuf = (ito::uint16*)m_myInternalBuffer;
        for (int n = 0; n < m_xsize * m_ysize; n++)
        {

            *ptrBuf++ = fastrand<ito::uint16>(seed, maxInt);
        }
    }
    else if (m_bpp < 32)
    {
        ito::int32 maxInt = cv::saturate_cast<ito::int32>(maxElemsBpp);
        ito::int32* ptrBuf = (ito::int32*)m_myInternalBuffer;
        for (int n = 0; n < m_xsize * m_ysize; n++)
        {
            *ptrBuf++ = fastrand<ito::int32>(seed, maxInt);
        }
    }
    else
    {
        return wrongBPP;
    }

    start = cv::getCPUTickCount() -start;
    int waitTime = (int)(m_frameTimeMS - start/cv::getTickFrequency() * 1000 + 0.5);
    if (waitTime > 0)
    {
        Sleep(waitTime);
    }
    return SimulatedCam::okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
SimulatedCam::SimulatedCam(void):
    m_frameTimeMS(0),
    m_binX(1),
    m_binY(1),
    m_xsize_max(42),
    m_ysize_max(42),
    m_xsize(42),
    m_ysize(42),
    m_bpp(8),
    m_initDone(false),
    m_started(false),
    m_grabbed (false),
    m_myInternalBuffer(NULL)
{}

//----------------------------------------------------------------------------------------------------------------------------------
SimulatedCam::~SimulatedCam(void)
{
    if (m_myInternalBuffer != NULL)
    {
        free(m_myInternalBuffer);
        m_myInternalBuffer = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::initCamera(int maxXSize, int maxYSize, int maxBitDepth)
{
    if (m_myInternalBuffer != NULL)
    {
        return SimulatedCam::triedInitTwice;
    }

    if (m_initDone)
    {
        return SimulatedCam::triedInitTwice;
    }

    m_bpp = maxBitDepth;
    int bytesPerPixel  = 1;

    if (m_bpp < 9)
    {
        //nichts
    }
    else if (m_bpp < 17)
    {
        bytesPerPixel = 2;
    }
    else if (m_bpp < 33)
    {
        bytesPerPixel = 4;
    }
    else
    {
        return SimulatedCam::wrongBPP;
    }
    m_xsize_max = m_xsize = maxXSize;
    m_ysize_max = m_ysize = maxYSize;

    m_myInternalBuffer = calloc(bytesPerPixel, m_xsize * m_ysize);

    if (m_myInternalBuffer == NULL)
    {
        return SimulatedCam::initFailedBuffer;
    }
    m_initDone = true;
    return SimulatedCam::okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::setBinning(int binX, int binY)
{
    if (!m_initDone)
    {
        return missingInit;
    }
    if (m_started)
    {
        return stillRunning;
    }

    if (binX > 4 || binY > 4 || binX < 1 || binY < 1)
    {
        return binningFailed;
    }
    if (m_xsize_max % binX != 0 || m_ysize_max % binY != 0)
    {
        return binningFailed;
    }
    m_binX = binX;
    m_binY = binY;
    m_xsize = m_xsize_max/binX;
    m_ysize = m_ysize_max/binY;
    return okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::getBinning(int &binX, int &binY)
{
    if (!m_initDone)
    {
        return missingInit;
    }
    binX = m_binX;
    binY = m_binY;
    return okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::getSize(int &sizeX, int &sizeY)
{
    if (!m_initDone)
    {
        return missingInit;
    }
    sizeX = m_xsize;
    sizeY = m_ysize;
    return okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
char* SimulatedCam::getImageBuffer(void)
{
    if (m_grabbed == true)
    {
        m_grabbed = false;
        return (char*)m_myInternalBuffer;
    }
    return NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::acquireImage(void)
{
    if (m_started != true)
    {
        return SimulatedCam::missingStart;
    }
    m_grabbed = true;
    return SimulatedCam::okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::prepareCamera(void)
{
    if (m_initDone != true)
    {
        return SimulatedCam::missingInit;
    }
    m_started = true;
    return SimulatedCam::okay;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SimulatedCam::stopCamera(void)
{
    if (m_initDone == true)
    {
        m_started = false;
        m_grabbed = false;
        return SimulatedCam::okay;
    }
    else
        return SimulatedCam::missingInit;
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
    DummyGrabber* newInst = new DummyGrabber();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

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
   if (*addInInst)
   {
      delete ((DummyGrabber *)*addInInst);
      int idx = m_InstList.indexOf(*addInInst);
      m_InstList.removeAt(idx);
   }

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
    char docstring[] = \
"The DummyGrabber is a virtual camera which emulates a camera with white noise. \n\
\n\
The camera is initialized with a maximum width and height of the simulated camera chip (both need to be a multiple of 4). \
The noise is always scaled in the range between 0 and the current bitdepth (bpp - bit per pixel). The real size of the camera \
image is controlled using the parameters x0, y0, x1 and y1 if the sizes stay within the limits given by the size of the camera chip.\n\
\n\
This plugin can also be used as template for other grabber. For the simulation of real measurement systems, \
please also check the plugin 'emulationGrabber'";
   
    m_description = QObject::tr("A virtual white noise grabber");
    m_detaildescription = QObject::tr(docstring);
    m_author = "C. Kohler, W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = tr("N.A.");      
    
    m_initParamsMand.clear();

    ito::Param param("maxXSize", ito::ParamBase::Int, 640, new ito::IntMeta(1, 4096), tr("Maximum x size of image").toAscii().data());
    m_initParamsOpt.append(param);

    param = ito::Param("maxYSize", ito::ParamBase::Int, 480, new ito::IntMeta(1, 4096), tr("Maximum y size of image").toAscii().data());
    m_initParamsOpt.append(param);

    param = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(1, 32), tr("Bits per Pixel, usually 8-16bit grayvalues").toAscii().data());
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
Q_EXPORT_PLUGIN2(DummyGrabberinterface, DummyGrabberInterface)

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
    dialogDummyGrabber *confDialog = new dialogDummyGrabber(this);

    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    QMetaObject::invokeMethod(this, "sendParameterRequest");

    if (confDialog->exec())
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        confDialog->sendVals();
    }
    else
    {
        disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), confDialog, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    }
    delete confDialog;

    return ito::retOk;
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
    m_isgrabbing(false)
{

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "DummyGrabber", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.005, 100.0, 12.5, tr("Integrationtime of CCD [s]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.05, 150.0, 33.333333, tr("Time between two frames").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double, 0.0, 1.0, 1.0, tr("Virtual gain").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double, 0.0, 1.0, 0.5, tr("Virtual offset").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int, 101, 404, 101, tr("Binning of different pixel").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("size in x (cols) [px]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("size in y (rows) [px]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int, 0, 2047, 0, tr("first pixel in x (cols) within ROI [zero-based, <= x1]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int, 0, 2047, 0, tr("first pixel in y (rows) within ROI [zero-based, <= y1]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x1", ito::ParamBase::Int, 0, 2047, 2047, tr("last pixel in x (cols) within ROI [zero-based, >= x0]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int, 0, 2047, 2047, tr("last pixel in y (rows) within ROI [zero-based, >= y0]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int, 8, 64, 8, tr("bitdepth of images").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("time_out", ito::ParamBase::Double, 0.1, 60.0, 60.0, tr("Timeout for acquiring images [s]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    //now create dock widget for this plugin
    DockWidgetDummyGrabber *dw = new DockWidgetDummyGrabber(m_params, getID());
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
DummyGrabber::~DummyGrabber()
{
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
    QMap<QString,ito::Param>::iterator it;

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
    QString key = val->getName();
    int ret = 0, sizeX = 0, sizeY = 0;
    int running = 0; // Used to check if grabber was running bevor

    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString,ito::Param>::iterator it;

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
        if (grabberStartedCount())
        {
            running = grabberStartedCount();
            setGrabberStarted(1);
            retValue += stopDevice(0);
        }
        else
        {
            setGrabberStarted(1);
            stopDevice(0);
        }
    }

    if (!retValue.containsError())
    {
        if (key == "binning")
        {
            int newValue = val->getVal<int>();
            int newbinX = newValue/100;
            int newbinY = newValue-newbinX *100;

            int maxbinX = (int)it->getMax()/100;
            int maxbinY = (int)it->getMax()- maxbinX * 100;

            int minbinX = (int)it->getMin()/100;
            int minbinY = (int)it->getMin()- minbinX * 100;

            if (newbinX > maxbinX)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("New value in X is larger than maximal value, input ignored").toAscii().data());
            }
            else if (newbinY > maxbinY)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("New value in Y is larger than maximal value, input ignored").toAscii().data());
            }
            else if (newbinX < minbinX)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("New value in X is smaller than parameter range, input ignored").toAscii().data());
            }
            else if (newbinY < minbinY)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("New value in Y is smaller than parameter range, input ignored").toAscii().data());
            }
            else
            {
                ret = myCam.setBinning(newbinX, newbinY);
                if (ret)
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("Set binning failed").toAscii().data());
                }
                ret = myCam.getBinning(newbinX, newbinY);
                it->setVal<int>(newbinX*100+newbinY);
            }
        }
        else //all other parameters
        {
            retValue += it->copyValueFrom(&(*val));

            //check further dependent stuff
            // Adapted parameters and send out depending parameter
            if (!retValue.containsError() && (key == "x0" || key == "y0" || key == "x1" || key == "y1"))
            {
                static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(m_params["x1"].getVal<int>());
                static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(m_params["y1"].getVal<int>());

                static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMin(m_params["x0"].getVal<int>());
                static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMin(m_params["y0"].getVal<int>());

                m_params["sizex"].setVal<int>(m_params["x1"].getVal<int>()-m_params["x0"].getVal<int>()+1);
                m_params["sizey"].setVal<int>(m_params["y1"].getVal<int>()-m_params["y0"].getVal<int>()+1);
            }
        }
    }

    if (!retValue.containsError())
    {
        retValue += checkData(); //check if image must be reallocated

        if (running)
        {
            retValue += this->startDevice(0);
            setGrabberStarted(running);
        }

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

    ito::RetVal retVal(ito::retOk,0,"");

    int sizeX = (*paramsOpt)[0].getVal<int>();    // is the first parameter in the opt-List, corresponding to the gabber width
    if (sizeX % 4 != 0)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during initialisation of DummyGrabber: SizeX must be a dividable of 4").toAscii().data());
    }

    int sizeY = (*paramsOpt)[1].getVal<int>();    // is the second parameter in the opt-List, corresponding to the gabber heigth
    if (sizeY == 1)
    {
        static_cast<ito::IntMeta*>(m_params["binning"].getMeta())->setMax(401);
    }
    else if (sizeY % 4 != 0)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during initialisation of DummyGrabber: SizeY must be a dividable of 4").toAscii().data());
    }

    int bpp = (*paramsOpt)[2].getVal<int>();    // is the third parameter in the opt-List, corresponding to the gabber bit depth per pixel
    m_params["bpp"].setVal<int>(bpp);

    m_params["sizex"].setVal<int>(sizeX);
    static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(sizeX);
    static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(sizeX-1);
    static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMax(sizeX-1);
    m_params["x1"].setVal<int>(sizeX-1);

    m_params["sizey"].setVal<int>(sizeY);
    static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(sizeY);
    static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(sizeY-1);
    static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMax(sizeY-1);
    m_params["y1"].setVal<int>(sizeY-1);

    int ret = myCam.initCamera(sizeX, sizeY, bpp);
    if (ret)
    {
        retVal += ito::RetVal(ito::retError, 0, tr("Error during initialisation of DummyGrabber").toAscii().data());
    }

    if (!retVal.containsError())
    {
        checkData(); //check if image must be reallocated
    }

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();

        setInitialized(true); //init method has been finished (independent on retval)
        return retVal;
    }
    else
    {
        setInitialized(true); //init method has been finished (independent on retval)
        return retVal;
    }
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
// function checkData() moved into addInGrabber.cpp -> standard for all cameras / ADDA

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
        int ret = myCam.prepareCamera();
        if (ret)
        {
            retValue += ito::RetVal(ito::retError, ret, tr("Error during virtual camera preparation.").toAscii().data());
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
    if (grabberStartedCount() == 0)
    {
        int ret = myCam.stopCamera();
        if (ret)
        {
            retValue += ito::RetVal(ito::retError, ret, tr("Error during virtual camera stop command.").toAscii().data());
        }

    }
    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("StopDevice of DummyGrabber can not be executed, since camera has not been started.").toAscii().data());
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

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of DummyGrabber can not be executed, since camera has not been started.").toAscii().data());
    }
    else
    {
        this->m_isgrabbing = true;
        int ret = myCam.acquireImage();
        if (ret)
        {
            retValue += ito::RetVal(ito::retError, ret, tr("Error during virtual camera acquisition.").toAscii().data());
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
            retValue += ito::RetVal(ito::retError, 1004, tr("data object of getVal is NULL or cast failed").toAscii().data());
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
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toAscii().data());
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
//! slot invoked if gain or offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void DummyGrabber::GainOffsetPropertiesChanged(double gain, double offset)
{
    if (checkNumericParamRange(m_params["offset"], offset))
    {
        m_params["offset"].setVal<double>(offset);
    }
    if (checkNumericParamRange(m_params["gain"], gain))
    {
        m_params["gain"].setVal<double>(gain);
    }
    updateCamParams();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain or offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void DummyGrabber::IntegrationPropertiesChanged(double integrationtime)
{
    if (checkNumericParamRange(m_params["integration_time"], integrationtime))
    {
        m_params["integration_time"].setVal<double>(integrationtime);
        updateCamParams();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot invoked if gain or offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
ito::RetVal DummyGrabber::updateCamParams(void)
{
    double intTime = m_params["integration_time"].getVal<double>();
    double gain = m_params["gain"].getVal<double>();
    double offset = m_params["offset"].getVal<double>();
    ito::RetVal retValue = ito::RetVal(ito::retOk, 0,"");

    int ret = myCam.setFrameTime(intTime);
    if (ret)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Set integrationtime failed").toAscii().data());
    }
    ret = myCam.setOffsetGain(gain, offset);
    if (ret)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Set offset and gain failed").toAscii().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DummyGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    int ret = 0;
    int timeOutMS = (int)(m_params["time_out"].getVal<double>() * 1000 + 0.5);

//    unsigned long imglength = 0;
//    long lcopysize = 0;
    long lsrcstrpos = 0;
//    int y  = 0;
    int maxxsize = (int)m_params["sizex"].getMax();
    int maxysize = (int)m_params["sizey"].getMax();
    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();

    bool hasListeners = false;
    bool copyExternal = false;
    if (m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }
    if (externalDataObject != NULL)
    {
        copyExternal = true;
    }

    ret = myCam.waitForImage(timeOutMS);
    //if (grabberStartedCount() <= 0 || this->m_isgrabbing != true)
    if (ret)
    {
        if (ret == SimulatedCam::missingAcquire)
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber can not be executed, since acquire was called.").toAscii().data());
        }
        else if (ret == SimulatedCam::missingStart)
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber can not be executed, since camera has not been started.").toAscii().data());
        }
        else if (ret == SimulatedCam::missingBuffer)
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber can not be executed, since no buffer allocated.").toAscii().data());
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber failed.").toAscii().data());
        }
    }
    else
    {
        int bpp = m_params["bpp"].getVal<int>();
        if (bpp <= 8)
        {
            ito::uint8 *cbuf=(ito::uint8*)myCam.getImageBuffer();
            if (cbuf == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber failed, since retrived NULL-Pointer.").toAscii().data());
            }
            else if (curxsize == maxxsize)
            {
                lsrcstrpos = y0 * maxxsize;
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, curysize);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, curysize);
            }
            else
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
            }
        }
        else if (bpp <= 16)
        {
            ito::uint16 *cbuf=(ito::uint16*)myCam.getImageBuffer();
            if (cbuf == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber failed, since retrived NULL-Pointer.").toAscii().data());
            }
            else if (curxsize == maxxsize)
            {
                lsrcstrpos = y0 * maxxsize;
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, curysize);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, curysize);
            }
            else
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
            }
        }
        else if (bpp <= 32)
        {
            ito::int32 *cbuf=(ito::int32*)myCam.getImageBuffer();
            if (cbuf == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber failed, since retrived NULL-Pointer.").toAscii().data());
            }
            else if (curxsize == maxxsize)
            {
                lsrcstrpos = y0 * maxxsize;
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::int32>((ito::int32*)cbuf+lsrcstrpos, maxxsize, curysize);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::int32>((ito::int32*)cbuf+lsrcstrpos, maxxsize, curysize);
            }
            else
            {
                if (copyExternal) retValue += externalDataObject->copyFromData2D<ito::int32>((ito::int32*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
                if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<ito::int32>((ito::int32*)cbuf, maxxsize, maxysize, x0, y0, curxsize, curysize);
            }
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 1002, tr("getVal of DummyGrabber failed, since undefined bitdepth.").toAscii().data());
        }
        this->m_isgrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DummyGrabber::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetDummyGrabber *dw = qobject_cast<DockWidgetDummyGrabber*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            connect(dw, SIGNAL(GainOffsetPropertiesChanged(double,double)), this, SLOT(GainOffsetPropertiesChanged(double,double)));
            connect(dw, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            disconnect(dw, SIGNAL(GainOffsetPropertiesChanged(double,double)), this, SLOT(GainOffsetPropertiesChanged(double,double)));
            disconnect(dw, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));
        }
    }
}
