/* ********************************************************************
    Plugin "FileGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#include "FileGrabber.h"

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
#include "dockWidgetFileGrabber.h"

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>

#if (CV_MAJOR_VERSION >= 3)
#include "opencv2/imgcodecs.hpp"
#endif

#include "common/helperCommon.h"
#include "DataObject/dataObjectFuncs.h"

#include "pluginVersion.h"
#include "gitVersion.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

Q_DECLARE_METATYPE(ito::DataObject)
//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class FileGrabberInterface
    \brief Small interface class for class FileGrabber. This class contains basic information about FileGrabber as is able to
        create one or more new instances of FileGrabber.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! creates new instance of FileGrabber and returns the instance-pointer.
/*!
    \param [in,out] addInInst is a double pointer of type ito::AddInBase. The newly created FileGrabber-instance is stored in *addInInst
    \return retOk
    \sa FileGrabber
*/
ito::RetVal FileGrabberInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(FileGrabber)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! deletes instance of FileGrabber. This instance is given by parameter addInInst.
/*!
    \param [in] double pointer to the instance which should be deleted.
    \return retOk
    \sa FileGrabber
*/
ito::RetVal FileGrabberInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(FileGrabber)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! constructor for interace
/*!
    defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: FileGrabber) should or must
    be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
    and m_initParamsOpt within this constructor.
*/
FileGrabberInterface::FileGrabberInterface()
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeDataIO | ito::typeGrabber;
    setObjectName("FileGrabber");

    m_description = QObject::tr("A virtual grabber");
/*    char docstring[] = \
"This plugin emulates a camera by grabbing images from files in a specific folder on the hard disk. Alternatively, it is possible to iteratively load images from \
a 3D-data object (stack). The first possibility uses the command imread from OpenCV (OpenCV library highgui required). \n\
\n\
The grabber can work in 3 different modes: \n\
1) Files in a specified folder are sequentially loaded each time a getVal / copyVal is performed. At the moment only 8 and 16 bit images are supported.\n\
2) Images are sequentially loaded from a 3D data object (stack of images). Each getVal / copyVal returns a reference/copy to the specific plane of the stack. Supports 8, 12, 14, 16, 24-bit.\n\
3) Files in a specified folder are scanned and preloaded to an image stack. Each getVal / copyVal returns a reference/copy to the specific plane of the stack. Supports 8, 12, 14, 16, 24-bit.\n\
\n\
In the second case, provide the objectStack argument, arguments bpp and sourceFolder are ignored. bpp is generated by the type and value range of the objectStack.";
    m_detaildescription = QObject::tr(docstring);*/

    m_detaildescription = QObject::tr(
"This plugin emulates a camera by grabbing images from files in a specific folder on the hard disk. Alternatively, it is possible to iteratively load images from \
a 3D-data object (stack). The first possibility uses the command imread from OpenCV (OpenCV library highgui required). \n\
\n\
The grabber can work in 3 different modes: \n\
1) Files in a specified folder are sequentially loaded each time a getVal / copyVal is performed. At the moment only 8 and 16 bit images are supported.\n\
2) Images are sequentially loaded from a 3D data object (stack of images). Each getVal / copyVal returns a reference/copy to the specific plane of the stack. Supports 8, 12, 14, 16, 24-bit.\n\
3) Files in a specified folder are scanned and preloaded to an image stack. Each getVal / copyVal returns a reference/copy to the specific plane of the stack. Supports 8, 12, 14, 16, 24-bit.\n\
\n\
In the second case, provide the objectStack argument, arguments bpp and sourceFolder are ignored. bpp is generated by the type and value range of the objectStack.");



    m_author = "W. Lyda, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("Licensed under LPGL.");
    m_aboutThis = QObject::tr(GITVERSION);

    m_initParamsOpt.clear();

    ito::Param param("fileFilter", ito::ParamBase::String, "tif", QObject::tr("File-Type, e.g. 'tif', 'png'").toLatin1().data());
    m_initParamsMand.append(param);

    param = ito::Param("sourceFolder", ito::ParamBase::String, "", QObject::tr("Absolute path of the source images").toLatin1().data());
    m_initParamsMand.append(param);

    param = ito::Param("bpp", ito::ParamBase::Int, 8, new ito::IntMeta(0,24), QObject::tr("Destination bit depth. 0: Auto or 8, 12, 14, 16, 24").toLatin1().data());
    m_initParamsMand.append(param);

    param = ito::Param("preloadImages", ito::ParamBase::Int, 0, new ito::IntMeta(0, 1000), QObject::tr("If 0, no preloading is active, else the first n image are loaded to a stack.").toLatin1().data());
    m_initParamsMand.append(param);

    param = ito::Param("objectStack", ito::ParamBase::DObjPtr, NULL, QObject::tr("If not NULL and preloading is active, an 3D-Object can to used for the grabber.").toLatin1().data());
    m_initParamsOpt.append(param);
}

//----------------------------------------------------------------------------------------------------------------------------------
//! destructor
/*!
    clears both vectors m_initParamsMand and m_initParamsOpt.
*/
FileGrabberInterface::~FileGrabberInterface()
{
    m_initParamsMand.clear();
    m_initParamsOpt.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
// this makro registers the class FileGrabberInterface with the name FileGrabberinterface as plugin for the Qt-System (see Qt-DOC)


//----------------------------------------------------------------------------------------------------------------------------------

/*!
    \class FileGrabber
    \brief Class for the FileGrabber. The FileGrabber is able to create noisy images or simulate a typical WLI or confocal image signal.

    Usually every method in this class can be executed in an own thread. Only the constructor, destructor, showConfDialog will be executed by the
    main (GUI) thread.
*/

//----------------------------------------------------------------------------------------------------------------------------------
//! shows the configuration dialog. This method must be executed in the main (GUI) thread and is usually called by the addIn-Manager.
/*!
    creates new instance of dialogFileGrabber, calls the method setVals of dialogFileGrabber, starts the execution loop and if the dialog
    is closed, reads the new parameter set and deletes the dialog.

    \return retOk
    \sa dialogFileGrabber
*/
const ito::RetVal FileGrabber::showConfDialog(void)
{
    dialogFileGrabber *confDialog = new dialogFileGrabber(this);

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
//! constructor for FileGrabber
/*!
    In this constructor the m_params-vector with all parameters, which are accessible by getParam or setParam, is built.
    Additionally the optional docking widget for the FileGrabber's toolbar is instantiated and created by createDockWidget.

    \param [in] uniqueID is an unique identifier for this FileGrabber-instance
    \sa ito::tParam, createDockWidget, setParam, getParam
*/
FileGrabber::FileGrabber() :
    AddInGrabber(),
    m_isgrabbing(false),
    m_fromStack(false),
    m_curTick(0.0),
    m_lastTick(0.0)
{
    //qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");
    //qRegisterMetaType<ito::DataObject>("ito::DataObject");

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "FileGrabber", "GrabberName");
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integration_time", ito::ParamBase::Double, 0.005, 100.0, 12.5, tr("Integrationtime of CCD programmed in s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("frame_time", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.05, 150.0, 33.333333, tr("Time between two frames").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("gain", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1.0, 1.0, tr("Virtual gain").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("offset", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.0, 1.0, 0.5, tr("Virtual offset").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("binning", ito::ParamBase::Int | ito::ParamBase::Readonly, 101, 101, 101, tr("Binning of different pixel").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 2048, 2048, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 2047, 0, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 2047, 0, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 2047, 2047, tr("Pixelsize in x (cols)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::Readonly, 0, 2047, 2047, tr("Pixelsize in y (rows)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::Readonly, 8, 64, 8, tr("Grabdepth of the images").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("time_out", ito::ParamBase::Double | ito::ParamBase::Readonly, 0.1, 60.0, 60.0, tr("Timeout for acquiring images").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("current_image", ito::ParamBase::Int, 0, 24, 24, tr("The current shown image").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("number_of_images", ito::ParamBase::Int | ito::ParamBase::Readonly, 1, 25, 25, tr("The maximal number of images").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    m_fileList.clear();

    m_searchFolder.setPath("");

    m_preloadedObject = ito::DataObject();

    if (hasGuiSupport())
    {
        //now create dock widget for this plugin
        DockWidgetFileGrabber *dw = new DockWidgetFileGrabber(m_params, getID());
        connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
        connect(dw, SIGNAL(GainOffsetPropertiesChanged(double, double)), this, SLOT(GainOffsetPropertiesChanged(double, double)));
        connect(dw, SIGNAL(IntegrationPropertiesChanged(double)), this, SLOT(IntegrationPropertiesChanged(double)));

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
FileGrabber::~FileGrabber()
{
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
ito::RetVal FileGrabber::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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

    \param [in] name is the key name of the parameter
    \param [in] val is the double value to set.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal FileGrabber::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();
    int running = 0; // Used to check if grabber was running bevor

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toLatin1().data());
    }
    else
    {

        if (grabberStartedCount())
        {
            running = grabberStartedCount();
            setGrabberStarted(1);
            retValue += this->stopDevice(0);
        }
        else
        {
            setGrabberStarted(1);
            this->stopDevice(0);
        }

        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {

            if (paramIt->getFlags() & ito::ParamBase::Readonly)
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toLatin1().data());
                goto end;
            }
            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if ( curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toLatin1().data());
                    goto end;
                }
                else
                {
                    paramIt.value().setVal<double>(curval);
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom( &(*val) );
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toLatin1().data());
                goto end;
            }

            if (!retValue.containsError())
            {

                // Adapted parameters and send out depending parameter
                if (!paramIt.key().compare("x0") ||
                   !paramIt.key().compare("x1") ||
                   !paramIt.key().compare("y0") ||
                   !paramIt.key().compare("y1"))
                {

                    static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax( m_params["x1"].getVal<int>() );
                    static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax( m_params["y1"].getVal<int>() );

                    static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMin( m_params["x0"].getVal<int>() );
                    static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMin( m_params["x0"].getVal<int>() );

                    m_params["sizex"].setVal<int>(m_params["x1"].getVal<int>()-m_params["x0"].getVal<int>()+1);
                    m_params["sizey"].setVal<int>(m_params["y1"].getVal<int>()-m_params["y0"].getVal<int>()+1);
                }
                else if (!paramIt.key().compare("frame_time"))
                {

                }
                else if (!paramIt.key().compare("integration_time"))
                {

                }
            }
            emit parametersChanged(m_params);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toLatin1().data());
        }
    }

end:
    retValue += checkData(); //check if image must be reallocated

    if (running)
    {
        retValue += this->startDevice(0);
        setGrabberStarted(running);
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! init method which is called by the addInManager after the initiation of a new instance of FileGrabber.
/*!
    This init method gets the mandatory and optional parameter vectors of type tParam and must copy these given parameters to the
    internal m_params-vector. Notice that this method is called after that this instance has been moved to its own (non-gui) thread.

    \param [in] paramsMand is a pointer to the vector of mandatory tParams.
    \param [in] paramsOpt is a pointer to the vector of optional tParams.
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
*/
ito::RetVal FileGrabber::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> * paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retVal;

    const char* fileEnding = (*paramsMand)[0].getVal<char*>();
    int bppFilter          = (*paramsMand)[2].getVal<int>();
    int nrPreLoading       = (*paramsMand)[3].getVal<int>();
    ito::DataObject* exObj = (*paramsOpt)[0].getVal<ito::DataObject*>();

    int sizeX = 0;
    int sizeY = 0;

    if ( paramsMand->at(1).getVal<char*>() )
    {
        m_searchFolder.setPath(QLatin1String(paramsMand->at(1).getVal<char*>()));
    }
    else
    {
        m_searchFolder.setPath("");
    }

    QString filter(fileEnding);

    if (filter.isEmpty())
    {
        filter = "*.tif";
    }

    if (exObj != NULL && nrPreLoading >= 0)   // Check if an external object provided and if so, set the interal image stack to this external data object
    {
        if (exObj->getDims() != 3 && !retVal.containsError())
        {
            retVal += ito::RetVal(ito::retError, 0, QObject::tr("Stack object must be 3 dimensional").toLatin1().data());
        }
        else
        {
            ito::float64 maxValD(0.0);
            ito::int32 maxValInt(0);
            ito::uint32 values[3];
            switch(exObj->getType())
            {
            case ito::tUInt8:
                bppFilter = 8;
                break;
            case ito::tUInt16:
                ito::dObjHelper::maxValue(exObj, maxValD, values);
                maxValInt = cv::saturate_cast<ito::uint16>(maxValD);
                if (maxValD < 1024) bppFilter = 10;
                else if (maxValD < 4096) bppFilter = 12;
                else if (maxValD < 16384) bppFilter = 14;
                else bppFilter = 16;
                break;
            case ito::tInt32:
                ito::dObjHelper::maxValue(exObj, maxValD, values);
                maxValInt = cv::saturate_cast<ito::int32>(maxValD);
                if (maxValD < 16777216) bppFilter = 24;
                else bppFilter = 30;
                break;
            default:
                retVal += ito::RetVal(ito::retError, 0, QObject::tr("Stack object type must be UInt8, UInt16 or Int32").toLatin1().data());
            }
        }
        if (!retVal.containsError())
        {
            if (exObj->getSize(0) <= nrPreLoading || nrPreLoading == 0) //if 0: auto, also take reference to the full data object stack
            {
                m_preloadedObject = (*exObj);
            }
            else
            {
                ito::Range ranges[] = { ito::Range(0,nrPreLoading), ito::Range::all(), ito::Range::all() };
                m_preloadedObject = exObj->at(ranges);
            }

            nrPreLoading = 0;
            sizeX = m_preloadedObject.getSize(2);
            sizeY = m_preloadedObject.getSize(1);

            m_params["number_of_images"].setVal<int>(m_preloadedObject.getSize(0));
            m_params["current_image"].setVal<int>(0);
            static_cast<ito::IntMeta*>(m_params["current_image"].getMeta())->setMax(m_preloadedObject.getSize(0)-1); //m_params["sizex"].setMax((double)sizeX);
            static_cast<ito::IntMeta*>(m_params["number_of_images"].getMeta())->setMax(m_preloadedObject.getSize(0)); //m_params["x0"].setMax((double)sizeX-1.0);
        }
    }
    else if (m_searchFolder.exists() && m_searchFolder.isReadable()) // Else scan the seachfolder and check if there is anything readable
    {
        QStringList filterList(filter);
        m_fileList = m_searchFolder.entryList(filterList, QDir::Files, QDir::Name);

        if (m_fileList.isEmpty())
        {
            retVal += ito::RetVal(ito::retError, 0, QObject::tr("Folder %1 does not contain any matching files").arg( m_searchFolder.absolutePath() ).toLatin1().data());
        }
    }
    else    // No search path and no object so this operation failed!
    {
        retVal += ito::RetVal(ito::retError, 0, QObject::tr("Folder %1 does not exist or is not readable").arg( m_searchFolder.absolutePath() ).toLatin1().data());
    }

    if (exObj == NULL && !retVal.containsError())    // If we have an external object we are finished. In other cases we have to scan the folder and do some magic stuff with it.
    {
        int i;
        cv::Mat loadedMat;
        QString fileName("");

#if (CV_MAJOR_VERSION >= 4)
        int openCVLoadingFlags = cv::IMREAD_UNCHANGED;
#else
        int openCVLoadingFlags = CV_LOAD_IMAGE_UNCHANGED;
#endif

        for(i = 0; i < m_fileList.size(); i++)
        {
#if (CV_MAJOR_VERSION >= 4)
            loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[i]).toLatin1().data(), cv::IMREAD_UNCHANGED);
#else
            loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[i]).toLatin1().data(), CV_LOAD_IMAGE_UNCHANGED);
#endif

            if (loadedMat.data != NULL && bppFilter == 0) // Loading okay and no bppFilter
            {
//                int type = loadedMat.type();

                sizeX = loadedMat.cols;
                sizeY = loadedMat.rows;
                if (loadedMat.type() == CV_8U)
                {
                    bppFilter = 8;
#if (CV_MAJOR_VERSION >= 4)
                    if (nrPreLoading == 0) openCVLoadingFlags = cv::IMREAD_GRAYSCALE;
#else
                    if (nrPreLoading == 0) openCVLoadingFlags = CV_LOAD_IMAGE_GRAYSCALE;
#endif
                }
                else if (loadedMat.type() == CV_16U)
                {
                    bppFilter = 16;
#if (CV_MAJOR_VERSION >= 4)
                    if (nrPreLoading == 0) openCVLoadingFlags = cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH;
#else
                    if (nrPreLoading == 0) openCVLoadingFlags = CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH;
#endif
                }
                else if (loadedMat.type() == CV_8UC3)
                {
                    bppFilter = 24;
#if (CV_MAJOR_VERSION >= 4)
                    if (nrPreLoading == 0) openCVLoadingFlags = cv::IMREAD_COLOR;
#else
                    if (nrPreLoading == 0) openCVLoadingFlags = CV_LOAD_IMAGE_COLOR;
#endif
                }
                else if (loadedMat.type() == CV_8UC4)
                {
                    bppFilter = 32;
#if (CV_MAJOR_VERSION >= 4)
                    if (nrPreLoading == 0) openCVLoadingFlags = cv::IMREAD_ANYCOLOR;
#else
                    if (nrPreLoading == 0) openCVLoadingFlags = CV_LOAD_IMAGE_ANYCOLOR;
#endif
                }

                i++;
                break;
            }
            else if (loadedMat.data != NULL) // Loading okay but we have to check if the bpp is valid
            {
//                int type = loadedMat.type();

                if ((loadedMat.type() == CV_8U && bppFilter == 8) ||
                   (loadedMat.type() == CV_16U && bppFilter > 8 && bppFilter < 17) ||
                   (loadedMat.type() == CV_8UC3 && bppFilter == 24) ||
                   (loadedMat.type() == CV_8UC4 && bppFilter == 32)
                   )
                {
                    sizeX = loadedMat.cols;
                    sizeY = loadedMat.rows;
                    i++;
                    break;
                }
                else
                {
                    m_fileList[i] = "";
                }
            }
            else    // Loading failed
            {
                m_fileList[i] = "";
            }

        }

        for(; i < m_fileList.size(); i++)
        {
            loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[i]).toLatin1().data(), openCVLoadingFlags);
            if (loadedMat.data != NULL)
            {
                if (!sizeX == loadedMat.cols ||
                   !sizeY == loadedMat.rows ||
                   (
                       !(bppFilter == 8 && loadedMat.type() == CV_8U)  &&
                       !((bppFilter < 17 && bppFilter >8)  && loadedMat.type() == CV_16U) &&
                       !(bppFilter == 24 && loadedMat.type() == CV_8UC3) &&
                       !(bppFilter == 32 && loadedMat.type() == CV_8UC4)
                   ))
                {
                    m_fileList[i] = "";
                }
            }
            else
            {
                m_fileList[i] = "";
            }
        }
        m_fileList.removeAll("");


        if (nrPreLoading > m_fileList.size())
        {
            nrPreLoading = m_fileList.size();
        }

        if (m_fileList.size() > 0 && nrPreLoading > 0)
        {
            int copySize = 0;
            if (bppFilter == 8)
            {
                copySize = 1;
                m_preloadedObject = ito::DataObject(nrPreLoading, sizeY, sizeX, ito::tUInt8);
            }
            else if (bppFilter < 17 && bppFilter >8)
            {
                copySize = 2;
                m_preloadedObject = ito::DataObject(nrPreLoading, sizeY, sizeX, ito::tUInt16);
            }
            else if (bppFilter == 24 || bppFilter == 30)
            {
                copySize = 4;
                m_preloadedObject = ito::DataObject(nrPreLoading, sizeY, sizeX, ito::tInt32);
            }
            for (i = 0; i < nrPreLoading; i++)
            {
                loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[i]).toLatin1().data(), openCVLoadingFlags);
                if (loadedMat.data != NULL && loadedMat.type() != CV_8UC3)
                {
                    memcpy(((cv::Mat*)m_preloadedObject.get_mdata()[i])->ptr(), loadedMat.ptr(), sizeX * sizeY * copySize);
                }
                else if (loadedMat.data != NULL && loadedMat.type() == CV_8UC3)
                {
                    cv::Vec3b intensity;
                    unsigned long blue;
                    unsigned long green;
                    unsigned long red;

                    ito::int32* rowPtr = NULL;

                    for(int y = 0; y < loadedMat.rows; y++)
                    {
                        rowPtr = (ito::int32*)((cv::Mat*)((cv::Mat*)m_preloadedObject.get_mdata()[i])->ptr(y));
                        for(int x = 0; x < loadedMat.cols; x++)
                        {
                            intensity = loadedMat.at<cv::Vec3b>(y, x);
                            blue = (unsigned long) intensity.val[2];
                            green = (unsigned long) intensity.val[1];
                            red = (unsigned long) intensity.val[0];
                            rowPtr[x] = (blue << 16) + (green << 8) + red;
                        }
                    }
                }
                else if (loadedMat.data != NULL && loadedMat.type() == CV_8UC4)
                {
                    cv::Vec4b intensity;
                    unsigned long alpha;
                    unsigned long blue;
                    unsigned long green;
                    unsigned long red;

                    ito::int32* rowPtr = NULL;

                    for(unsigned int y = 0; y < (unsigned int)loadedMat.rows; y++)
                    {
                        rowPtr = (ito::int32*)((cv::Mat*)((cv::Mat*)m_preloadedObject.get_mdata()[i])->ptr(y));
                        for(unsigned int x = 0; x < (unsigned int)loadedMat.cols; x++)
                        {
                            intensity = loadedMat.at<cv::Vec4b>(y, x);
                            alpha = (unsigned long) intensity.val[3];
                            blue = (unsigned long) intensity.val[2];
                            green = (unsigned long) intensity.val[1];
                            red = (unsigned long) intensity.val[0];
                            rowPtr[x] = (alpha << 24) + (blue << 16) + (green << 8) + red;
                        }
                    }
                }
                else
                {
                    retVal += ito::RetVal(ito::retError, 0, QObject::tr("Image loading failed").toLatin1().data());
                }

            }

            m_params["number_of_images"].setVal<int>(nrPreLoading);
            m_params["current_image"].setVal<int>(0);
            static_cast<ito::IntMeta*>(m_params["current_image"].getMeta())->setMax(nrPreLoading-1); //m_params["sizex"].setMax((double)sizeX);
            static_cast<ito::IntMeta*>(m_params["number_of_images"].getMeta())->setMax(nrPreLoading); //m_params["x0"].setMax((double)sizeX-1.0);
        }
        else if (m_fileList.size() > 0)  // we have files but no preloading
        {
            m_params["number_of_images"].setVal<int>(m_fileList.size());
            m_params["current_image"].setVal<int>(0);
            static_cast<ito::IntMeta*>(m_params["current_image"].getMeta())->setMax(m_fileList.size()-1); //m_params["sizex"].setMax((double)sizeX);
            static_cast<ito::IntMeta*>(m_params["number_of_images"].getMeta())->setMax(m_fileList.size()); //m_params["x0"].setMax((double)sizeX-1.0);
        }
        else
        {
            retVal += ito::RetVal(ito::retError, 0, QObject::tr("No suitable images found").toLatin1().data());
        }
    }

    if (m_preloadedObject.getDims() == 3)
    {
        m_fromStack = true;
    }
    else
    {
        m_fromStack = false;
    }

    m_params["bpp"].setVal<int>(bppFilter);

    m_params["sizex"].setVal<int>(sizeX);
    static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(sizeX); //m_params["sizex"].setMax((double)sizeX);
    static_cast<ito::IntMeta*>(m_params["x0"].getMeta())->setMax(sizeX-1); //m_params["x0"].setMax((double)sizeX-1.0);
    static_cast<ito::IntMeta*>(m_params["x1"].getMeta())->setMax(sizeX-1); //m_params["x1"].setMax((double)sizeX-1.0);
    m_params["x1"].setVal<int>(sizeX-1);

    m_params["sizey"].setVal<int>(sizeY);
    static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(sizeY); //m_params["sizey"].setMax((double)sizeY);
    static_cast<ito::IntMeta*>(m_params["y0"].getMeta())->setMax(sizeY-1); //m_params["y0"].setMax((double)sizeY-1.0);
    static_cast<ito::IntMeta*>(m_params["y1"].getMeta())->setMax(sizeY-1); //m_params["y1"].setMax((double)sizeY-1.0);
    m_params["y1"].setVal<int>(sizeY-1);

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
//! close method which is called before that this instance is deleted by the FileGrabberInterface
/*!
    notice that this method is called in the actual thread of this instance.

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk
    \sa ItomSharedSemaphore
*/
ito::RetVal FileGrabber::close(ItomSharedSemaphore *waitCond)
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

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! With startDevice this camera is initialized.
/*!
    In the FileGrabber, this method does nothing. In general, the hardware camera should be intialized in this method and necessary memory should be allocated.

    \note This method is similar to VideoCapture::open() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if starting was successfull, retWarning if startDevice has been calling at least twice.
*/
ito::RetVal FileGrabber::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    checkData(); //this will be reallocated in this method.

    incGrabberStarted();

    if (grabberStartedCount() == 1)
    {
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
    In this FileGrabber, this method does nothing. In general, the hardware camera should be closed in this method.

    \note This method is similar to VideoCapture::release() of openCV

    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk if everything is ok, retError if camera wasn't started before
    \sa startDevice
*/
ito::RetVal FileGrabber::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    decGrabberStarted();
    if (grabberStartedCount() == 0)
    {

    }
    else if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 1001, tr("StopDevice of FileGrabber can not be executed, since camera has not been started.").toLatin1().data());
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
ito::RetVal FileGrabber::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue = ito::retOk;

    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(ito::retError, 1002, tr("Acquire of FileGrabber can not be executed, since camera has not been started.").toLatin1().data());
    }
    else
    {
        this->m_isgrabbing = true;

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
ito::RetVal FileGrabber::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
ito::RetVal FileGrabber::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
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
//! slot invoked if gain or offset parameters in docking toolbox have been manually changed
/*!
    \param [in] gain
    \param [in] offset
*/
void FileGrabber::GainOffsetPropertiesChanged(double gain, double offset)
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
void FileGrabber::IntegrationPropertiesChanged(double integrationtime)
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
ito::RetVal FileGrabber::updateCamParams(void)
{
//    double intTime = m_params["integration_time"].getVal<double>();
//    double gain = m_params["gain"].getVal<double>();
//    double offset = m_params["offset"].getVal<double>();
    ito::RetVal retValue = ito::RetVal(ito::retOk, 0,"");

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
template<typename _Tp> inline ito::RetVal FileGrabber::transferDataFromStack(const int current_image, const bool hasListeners, const bool copyExternal, ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    long lsrcstrpos = 0;
    int maxxsize = (int)m_params["sizex"].getMax();
    int maxysize = (int)m_params["sizey"].getMax();
    int x0 = m_params["x0"].getVal<int>();
    int y0 = m_params["y0"].getVal<int>();
    int sizes[2] = {m_params["sizey"].getVal<int>(), m_params["sizex"].getVal<int>()};

    _Tp *cbuf = (_Tp*)((cv::Mat*)m_preloadedObject.get_mdata()[current_image])->ptr();

    if (sizes[1] == maxxsize && sizes[0] == maxysize)   // stack and ROI are of equal size
    {
        if (copyExternal)
        {

            retValue += externalDataObject->copyFromData2D<_Tp>((_Tp*)cbuf, maxxsize, maxysize);
        }
        if (!copyExternal || hasListeners)
        {
            m_data = ito::DataObject(2, sizes, m_preloadedObject.getType(), (cv::Mat*)(m_preloadedObject.get_mdata()[current_image]), 1);
        }
    }
    else if (sizes[1] == maxxsize)
    {
        lsrcstrpos = y0 * maxxsize;
        if (copyExternal)
        {
            retValue += externalDataObject->copyFromData2D<_Tp>((_Tp*)cbuf+lsrcstrpos, maxxsize, maxysize);
        }
        if (!copyExternal || hasListeners)
        {
            retValue += m_data.copyFromData2D<_Tp>((_Tp*)cbuf+lsrcstrpos, maxxsize, maxysize);
        }
    }
    else
    {
        if (copyExternal) retValue += externalDataObject->copyFromData2D<_Tp>((_Tp*)cbuf, maxxsize, maxysize, x0, y0, sizes[1], sizes[0]);
        if (!copyExternal || hasListeners) retValue += m_data.copyFromData2D<_Tp>((_Tp*)cbuf, maxxsize, maxysize, x0, y0, sizes[1], sizes[0]);
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal FileGrabber::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);
    int ret = 0;
//    int timeOutMS = (int)(m_params["time_out"].getVal<double>() * 1000 + 0.5);

//    unsigned long imglength = 0;
//    long lcopysize = 0;

    int current_image = m_params["current_image"].getVal<int>();
    m_params["current_image"].setVal<int>((current_image + 1) % m_params["number_of_images"].getVal<int>());

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

    //if (grabberStartedCount() <= 0 || this->m_isgrabbing != true)
    if (ret)
    {

    }
    else
    {
        int bpp = m_params["bpp"].getVal<int>();

        if (m_fromStack)
        {
            if (bpp <= 8)
            {
                retValue += transferDataFromStack<ito::uint8>(current_image, hasListeners, copyExternal, externalDataObject);
            }
            else if (bpp <= 16)
            {
                retValue += transferDataFromStack<ito::uint16>(current_image, hasListeners, copyExternal, externalDataObject);
            }
            else if (bpp <= 32)
            {
                retValue += transferDataFromStack<ito::int32>(current_image, hasListeners, copyExternal, externalDataObject);
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("getVal of FileGrabber failed, since undefined bitdepth.").toLatin1().data());
            }
        }
        else
        {
            cv::Mat loadedMat;
            //loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[current_image]).toLatin1().data(), CV_LOAD_IMAGE_UNCHANGED);
#if (CV_MAJOR_VERSION >= 4)
            loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[current_image]).toLatin1().data(), cv::IMREAD_GRAYSCALE | cv::IMREAD_ANYDEPTH);
#else
            loadedMat = cv::imread(m_searchFolder.absoluteFilePath(m_fileList[current_image]).toLatin1().data(), CV_LOAD_IMAGE_GRAYSCALE | CV_LOAD_IMAGE_ANYDEPTH);
#endif
            long lsrcstrpos = 0;
            int maxxsize = (int)m_params["sizex"].getMax();
            int maxysize = (int)m_params["sizey"].getMax();
            int x0 = m_params["x0"].getVal<int>();
            int y0 = m_params["y0"].getVal<int>();

            int sizes[2] = {m_params["sizey"].getVal<int>(), m_params["sizex"].getVal<int>()};

            if (loadedMat.data == NULL)
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("Unable to load file").toLatin1().data());
            }
            else if (loadedMat.type() == CV_8U)
            {
                m_params["bpp"].setVal<int>(8);
                ito::uint8 *cbuf = (ito::uint8*)loadedMat.ptr();

                if (sizes[1] == loadedMat.cols && sizes[0] == loadedMat.rows)   // stack and ROI are of equal size
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        m_data = ito::DataObject(2, sizes, ito::tUInt8, &loadedMat, 1);
                    }
                }
                else if (sizes[1] == (int)loadedMat.cols && sizes[0] < (int)loadedMat.rows)
                {
                    lsrcstrpos = y0 * maxxsize;
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, maxysize);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf+lsrcstrpos, maxxsize, maxysize);
                    }
                }
                else if (sizes[1] < (int)loadedMat.cols && sizes[0] < (int)loadedMat.rows)
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, sizes[1], sizes[0]);
                    }

                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint8>((ito::uint8*)cbuf, maxxsize, maxysize, x0, y0, sizes[1], sizes[0]);
                    }
                }
            }
            else if (loadedMat.type() == CV_16U)
            {
                m_params["bpp"].setVal<int>(16);
                ito::uint16 *cbuf = (ito::uint16*)loadedMat.ptr();

                if (sizes[1] == loadedMat.cols && sizes[0] == loadedMat.rows)   // stack and ROI are of equal size
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        m_data = ito::DataObject(2, sizes, ito::tUInt16, &loadedMat, 1);
                    }
                }
                else if (sizes[1] == (int)loadedMat.cols && sizes[0] < (int)loadedMat.rows)
                {
                    lsrcstrpos = y0 * maxxsize;
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, maxysize);
                    }
                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf+lsrcstrpos, maxxsize, maxysize);
                    }
                }
                else if (sizes[1] < (int)loadedMat.cols && sizes[0] < (int)loadedMat.rows)
                {
                    if (copyExternal)
                    {
                        retValue += externalDataObject->copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, sizes[1], sizes[0]);
                    }

                    if (!copyExternal || hasListeners)
                    {
                        retValue += m_data.copyFromData2D<ito::uint16>((ito::uint16*)cbuf, maxxsize, maxysize, x0, y0, sizes[1], sizes[0]);
                    }
                }
            }/*
            else if (loadedMat.type() == CV_8UC3)
            {
                cv::Vec3b intensity;
                unsigned long blue;
                unsigned long green;
                unsigned long red;

                ito::int32* rowPtr = NULL;

                for(unsigned int y = 0; y < loadedMat.rows; y++)
                {
                    rowPtr = (ito::int32*)((cv::Mat*)((cv::Mat*)m_preloadesObject.get_mdata()[i])->ptr(y));
                    for(unsigned int x = 0; x < loadedMat.cols; x++)
                    {
                        intensity = loadedMat.at<cv::Vec3b>(y, x);
                        blue = (unsigned long) intensity.val[2];
                        green = (unsigned long) intensity.val[1];
                        red = (unsigned long) intensity.val[0];
                        rowPtr[x] = (blue << 16) + (green << 8) + red;
                    }
                }
            }
            else if (loadedMat.type() == CV_8UC4)
            {
                cv::Vec4b intensity;
                unsigned long alpha;
                unsigned long blue;
                unsigned long green;
                unsigned long red;

                ito::int32* rowPtr = NULL;

                for(unsigned int y = 0; y < loadedMat.rows; y++)
                {
                    rowPtr = (ito::int32*)((cv::Mat*)((cv::Mat*)m_preloadesObject.get_mdata()[i])->ptr(y));
                    for(unsigned int x = 0; x < loadedMat.cols; x++)
                    {
                        intensity = loadedMat.at<cv::Vec4b>(y, x);
                        alpha = (unsigned long) intensity.val[3];
                        blue = (unsigned long) intensity.val[2];
                        green = (unsigned long) intensity.val[1];
                        red = (unsigned long) intensity.val[0];
                        rowPtr[x] = (alpha << 24) + (blue << 16) + (green << 8) + red;
                    }
                }
            }*/
            else
            {
                retValue += ito::RetVal(ito::retError, 1002, tr("Unsupported image data type.").toLatin1().data());
            }
        }
        this->m_isgrabbing = false;
    }

    if (current_image == 0)
    {
        m_curTick = (double)(cv::getTickCount())/cv::getTickFrequency();
        m_params["frame_time"].setVal<double>((m_curTick-m_lastTick)/m_params["number_of_images"].getVal<int>());
        m_lastTick = m_curTick;
    }

    return retValue;
}
