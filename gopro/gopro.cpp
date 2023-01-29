#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "GoPro.h"
#include "pluginVersion.h"
#include "gitVersion.h"

#include <qstring.h>
#include <qstringlist.h>
#include <qplugin.h>
#include <qurl.h>

#include <qobject.h>

#include <qnetworkreply.h>
#include <qnetworkrequest.h>
#include <qurlquery.h>
#include <qtimer.h>
#include <qeventloop.h>
#include <qjsondocument.h>

#include "dockWidgetGoPro.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/utils/logger.hpp>

using namespace std;
using namespace cv;

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of Interface Class.
GoProInterface::GoProInterface()
{
    m_type = ito::typeDataIO | ito::typeGrabber; //any grabber is a dataIO device AND its subtype grabber (bitmask -> therefore the OR-combination).
    setObjectName("GoPro");

    m_description = QObject::tr("GoPro");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"This template can be used for implementing a new type of camera or grabber plugin \n\
\n\
Put a detailed description about what the plugin is doing, what is needed to get it started, limitations...";
    m_detaildescription = QObject::tr(docstring);

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("The plugin's license string");
    m_aboutThis = QObject::tr(GITVERSION); 

    ito::Param paramVal = ito::Param(
        "colorMode",
        ito::ParamBase::String,
        "auto",
        tr("color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)")
            .toLatin1()
            .data());
    ito::StringMeta meta(ito::StringMeta::String);
    meta.addItem("auto");
    meta.addItem("color");
    meta.addItem("red");
    meta.addItem("green");
    meta.addItem("blue");
    meta.addItem("gray");
    paramVal.setMeta(&meta, false);
    m_initParamsOpt.append(paramVal);

    paramVal = ito::Param(
        "OpenCVLoggerLevel",
        ito::ParamBase::Int,
        utils::logging::LOG_LEVEL_WARNING,
        new ito::IntMeta(0, 6, 1),
        tr("OpenCV logger level for debugging. (SILENT = %1, FATAL = %2, ERROR = %3, WARNING = %4, "
           "INFO = %5, DEBUG = %6, VERBOSE = %7).")
            .arg(utils::logging::LOG_LEVEL_SILENT)
            .arg(utils::logging::LOG_LEVEL_FATAL)
            .arg(utils::logging::LOG_LEVEL_ERROR)
            .arg(utils::logging::LOG_LEVEL_WARNING)
            .arg(utils::logging::LOG_LEVEL_INFO)
            .arg(utils::logging::LOG_LEVEL_DEBUG)
            .arg(utils::logging::LOG_LEVEL_VERBOSE)
            .toUtf8()
            .data());
    m_initParamsOpt.append(paramVal);
}
       

//----------------------------------------------------------------------------------------------------------------------------------
//! Destructor of Interface Class.
GoProInterface::~GoProInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoProInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(GoPro) //the argument of the macro is the classname of the plugin
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoProInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(GoPro) //the argument of the macro is the classname of the plugin
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
#if QT_VERSION < 0x050000
    Q_EXPORT_PLUGIN2(GoProinterface, GoProInterface) //the second parameter must correspond to the class-name of the interface class, the first parameter is arbitrary (usually the same with small letters only)
#endif

//----------------------------------------------------------------------------------------------------------------------------------
//! Constructor of plugin.
    GoPro::GoPro() : AddInGrabber(), m_isGrabbing(false), m_NetworkManager(this), m_pDataMatBuffer(cv::Mat())
    {
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "GoPro", NULL);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "x0",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        848,
        0,
        tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param(
        "y0",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        480,
        0,
        tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param(
        "x1",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        1920,
        848,
        tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param(
        "y1",
        ito::ParamBase::Int | ito::ParamBase::In,
        0,
        1920,
        480,
        tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param(
        "sizex",
        ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In,
        1,
        3840,
        848,
        tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param(
        "sizey",
        ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In,
        1,
        3840,
        848,
        tr("height of ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    int roi[] = {0, 0, 848, 480};
    paramVal = ito::Param(
        "roi",
        ito::ParamBase::IntArray | ito::ParamBase::In,
        4,
        roi,
        tr("ROI (x,y,width,height)").toLatin1().data());
    ito::RectMeta* rm = new ito::RectMeta(ito::RangeMeta(0, 3840), ito::RangeMeta(0, 3840));
    paramVal.setMeta(rm, true);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param(
        "color_mode",
        ito::ParamBase::String,
        "auto",
        tr("color mode of camera (auto|color|red|green|blue|gray, default: auto -> color or gray)")
            .toLatin1()
            .data());
    ito::StringMeta meta(ito::StringMeta::String);
    meta.addItem("auto");
    meta.addItem("color");
    meta.addItem("red");
    meta.addItem("green");
    meta.addItem("blue");
    meta.addItem("gray");
    paramVal.setMeta(&meta, false);
    m_params.insert(paramVal.getName(), paramVal);

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 3*8, 3*8, tr("bpp").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("integrationTime", ito::ParamBase::Double | ito::ParamBase::In, 0.0, 1.0, 0.01, tr("Integrationtime of CCD [0..1] (no unit)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    
    //the following lines create and register the plugin's dock widget. Delete these lines if the plugin does not have a dock widget.
    DockWidgetGoPro *dw = new DockWidgetGoPro(this);
    
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);   
}

//----------------------------------------------------------------------------------------------------------------------------------
GoPro::~GoPro()
{
}

void GoPro::readyRead()
{
    QNetworkReply* reply = qobject_cast<QNetworkReply*>(sender());
    qDebug() << reply->readAll();

}

void GoPro::replyFinished(QNetworkReply& reply)
{
    if (reply.isFinished())
    {
        qDebug() << reply.readAll();
    }
}

void GoPro::slotReadyRead()
{
    bool a = true;
}

void GoPro::slotError()
{
    bool a = true;
}

void GoPro::slotSslErrors(QNetworkReply& reply, const QList<QSslError>& error)
{
    bool a = true;
}

void GoPro::get(QString location)
{
    QNetworkReply* reply = m_NetworkManager.get(QNetworkRequest(QUrl(location)));
    connect(reply, &QNetworkReply::readyRead, this, &GoPro::readyRead);
}

void GoPro::post(QString location, QByteArray data)
{
    QNetworkRequest request = QNetworkRequest(QUrl(location));
    request.setHeader(QNetworkRequest::ContentTypeHeader, "test/plain");
    QNetworkReply* reply = m_NetworkManager.post(request, data);
    connect(reply, &QNetworkReply::readyRead, this, &GoPro::readyRead);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::checkData(ito::DataObject* externalDataObject)
{
    ito::RetVal retValue = ito::retOk;

    if (!m_VideoCapture.grab())
    {
        retValue += ito::RetVal(ito::retError, 0, "could not acquire one test image");
    }

    if (!retValue.containsError())
    {
        if (!m_VideoCapture.retrieve(m_pDataMatBuffer))
        {
            QThread::msleep(200);
    
        }
    }

    if (!retValue.containsError())
    {
        m_imgChannels = m_pDataMatBuffer.channels();
        m_imgCols = m_pDataMatBuffer.cols;
        m_imgRows = m_pDataMatBuffer.rows;
        m_imgBpp = (int)m_pDataMatBuffer.elemSize1() * 8;

        static_cast<ito::IntMeta*>(m_params["sizex"].getMeta())->setMax(m_imgCols);
        static_cast<ito::IntMeta*>(m_params["sizey"].getMeta())->setMax(m_imgRows);
        m_params["sizex"].setVal<int>(m_imgCols);
        m_params["sizey"].setVal<int>(m_imgRows);

        ito::RectMeta* rm = new ito::RectMeta(
            ito::RangeMeta(0, m_imgCols, 1, 1, m_imgCols, 1),
            ito::RangeMeta(0, m_imgRows, 1, 1, m_imgRows, 1));
        m_params["roi"].setMeta(rm, true);
        int* roi = m_params["roi"].getVal<int*>();
        roi[0] = 0;
        roi[1] = 0;
        roi[2] = m_imgCols;
        roi[3] = m_imgRows;

        m_params["bpp"].setMeta(new ito::IntMeta(8, m_imgBpp), true);
        m_params["bpp"].setVal<int>(m_imgBpp);

        if (m_imgBpp < 8 || m_imgBpp > 32)
        {
            return ito::RetVal(ito::retError, 0, tr("unknown bpp").toLatin1().data());
        }
    }

    

    int futureHeight = m_params["sizey"].getVal<int>();
    int futureWidth = m_params["sizex"].getVal<int>();
    int futureChannels;
    int futureType;

    const char* colorMode = m_params["color_mode"].getVal<char*>();
    if (m_imgChannels == 1 && (m_colorMode == modeGray || m_colorMode == modeAuto))
    {
        futureChannels = 1;
    }
    else if (m_imgChannels == 3 && (m_colorMode == modeColor || m_colorMode == modeAuto))
    {
        futureChannels = 3;
    }
    else
    {
        futureChannels = 1;
    }

    int bpp = m_params["bpp"].getVal<int>();

    if (bpp <= 8 && futureChannels == 1)
    {
        futureType = ito::tUInt8;
    }
    else if (bpp <= 16 && futureChannels == 1)
    {
        futureType = ito::tUInt16;
    }
    else if (bpp <= 32 && futureChannels == 1)
    {
        futureType = ito::tInt32;
    }
    else if (futureChannels == 1)
    {
        futureType = ito::tFloat64;
    }
    else if (futureChannels == 3 && bpp <= 8)
    {
        futureType = ito::tRGBA32;
    }
    else
    {
        return ito::RetVal(
            ito::retError,
            0,
            tr("A camera with a bitdepth > 8 cannot be operated in color mode.").toLatin1().data());
    }

    if (futureType == ito::tRGBA32 &&
        (m_alphaChannel.cols != futureWidth || m_alphaChannel.rows != futureHeight))
    {
        m_alphaChannel = cv::Mat(futureHeight, futureWidth, CV_8UC1, cv::Scalar(255));
    }

    if (!externalDataObject)
    {
        if (m_data.getDims() != 2 || m_data.getSize(0) != (unsigned int)futureHeight ||
            m_data.getSize(1) != (unsigned int)futureWidth || m_data.getType() != futureType)
        {
            m_data = ito::DataObject(futureHeight, futureWidth, futureType);

            if (futureType == ito::tRGBA32)
            {
                // copy alpha channel to 4th channel in m_data
                const int relations[] = {0, 3};
                cv::mixChannels(&m_alphaChannel, 1, m_data.get_mdata()[0], 1, relations, 1);
            }
        }
    }
    else
    {
        int dims = externalDataObject->getDims();
        if (externalDataObject->getDims() == 0) // empty external dataObject
        {
            *externalDataObject = ito::DataObject(futureHeight, futureWidth, futureType);
        }
        else if (externalDataObject->calcNumMats() != 1)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Error during check data, external dataObject invalid. Object has more than 1 "
                   "plane or 0 planes. It must be of right size and type or an uninitialized "
                   "image.")
                    .toLatin1()
                    .data());
        }
        else if (
            externalDataObject->getSize(dims - 2) != (unsigned int)futureHeight ||
            externalDataObject->getSize(dims - 1) != (unsigned int)futureWidth ||
            externalDataObject->getType() != futureType)
        {
            return ito::RetVal(
                ito::retError,
                0,
                tr("Error during check data, external dataObject invalid. Object must be of right "
                   "size and type or a uninitialized image.")
                    .toLatin1()
                    .data());
        }

        if (futureType == ito::tRGBA32)
        {
            // copy alpha channel to 4th channel in m_data
            const int relations[] = {0, 3};
            cv::mixChannels(
                &m_alphaChannel,
                1,
                (cv::Mat*)externalDataObject->get_mdata()[externalDataObject->seekMat(0)],
                1,
                relations,
                1);
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! initialization of plugin
ito::RetVal GoPro::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //post("https://postman-echo.com/post", data);
    get("http://10.5.5.9/gp/gpControl/execute?p1=gpStream&a1=proto_v2&c1=restart");

    m_VideoCapture = VideoCapture();

    // set optional parameters
    if (!retValue.containsError())
    {
        QSharedPointer<ito::ParamBase> colorMode(new ito::ParamBase(
            "color_mode", ito::ParamBase::String, paramsOpt->at(0).getVal<char*>()));
        retValue += setParam(colorMode, NULL);

        utils::logging::setLogLevel(saturate_cast<utils::logging::LogLevel>(paramsOpt->at(1).getVal<int>()));
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

    setInitialized(true); //init method has been finished (independent on retval)
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! shutdown of plugin
ito::RetVal GoPro::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (m_VideoCapture.isOpened())
    {
        m_VideoCapture.release();
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
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
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
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
ito::RetVal GoPro::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    QMap<QString, ito::Param>::iterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if (!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if (!retValue.containsError())
    {
        //here the new parameter is checked whether its type corresponds or can be cast into the
        // value in m_params and whether the new type fits to the requirements of any possible
        // meta structure.
        retValue += apiValidateParam(*it, *val, false, true);
    }

    if (!retValue.containsError())
    {
        if (key == "color_mode")
        {
            const char* mode = val->getVal<char*>();

            if (m_imgChannels == 1)
            {
                if (QString::compare(mode, "auto") != 0 || QString::compare(mode, "gray") != 0)
                {
                    retValue += ito::RetVal(
                        ito::retError,
                        0,
                        "The connected grayscale camera cannot be operated in any colored "
                        "colorMode");
                }
            }
            else if (m_imgChannels == 3 && m_imgBpp > 8)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    "The connected color camera cannot output an color image since the bit depth "
                    "is > 8");
            }

            if (!retValue.containsError())
            {
                switch (mode[0])
                {
                case 'a':
                    m_colorMode = modeAuto;
                    break;
                case 'c':
                    m_colorMode = modeColor;
                    break;
                case 'g':
                    if (mode[2] == 'a')
                        m_colorMode = modeGray;
                    else
                        m_colorMode = modeGreen;
                    break;
                case 'r':
                    m_colorMode = modeRed;
                    break;
                case 'b':
                    m_colorMode = modeBlue;
                    break;
                }
            }
        }
    }

    if (!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::startDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    
    incGrabberStarted(); //increment a counter to see how many times startDevice has been called


    QThread* awakeThread = new QThread(this);
    QTimer* aliveTimer = new QTimer(nullptr);
    aliveTimer->setInterval(500);
    aliveTimer->stop();
    aliveTimer->moveToThread(awakeThread);
    aliveTimer->start();
    connect(aliveTimer, SIGNAL(timeout()), SLOT(videoCaptureTimerCallBack()), Qt::DirectConnection);
    connect(awakeThread, SIGNAL(started()), aliveTimer, SLOT(start()));
    awakeThread->start();

    if (m_VideoCapture.isOpened())
    {
        m_VideoCapture.release();
    }

    m_VideoCapture.open("udp://10.5.5.9:8554");

    awakeThread->quit();
    awakeThread->wait();
    aliveTimer->deleteLater();
    delete awakeThread;
    awakeThread = nullptr;

    //TODO add local loop single shot to send isAlive as long as open takes
    // check if succeeded
    if (!m_VideoCapture.isOpened())
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Connecting to video stream was not successful!").toUtf8().data());
    }

    if (!retValue.containsError())
    {
        checkData();
    }
    
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return retValue;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::stopDevice(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    decGrabberStarted(); //decrements the counter (see startDevice)

    if (grabberStartedCount() < 0)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("The grabber has already been stopped.").toLatin1().data());
        setGrabberStarted(0);
    }
    
    //todo:
    // if the counter (obtained by grabberStartedCount()) drops to zero again, stop the camera, free all allocated
    // image buffers of the camera... (it is the opposite from all things that have been started, allocated... in startDevice)

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    return ito::retOk;
}
         
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //now the wait condition is released, such that itom (caller) stops waiting and continuous with its own execution.
    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();  
    }
    
    if (grabberStartedCount() <= 0)
    {
        retValue += ito::RetVal(
            ito::retError,
            0,
            tr("Tried to acquire an image without having started the device.").toLatin1().data());
    }
    else
    {
        m_isGrabbing = true;
        //m_VideoCapture.retrieve(m_pDataMatBuffer);
        m_VideoCapture.read(m_pDataMatBuffer);

        if (!m_VideoCapture.grab())
        {
            retValue +=
                ito::RetVal(ito::retError, 0, tr("Could not acquire a new image!").toUtf8().data());
        }
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::retrieveData(ito::DataObject *externalDataObject)
{
    ito::RetVal retValue(ito::retOk);

    bool RetCode = false;
    cv::Mat* internalMat = NULL;

    int curxsize = m_params["sizex"].getVal<int>();
    int curysize = m_params["sizey"].getVal<int>();
    const int* roi = m_params["roi"].getVal<int*>();
    int x0 = roi[0];
    int y0 = roi[1];
    bool resizeRequired = (x0 > 0 || y0 > 0);

    ito::DataObject* dataObj = &m_data;
    if (externalDataObject)
    {
        dataObj = externalDataObject;
    }

    bool hasListeners = false;
    if (m_autoGrabbingListeners.size() > 0)
    {
        hasListeners = true;
    }

    if (m_isGrabbing == false)
    {
        retValue += ito::RetVal(
            ito::retWarning,
            0,
            tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //retValue += m_acquisitionRetVal;
        //m_acquisitionRetVal = ito::retOk;
    }

    if (!retValue.containsError())
    {
        if (m_pDataMatBuffer.cols == 0 || m_pDataMatBuffer.rows == 0)
        {
            retValue += ito::RetVal(
                ito::retError, 0, tr("Error: grabbed image is empty").toLatin1().data());
        }
        else
        {
            int desiredBpp = m_params["bpp"].getVal<int>();
            cv::Mat tempImage;

            if (m_imgCols != curxsize || m_imgRows != curysize)
            {
                resizeRequired = true;
            }

            if (m_imgBpp != 8 && m_imgBpp != 16)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Error: bpp other than 8 or 16 not allowed.").toLatin1().data());
            }
            else if (m_imgChannels != 1 && m_imgChannels != 3)
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Error: channels sizes other than 1 or 3 not allowed.").toLatin1().data());
            }
            else if ((desiredBpp != 8 && desiredBpp != 16))
            {
                retValue += ito::RetVal(
                    ito::retError,
                    0,
                    tr("Error: desired bpp must be 8 or 16 bit.").toLatin1().data());
            }
            else
            {
                // step 1. check ROI
                if (resizeRequired == false)
                {
                    tempImage = m_pDataMatBuffer;
                }
                else
                {
                    cv::Range ranges[] = {
                        cv::Range(y0, y0 + curysize), cv::Range(x0, x0 + curxsize)};
                    tempImage = cv::Mat(m_pDataMatBuffer, ranges);
                }

                // step 2. check whether 3 channel color should be transformed to 1 channel
                // grayscale
                if (m_imgChannels == 3 && m_colorMode == modeGray)
                {
#if (CV_MAJOR_VERSION >= 4)
                    cv::cvtColor(
                        tempImage,
                        tempImage,
                        cv::COLOR_BGR2GRAY,
                        0); // camera provides BGR images in OpenCV
#else
                    cv::cvtColor(
                        tempImage,
                        tempImage,
                        CV_BGR2GRAY,
                        0); // camera provides BGR images in OpenCV
#endif
                }

                // step 3: create m_data (if not yet available)
                if (externalDataObject && hasListeners)
                {
                    retValue += checkData(NULL); // update m_data
                    retValue += checkData(externalDataObject); // update external object
                }
                else
                {
                    retValue += checkData(externalDataObject); // update external object or m_data
                }

                if (!retValue.containsError())
                {
                    // step 4: check whether tempImage must be converted to other type
                    if (desiredBpp != m_imgBpp)
                    {
                        if (desiredBpp == 8)
                        {
                            tempImage.convertTo(tempImage, CV_8U);
                        }
                        else if (desiredBpp == 16)
                        {
                            tempImage.convertTo(tempImage, CV_16U);
                        }
                        else
                        {
                            retValue += ito::RetVal(
                                ito::retError,
                                0,
                                tr("Error while converting data format. Unsupported format.")
                                    .toLatin1()
                                    .data());
                        }
                    }
                }

                if (!retValue.containsError())
                {
                    if (tempImage.channels() == 1)
                    {
                        internalMat = dataObj->getCvPlaneMat(0);
                        tempImage.copyTo(*(internalMat));

                        if (externalDataObject && hasListeners)
                        {
                            internalMat = m_data.getCvPlaneMat(0);
                            tempImage.copyTo(*(internalMat));
                        }
                    }
                    else if (
                        tempImage.channels() == 3 &&
                        (m_colorMode == modeAuto || m_colorMode == modeColor))
                    {
                        cv::Mat out[] = {*(
                            dataObj->getCvPlaneMat(0))}; //{ *(cv::Mat*)(dataObj->get_mdata()[0]) ,
                                                         //*(cv::Mat*)(dataObj->get_mdata()[1]) ,
                                                         //*(cv::Mat*)(dataObj->get_mdata()[2]) };
                        int fromTo[] = {0, 0, 1, 1, 2, 2}; //{0,2,1,1,2,0}; //implicit BGR (camera)
                                                           //-> BGR (dataObject style) conversion

                        cv::mixChannels(&tempImage, 1, out, 1, fromTo, 3);

                        if (externalDataObject && hasListeners)
                        {
                            cv::Mat out[] = {*(dataObj->getCvPlaneMat(0))};
                            cv::mixChannels(&tempImage, 1, out, 1, fromTo, 3);
                        }
                    }
                    else if (tempImage.channels() == 3) // R,G,B selection
                    {
                        cv::Mat out[] = {*(dataObj->getCvPlaneMat(0))};
                        int fromTo[] = {0, 0};
                        switch (m_colorMode)
                        {
                        case modeRed:
                            fromTo[0] = 2;
                            break; // red
                        case modeGreen:
                            fromTo[0] = 1;
                            break; // green
                        default /*3*/:
                            fromTo[0] = 0;
                            break; // blue
                        }
                        cv::mixChannels(&tempImage, 1, out, 1, fromTo, 1);

                        if (externalDataObject && hasListeners)
                        {
                            cv::Mat out[] = {*(m_data.getCvPlaneMat(0))};
                            cv::mixChannels(&tempImage, 1, out, 1, fromTo, 1);
                        }
                    }
                    else
                    {
                        retValue += ito::RetVal(
                            ito::retError,
                            0,
                            tr("unknown color, conversion... combination in retrieveImage")
                                .toLatin1()
                                .data());
                    }
                }
            }
        }

        m_isGrabbing = false;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! Returns the grabbed camera frame as reference.

ito::RetVal GoPro::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    //call retrieveData without argument. Retrieve data should then put the currently acquired image into the dataObject m_data of the camera.
    retValue += retrieveData();

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.

        if (dObj)
        {
            (*dObj) = this->m_data; //copy reference to externally given object
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
ito::RetVal GoPro::copyVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    
    if (!dObj)
    {
        retValue += ito::RetVal(ito::retError, 0, tr("Empty object handle retrieved from caller").toLatin1().data());
    }
    
    if (!retValue.containsError())
    {
        //this method calls retrieveData with the passed dataObject as argument such that retrieveData is able to copy the image obtained
        //by the camera directly into the given, external dataObject
        retValue += retrieveData(dObj);  //checkData is executed inside of retrieveData
    }

    if (!retValue.containsError())
    {
        //send newly acquired image to possibly connected live images
        sendDataToListeners(0); //don't wait for live data, since user should get the data as fast as possible.
    }
    
    if (waitCond) 
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }
    
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
//! slot called if the dock widget of the plugin becomes (in)visible
void GoPro::dockWidgetVisibilityChanged(bool visible)
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

//----------------------------------------------------------------------------------------------------------------------------------
//! method called to show the configuration dialog
const ito::RetVal GoPro::showConfDialog(void)
{
    return apiShowConfigurationDialog(this, new DialogGoPro(this));
}

//----------------------------------------------------------------------------------------------------------------------------------
void GoPro::videoCaptureTimerCallBack()
{
    setAlive();
}