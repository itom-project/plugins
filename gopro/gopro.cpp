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

    //add mandatory and optional parameters for the initialization here.
    //append them to m_initParamsMand or m_initParamsOpt.
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

    paramVal = ito::Param("x0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y0", ito::ParamBase::Int | ito::ParamBase::In, 0, 2048, 0, tr("first pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("x1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1279, 1279, tr("last pixel index in ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("y1", ito::ParamBase::Int | ito::ParamBase::In, 0, 1023, 1023, tr("last pixel index in ROI (y-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizex", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("width of ROI (x-direction)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sizey", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::In, 1, 2048, 2048, tr("height of ROI (y-direction)").toLatin1().data());
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

    paramVal = ito::Param("bpp", ito::ParamBase::Int | ito::ParamBase::In, 8, 8, 8, tr("bpp").toLatin1().data());
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
//! initialization of plugin
ito::RetVal GoPro::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    //post("https://postman-echo.com/post", data);
    get("http://10.5.5.9/gp/gpControl/execute?p1=gpStream&a1=proto_v2&c1=restart");

    m_VideoCapture = VideoCapture();
    
    m_params["sizex"].setVal<int>(848);
    m_params["sizey"].setVal<int>(480);
    m_params["bpp"].setVal<int>(24);


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
        //if (key == "demoKey1")
        //{
        //    //check the new value and if ok, assign it to the internal parameter
        //    retValue += it->copyValueFrom( &(*val) );
        //}
        //else if (key == "demoKey2")
        //{
        //    //check the new value and if ok, assign it to the internal parameter
        //    retValue += it->copyValueFrom( &(*val) );
        //}
        //else
        //{
        //    //all parameters that don't need further checks can simply be assigned
        //    //to the value in m_params (the rest is already checked above)
        //    retValue += it->copyValueFrom( &(*val) );
        //}
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
    QTimer* aliveTimer = new QTimer(this);
    aliveTimer->setInterval(500);
    aliveTimer->stop();
    aliveTimer->moveToThread(awakeThread);
    aliveTimer->start();
    connect(aliveTimer, SIGNAL(timeout()), SLOT(videoCaptureTimerCallBack()), Qt::DirectConnection);
    connect(awakeThread, SIGNAL(started()), aliveTimer, SLOT(started()));
    awakeThread->start();

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
        cv::Mat *internalMat = m_data.getCvPlaneMat(0);
        cv::Mat temp = m_pDataMatBuffer;
        m_data = m_pDataMatBuffer;
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal GoPro::retrieveData(ito::DataObject *externalDataObject)
{
    //todo: this is just a basic example for getting the buffered image to m_data or the externally given data object
    //enhance it and adjust it for your needs
    ito::RetVal retValue(ito::retOk);

    ito::DataObject *dataObj = externalDataObject ? externalDataObject : &m_data;

    bool hasListeners = (m_autoGrabbingListeners.size() > 0);
    bool copyExternal = (externalDataObject != NULL);
    
    const int bufferWidth = m_params["sizex"].getVal<int>();
    const int bufferHeight = m_params["sizey"].getVal<int>();

    qDebug() << "channels: " << m_pDataMatBuffer.channels();
    qDebug() << "width: " << m_pDataMatBuffer.size().width;
    qDebug() << "height: " << m_pDataMatBuffer.size().height;
    qDebug() << "dtype: " << m_pDataMatBuffer.type();


    if (m_isGrabbing == false)
    {
        retValue += ito::RetVal(ito::retWarning, 0, tr("Tried to get picture without triggering exposure").toLatin1().data());
    }
    else
    {
        //step 1: create m_data (if not yet available)
        if (externalDataObject && hasListeners)
        {
            retValue += checkData(NULL); //update m_data
            retValue += checkData(externalDataObject); //update external object
        }
        else
        {
            retValue += checkData(externalDataObject); //update external object or m_data
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
                m_data.deepCopyPartial(*externalDataObject);
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