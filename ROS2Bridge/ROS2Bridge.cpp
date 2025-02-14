// Ro s2 bridge.cpp
#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "ROS2Bridge.h"
#include "dockWidgetROS2Bridge.h"
#include <math.h>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>
#include <qwaitcondition.h>
#include "pluginVersion.h"
#include "gitVersion.h"
#include <qmetaobject.h>
#include "common/helperCommon.h"
#include "common/paramMeta.h"

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

#include <iostream>
#include <qdebug.h>


ito::RetVal ROS2BridgeInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(ROS2Bridge)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2BridgeInterface::closeThisInst(ito::AddInBase **addInInst)
{
    REMOVE_PLUGININSTANCE(ROS2Bridge)
    return ito::retOk;
}


//----------------------------------------------------------------------------------------------------------------------------------
ROS2BridgeInterface::ROS2BridgeInterface()
{


    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("ROS2Bridge");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The ROS2Bridge is a virtual actuator plugin that emulates up to 10 linear axes. \n\
\n\
The real number of simulated axes is given by the initialization parameter 'numAxis'. Use this plugin \
to simulate or develop your measurement system at another computer. Whenever a position command is executed, \
this plugin sleeps until the time needed for the positioning (with respect to the speed of the axis) \
expired.";*/

    m_description = tr("A Bridge between ITOM and ROS2.");
//    m_detaildescription = QObject::tr(docstring);
    m_detaildescription = QObject::tr(
"The RO24Bridge is a virtual actuator plugin that emulates up to 10 linear axes. \n\
\n\
The real number of simulated axes is given by the initialization parameter 'numAxis'. Use this plugin \
to simulate or develop your measurement system at another computer. Whenever a position command is executed, \
this plugin sleeps until the time needed for the positioning (with respect to the speed of the axis) \
expired.");

    m_author = PLUGIN_AUTHOR;
    m_version = PLUGIN_VERSION;
    m_minItomVer = PLUGIN_MIN_ITOM_VERSION;
    m_maxItomVer = PLUGIN_MAX_ITOM_VERSION;
    m_license = QObject::tr(PLUGIN_LICENCE);
    m_aboutThis = QObject::tr(GITVERSION);

    ito::Param paramVal("ServiceServerName", ito::ParamBase::String, "default Name", tr("The name of the service server to connect to").toLatin1().data() );
    m_initParamsMand.append(paramVal);


    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
ROS2BridgeInterface::~ROS2BridgeInterface()
{
 
}

const ito::RetVal ROS2Bridge::showConfDialog(void)
{
return ito::retOk;
}

ROS2Bridge::ROS2Bridge() : AddInDataIO()
{
    // Initialize the parameters related to the itom plug-in
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ROS2Bridge", "name of the plugin");
    m_params.insert(paramVal.getName(), paramVal);
    ito::RetVal retValue;
    paramVal = ito::Param(
        "ServiceServerName",
        ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave,
        "/add_two_ints",
        tr("The name of the service server to connect to").toLatin1().data() );
    m_params.insert(paramVal.getName(), paramVal);

    
    int argc = 1;
    
    const char* argv[] = {"ros2_itom_bridge", nullptr};

    // Call rclcpp::init
    try
    {
        RCLCPP_INFO(rclcpp::get_logger("ROS2Bridge"), "Initializing ROS...");
        rclcpp::init(argc, const_cast<char**>(argv));
    }
    catch(const std::exception &ex)
    {
        std::cerr << "[ROS2Bridge] rclcpp::init() failed: " << ex.what() << std::endl;
        throw;
    }

    // Create a ROS 2 node
    try
    {
        node_ = rclcpp::Node::make_shared("ros2_itom_bridge");
        RCLCPP_INFO(rclcpp::get_logger("ROS2Bridge"), "ROS Node created: ros2_itom_bridge");
    }
    catch(const std::exception &ex)
    {
        std::cerr << "[ROS2Bridge] Failed to create node: " << ex.what() << std::endl;
        throw;
    }

    // 5. Periodically call spin_some through the Qt timer
    rosSpinTimer_ = new QTimer(this);
    connect(rosSpinTimer_, &QTimer::timeout, this, [this]() {
        rclcpp::spin_some(node_);
    });
    rosSpinTimer_->start(10); // Process callbacks every 10 milliseconds

    // 6. Create a service
    RCLCPP_INFO(node_->get_logger(), "Before creating Test Server.");
    try
    {
        test_server_ = node_->create_service<itom_ros2_test::srv::Exchange2DArray>(
            "image_exchange",
            std::bind(&ROS2Bridge::handle_image_request, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(node_->get_logger(), "After creating Test Server.");
    }
    catch(const std::exception &ex)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create service: %s", ex.what());
        throw;
    }

    // 7. Create and set DockWidget (itom GUI related)
    if (hasGuiSupport())
    {
        DockWidgetROS2Bridge *dw = new DockWidgetROS2Bridge(m_params, getID());
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features =
            QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);

        connect(this, SIGNAL(dockWidgetVisibilityChanged(bool)), this, SLOT(dockWidgetVisibilityChanged(bool)));
    }
}


ROS2Bridge::~ROS2Bridge()
{
    // Stop timer
    if (rosSpinTimer_)
    {
        rosSpinTimer_->stop();
        delete rosSpinTimer_;
    }

    // Clean up ROS 2 resources`
    rclcpp::shutdown();
}


//--------------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setCvMatToDataObject(ito::DataObject &dataObj, const cv::Mat *mat)
{
    ito::RetVal retval;
    RCLCPP_INFO(node_->get_logger(), "test1.");
    // 1. Check whether the input mat is empty
    if (!mat)
    {
        return ito::RetVal(ito::retError, 0, "mat must not be NULL");
    }
    RCLCPP_INFO(node_->get_logger(), "test2.");
    // 2. Determine the data type
    ito::tDataType cameraMatrixType;
    cv::Mat mat_;
    RCLCPP_INFO(node_->get_logger(), "test3.");
    if (mat->type() == CV_8UC3)  // BGR -> RGBA
    {
        cameraMatrixType = ito::tRGBA32;

        cv::Mat in[] = { *mat, cv::Mat(mat->rows, mat->cols, CV_8UC1) };
        in[1].setTo(255); // Alpha channels are all 255
        cv::Mat out;
        cv::merge(in, 2, out);
        out.convertTo(mat_, cv::DataType<ito::Rgba32>::type);
            RCLCPP_INFO(node_->get_logger(), "test4.");
    }
    else if (mat->type() == CV_8UC4)  // Convert directly to RGBA32
    {
        cameraMatrixType = ito::tRGBA32;
        mat->convertTo(mat_, cv::DataType<ito::Rgba32>::type);
            RCLCPP_INFO(node_->get_logger(), "test5.");
    }
    else
    {
        cameraMatrixType = ito::guessDataTypeFromCVMat(mat, retval);
        mat_ = *mat;
            RCLCPP_INFO(node_->get_logger(), "test6.");
    }

    // 3. Check whether existing dataObj can be reused
    if (!retval.containsError() &&
        dataObj.calcNumMats() == 1 &&
        dataObj.getDims() == mat_.dims &&
        dataObj.getType() == cameraMatrixType)
    {
        cv::Mat *plane = dataObj.getCvPlaneMat(0);
            RCLCPP_INFO(node_->get_logger(), "test7.");
        if (plane->data == mat_.data && plane->size == mat_.size)
        {    RCLCPP_INFO(node_->get_logger(), "test8.");
            return retval;
        }
    }

    // 4. Assign dataObj
    if (!retval.containsError())
    {
        if (mat_.dims == 0)
        {
            dataObj = ito::DataObject();
                RCLCPP_INFO(node_->get_logger(), "test9.");
        }
        else
        {
            dataObj = ito::DataObject(mat_.dims, mat_.size, cameraMatrixType, &mat_, 1);
                RCLCPP_INFO(node_->get_logger(), "test10.");
        }
    }

//     Retval = Ito::Retok;
        RCLCPP_INFO(node_->get_logger(), "test11.");
    return retval;
}


//Test for 2D Data

void ROS2Bridge::handle_image_request(
        const std::shared_ptr<itom_ros2_test::srv::Exchange2DArray::Request> request,
        std::shared_ptr<itom_ros2_test::srv::Exchange2DArray::Response> response)
    {
        RCLCPP_INFO(node_->get_logger(), "Received an image request.");

        // Convert ros image message to open cv image

        try
        {
            input_image = cv_bridge::toCvCopy(request->request_image, "bgr8")->image;
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_INFO(node_->get_logger(), "Failed to convert ROS image message to OpenCV image: %s", e.what());
            return;
        }

        // Check if the input image is valid
        if (input_image.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "Input image is empty.");
            return;
        }

        // Rotate the image (for example, rotate 90 degrees)
        cv::Mat rotated_image;
        cv::rotate(input_image, rotated_image, cv::ROTATE_90_CLOCKWISE);
        RCLCPP_INFO(node_->get_logger(), "Image rotated successfully.");

        // Convert to ros image message
        auto ros_image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rotated_image).toImageMsg();

        // Fill the response
        response->response_image = *ros_image_msg;
        RCLCPP_INFO(node_->get_logger(), "Response sent back to client.");
    }




void ROS2Bridge::dockWidgetVisibilityChanged(bool visible)
{
    if (getDockWidget())
    {
        DockWidgetROS2Bridge *dw = qobject_cast<DockWidgetROS2Bridge*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(updateTopicsList(QStringList, QStringList)), dw, SLOT(updateTopicsList(QStringList, QStringList)));
            connect(dw, SIGNAL(requestRefreshTopics()), this, SLOT(onRequestRefreshTopics()));
        }
        else
        {
            disconnect(this, SIGNAL(updateTopicsList(QStringList, QStringList)), dw, SLOT(updateTopicsList(QStringList, QStringList)));
            disconnect(dw, SIGNAL(requestRefreshTopics()), this, SLOT(onRequestRefreshTopics()));
        }
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;

    if (!retValue.containsError())
    {
        //checkData(); //check if image must be reallocated

        //emit parametersChanged(m_params);
    }

    retValue == ito::retOk;

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;

    if (!retValue.containsError())
    {
        //checkData(); //check if image must be reallocated

        //emit parametersChanged(m_params);
    }


    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retVal(ito::retOk);

    if (!retVal.containsError())
    {
        //checkData(); //check if image must be reallocated

        //emit parametersChanged(m_params);
    }

    setIdentifier(QString::number(getID()));

    if (waitCond)
    {
        waitCond->returnValue = retVal;
        waitCond->release();
    }

    setInitialized(true);
    return retVal;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);

    if (waitCond)
    {
        waitCond->release();
        waitCond->returnValue = retValue;
    }
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::startDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::acquire(const int trigger, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}



//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::getVal(void *vpdObj, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    ito::RetVal retval;


//     ItomSharedSemaphoreLocker locker(waitCond);
//     retValue == ito::retOk;
//     if (waitCond)
//     {
//         waitCond->returnValue = retValue;
//         waitCond->release();
//     }
        ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);

        if (input_image.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "input_image is empty.");
        }

        // 3. Call setOutputArrayToDataObject to convert
        retValue += setCvMatToDataObject(*dObj, &input_image);

//         if (retValue.containsError())
//         {
//             std::cout << "Error converting cv::Mat to ito::DataObject" << std::endl;
//         }
//         else
//         {
//             std::cout << "Conversion successful!" << std::endl;
//         }

    return retValue;
}


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setVal(const char *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    //const char *buf = data;
    //char endline[3] = {0, 0, 0};
    ito::RetVal retval(ito::retOk);

    //m_serport.getendline(endline);
    //if (m_debugMode)
    //{
    //    emit serialLog(QByteArray(buf,datalength), QByteArray(endline, (int)strlen(endline)), '>');
    //}
    //retval = m_serport.swrite(buf, datalength, m_params["sendDelay"].getVal<int>());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}
//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}


// The rest of the code remains the same
