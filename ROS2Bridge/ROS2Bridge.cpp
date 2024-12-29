// ROS2Bridge.cpp
#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "ROS2Bridge.h"

#include <math.h>
#include <qstring.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qelapsedtimer.h>
#include <qwaitcondition.h>
#include "pluginVersion.h"
#include "gitVersion.h"

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
ROS2BridgeInterface::ROS2BridgeInterface(QObject * /*parent*/)
{
    m_autoLoadPolicy = ito::autoLoadKeywordDefined;
    m_autoSavePolicy = ito::autoSaveAlways;

    m_type = ito::typeActuator;
    setObjectName("ROS2Bridge");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"The ROS2Bridge is a virtual actuator plugin that emulates up to 10 linear axes. \n\
\n\
The real number of simulated axes is given by the initialization parameter 'numAxis'. Use this plugin \
to simulate or develop your measurement system at another computer. Whenever a position command is executed, \
this plugin sleeps until the time needed for the positioning (with respect to the speed of the axis) \
expired.";*/

    m_description = QObject::tr("A virtual motor to test real actuators.");
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

ROS2Bridge::ROS2Bridge() : AddInActuator()
{
    // 初始化参数
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly, "ROS2Bridge", "name of the plugin");
    m_params.insert(paramVal.getName(), paramVal);

    // 创建ROS 2节点
    // rclcpp::init(0, nullptr);
    // node_ = rclcpp::Node::make_shared("ros2_itom_bridge");

    // // 注册服务
    // register_services();

    // 创建并设置 DockWidget
    if (hasGuiSupport())
    {
        DockWidgetROS2Bridge *dw = new DockWidgetROS2Bridge(m_params, getID());
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char*>()), features, areas, dw);

        // 注册 dockWidgetVisibilityChanged 槽函数
        connect(this, SIGNAL(dockWidgetVisibilityChanged(bool)), this, SLOT(dockWidgetVisibilityChanged(bool)));
    }

    // // 初始化并启动定时器
    // rosSpinTimer_ = new QTimer(this);
    // connect(rosSpinTimer_, &QTimer::timeout, this, [this]() {
    //     rclcpp::spin_some(node_);
    // });
    // rosSpinTimer_->start(10); // 每10毫秒调用一次
}

ROS2Bridge::~ROS2Bridge()
{
    // 停止定时器
    if (rosSpinTimer_)
    {
        rosSpinTimer_->stop();
        delete rosSpinTimer_;
    }

    // 清理ROS 2资源`
    rclcpp::shutdown();
}

void ROS2Bridge::register_services()
{
    get_topics_service_ = node_->create_service<rosapi_msgs::srv::Topics>(
        "/rosapi/topics",
        std::bind(&ROS2Bridge::handle_get_topics, this, std::placeholders::_1, std::placeholders::_2));
}

void ROS2Bridge::handle_get_topics(const std::shared_ptr<rosapi_msgs::srv::Topics::Request> /*request*/,
                                   std::shared_ptr<rosapi_msgs::srv::Topics::Response> response)
{
    auto topics_and_types = node_->get_topic_names_and_types();
    for (const auto &topic : topics_and_types)
    {
        response->topics.push_back(topic.first);
        response->types.push_back(topic.second[0]);
    }
}

void ROS2Bridge::onRequestRefreshTopics()
{
    try
    {
        auto topics_and_types = node_->get_topic_names_and_types();
        QStringList topics, types;
        for (const auto &topic : topics_and_types)
        {
            topics.append(QString::fromStdString(topic.first));

            QString typeList;
            for (const auto &type : topic.second)
            {
                if (!typeList.isEmpty())
                {
                    typeList += ", ";
                }
                typeList += QString::fromStdString(type);
            }
            types.append(typeList);
        }
        // 发出信号，更新 UI
        emit updateTopicsList(topics, types);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get topics: %s", e.what());
        // 根据需要处理错误
    }
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
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::init(QVector<ito::ParamBase> * /*paramsMand*/, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    setInitialized(true);
    return retValue;
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
ito::RetVal ROS2Bridge::calib(const int axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::calib(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setOrigin(const int axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal ROS2Bridge::waitForDone(const int timeoutMS, const QVector<int> axis /*if empty -> all axis*/, const int /*flags*/ /*for your use*/)
{
    ito::RetVal retValue;
    retValue == ito::retOk;
    return retValue;
}


// 其余代码保持不变
