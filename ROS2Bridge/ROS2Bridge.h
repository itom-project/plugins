// ROS2Bridge.h
#ifndef ROS2BRIDGE_H
#define ROS2BRIDGE_H

#include "common/addInInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <rosapi_msgs/srv/topics.hpp>
#include <QTimer>
#include "dockWidgetROS2Bridge.h"

#include <qsharedpointer.h>
#include <qmetatype.h>
#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

class ROS2BridgeInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddIn`nterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

   
    public:
        ROS2BridgeInterface(QObject *parent = 0);
        ~ROS2BridgeInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);    //!< Creates a new ROS4Bridge and gives it a unique identification
        bool hasDockWidget() { return true; }

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};


class ROS2Bridge : public ito::AddInActuator
{
    Q_OBJECT


public:
    friend class ROS2BridgeInterface;
    const ito::RetVal showConfDialog(void);    //!< This calls the modal Configuration Dialog
    int hasConfDialog(void) { return 1; } 

protected:
    ~ROS2Bridge();
    ROS2Bridge();
     ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);


public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
    void onRequestRefreshTopics(); // 添加此槽函数

signals:
    void updateTopicsList(const QStringList &topics, const QStringList &types); // 添加此信号

private:
    void register_services();

    // 服务回调函数
    void handle_get_topics(const std::shared_ptr<rosapi_msgs::srv::Topics::Request> request,
                           std::shared_ptr<rosapi_msgs::srv::Topics::Response> response);

    // ROS 2 节点和服务成员变量
    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<rosapi_msgs::srv::Topics>::SharedPtr get_topics_service_;

    // 插件参数
    QMap<QString, ito::Param> m_params;

    // 定时器用于处理ROS 2事件循环
    QTimer *rosSpinTimer_;
};

#endif // ROS2BRIDGE_H
