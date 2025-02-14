// Ro s2 bridge.h
#ifndef ROS2BRIDGE_H
#define ROS2BRIDGE_H

#include "DataObject/dataobj.h"
#include "common/addInInterface.h"
#include "common/typeDefs.h"
#include "common/retVal.h"
#include "common/param.h"

#include <rclcpp/rclcpp.hpp>
#include <rosapi_msgs/srv/topics.hpp>
#include <QTimer>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <itom_ros2_test/srv/exchange2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <qsharedpointer.h>
#include <qbytearray.h>
#include <qmetatype.h>
#ifdef WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


class ROS2Bridge : public ito::AddInDataIO
{
    Q_OBJECT

protected:
    virtual ~ROS2Bridge();
    ROS2Bridge();


public:
    friend class ROS2BridgeInterface;
    const ito::RetVal showConfDialog(void);    //!< This calls the modal Configuration Dialog
    int hasConfDialog(void) { return 1; }


public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);
        void register_services();
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitConde = NULL);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond);
        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);



private slots:
    void dockWidgetVisibilityChanged(bool visible);

signals:
    void updateTopicsList(const QStringList &topics, const QStringList &types); // Add this signal

private:
// DataObject Conversion relevant Function

    ito::RetVal setCvMatToDataObject(ito::DataObject &dataObj, const cv::Mat *mat);
    void handle_image_request(
        const std::shared_ptr<itom_ros2_test::srv::Exchange2DArray::Request> request,
        std::shared_ptr<itom_ros2_test::srv::Exchange2DArray::Response> response);
    void send_service_request(int64_t a, int64_t b);
    // Service callback function
    void handle_get_topics(const std::shared_ptr<rosapi_msgs::srv::Topics::Request> request,
                           std::shared_ptr<rosapi_msgs::srv::Topics::Response> response);
    // ROS 2 Node and Service Member Variables
    rclcpp::Node::SharedPtr node_;

    // Test for 2D Data
    rclcpp::Service<itom_ros2_test::srv::Exchange2DArray>::SharedPtr test_server_;
    // Plugin parameters
    QMap<QString, ito::Param> m_params;
    cv::Mat input_image;
    // Timer is used to handle ROS 2 event loops
    QTimer *rosSpinTimer_;
};


class ROS2BridgeInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddIn`nterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:


    public:
        ROS2BridgeInterface();
        ~ROS2BridgeInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

};

#endif // Ro s2 bridge h
