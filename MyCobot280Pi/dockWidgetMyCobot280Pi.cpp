#include "dockWidgetMyCobot280Pi.h"
#include <QVector>
#include "MyCobot280Pi.h"


//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetMyCobot280Pi::DockWidgetMyCobot280Pi(int uniqueID, ito::AddInActuator *myPlugin) :
    ito::AbstractAddInDockWidget(myPlugin),
    m_pActuator(myPlugin),
    m_isVisible(false),
    m_numaxis(6)  // Assuming the robot has 6 axes
{
    ui.setupUi(this);

    // Set the unique ID in the UI
    // ui.lblID->setText(QString::number(uniqueID));

    // Initialize sliders for each joint
    m_jointSliders.append(ui.sliderWidget);
    m_jointSliders.append(ui.sliderWidget_2);
    m_jointSliders.append(ui.sliderWidget_3);
    m_jointSliders.append(ui.sliderWidget_4);
    m_jointSliders.append(ui.sliderWidget_5);
    m_jointSliders.append(ui.sliderWidget_6);

    // Connect the "Send Command" button to its click handler
    connect(ui.pushButton_3, &QPushButton::clicked, this, &DockWidgetMyCobot280Pi::onStartCommandClicked);
    connect(ui.pushButton, &QPushButton::clicked, this, &DockWidgetMyCobot280Pi::onStartRobotClicked);
    connect(ui.pushButton_2, &QPushButton::clicked, this, &DockWidgetMyCobot280Pi::onGetJointStatesClicked);
    connect(ui.pushButton_4, &QPushButton::clicked, this, &DockWidgetMyCobot280Pi::onConnectSocketServer);
    connect(ui.pushButton_5, &QPushButton::clicked, this, &DockWidgetMyCobot280Pi::onStartArmControlClicked);
    connect(ui.pushButton_6, &QPushButton::clicked, this, &DockWidgetMyCobot280Pi::onConnectArmControlClicked);

    ui.textBrowser->append("Initializing joint states...");
    ui.textBrowser_2->append("Initializing... Please connect to the server.");
    // Initially, enable all widgets
    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This slot is triggered when the "Send Command" button is clicked */
void DockWidgetMyCobot280Pi::onStartCommandClicked()
{
    sendJointData();  // Collect and send the joint data to the server
}

void DockWidgetMyCobot280Pi::onStartArmControlClicked()
{
    sendStartArmcontrolCommand();  // Collect and send the joint data to the server
}

void DockWidgetMyCobot280Pi::onConnectArmControlClicked()
{
    sendConnectArmcontrolCommand();  // Collect and send the joint data to the server
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This slot is triggered when the "Start Robot" button is clicked */
void DockWidgetMyCobot280Pi::onStartRobotClicked()
{
    sendStartRobot();  // Send the start command to the server
}

void DockWidgetMyCobot280Pi::onGetJointStatesClicked()
{
    getCurrentJointStates();  // Send the start command to the server
}

void DockWidgetMyCobot280Pi::onConnectSocketServer()
{
    connectSocketServer();  
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This function collects the data from all joint sliders and sends it via socket */
void DockWidgetMyCobot280Pi::sendJointData()
{
    QString dataToSend;

    // Collect joint data based on the checkboxes and slider values
    if (ui.checkBox->isChecked()) {
        double value = ui.sliderWidget->value() / 100.0;  // 将整数值转换为小数
        dataToSend += QString("joint2_to_joint1:%1 ").arg(value, 0, 'f', 2);  // 保留两位小数
    } else {
        dataToSend += "joint2_to_joint1: ";
    }

    if (ui.checkBox_2->isChecked()) {
        double value = ui.sliderWidget_2->value() / 100.0;  // 将整数值转换为小数
        dataToSend += QString("joint3_to_joint2:%1 ").arg(value, 0, 'f', 2);
    } else {
        dataToSend += "joint3_to_joint2: ";
    }

    if (ui.checkBox_3->isChecked()) {
        double value = ui.sliderWidget_3->value() / 100.0;  // 将整数值转换为小数
        dataToSend += QString("joint4_to_joint3:%1 ").arg(value, 0, 'f', 2);
    } else {
        dataToSend += "joint4_to_joint3: ";
    }

    if (ui.checkBox_4->isChecked()) {
        double value = ui.sliderWidget_4->value() / 100.0;  // 将整数值转换为小数
        dataToSend += QString("joint5_to_joint4:%1 ").arg(value, 0, 'f', 2);
    } else {
        dataToSend += "joint5_to_joint4: ";
    }

    if (ui.checkBox_5->isChecked()) {
        double value = ui.sliderWidget_5->value() / 100.0;  // 将整数值转换为小数
        dataToSend += QString("joint6_to_joint5:%1 ").arg(value, 0, 'f', 2);
    } else {
        dataToSend += "joint6_to_joint5: ";
    }

    if (ui.checkBox_6->isChecked()) {
        double value = ui.sliderWidget_6->value() / 100.0;  // 将整数值转换为小数
        dataToSend += QString("joint6output_to_joint6:%1 ").arg(value, 0, 'f', 2);
    } else {
        dataToSend += "joint6output_to_joint6: ";
    }

    // Send the collected data if any slider value is updated
    MyCobot280Pi *myCobot = dynamic_cast<MyCobot280Pi *>(m_pActuator);
    if (myCobot) {
        myCobot->sendSocketData(dataToSend);
    }
}


//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This function sends the "start_slider_control" command to the robot server */
void DockWidgetMyCobot280Pi::sendStartRobot()
{
    QString startCommand = "start_slider_control";
    MyCobot280Pi *myCobot = dynamic_cast<MyCobot280Pi *>(m_pActuator);
    if (myCobot) {
        myCobot->sendSocketData(startCommand);
    }
}

void DockWidgetMyCobot280Pi::sendStartArmcontrolCommand()
{
    QString startCommand = "start_arm_control_service";
    MyCobot280Pi *myCobot = dynamic_cast<MyCobot280Pi *>(m_pActuator);
    if (myCobot) {
        myCobot->sendSocketData(startCommand);
    }
}


void DockWidgetMyCobot280Pi::sendConnectArmcontrolCommand()
{
    QString startCommand = "connect_services";
    MyCobot280Pi *myCobot = dynamic_cast<MyCobot280Pi *>(m_pActuator);
    if (myCobot) {
        myCobot->sendSocketData(startCommand);
    }
}


void DockWidgetMyCobot280Pi::getCurrentJointStates()
{
    QString getJointStates = "get_joint_states";
    MyCobot280Pi *myCobot = dynamic_cast<MyCobot280Pi *>(m_pActuator);
    if (myCobot) {
        // 调用 sendSocketData 函数，并获取返回结果
        ito::RetVal result = myCobot->sendSocketData(getJointStates);

        // 检查返回值是否包含错误
        if (result.containsError()) {
            // 如果有错误，显示错误信息到 textBrowser
            ui.textBrowser->append("Error getting joint states: " + QString::fromStdString(result.errorMessage()));
        } else {
            // 如果成功，显示服务器返回的关节状态到 textBrowser
            ui.textBrowser->append("Joint states received:\n" + QString::fromStdString(result.errorMessage()));
        }
    } else {
        ui.textBrowser->append("Error: Unable to get joint states. MyCobot280Pi instance not found.");
    }
}



void DockWidgetMyCobot280Pi::connectSocketServer()
{
    // 动态转换 m_pActuator 为 MyCobot280Pi
    MyCobot280Pi *myCobot = dynamic_cast<MyCobot280Pi *>(m_pActuator);

    if (myCobot) {
        // 调用 connectToSocket() 方法以连接服务器
        ito::RetVal result = myCobot->connectToSocket();

        // 检查连接结果
        if (result.containsError()) {
            // 如果连接失败，显示错误消息到 textBrowser_2
            ui.textBrowser_2->append("Failed to connect to server: " + QString::fromStdString(result.errorMessage()));
        } else {
            // 如果连接成功，显示成功消息到 textBrowser_2
            ui.textBrowser_2->append("Successfully connected to server");
        }
    } else {
        // 如果动态转换失败，显示错误消息到 textBrowser_2
        ui.textBrowser_2->append("Error: Actuator is not a MyCobot280Pi instance");
    }
}



//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMyCobot280Pi::parametersChanged(QMap<QString, ito::Param> params)
{
    // bool newNumaxis = m_numaxis != params["numaxis"].getVal<int>();
    // if (newNumaxis)
    // {
    //     m_numaxis = params["numaxis"].getVal<int>();
    //     ui.lblAxis->setText(QString::number(m_numaxis));

    //     // Adjust sliders based on the number of axes
    //     for (int i = 0; i < m_jointSliders.size(); i++)
    //     {
    //         m_jointSliders[i]->setVisible(m_numaxis > i);  // Show/hide sliders based on the number of axes
    //     }
    // }

    // enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMyCobot280Pi::dockWidgetVisibilityChanged(bool visible)
{
    m_isVisible = visible;
    if (visible)
    {
        // Perform actions when the widget becomes visible, such as updating the UI
        // based on the current status of the robot
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMyCobot280Pi::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
    // Update the joint sliders based on the current positions
    for (int i = 0; i < m_jointSliders.size(); i++)
    {
        if (i < positions.size())
        {
            m_jointSliders[i]->setValue(positions[i]);  // Update the slider value to match the joint position
        }
    }

    // Optionally, disable the GUI if the robot is moving
    bool isMoving = false;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            isMoving = true;
            break;
        }
    }
    // ui.groupProperties->setEnabled(!isMoving);  // Disable controls if the robot is moving
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMyCobot280Pi::enableWidget(bool enabled)
{
    for (int i = 0; i < m_jointSliders.size(); i++)
    {
        m_jointSliders[i]->setEnabled(enabled && m_numaxis > i);  // Enable/disable sliders based on the number of axes
    }

    // Optionally, enable or disable other UI elements as needed
}
