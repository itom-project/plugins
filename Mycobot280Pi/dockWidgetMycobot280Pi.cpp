#include "dockWidgetMycobotControl.h"
#include <QVector>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetMycobotControl::DockWidgetMycobotControl(int uniqueID, ito::AddInActuator *myPlugin) :
    ito::AbstractAddInDockWidget(myPlugin),
    m_pActuator(myPlugin),
    m_isVisible(false),
    m_numaxis(6)  // Assuming the robot has 6 axes
{
    ui.setupUi(this);

    // Set the unique ID in the UI
    ui.lblID->setText(QString::number(uniqueID));

    // Initialize sliders for each joint
    m_jointSliders.append(ui.sliderJoint1);
    m_jointSliders.append(ui.sliderJoint2);
    m_jointSliders.append(ui.sliderJoint3);
    m_jointSliders.append(ui.sliderJoint4);
    m_jointSliders.append(ui.sliderJoint5);
    m_jointSliders.append(ui.sliderJoint6);

    // Connect the "Send Command" button to its click handler
    connect(ui.FontButton, SIGNAL(clicked()), this, SLOT(onStartCommandClicked()));

    // Connect the "Start Robot" button to its click handler
    connect(ui.FontButton_2, SIGNAL(clicked()), this, SLOT(onStartRobotClicked()));

    // Initially, enable all widgets
    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This slot is triggered when the "Send Command" button is clicked */
void DockWidgetMycobotControl::onStartCommandClicked()
{
    sendJointData();  // Collect and send the joint data to the server
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This slot is triggered when the "Start Robot" button is clicked */
void DockWidgetMycobotControl::onStartRobotClicked()
{
    sendStartCommand();  // Send the start command to the server
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This function collects the data from all joint sliders and sends it via socket */
void DockWidgetMycobotControl::sendJointData()
{
    QString dataToSend;

    // Collect joint data based on the checkboxes and slider values
    if (ui.checkBox->isChecked()) {
        double value = ui.sliderJoint1->value();
        dataToSend += QString("joint2_to_joint1:%1 ").arg(value);
    } else {
        dataToSend += "joint2_to_joint1: ";
    }

    if (ui.checkBox_2->isChecked()) {
        double value = ui.sliderJoint2->value();
        dataToSend += QString("joint3_to_joint2:%1 ").arg(value);
    } else {
        dataToSend += "joint3_to_joint2: ";
    }

    if (ui.checkBox_3->isChecked()) {
        double value = ui.sliderJoint3->value();
        dataToSend += QString("joint4_to_joint3:%1 ").arg(value);
    } else {
        dataToSend += "joint4_to_joint3: ";
    }

    if (ui.checkBox_4->isChecked()) {
        double value = ui.sliderJoint4->value();
        dataToSend += QString("joint5_to_joint4:%1 ").arg(value);
    } else {
        dataToSend += "joint5_to_joint4: ";
    }

    if (ui.checkBox_5->isChecked()) {
        double value = ui.sliderJoint5->value();
        dataToSend += QString("joint6_to_joint5:%1 ").arg(value);
    } else {
        dataToSend += "joint6_to_joint5: ";
    }

    if (ui.checkBox_6->isChecked()) {
        double value = ui.sliderJoint6->value();
        dataToSend += QString("joint6output_to_joint6:%1 ").arg(value);
    } else {
        dataToSend += "joint6output_to_joint6: ";
    }

    // Send the collected data if any slider value is updated
    if (!dataToSend.isEmpty()) {
        m_pActuator->sendSocketData(dataToSend.trimmed());  // Send the data to the socket server
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This function sends the "start_slider_control" command to the robot server */
void DockWidgetMycobotControl::sendStartCommand()
{
    QString startCommand = "start_slider_control";
    m_pActuator->sendSocketData(startCommand);  // Send the start command to the socket server
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMycobotControl::parametersChanged(QMap<QString, ito::Param> params)
{
    bool newNumaxis = m_numaxis != params["numaxis"].getVal<int>();
    if (newNumaxis)
    {
        m_numaxis = params["numaxis"].getVal<int>();
        ui.lblAxis->setText(QString::number(m_numaxis));

        // Adjust sliders based on the number of axes
        for (int i = 0; i < m_jointSliders.size(); i++)
        {
            m_jointSliders[i]->setVisible(m_numaxis > i);  // Show/hide sliders based on the number of axes
        }
    }

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMycobotControl::dockWidgetVisibilityChanged(bool visible)
{
    m_isVisible = visible;
    if (visible)
    {
        // Perform actions when the widget becomes visible, such as updating the UI
        // based on the current status of the robot
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMycobotControl::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
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
    ui.groupProperties->setEnabled(!isMoving);  // Disable controls if the robot is moving
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMycobotControl::enableWidget(bool enabled)
{
    for (int i = 0; i < m_jointSliders.size(); i++)
    {
        m_jointSliders[i]->setEnabled(enabled && m_numaxis > i);  // Enable/disable sliders based on the number of axes
    }

    // Optionally, enable or disable other UI elements as needed
}
