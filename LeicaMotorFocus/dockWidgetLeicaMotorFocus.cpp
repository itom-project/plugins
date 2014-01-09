#include "dockWidgetLeicaMotorFocus.h"

#include "common/addInInterface.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetLeicaMotorFocus::DockWidgetLeicaMotorFocus(QMap<QString, ito::Param> params, int uniqueID,ito::AddInActuator *actuator) : m_actuator(actuator)
{
    ui.setupUi(this); 

    char* temp = params["name"].getVal<char*>(); //borrowed reference
    ui.lblName->setText(temp);
    ui.lblID->setText(QString::number(uniqueID));

    valuesChanged(params);
    setMotorStatus(false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::valuesChanged(QMap<QString, ito::Param> /*params*/)
{

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::setMotorStatus(bool busy)
{
    ui.btnDown->setEnabled(!busy);
    ui.btnUp->setEnabled(!busy);
    ui.btnStart->setVisible(!busy);
    ui.btnStop->setVisible(busy);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::on_btnUp_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::on_btnDown_clicked()
{
    double dpos = -1 * ui.spinStepSize->value() / 1e3;
    emit MoveRelative(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::on_btnRefresh_clicked()
{
    emit MotorTriggerStatusRequest(true, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::on_btnStart_clicked()
{
    double dpos = ui.doubleSpinBox_tarpos->value();
    emit MoveAbsolute(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::on_btnStop_clicked()
{
    if (m_actuator)
    {
        m_actuator->setInterrupt();
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
{
    if (actPosition.size() > 0)
    {
        ui.doubleSpinBox_actpos->setValue(actPosition[0]);
    }

    bool running = false;
    QString style;

    if (status.size() > 0)
    {
        if (status[0] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
            running = true;
        }
        else if (status[0] & ito::actuatorInterrupted)
        {
            style = "background-color: red";
        }
        else if (status[0] & ito::actuatorTimeout)
        {
            style = "background-color: #FFA3FD";
        }
        else
        {
            style = "background-color: ";
        }

        ui.doubleSpinBox_actpos->setStyleSheet(style);
    }

    setMotorStatus(running);

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetLeicaMotorFocus::targetChanged(QVector<double> targetPositions)
{
    if(targetPositions.size()>0)
    {
        ui.doubleSpinBox_tarpos->setValue(targetPositions[0]);
    }
}
