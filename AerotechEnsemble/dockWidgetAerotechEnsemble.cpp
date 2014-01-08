/**\file dockWidgetDummyMotor.cpp
* \brief In this file the functions of the non modal dialog for the DummyMotor are specified
*
*	This file defines the functions of the DockWidgetDummyMotor-Class defined in the file "dockWidgetDummyMotor.h"
* 
*\sa dockWidgetDummyMotor, DummyMotor
*\author Wolfram Lyda
*\date	Oct2011
*/

#include "dockWidgetAerotechEnsemble.h"
#include "common/addInInterface.h"

/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params		m_params-Variable containg the parameters of the DummyMotor
*\param[in] uniqueID	The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetAerotechEnsemble::DockWidgetAerotechEnsemble(ito::AddInActuator *actuator) : m_actuator(actuator)
{
    ui.setupUi(this); 
    enableWidget(true);
}

void DockWidgetAerotechEnsemble::basicInformationChanged(QString name, QString id, QString axis, QVector<bool> available)
{
//    ui.lblName->setText(name);
    ui.lblID->setText(id);
    ui.lblAxis->setText(axis);

    ui.label_xachse->setVisible(available[0]);
    ui.label_yachse->setVisible(available[1]);
    ui.label_zachse->setVisible(available[2]);

    ui.doubleSpinBox_actpos_x->setVisible(available[0]);
    ui.doubleSpinBox_actpos_y->setVisible(available[1]);
    ui.doubleSpinBox_actpos_z->setVisible(available[2]);

    ui.doubleSpinBox_tarpos_x->setVisible(available[0]);
    ui.doubleSpinBox_tarpos_y->setVisible(available[1]);
    ui.doubleSpinBox_tarpos_z->setVisible(available[2]);

    ui.btn_relPlus1->setVisible(available[0]);
    ui.btn_relMinus1->setVisible(available[0]);
    ui.btn_relPlus2->setVisible(available[1]);
    ui.btn_relMinus2->setVisible(available[1]);
    ui.btn_relPlus3->setVisible(available[2]);
    ui.btn_relMinus3->setVisible(available[2]);
}

void DockWidgetAerotechEnsemble::on_btnStartAbsolute_clicked()
{
    emit setAbsTargetDegree( ui.doubleSpinBox_tarpos_x->value(), ui.doubleSpinBox_tarpos_y->value(), ui.doubleSpinBox_tarpos_z->value() );
}

void DockWidgetAerotechEnsemble::on_btn_relPlus1_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(0, stepDeg);
}

void DockWidgetAerotechEnsemble::on_btn_relMinus1_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(0, -stepDeg);
}

void DockWidgetAerotechEnsemble::on_btn_relPlus2_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(1, stepDeg);
}

void DockWidgetAerotechEnsemble::on_btn_relMinus2_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(1, -stepDeg);
}

void DockWidgetAerotechEnsemble::on_btn_relPlus3_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(2, stepDeg);
}

void DockWidgetAerotechEnsemble::on_btn_relMinus3_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(2, -stepDeg);
}

void DockWidgetAerotechEnsemble::on_btnStop_clicked()
{
    if(m_actuator) m_actuator->setInterrupt();
}

void DockWidgetAerotechEnsemble::targetChanged(QVector<double> targetPositions)
{
    int i=targetPositions.size();
    if(i>=0) ui.doubleSpinBox_tarpos_x->setValue(targetPositions[0]);
    if(i>=1) ui.doubleSpinBox_tarpos_y->setValue(targetPositions[1]);
    if(i>=2) ui.doubleSpinBox_tarpos_z->setValue(targetPositions[2]);
}

void DockWidgetAerotechEnsemble::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition)
{

    ui.doubleSpinBox_tarpos_x->setEnabled(status[0] & ito::actuatorEnabled);
    ui.doubleSpinBox_tarpos_y->setEnabled(status[1] & ito::actuatorEnabled);
    ui.doubleSpinBox_tarpos_z->setEnabled(status[2] & ito::actuatorEnabled);

    if(actPosition.size() > 0)
    {
        ui.doubleSpinBox_actpos_x->setValue(actPosition[0]);
        ui.doubleSpinBox_actpos_y->setValue(actPosition[1]);
        ui.doubleSpinBox_actpos_z->setValue(actPosition[2]);
    }

    bool running = false;
    QString style;

    for(int i=0;i<status.size();i++)
    {
        if(status[i] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
            running = true;
        }
        else if(status[i] & ito::actuatorInterrupted)
        {
            style = "background-color: red";
        }
        else if(status[i] & ito::actuatorTimeout)
        {
            style = "background-color: #FFA3FD";
        }
        else
        {
            style = "background-color: ";
        }

         switch(i)
         {
         case 0:
             ui.doubleSpinBox_actpos_x->setStyleSheet(style);
             break;
         case 1:
            ui.doubleSpinBox_actpos_y->setStyleSheet(style);
            break;
         case 2:
            ui.doubleSpinBox_actpos_z->setStyleSheet(style);
            break;
         }
     }

     enableWidget(!running);
}

void DockWidgetAerotechEnsemble::enableWidget(bool enabled)
{
    ui.btn_relPlus1->setEnabled(enabled);
    ui.btn_relPlus2->setEnabled(enabled);
    ui.btn_relPlus3->setEnabled(enabled);
    ui.btn_relMinus1->setEnabled(enabled);
    ui.btn_relMinus2->setEnabled(enabled);
    ui.btn_relMinus3->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_x->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_y->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_z->setEnabled(enabled);

    ui.btnStartAbsolute->setVisible(enabled);
    ui.btnStop->setVisible(!enabled);
}