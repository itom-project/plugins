/**\file dockWidgetDummyMotor.cpp
* \brief In this file the functions of the non modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the DockWidgetDummyMotor-Class defined in the file "dockWidgetDummyMotor.h"
* 
*\sa dockWidgetDummyMotor, DummyMotor
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "dockWidgetAerotechEnsemble.h"
#include "common/addInInterface.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetAerotechEnsemble::DockWidgetAerotechEnsemble(QMap<QString, ito::Param> params, int uniqueID, ito::AddInActuator *actuator) :
    m_actuator(actuator)
{
    ui.setupUi(this);

    char* temp = params["name"].getVal<char*>(); //borrowed reference
    ui.lblID->setText(QString::number(uniqueID));

    setAxisVisible(0);

    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::CheckAxisNums(QMap<QString, ito::Param> params)
{
//    ui.cb0_Name->setChecked(params["axisEnable"].getVal<bool>());
    on_cb0_Name_clicked();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks all parameters, currently only calling CheckAxisNums
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*
*/
void DockWidgetAerotechEnsemble::valuesChanged(QMap<QString, ito::Param> params)
{
     CheckAxisNums(params);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::init(QMap<QString, ito::Param> params, QStringList axisNames)
{
    int numaxis = params["numAxis"].getVal<int>();
    ui.lblAxis->setText(QString::number(numaxis));

    setAxisVisible(numaxis);

    setMinimumHeight(77 + (29 * numaxis) + 107);

    if (numaxis > 0)
    {
        ui.cb0_Name->setText(axisNames[0]);
    }

    if (numaxis > 1)
    {
        ui.cb1_Name->setText(axisNames[1]);
    }

    if (numaxis > 2)
    {
        ui.cb2_Name->setText(axisNames[2]);
    }

    if (numaxis > 3)
    {
        ui.cb3_Name->setText(axisNames[3]);
    }

    if (numaxis > 4)
    {
        ui.cb4_Name->setText(axisNames[4]);
    }

    if (numaxis > 5)
    {
        ui.cb5_Name->setText(axisNames[5]);
    }

    if (numaxis > 6)
    {
        ui.cb6_Name->setText(axisNames[6]);
    }

    if (numaxis > 7)
    {
        ui.cb7_Name->setText(axisNames[7]);
    }

    if (numaxis > 8)
    {
        ui.cb8_Name->setText(axisNames[8]);
    }

    if (numaxis > 9)
    {
        ui.cb9_Name->setText(axisNames[9]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::setAxisVisible(int numaxis)
{
    ui.cb0_Name->setVisible(numaxis > 0);
    ui.cb1_Name->setVisible(numaxis > 1);
    ui.cb2_Name->setVisible(numaxis > 2);
    ui.cb3_Name->setVisible(numaxis > 3);
    ui.cb4_Name->setVisible(numaxis > 4);
    ui.cb5_Name->setVisible(numaxis > 5);
    ui.cb6_Name->setVisible(numaxis > 6);
    ui.cb7_Name->setVisible(numaxis > 7);
    ui.cb8_Name->setVisible(numaxis > 8);
    ui.cb9_Name->setVisible(numaxis > 9);

    ui.dsb0_Actual->setVisible(numaxis > 0);
    ui.dsb1_Actual->setVisible(numaxis > 1);
    ui.dsb2_Actual->setVisible(numaxis > 2);
    ui.dsb3_Actual->setVisible(numaxis > 3);
    ui.dsb4_Actual->setVisible(numaxis > 4);
    ui.dsb5_Actual->setVisible(numaxis > 5);
    ui.dsb6_Actual->setVisible(numaxis > 6);
    ui.dsb7_Actual->setVisible(numaxis > 7);
    ui.dsb8_Actual->setVisible(numaxis > 8);
    ui.dsb9_Actual->setVisible(numaxis > 9);

    ui.dsb0_Target->setVisible(numaxis > 0);
    ui.dsb1_Target->setVisible(numaxis > 1);
    ui.dsb2_Target->setVisible(numaxis > 2);
    ui.dsb3_Target->setVisible(numaxis > 3);
    ui.dsb4_Target->setVisible(numaxis > 4);
    ui.dsb5_Target->setVisible(numaxis > 5);
    ui.dsb6_Target->setVisible(numaxis > 6);
    ui.dsb7_Target->setVisible(numaxis > 7);
    ui.dsb8_Target->setVisible(numaxis > 8);
    ui.dsb9_Target->setVisible(numaxis > 9);

    ui.pb0_Add->setVisible(numaxis > 0);
    ui.pb1_Add->setVisible(numaxis > 1);
    ui.pb2_Add->setVisible(numaxis > 2);
    ui.pb3_Add->setVisible(numaxis > 3);
    ui.pb4_Add->setVisible(numaxis > 4);
    ui.pb5_Add->setVisible(numaxis > 5);
    ui.pb6_Add->setVisible(numaxis > 6);
    ui.pb7_Add->setVisible(numaxis > 7);
    ui.pb8_Add->setVisible(numaxis > 8);
    ui.pb9_Add->setVisible(numaxis > 9);

    ui.pb0_Sub->setVisible(numaxis > 0);
    ui.pb1_Sub->setVisible(numaxis > 1);
    ui.pb2_Sub->setVisible(numaxis > 2);
    ui.pb3_Sub->setVisible(numaxis > 3);
    ui.pb4_Sub->setVisible(numaxis > 4);
    ui.pb5_Sub->setVisible(numaxis > 5);
    ui.pb6_Sub->setVisible(numaxis > 6);
    ui.pb7_Sub->setVisible(numaxis > 7);
    ui.pb8_Sub->setVisible(numaxis > 8);
    ui.pb9_Sub->setVisible(numaxis > 9);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::targetChanged(QVector<double> targetPositions)
{
    int i = targetPositions.size();
    if (i >= 0)
    {
        ui.dsb0_Target->setValue(targetPositions[0]);
    }
    if (i >= 1)
    {
        ui.dsb1_Target->setValue(targetPositions[1]);
    }
    if (i >= 2)
    {
        ui.dsb2_Target->setValue(targetPositions[2]);
    }
    if (i >= 3)
    {
        ui.dsb3_Target->setValue(targetPositions[3]);
    }
    if (i >= 4)
    {
        ui.dsb4_Target->setValue(targetPositions[4]);
    }
    if (i >= 5)
    {
        ui.dsb5_Target->setValue(targetPositions[5]);
    }
    if (i >= 6)
    {
        ui.dsb6_Target->setValue(targetPositions[6]);
    }
    if (i >= 7)
    {
        ui.dsb7_Target->setValue(targetPositions[7]);
    }
    if (i >= 8)
    {
        ui.dsb8_Target->setValue(targetPositions[8]);
    }
    if (i >= 9)
    {
        ui.dsb9_Target->setValue(targetPositions[9]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition)
{
    ui.dsb0_Target->setEnabled(status[0] & ito::actuatorEnabled);
    ui.dsb1_Target->setEnabled(status[1] & ito::actuatorEnabled);
    ui.dsb2_Target->setEnabled(status[2] & ito::actuatorEnabled);
    ui.dsb3_Target->setEnabled(status[3] & ito::actuatorEnabled);
    ui.dsb4_Target->setEnabled(status[4] & ito::actuatorEnabled);
    ui.dsb5_Target->setEnabled(status[5] & ito::actuatorEnabled);
    ui.dsb6_Target->setEnabled(status[6] & ito::actuatorEnabled);
    ui.dsb7_Target->setEnabled(status[7] & ito::actuatorEnabled);
    ui.dsb8_Target->setEnabled(status[8] & ito::actuatorEnabled);
    ui.dsb9_Target->setEnabled(status[9] & ito::actuatorEnabled);

    if (actPosition.size() > 0)
    {
        ui.dsb0_Actual->setValue(actPosition[0]);
        ui.dsb1_Actual->setValue(actPosition[1]);
        ui.dsb2_Actual->setValue(actPosition[2]);
        ui.dsb2_Actual->setValue(actPosition[2]);
        ui.dsb3_Actual->setValue(actPosition[3]);
        ui.dsb4_Actual->setValue(actPosition[4]);
        ui.dsb5_Actual->setValue(actPosition[5]);
        ui.dsb6_Actual->setValue(actPosition[6]);
        ui.dsb7_Actual->setValue(actPosition[7]);
        ui.dsb8_Actual->setValue(actPosition[8]);
        ui.dsb9_Actual->setValue(actPosition[9]);
    }

    bool running = false;
    QString style;

    for (int i = 0; i < status.size(); i++)
    {
        if (status[i] & ito::actuatorMoving)
        {
            style = "background-color: yellow";
            running = true;
        }
        else if (status[i] & ito::actuatorInterrupted)
        {
            style = "background-color: red";
        }
        else if (status[i] & ito::actuatorTimeout)
        {
            style = "background-color: #FFA3FD";
        }
        else
        {
            style = "background-color: ";
        }

         switch (i)
         {
             case 0:
                 ui.dsb0_Actual->setStyleSheet(style);
                 break;
             case 1:
                ui.dsb1_Actual->setStyleSheet(style);
                break;
             case 2:
                ui.dsb2_Actual->setStyleSheet(style);
                break;
             case 3:
                ui.dsb3_Actual->setStyleSheet(style);
                break;
             case 4:
                ui.dsb4_Actual->setStyleSheet(style);
                break;
             case 5:
                ui.dsb5_Actual->setStyleSheet(style);
                break;
             case 6:
                ui.dsb6_Actual->setStyleSheet(style);
                break;
             case 7:
                ui.dsb7_Actual->setStyleSheet(style);
                break;
             case 8:
                ui.dsb8_Actual->setStyleSheet(style);
                break;
             case 9:
                ui.dsb9_Actual->setStyleSheet(style);
                break;
         }
     }

     enableWidget(!running);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb0_Name_clicked()
{
//    ui.dsb0_Actual->setEnabled(ui.cb0_Name->isChecked());
    ui.dsb0_Target->setEnabled(ui.cb0_Name->isChecked());
    ui.pb0_Add->setEnabled(ui.cb0_Name->isChecked());
    ui.pb0_Sub->setEnabled(ui.cb0_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb1_Name_clicked()
{
//    ui.dsb1_Actual->setEnabled(ui.cb1_Name->isChecked());
    ui.dsb1_Target->setEnabled(ui.cb1_Name->isChecked());
    ui.pb1_Add->setEnabled(ui.cb1_Name->isChecked());
    ui.pb1_Sub->setEnabled(ui.cb1_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb2_Name_clicked()
{
//    ui.dsb2_Actual->setEnabled(ui.cb2_Name->isChecked());
    ui.dsb2_Target->setEnabled(ui.cb2_Name->isChecked());
    ui.pb2_Add->setEnabled(ui.cb2_Name->isChecked());
    ui.pb2_Sub->setEnabled(ui.cb2_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb3_Name_clicked()
{
//    ui.dsb3_Actual->setEnabled(ui.cb3_Name->isChecked());
    ui.dsb3_Target->setEnabled(ui.cb3_Name->isChecked());
    ui.pb3_Add->setEnabled(ui.cb3_Name->isChecked());
    ui.pb3_Sub->setEnabled(ui.cb3_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb4_Name_clicked()
{
//    ui.dsb4_Actual->setEnabled(ui.cb4_Name->isChecked());
    ui.dsb4_Target->setEnabled(ui.cb4_Name->isChecked());
    ui.pb4_Add->setEnabled(ui.cb4_Name->isChecked());
    ui.pb4_Sub->setEnabled(ui.cb4_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb5_Name_clicked()
{
//    ui.dsb5_Actual->setEnabled(ui.cb5_Name->isChecked());
    ui.dsb5_Target->setEnabled(ui.cb5_Name->isChecked());
    ui.pb5_Add->setEnabled(ui.cb5_Name->isChecked());
    ui.pb5_Sub->setEnabled(ui.cb5_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb6_Name_clicked()
{
//    ui.dsb6_Actual->setEnabled(ui.cb6_Name->isChecked());
    ui.dsb6_Target->setEnabled(ui.cb6_Name->isChecked());
    ui.pb6_Add->setEnabled(ui.cb6_Name->isChecked());
    ui.pb6_Sub->setEnabled(ui.cb6_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb7_Name_clicked()
{
//    ui.dsb7_Actual->setEnabled(ui.cb7_Name->isChecked());
    ui.dsb7_Target->setEnabled(ui.cb7_Name->isChecked());
    ui.pb7_Add->setEnabled(ui.cb7_Name->isChecked());
    ui.pb7_Sub->setEnabled(ui.cb7_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb8_Name_clicked()
{
//    ui.dsb8_Actual->setEnabled(ui.cb8_Name->isChecked());
    ui.dsb8_Target->setEnabled(ui.cb8_Name->isChecked());
    ui.pb8_Add->setEnabled(ui.cb8_Name->isChecked());
    ui.pb8_Sub->setEnabled(ui.cb8_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_cb9_Name_clicked()
{
//    ui.dsb9_Actual->setEnabled(ui.cb9_Name->isChecked());
    ui.dsb9_Target->setEnabled(ui.cb9_Name->isChecked());
    ui.pb9_Add->setEnabled(ui.cb9_Name->isChecked());
    ui.pb9_Sub->setEnabled(ui.cb9_Name->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb0_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb0_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb1_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(1, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb1_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(1, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb2_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(2, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb2_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(2, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb3_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(3, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb3_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(3, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb4_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(4, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb4_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(4, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb5_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(5, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb5_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(5, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb6_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(6, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb6_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(6, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb7_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(7, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb7_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(7, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb8_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(8, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb8_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(8, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb9_Add_clicked()
{
    double dpos = ui.dsb_StepSize->value() / 1e3;
    emit MoveRelative(9, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb9_Sub_clicked()
{
    double dpos = ui.dsb_StepSize->value() / -1e3;
    emit MoveRelative(9, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::enableWidget(bool enabled)
{
/*    ui.btn_relPlus1->setEnabled(enabled);
    ui.btn_relPlus2->setEnabled(enabled);
    ui.btn_relPlus3->setEnabled(enabled);
    ui.btn_relMinus1->setEnabled(enabled);
    ui.btn_relMinus2->setEnabled(enabled);
    ui.btn_relMinus3->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_x->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_y->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_z->setEnabled(enabled);*/

    ui.pb_Start->setVisible(enabled);
    ui.pb_Stop->setVisible(!enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb_Start_clicked()
{
    QVector<int> axis;
    QVector<double> dpos;

    if (ui.cb0_Name->isVisible() && ui.cb0_Name->isChecked())
    {
        axis << 0;
        dpos << ui.dsb0_Target->value();
    }
    if (ui.cb1_Name->isVisible() && ui.cb1_Name->isChecked())
    {
        axis << 1;
        dpos << ui.dsb1_Target->value();
    }
    if (ui.cb2_Name->isVisible() && ui.cb2_Name->isChecked())
    {
        axis << 2;
        dpos << ui.dsb2_Target->value();
    }
    if (ui.cb3_Name->isVisible() && ui.cb3_Name->isChecked())
    {
        axis << 3;
        dpos << ui.dsb3_Target->value();
    }
    if (ui.cb4_Name->isVisible() && ui.cb4_Name->isChecked())
    {
        axis << 4;
        dpos << ui.dsb4_Target->value();
    }
    if (ui.cb5_Name->isVisible() && ui.cb5_Name->isChecked())
    {
        axis << 5;
        dpos << ui.dsb5_Target->value();
    }
    if (ui.cb6_Name->isVisible() && ui.cb6_Name->isChecked())
    {
        axis << 6;
        dpos << ui.dsb6_Target->value();
    }
    if (ui.cb7_Name->isVisible() && ui.cb7_Name->isChecked())
    {
        axis << 7;
        dpos << ui.dsb7_Target->value();
    }
    if (ui.cb8_Name->isVisible() && ui.cb8_Name->isChecked())
    {
        axis << 8;
        dpos << ui.dsb8_Target->value();
    }
    if (ui.cb9_Name->isVisible() && ui.cb9_Name->isChecked())
    {
        axis << 9;
        dpos << ui.dsb9_Target->value();
    }

    emit MoveAbsolute(axis, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb_Stop_clicked()
{
    if (m_actuator) m_actuator->setInterrupt();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_pb_Refresh_clicked()
{
    emit MotorTriggerStatusRequest(true, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*void DockWidgetAerotechEnsemble::basicInformationChanged(QString name, QString id, QString axis, QVector<bool> available)
{
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

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_btnStartAbsolute_clicked()
{
    emit setAbsTargetDegree( ui.doubleSpinBox_tarpos_x->value(), ui.doubleSpinBox_tarpos_y->value(), ui.doubleSpinBox_tarpos_z->value() );
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_btn_relPlus1_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(0, stepDeg);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_btn_relMinus1_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(0, -stepDeg);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_btn_relPlus2_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(1, stepDeg);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAerotechEnsemble::on_btn_relMinus2_clicked()
{
    double stepDeg = ui.spinStepSize->value();
    emit setRelTargetDegree(1, -stepDeg);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
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
}*/