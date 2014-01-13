/**\file dockWidgetDummyMotor.cpp
* \brief In this file the functions of the non modal dialog for the DummyMotor are specified
*
*    This file defines the functions of the DockWidgetDummyMotor-Class defined in the file "dockWidgetDummyMotor.h"
* 
*\sa dockWidgetDummyMotor, DummyMotor
*\author Wolfram Lyda
*\date    Oct2011
*/

#include "dockWidgetDummyMotor.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail The constructor by the constructor of the DummyMotor during initialisation of the DummyMotor-Instance.
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*\param[in] uniqueID    The unique Id of the DummyMotor-Instance connected to this dialog
*
*\sa DummyMotor
*/
DockWidgetDummyMotor::DockWidgetDummyMotor(QMap<QString, ito::Param> params, int uniqueID, ito::AddInActuator * myPlugin) :
    m_pMyPlugin(myPlugin),
    m_isVisible(false),
    m_numaxis(-1)
{
//    m_numaxis = params["numaxis"].getVal<int>();     
    ui.setupUi(this);

    char* temp = params["name"].getVal<char*>(); //borrowed reference
//    ui.lblName->setText(temp);
    ui.lblID->setText(QString::number(uniqueID));

    CheckAxisNums(params);
    enableWidget(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks the axis-numbers and enables the corresponding GUI-elements
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*
*/
void  DockWidgetDummyMotor::CheckAxisNums(QMap<QString, ito::Param> params)
{
    bool newNumaxis = m_numaxis != params["numaxis"].getVal<int>();
    if (newNumaxis)
    {
        m_numaxis = params["numaxis"].getVal<int>();
        ui.lblAxis->setText(QString::number(m_numaxis));

        visibleWidget();
    }

    ui.checkBox_enablex->setEnabled(m_numaxis > 0);
    ui.checkBox_enablex->setChecked(m_numaxis > 0);
    on_checkBox_enablex_clicked();

    ui.checkBox_enabley->setEnabled(m_numaxis > 1);
    ui.checkBox_enabley->setChecked(m_numaxis > 1);
    on_checkBox_enabley_clicked();

    ui.checkBox_enablez->setEnabled(m_numaxis > 2);
    ui.checkBox_enablez->setChecked(m_numaxis > 2);
    on_checkBox_enablez_clicked();

    ui.checkBox_enablea->setEnabled(m_numaxis > 3);
    ui.checkBox_enablea->setChecked(m_numaxis > 3);
    on_checkBox_enablea_clicked();

    ui.checkBox_enableb->setEnabled(m_numaxis > 4);
    ui.checkBox_enableb->setChecked(m_numaxis > 4);
    on_checkBox_enableb_clicked();

    ui.checkBox_enablec->setEnabled(m_numaxis > 5);
    ui.checkBox_enablec->setChecked(m_numaxis > 5);
    on_checkBox_enablec_clicked();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/** @detail This Slot checks all parameters, currently only calling CheckAxisNums
*
*\param[in] params        m_params-Variable containg the parameters of the DummyMotor
*
*/
void DockWidgetDummyMotor::valuesChanged(QMap<QString, ito::Param> params)
{
     CheckAxisNums(params);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::targetChanged(QVector<double> targetPos)
{
    for (int i = 0; i < targetPos.size(); i++)
    {
        switch(i)
        {
        case 0:
            ui.doubleSpinBox_tarpos_x->setValue(targetPos[i]);
            break;
        case 1:
            ui.doubleSpinBox_tarpos_y->setValue(targetPos[i]);
            break;
        case 2:
            ui.doubleSpinBox_tarpos_z->setValue(targetPos[i]);
            break;
        case 3:
            ui.doubleSpinBox_tarpos_a->setValue(targetPos[i]);
            break;
        case 4:
            ui.doubleSpinBox_tarpos_b->setValue(targetPos[i]);
            break;
        case 5:
            ui.doubleSpinBox_tarpos_c->setValue(targetPos[i]);
            break;
        }
    }
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::actuatorStatusChanged(QVector<int> status, QVector<double> positions)
{
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
            /*else if (status[i] & ito::actuatorTimeout) //timeout is bad for dummyMotor, since the waitForDone-method always drops into a timeout
            {
                style = "background-color: green";
            }*/
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
            case 3:
                ui.doubleSpinBox_actpos_a->setStyleSheet(style);
                break;
            case 4:
                ui.doubleSpinBox_actpos_b->setStyleSheet(style);
                break;
            case 5:
                ui.doubleSpinBox_actpos_c->setStyleSheet(style);
                break;
            }
        }

        enableWidget(!running);

        if (positions.size() < 6)
        {
            positions.resize(6);
        }

    ui.doubleSpinBox_actpos_x->setValue(positions[0]);
    ui.doubleSpinBox_actpos_y->setValue(positions[1]);
    ui.doubleSpinBox_actpos_z->setValue(positions[2]);
    ui.doubleSpinBox_actpos_a->setValue(positions[3]);
    ui.doubleSpinBox_actpos_b->setValue(positions[4]);
    ui.doubleSpinBox_actpos_c->setValue(positions[5]);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_xp_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_xm_clicked()
{
    double dpos = ui.spinStepSize->value() / -1e3;
    emit MoveRelative(0, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_yp_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(1, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_ym_clicked()
{
    double dpos = ui.spinStepSize->value() / -1e3;
    emit MoveRelative(1, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_up_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(2, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_down_clicked()
{
    double dpos = ui.spinStepSize->value() / -1e3;
    emit MoveRelative(2, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_ap_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(3, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_am_clicked()
{
    double dpos = ui.spinStepSize->value() / -1e3;
    emit MoveRelative(3, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_bp_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(4, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_bm_clicked()
{
    double dpos = ui.spinStepSize->value() / -1e3;
    emit MoveRelative(4, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_cp_clicked()
{
    double dpos = ui.spinStepSize->value() / 1e3;
    emit MoveRelative(5, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_cm_clicked()
{
    double dpos = ui.spinStepSize->value() / -1e3;
    emit MoveRelative(5, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_refresh_clicked()
{
    emit MotorTriggerStatusRequest(true, false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_stop_clicked()
{
    m_pMyPlugin->setInterrupt();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_pushButton_start_clicked()
{
    QVector<int> axis;
    QVector<double> dpos;

    if (ui.checkBox_enablex->isChecked() == 1)
    {
        axis << 0;
        dpos << ui.doubleSpinBox_tarpos_x->value();
    }
    if (ui.checkBox_enabley->isChecked() == 1)
    {
        axis << 1;
        dpos << ui.doubleSpinBox_tarpos_y->value();
    }
    if (ui.checkBox_enablez->isChecked() == 1)
    {
        axis << 2;
        dpos << ui.doubleSpinBox_tarpos_z->value();
    }
    if (ui.checkBox_enablea->isChecked() == 1)
    {
        axis << 3;
        dpos << ui.doubleSpinBox_tarpos_a->value();
    }
    if (ui.checkBox_enableb->isChecked() == 1)
    {
        axis << 4;
        dpos << ui.doubleSpinBox_tarpos_b->value();
    }
    if (ui.checkBox_enablec->isChecked() == 1)
    {
        axis << 5;
        dpos << ui.doubleSpinBox_tarpos_c->value();
    }

    emit MoveAbsolute(axis, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_checkBox_enablex_clicked()
{
    ui.pushButton_xm->setEnabled(ui.checkBox_enablex->isChecked());
    ui.pushButton_xp->setEnabled(ui.checkBox_enablex->isChecked());
    ui.doubleSpinBox_tarpos_x->setEnabled(ui.checkBox_enablex->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_checkBox_enabley_clicked()
{
    ui.pushButton_ym->setEnabled(ui.checkBox_enabley->isChecked());
    ui.pushButton_yp->setEnabled(ui.checkBox_enabley->isChecked());
    ui.doubleSpinBox_tarpos_y->setEnabled(ui.checkBox_enabley->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_checkBox_enablez_clicked()
{
    ui.pushButton_up->setEnabled(ui.checkBox_enablez->isChecked());
    ui.pushButton_down->setEnabled(ui.checkBox_enablez->isChecked());
    ui.doubleSpinBox_tarpos_z->setEnabled(ui.checkBox_enablez->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_checkBox_enablea_clicked()
{
    ui.pushButton_ap->setEnabled(ui.checkBox_enablea->isChecked());
    ui.pushButton_am->setEnabled(ui.checkBox_enablea->isChecked());
    ui.doubleSpinBox_tarpos_a->setEnabled(ui.checkBox_enablea->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_checkBox_enableb_clicked()
{
    ui.pushButton_bp->setEnabled(ui.checkBox_enableb->isChecked());
    ui.pushButton_bm->setEnabled(ui.checkBox_enableb->isChecked());
    ui.doubleSpinBox_tarpos_b->setEnabled(ui.checkBox_enableb->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::on_checkBox_enablec_clicked()
{
    ui.pushButton_cp->setEnabled(ui.checkBox_enablec->isChecked());
    ui.pushButton_cm->setEnabled(ui.checkBox_enablec->isChecked());
    ui.doubleSpinBox_tarpos_c->setEnabled(ui.checkBox_enablec->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::visibleWidget()
{
    ui.checkBox_enablex->setVisible(m_numaxis > 0);
    ui.checkBox_enabley->setVisible(m_numaxis > 1);
    ui.checkBox_enablez->setVisible(m_numaxis > 2);
    ui.checkBox_enablea->setVisible(m_numaxis > 3);
    ui.checkBox_enableb->setVisible(m_numaxis > 4);
    ui.checkBox_enablec->setVisible(m_numaxis > 5);

    ui.pushButton_xm->setVisible(m_numaxis > 0);
    ui.pushButton_xp->setVisible(m_numaxis > 0);
    ui.pushButton_ym->setVisible(m_numaxis > 1);
    ui.pushButton_yp->setVisible(m_numaxis > 1);
    ui.pushButton_up->setVisible(m_numaxis > 2);
    ui.pushButton_down->setVisible(m_numaxis > 2);
    ui.pushButton_am->setVisible(m_numaxis > 3);
    ui.pushButton_ap->setVisible(m_numaxis > 3);
    ui.pushButton_bm->setVisible(m_numaxis > 4);
    ui.pushButton_bp->setVisible(m_numaxis > 4);
    ui.pushButton_cm->setVisible(m_numaxis > 5);
    ui.pushButton_cp->setVisible(m_numaxis > 5);

    ui.doubleSpinBox_actpos_x->setVisible(m_numaxis > 0);
    ui.doubleSpinBox_tarpos_x->setVisible(m_numaxis > 0);
    ui.doubleSpinBox_actpos_y->setVisible(m_numaxis > 1);
    ui.doubleSpinBox_tarpos_y->setVisible(m_numaxis > 1);
    ui.doubleSpinBox_actpos_z->setVisible(m_numaxis > 2);
    ui.doubleSpinBox_tarpos_z->setVisible(m_numaxis > 2);
    ui.doubleSpinBox_actpos_a->setVisible(m_numaxis > 3);
    ui.doubleSpinBox_tarpos_a->setVisible(m_numaxis > 3);
    ui.doubleSpinBox_actpos_b->setVisible(m_numaxis > 4);
    ui.doubleSpinBox_tarpos_b->setVisible(m_numaxis > 4);
    ui.doubleSpinBox_actpos_c->setVisible(m_numaxis > 5);
    ui.doubleSpinBox_tarpos_c->setVisible(m_numaxis > 5);

    ui.label_xachse->setVisible(m_numaxis > 0);
    ui.label_yachse->setVisible(m_numaxis > 1);
    ui.label_zachse->setVisible(m_numaxis > 2);
    ui.label_aachse->setVisible(m_numaxis > 3);
    ui.label_bachse->setVisible(m_numaxis > 4);
    ui.label_cachse->setVisible(m_numaxis > 5);

    ui.label_actpos_t->setVisible(m_numaxis > 3);
    ui.label_tarpos_r->setVisible(m_numaxis > 3);

    if (m_numaxis < 4)
    {
        this->setMinimumHeight(278);
    }
    else
    {
        this->setMinimumHeight(366);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetDummyMotor::enableWidget(bool enabled)
{
    ui.checkBox_enablex->setEnabled(enabled && m_numaxis > 0);
    ui.checkBox_enabley->setEnabled(enabled && m_numaxis > 1);
    ui.checkBox_enablez->setEnabled(enabled && m_numaxis > 2);
    ui.checkBox_enablea->setEnabled(enabled && m_numaxis > 3);
    ui.checkBox_enableb->setEnabled(enabled && m_numaxis > 4);
    ui.checkBox_enablec->setEnabled(enabled && m_numaxis > 5);

    ui.pushButton_xm->setEnabled(enabled && m_numaxis > 0);
    ui.pushButton_xp->setEnabled(enabled && m_numaxis > 0);
    ui.pushButton_ym->setEnabled(enabled && m_numaxis > 1);
    ui.pushButton_yp->setEnabled(enabled && m_numaxis > 1);
    ui.pushButton_up->setEnabled(enabled && m_numaxis > 2);
    ui.pushButton_down->setEnabled(enabled && m_numaxis > 2);
    ui.pushButton_am->setEnabled(enabled && m_numaxis > 3);
    ui.pushButton_ap->setEnabled(enabled && m_numaxis > 3);
    ui.pushButton_bm->setEnabled(enabled && m_numaxis > 4);
    ui.pushButton_bp->setEnabled(enabled && m_numaxis > 4);
    ui.pushButton_cm->setEnabled(enabled && m_numaxis > 5);
    ui.pushButton_cp->setEnabled(enabled && m_numaxis > 5);

    ui.pushButton_start->setVisible(enabled);
    ui.pushButton_stop->setVisible(!enabled);

    ui.spinStepSize->setFocus();
}