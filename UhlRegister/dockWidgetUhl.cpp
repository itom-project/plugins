#include "dockWidgetUhl.h"
#include "common/addInInterface.h"

#include <qmessagebox.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetUhl::DockWidgetUhl(int uniqueID, ito::AddInActuator *actuator) : m_pUhlMotor(actuator)
{
    ui.setupUi(this);

    ui.lblID->setText(QString::number(uniqueID));

    enableWidget(true);

    ui.pushButton_stop->setEnabled(false);

/*    m_numaxis = 0; //at startup we don't have information about the real number, this is sent later by the slot valuesChanged
    ui.setupUi(this);
    ui.lblID->setText(QString::number(uniqueID));
    setMotorStatus(false);*/
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*void  DockWidgetUhl::CheckAxisNums(QMap<QString, ito::tParam> params)
{
    m_numaxis = params["numaxis"].getVal<int>();
    ui.lblAxis->setText(QString::number(m_numaxis));

    ui.checkBox_enablex->setEnabled(m_numaxis > 0);
    ui.checkBox_enablex->setChecked(m_numaxis > 0);
    on_checkBox_enablex_clicked();

    ui.checkBox_enabley->setEnabled(m_numaxis > 1);
    ui.checkBox_enabley->setChecked(m_numaxis > 1);
    on_checkBox_enabley_clicked();

    ui.checkBox_enablez->setEnabled(m_numaxis > 2);
    ui.checkBox_enablez->setChecked(m_numaxis > 2);
    on_checkBox_enablez_clicked();
}
*/
//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::enableWidget(bool enabled)
{
    ui.groupBoxProperties->setEnabled(enabled);
    ui.groupBoxJoystick->setEnabled(enabled);
    ui.groupBoxRelPos->setEnabled(enabled);
//    ui.groupBoxAbsPos->setEnabled(enabled);
    ui.pushButton_start->setEnabled(enabled);
    ui.pushButton_refresh->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_x->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_y->setEnabled(enabled);
    ui.doubleSpinBox_tarpos_z->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::valuesChanged(QMap<QString, ito::Param> params)
{
    int numaxis = params["numaxis"].getVal<int>();

    ui.lblAxis->setText( QString("%1").arg(numaxis) );
//    ui.lblName->setText(params["name"].getVal<char*>());

    ui.checkBox_enablex->setEnabled(numaxis > 0);
    ui.checkBox_enablex->setChecked(numaxis > 0);
    on_checkBox_enablex_clicked();

    ui.checkBox_enabley->setEnabled(numaxis > 1);
    ui.checkBox_enabley->setChecked(numaxis > 1);
    on_checkBox_enabley_clicked();

    ui.checkBox_enablez->setEnabled(numaxis > 2);
    ui.checkBox_enablez->setChecked(numaxis > 2);
    on_checkBox_enablez_clicked();

    QMap<QString, ito::Param>::const_iterator paramIt = params.constFind("joyenabled");
    ui.checkBox_enablejoy->setEnabled(paramIt != params.constEnd());
    if (ui.checkBox_enablejoy->isEnabled())
    {
        ui.checkBox_enablejoy->setChecked(paramIt->getVal<int>());
    }
    else
    {
        ui.checkBox_enablejoy->setChecked(false);
    }

/*    ui.lblName->setText(params["name"].getVal<char*>());

    CheckAxisNums(params);*/
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
/*void DockWidgetUhl::setMotorStatus(bool busy)
{
    ui.pushButton_start->setEnabled(!busy);
    ui.pushButton_stop->setEnabled(busy);
    ui.groupBoxRelPos->setEnabled(!busy);
    ui.groupBoxProperties->setEnabled(!busy);
}*/

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
{
    switch( actPosition.size() )
    {
    case 3:
        ui.doubleSpinBox_actpos_z->setValue(actPosition[2]);
    case 2:
        ui.doubleSpinBox_actpos_y->setValue(actPosition[1]);
    case 1:
        ui.doubleSpinBox_actpos_x->setValue(actPosition[0]);
    }

    bool running = false;
    QString style;

    for (int i=0; i<status.size(); i++)
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

        switch(i)
        {
        case 0: ui.doubleSpinBox_actpos_x->setStyleSheet(style); break;
        case 1: ui.doubleSpinBox_actpos_y->setStyleSheet(style); break;
        case 2: ui.doubleSpinBox_actpos_z->setStyleSheet(style); break;
        }
    }

    enableWidget(!running);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::targetChanged(QVector<double> targetPositions)
{
    switch( targetPositions.size() )
    {
    case 3:
        ui.doubleSpinBox_tarpos_z->setValue( targetPositions[2] );
    case 2:
        ui.doubleSpinBox_tarpos_y->setValue( targetPositions[1] );
    case 1:
        ui.doubleSpinBox_tarpos_x->setValue( targetPositions[0] );
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_xp_clicked()
{
//    double dpos = ui.spinStepSize->value() / 1e3;
//    emit MoveRelative(0, dpos, 0);
    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosRel", Q_ARG(int,0), Q_ARG(double,ui.spinStepSize->value() / 1e3), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_xm_clicked()
{
//    double dpos = ui.spinStepSize->value() / -1e3;
//    emit MoveRelative(0, dpos, 0);
    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosRel", Q_ARG(int,0), Q_ARG(double,ui.spinStepSize->value() / -1e3), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_yp_clicked()
{
//    double dpos = ui.spinStepSize->value() / 1e3;
//    emit MoveRelative(1, dpos, 0);
    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosRel", Q_ARG(int,1), Q_ARG(double,ui.spinStepSize->value() / 1e3), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_ym_clicked()
{
//    double dpos = ui.spinStepSize->value() / -1e3;
//    emit MoveRelative(1, dpos, 0);
    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosRel", Q_ARG(int,1), Q_ARG(double,ui.spinStepSize->value() / -1e3), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_up_clicked()
{
//    double dpos = ui.spinStepSize->value() / 1e3;
//    emit MoveRelative(2, dpos, 0);
    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosRel", Q_ARG(int,2), Q_ARG(double,ui.spinStepSize->value() / 1e3), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_down_clicked()
{
//    double dpos = ui.spinStepSize->value() / -1e3;
//    emit MoveRelative(2, dpos, 0);
    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosRel", Q_ARG(int,2), Q_ARG(double,ui.spinStepSize->value() / -1e3), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_refresh_clicked()
{
//    emit MotorTriggerStatusRequest(true,true);
    if (m_pUhlMotor)
    {
        QMetaObject::invokeMethod(m_pUhlMotor, "requestStatusAndPosition", Q_ARG(bool,true), Q_ARG(bool,true));
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_start_clicked()
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

    if (m_pUhlMotor)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setPosAbs", Q_ARG(QVector<int>,axis), Q_ARG(QVector<double>,dpos), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
//    emit MoveAbsolute(axis, dpos, 0);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_pushButton_stop_clicked()
{
/*    if (m_actuator)
    {
        m_actuator->setInterrupt();
    }*/
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_checkBox_enablex_clicked()
{
    ui.pushButton_xm->setEnabled(ui.checkBox_enablex->isChecked());
    ui.pushButton_xp->setEnabled(ui.checkBox_enablex->isChecked());
    ui.doubleSpinBox_tarpos_x->setEnabled(ui.checkBox_enablex->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_checkBox_enabley_clicked()
{
    ui.pushButton_ym->setEnabled(ui.checkBox_enabley->isChecked());
    ui.pushButton_yp->setEnabled(ui.checkBox_enabley->isChecked());
    ui.doubleSpinBox_tarpos_y->setEnabled(ui.checkBox_enabley->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_checkBox_enablez_clicked()
{
    ui.pushButton_up->setEnabled(ui.checkBox_enablez->isChecked());
    ui.pushButton_down->setEnabled(ui.checkBox_enablez->isChecked());
    ui.doubleSpinBox_tarpos_z->setEnabled(ui.checkBox_enablez->isChecked());
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::on_checkBox_enablejoy_clicked()
{
    QVector<QSharedPointer<ito::ParamBase> > values;

    int v = ui.checkBox_enablejoy->isChecked() ? 1 : 0;
    values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("joyenabled", ito::ParamBase::Int, v) ) );

    if (m_pUhlMotor)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pUhlMotor, "setParamVector", Q_ARG( const QVector< QSharedPointer<ito::ParamBase> >, values), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetUhl::waitForDoneAndCheckRetVal(ItomSharedSemaphore *waitCond)
{
    int done = 0;
    QElapsedTimer time;
    QMessageBox msgBox;
    time.start();
    if (m_pUhlMotor)
    {
        while(done == 0 && waitCond->waitAndProcessEvents(PLUGINWAIT/2) == false)
        {
            if (time.elapsed() > PLUGINWAIT)
            {
                if (m_pUhlMotor->isAlive() == false)
                {
                    msgBox.setText("Error while execution");
                    msgBox.setInformativeText("The plugin is not reacting any more");
                    msgBox.setIcon(QMessageBox::Critical);
                    msgBox.exec();
                    done = 1;
                }
                time.restart();
            }
        }
        //while(done == 0 && waitCond->wait(10) == false)
        //{
        //    if(waitCond->wait(10) == true)
        //    {
        //        qDebug("bruch");
        //        break;
        //    }
        //    QCoreApplication::processEvents();
        //    if (time.elapsed() > PLUGINWAIT)
        //    {
        //        if (m_pUhlMotor->isAlive() == false)
        //        {
        //            msgBox.setText("Error while execution");
        //            msgBox.setInformativeText("The plugin is not reacting any more");
        //            msgBox.setIcon(QMessageBox::Critical);
        //            msgBox.exec();
        //            done = 1;
        //        }
        //        time.restart();
        //    }
        //}

        if (done == 0) //ok, check returnValue of waitCond
        {
            ito::RetVal retVal = waitCond->returnValue;
            if (retVal.containsError())
            {
                msgBox.setText("Error while execution");
                if (retVal.errorMessage())
                {
                    msgBox.setInformativeText( retVal.errorMessage() );
                }
                msgBox.setIcon(QMessageBox::Critical);
                msgBox.exec();
            }
            else if (retVal.containsWarning())
            {
                msgBox.setText("Warning while execution");
                if (retVal.errorMessage())
                {
                    msgBox.setInformativeText( retVal.errorMessage() );
                }
                msgBox.setIcon(QMessageBox::Warning);
                msgBox.exec();
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
