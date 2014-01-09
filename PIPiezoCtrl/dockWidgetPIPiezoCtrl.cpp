#include "dockWidgetPIPiezoCtrl.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
 DockWidgetPIPiezoCtrl::DockWidgetPIPiezoCtrl(int uniqueID, ito::AddInActuator *actuator) : m_pPlugin(actuator)
 {
     ui.setupUi(this); 

     ui.lblID->setText(QString::number(uniqueID));

     enableWidget(true);
 }
 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetPIPiezoCtrl::valuesChanged(QMap<QString, ito::Param> params)
 {
    ui.lblDevice1->setText( params["ctrlType"].getVal<char*>() );
    ui.lblDevice2->setText( params["ctrlName"].getVal<char*>() );
    ui.lblPiezo->setText( params["piezoName"].getVal<char*>() );

    bool hasMode = params["hasLocalRemote"].getVal<int>() > 0;
    ui.groupBoxMode->setVisible(hasMode);

    if (params["local"].getVal<int>() > 0)
    {
        ui.radioLocal->setChecked(true);
    }
    else
    {
        ui.radioRemote->setChecked(true);
    }
 }

 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetPIPiezoCtrl::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
 {
    ui.spinBoxTargetPos->setEnabled(status[0] & ito::actuatorEnabled);

    if (actPosition.size() > 0)
    {
        ui.spinBoxActPos->setValue(actPosition[0] * 1000);
    }

    bool running = false;
    QString style;

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
    ui.spinBoxActPos->setStyleSheet(style);

     enableWidget(!running);
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::targetChanged(QVector<double> targetPositions)
{
    if (targetPositions.size() > 0)
    {
        ui.spinBoxTargetPos->setValue( targetPositions[0] * 1000 );
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::enableWidget(bool enabled)
{
    ui.spinBoxTargetPos->setEnabled(enabled);
    ui.btnUp->setEnabled(enabled);
    ui.btnDown->setEnabled(enabled);
    ui.groupBoxMode->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::waitForDoneAndCheckRetVal(ItomSharedSemaphore *waitCond)
{
    int done = 0;
    QTime time;
    QMessageBox msgBox;
    time.start();
    if (m_pPlugin)
    {
        while(done == 0 && waitCond->wait(10) == false)
        {
            QCoreApplication::processEvents();
            if (time.elapsed() > PLUGINWAIT)
            {
                if (m_pPlugin->isAlive() == false)
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
void DockWidgetPIPiezoCtrl::on_radioLocal_clicked()
{
    QSharedPointer<ito::ParamBase> param;
    if (ui.radioLocal->isChecked())
    {
        param = QSharedPointer<ito::ParamBase>(new ito::ParamBase("local",ito::ParamBase::Int,1.0));
    }
    else
    {
        param = QSharedPointer<ito::ParamBase>(new ito::ParamBase("local",ito::ParamBase::Int,0.0));
    }

    if (m_pPlugin)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pPlugin, "setParam", Q_ARG(QSharedPointer<ito::ParamBase>,param), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnUp_clicked()
{
    if (m_pPlugin)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pPlugin, "setPosRel", Q_ARG(int,0), Q_ARG(double,ui.spinBoxStepSize->value() / 1000.0), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnDown_clicked()
{
    if (m_pPlugin)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pPlugin, "setPosRel", Q_ARG(int,0), Q_ARG(double,-ui.spinBoxStepSize->value() / 1000.0), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnStart_clicked()
{
    if (m_pPlugin)
    {
        enableWidget(false);
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_pPlugin, "setPosAbs", Q_ARG(int,0), Q_ARG(double,ui.spinBoxTargetPos->value() / 1000.0), Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        waitForDoneAndCheckRetVal(locker.getSemaphore());
        enableWidget(true);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPIPiezoCtrl::on_btnRefresh_clicked()
{
    if (m_pPlugin)
    {
        QMetaObject::invokeMethod(m_pPlugin, "RequestStatusAndPosition", Q_ARG(bool,true), Q_ARG(bool,true));
    }
}
