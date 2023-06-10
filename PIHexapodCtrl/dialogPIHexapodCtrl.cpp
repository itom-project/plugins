#include "dialogPIHexapodCtrl.h"
#include "PIHexapodCtrl.h"

#include "common/addInInterface.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogPIHexapodCtrl::DialogPIHexapodCtrl(ito::AddInActuator *motor) : m_pPIHexapod(motor)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogPIHexapodCtrl::parametersChanged(QMap<QString, ito::Param> params)
{
    setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
//    QString temp;
    ui.lblDevice1->setText( params["ctrlType"].getVal<char*>() );
    ui.lblDevice2->setText( params["ctrlName"].getVal<char*>() );
    //ui.lblHexapod->setText( params["HexapodName"].getVal<char*>() );

    bool hasMode = params["hasLocalRemote"].getVal<int>() > 0;
    ui.comboMode->setVisible(hasMode);
    ui.lblMode->setVisible(hasMode);

    if (params["local"].getVal<int>() > 0)
    {
        ui.comboMode->setCurrentIndex(1);
    }
    else
    {
        ui.comboMode->setCurrentIndex(0);
    }

    ui.checkAsync->setChecked( params["async"].getVal<int>() > 0 );
    ui.dblSpinPosLimitHigh->setValue( params["posLimitHigh"].getVal<double>() * 1000 );
    ui.dblSpinPosLimitLow->setValue( params["posLimitLow"].getVal<double>() * 1000 );

    ui.spinDelayOffset->setMinimum(0);
    ui.spinDelayOffset->setMaximum( params["delayOffset"].getMax() * 1000 );
    ui.spinDelayOffset->setValue( params["delayOffset"].getVal<double>() * 1000 );

    ui.spinDelayProp->setMinimum(0);
    ui.spinDelayProp->setMaximum( params["delayProp"].getMax() * 1000 );
    ui.spinDelayProp->setValue( params["delayProp"].getVal<double>() );

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_actualParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPIHexapodCtrl::checkParameters()
{
    ito::RetVal retValue(ito::retOk);
    if (ui.dblSpinPosLimitHigh->value() < ui.dblSpinPosLimitLow->value())
    {
        retValue += ito::RetVal(ito::retError, 0, tr("the upper position limit must be higher than the lower one").toLatin1().data());
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPIHexapodCtrl::sendParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed

    double v = ui.comboMode->currentIndex() > 0 ? 1.0 : 0.0;
    if (m_actualParameters["local"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("local", ito::ParamBase::Int, v) ) );
    }

    v = ui.checkAsync->isChecked() ? 1.0 : 0.0;
    if (m_actualParameters["async"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("async", ito::ParamBase::Int, v) ) );
    }

    v = ui.dblSpinPosLimitHigh->value() / 1000.0;
    if (m_actualParameters["posLimitHigh"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("posLimitHigh", ito::ParamBase::Double, v) ) );
    }

    v = ui.dblSpinPosLimitLow->value() / 1000.0;
    if (m_actualParameters["posLimitLow"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("posLimitLow", ito::ParamBase::Double, v) ) );
    }

    v = ui.spinDelayOffset->value() / 1000.0;
    if (m_actualParameters["delayOffset"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("delayOffset", ito::ParamBase::Double, v) ) );
    }

    v = ui.spinDelayProp->value();
    if (m_actualParameters["delayProp"].getVal<double>() != v)
    {
        values.append( QSharedPointer<ito::ParamBase>( new ito::ParamBase("delayProp", ito::ParamBase::Double, v) ) );
    }

    if (m_pPIHexapod)
    {
        if (values.size() > 0)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(m_pPIHexapod, "setParamVector", Q_ARG( const QVector< QSharedPointer<ito::ParamBase> >, values), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            while(!success)
            {
                if (locker.getSemaphore()->wait(PLUGINWAIT) == true)
                {
                    success = true;
                }
                if (!m_pPIHexapod->isAlive()) break;
            }

            if (!success)
            {
                retValue += ito::RetVal(ito::retError, 0, tr("timeout while setting parameters of plugin.").toLatin1().data());
            }
        }
    }
    else
    {
        retValue += ito::RetVal(ito::retError, 0, tr("plugin instance not defined.").toLatin1().data());
    }

    return retValue;

}

//---------------------------------------------------------------------------------------------------------------------
void DialogPIHexapodCtrl::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    else //ApplyRole or AcceptRole
    {
        retValue += checkParameters();
        if (retValue.containsError())
        {
            QMessageBox msgBox(this);
            QString text = tr("Invalid parameter input");
            msgBox.setText( text );
            msgBox.setIcon(QMessageBox::Critical);
            if (retValue.errorMessage()) //if no error message indicates, this is NULL
            {
                msgBox.setInformativeText( retValue.errorMessage() );
            }

            msgBox.exec();
        }
        else
        {
            retValue += sendParameters();

            if (retValue.containsError())
            {
                QMessageBox msgBox(this);
                QString text = tr("Error while setting parameters of plugin.");
                msgBox.setText( text );
                msgBox.setIcon(QMessageBox::Critical);
                if (retValue.errorMessage()) //if no error message indicates, this is NULL
                {
                    msgBox.setInformativeText( retValue.errorMessage() );
                }

                msgBox.exec();
            }
            else if (role == QDialogButtonBox::AcceptRole)
            {
                accept(); //close dialog with accept
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPIHexapodCtrl::enableDialog(bool enabled)
{
    ui.group1->setEnabled(enabled);
    ui.group2->setEnabled(enabled);
    ui.group3->setEnabled(enabled);
    ui.group4->setEnabled(enabled);
}
