#include "dialogUhl.h"
//#include "UhlRegister.h"

#include "common/addInInterface.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
//dialogUhl::dialogUhl(ito::AddInActuator *motor, int axisnums) : m_pUhlMotor(motor), m_numaxis(axisnums)
dialogUhl::dialogUhl(ito::AddInActuator *motor) : m_pUhlMotor(motor)
{
    memset(m_invert, 0, 4 * sizeof(int));
    memset(m_enable, 0, 4 * sizeof(int));

    ui.setupUi(this);

    m_numaxis = -1;

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::parametersChanged(QMap<QString, ito::Param> params)
{
    QVariant qvar;
    double dtemp = 0;

    setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

    dtemp = ((params)["speed"]).getVal<double>();
    ui.doubleSpinBox_Speed->setValue(dtemp);
    dtemp = ((params)["speed"]).getMax();
    ui.doubleSpinBox_Speed->setMaximum(dtemp);
    dtemp = ((params)["speed"]).getMin();
    ui.doubleSpinBox_Speed->setMinimum(dtemp);

    dtemp = ((params)["accel"]).getVal<double>();
    ui.doubleSpinBox_Accel->setValue(dtemp);
    dtemp = ((params)["accel"]).getMax();
    ui.doubleSpinBox_Accel->setMaximum(dtemp);
    dtemp = ((params)["accel"]).getMin();
    ui.doubleSpinBox_Accel->setMinimum(dtemp);

    m_invert[0] = (params)["inversex"].getVal<int>();
    ui.checkBox_InvertX->setChecked(m_invert[0]);
    m_invert[1] = (params)["inversey"].getVal<int>();
    ui.checkBox_InvertY->setChecked(m_invert[1]);
    m_invert[2] = (params)["inversez"].getVal<int>();
    ui.checkBox_InvertZ->setChecked(m_invert[2]);

    if (m_numaxis != (params)["numaxis"].getVal<int>())
    {
        m_numaxis = (params)["numaxis"].getVal<int>();

        m_enable[0] = (m_numaxis > 0);
        ui.checkBox_InvertX->setEnabled(m_enable[0]);
        ui.checkBox_EnableX->setEnabled(m_enable[0]);
        ui.checkBox_EnableX->setChecked(m_enable[0]);
        ui.OriginXButton->setEnabled(m_enable[0]);

        m_enable[1] = (m_numaxis > 1);
        ui.checkBox_InvertY->setEnabled(m_enable[1]);
        ui.checkBox_EnableY->setEnabled(m_enable[1]);
        ui.checkBox_EnableY->setChecked(m_enable[1]);
        ui.OriginYButton->setEnabled(m_enable[1]);

        m_enable[2] = (m_numaxis>2);
        ui.checkBox_InvertZ->setEnabled(m_enable[2]);
        ui.checkBox_EnableZ->setEnabled(m_enable[2]);
        ui.checkBox_EnableZ->setChecked(m_enable[2]);
        ui.OriginZButton->setEnabled(m_enable[2]);
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_actualParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::setUhlAxisOrigin(int axis)
{
    if (m_pUhlMotor)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());

        QMetaObject::invokeMethod(m_pUhlMotor,"setOrigin",Q_ARG(int,axis),Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        locker.getSemaphore()->wait(60000);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal dialogUhl::sendParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;
    double dtemp = 0.0;

    //only send parameters which are changed

    dtemp = ui.doubleSpinBox_Speed->value();
    if (m_actualParameters["speed"].getVal<double>() != dtemp)
    {
//        m_pUhlMotor->setParam(QSharedPointer<ito::tParam>(new ito::tParam("speed", ito::ParamBase::Double, dtemp, dtemp, dtemp, NULL)), NULL);
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("speed", ito::ParamBase::Int, dtemp)));
    }

    dtemp = ui.doubleSpinBox_Accel->value();
    if (m_actualParameters["accel"].getVal<double>() != dtemp)
    {
//        m_pUhlMotor->setParam(QSharedPointer<ito::tParam>(new ito::tParam("accel", ito::ParamBase::Double, dtemp, dtemp, dtemp, NULL)), NULL);
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("accel", ito::ParamBase::Int, dtemp)));
    }

    dtemp = ui.checkBox_InvertX->isChecked() ? 1.0 : 0.0;
    if (m_actualParameters["inversex"].getVal<double>() != dtemp)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("inversex", ito::ParamBase::Int, dtemp)));
        m_invert[0] = ui.checkBox_InvertX->isChecked();
    }

    dtemp = ui.checkBox_InvertY->isChecked() ? 1.0 : 0.0;
    if (m_actualParameters["inversey"].getVal<double>() != dtemp)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("inversey", ito::ParamBase::Int, dtemp)));
        m_invert[1] = ui.checkBox_InvertY->isChecked();
    }

    dtemp = ui.checkBox_InvertZ->isChecked() ? 1.0 : 0.0;
    if (m_actualParameters["inversez"].getVal<double>() != dtemp)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("inversez", ito::ParamBase::Int, dtemp)));
        m_invert[2] = ui.checkBox_InvertZ->isChecked();
    }

    dtemp = ui.checkBox_InvertZ->isChecked() ? 1.0 : 0.0;
    if (m_actualParameters["inversez"].getVal<double>() != dtemp)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("inversez", ito::ParamBase::Int, dtemp)));
        m_invert[2] = ui.checkBox_InvertZ->isChecked();
    }

    if (m_pUhlMotor)
    {
        if (values.size() > 0)
        {
            ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
            QMetaObject::invokeMethod(m_pUhlMotor, "setParamVector", Q_ARG(const QVector< QSharedPointer<ito::ParamBase> >, values), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

            while(!success)
            {
                if (locker.getSemaphore()->wait(PLUGINWAIT) == true)
                {
                    success = true;
                }

                if (!m_pUhlMotor->isAlive())
                {
                    break;
                }
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
void dialogUhl::on_pushButtonCalib_clicked()
{
    if (m_pUhlMotor)
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        int i = 0;
        QVector<int> axis;

        for (i = 0; i < m_numaxis; i++)
        {
            if (m_enable[i])
            {
                axis << i;
            }
        }

        enableDialog(false);
        qApp->processEvents();
        QMetaObject::invokeMethod(m_pUhlMotor,"calib",Q_ARG(QVector<int>,axis),Q_ARG(ItomSharedSemaphore*,locker.getSemaphore()));
        locker.getSemaphore()->wait(60000);
        enableDialog(true);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_checkBox_InvertX_clicked()
{
//  m_invert[0] = ui.checkBox_InvertX->isChecked();
//  m_pUhlMotor->setParam(QSharedPointer<ito::tParam>(new ito::tParam("inversex",ito::ParamBase::Int,m_invert[0],m_invert[0],m_invert[0], NULL)), NULL);
    sendParameters();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_checkBox_InvertY_clicked()
{
//    m_invert[1] = ui.checkBox_InvertY->isChecked();
//    m_pUhlMotor->setParam(QSharedPointer<ito::tParam>(new ito::tParam("inversey",ito::ParamBase::Int,m_invert[0],m_invert[0],m_invert[0], NULL)), NULL);
    sendParameters();
}

//---------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_checkBox_InvertZ_clicked()
{
//    m_invert[2] = ui.checkBox_InvertZ->isChecked();
//    m_pUhlMotor->setParam(QSharedPointer<ito::tParam>(new ito::tParam("inversez",ito::ParamBase::Int,m_invert[0],m_invert[0],m_invert[0], NULL)), NULL);
    sendParameters();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_checkBox_EnableX_clicked()
{
    m_enable[0] = ui.checkBox_EnableX->isChecked();
    ui.checkBox_InvertX->setEnabled(m_enable[0]);
    ui.OriginXButton->setEnabled(m_enable[0]);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_checkBox_EnableY_clicked()
{
    m_enable[1] = ui.checkBox_EnableY->isChecked();
    ui.checkBox_InvertY->setEnabled(m_enable[1]);
    ui.OriginYButton->setEnabled(m_enable[1]);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_checkBox_EnableZ_clicked()
{
    m_enable[2] = ui.checkBox_EnableZ->isChecked();
    ui.checkBox_InvertZ->setEnabled(m_enable[2]);
    ui.OriginZButton->setEnabled(m_enable[2]);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::enableDialog(bool enabled)
{
    ui.pushButtonCalib->setEnabled(enabled);
    ui.doubleSpinBox_Accel->setEnabled(enabled);
    ui.doubleSpinBox_Speed->setEnabled(enabled);
    ui.cancelButton->setEnabled(enabled);
    ui.okButton->setEnabled(enabled);
    ui.checkBox_EnableX->setEnabled(enabled);
    ui.checkBox_EnableY->setEnabled(enabled);
    ui.checkBox_EnableZ->setEnabled(enabled);
    ui.checkBox_InvertX->setEnabled(enabled && ui.checkBox_EnableX->isChecked());
    ui.checkBox_InvertY->setEnabled(enabled && ui.checkBox_EnableY->isChecked());
    ui.checkBox_InvertZ->setEnabled(enabled && ui.checkBox_EnableZ->isChecked());
    ui.OriginXButton->setEnabled(enabled && ui.checkBox_EnableX->isChecked());
    ui.OriginYButton->setEnabled(enabled && ui.checkBox_EnableY->isChecked());
    ui.OriginZButton->setEnabled(enabled && ui.checkBox_EnableZ->isChecked());
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_OriginXButton_clicked()
{
    setUhlAxisOrigin(0);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_OriginYButton_clicked()
{
    setUhlAxisOrigin(1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_OriginZButton_clicked()
{
    setUhlAxisOrigin(2);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogUhl::on_okButton_clicked()
{
    if (m_pUhlMotor)
    {
        QMetaObject::invokeMethod(m_pUhlMotor, "RequestStatusAndPosition", Q_ARG(bool,true), Q_ARG(bool,true));
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
