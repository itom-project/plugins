#include "dialogPiezosystemJena_NV40_1.h"
#include "piezosystemJena_NV40_1.h"

#include "common/addInInterface.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>


//----------------------------------------------------------------------------------------------------------------------------------
DialogPiezosystemJena_NV40_1::DialogPiezosystemJena_NV40_1(ito::AddInBase *motor) :
    AbstractAddInConfigDialog(motor),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};



//----------------------------------------------------------------------------------------------------------------------------------
void DialogPiezosystemJena_NV40_1::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        m_firstRun = false;
    }

    ui.checkAsync->setChecked(params["async"].getVal<int>() > 0);
    ui.radioRemoteOn->setChecked(params["remote"].getVal<int>() > 0);
    ui.radioClosedLoop->setChecked(params["closedLoop"].getVal<int>() > 0);

    ui.spinSleep->setMinimum(0);
    ui.spinSleep->setMaximum(params["delayTime"].getMax() * 1000);
    ui.spinSleep->setValue(params["delayTime"].getVal<double>() * 1000);

    ui.radioDelayAsk->setChecked(params["delayMode"].getVal<int>() == 0);

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPiezosystemJena_NV40_1::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed

    int i = ui.radioRemoteOn->isChecked() ? 1 : 0;
    if (m_currentParameters["remote"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("remote", ito::ParamBase::Int, i)));
    }

    i = ui.radioClosedLoop->isChecked() ? 1 : 0;
    if (m_currentParameters["closedLoop"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("closedLoop", ito::ParamBase::Int, i)));
    }

    i = ui.checkAsync->isChecked() ? 1 : 0;
    if (m_currentParameters["async"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("async", ito::ParamBase::Int, i)));
    }

    i = ui.radioDelayAsk->isChecked() ? 0 : 1;
    if (m_currentParameters["delayMode"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delayMode", ito::ParamBase::Int, i)));
    }

    double v = ui.spinSleep->value() / 1000.0;
    if (qAbs(m_currentParameters["delayTime"].getVal<double>() - v) > 0.001)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delayTime", ito::ParamBase::Double, v)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPiezosystemJena_NV40_1::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else
    {
        applyParameters(); //ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPiezosystemJena_NV40_1::enableDialog(bool enabled)
{
    ui.group1->setEnabled(enabled);
    ui.group2->setEnabled(enabled);
    ui.group3->setEnabled(enabled);
    ui.group4->setEnabled(enabled);
}
