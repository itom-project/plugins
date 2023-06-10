/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dialogNewport2936.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogNewport2936::DialogNewport2936(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_plugin(grabber)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known yet. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogNewport2936::parametersChanged(QMap<QString, ito::Param> params)
{
    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        ui.spinWavelengthA->setMinimum(params["wavelengthA"].getMin());
        ui.spinWavelengthA->setMaximum(params["wavelengthA"].getMax());

        if (params["channels"].getVal<int>() == 1)
        {
            ui.spinWavelengthB->setEnabled(false);
            ui.comboFilterB->setEnabled(false);
            ui.checkAttenB->setEnabled(false);
            ui.btnZeroB->setEnabled(false);
        }
        else
        {
            ui.spinWavelengthB->setMinimum(params["wavelengthB"].getMin());
            ui.spinWavelengthB->setMaximum(params["wavelengthB"].getMax());
        }
        m_firstRun = false;
        enableDialog(true);
    }
    ui.comboFilterA->setCurrentIndex(params["filterTypeA"].getVal<int>());
    ui.spinWavelengthA->setValue(params["wavelengthA"].getVal<int>());
    ui.checkAttenA->setChecked(params["attenuatorA"].getVal<int>());
    if (params["channels"].getVal<int>() == 1)
    {
        ui.comboFilterB->setCurrentIndex(params["filterTypeB"].getVal<int>());
        ui.spinWavelengthB->setValue(params["wavelengthB"].getVal<int>());
        ui.checkAttenB->setChecked(params["attenuatorB"].getVal<int>());
    }

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogNewport2936::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;
    if (m_currentParameters["wavelengthA"].getVal<int>() != ui.spinWavelengthA->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("wavelengthA", ito::ParamBase::Int, ui.spinWavelengthA->value())));
    }
    if (m_currentParameters["filterTypeA"].getVal<int>() != ui.comboFilterA->currentIndex())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("filterTypeA", ito::ParamBase::Int, ui.comboFilterA->currentIndex())));
    }
    if (m_currentParameters["attenuatorA"].getVal<int>() != ui.checkAttenA->isChecked())
    {
        if (ui.checkAttenA->isChecked())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("attenuatorA", ito::ParamBase::Int, 1)));
        }
        else
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("attenuatorA", ito::ParamBase::Int, 0)));
        }
    }
    if (m_currentParameters["channels"].getVal<int>() == 2)
    {
        if (m_currentParameters["attenuatorB"].getVal<int>() != ui.checkAttenB->isChecked())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("attenuatorB", ito::ParamBase::Double, ui.checkAttenB->isChecked())));
        }
        if (m_currentParameters["filterTypeB"].getVal<int>() != ui.comboFilterB->currentIndex())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("filterTypeB", ito::ParamBase::Double, ui.comboFilterB->currentIndex())));
        }
        if (m_currentParameters["wavelengthB"].getVal<int>() != ui.spinWavelengthB->value())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("wavelengthB", ito::ParamBase::Int, ui.spinWavelengthB->value())));
        }
    }



    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogNewport2936::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogNewport2936::on_btnZeroA_clicked()
{
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
    ui.group1->setEnabled(false);
    QMetaObject::invokeMethod(m_plugin, "zeroDevice",Q_ARG(int, 1) ,Q_ARG(ItomSharedSemaphore*, waitCond));
    observeInvocation(waitCond, msgLevelWarningAndError);
    ui.group1->setEnabled(true);
    waitCond->deleteSemaphore();
    waitCond = NULL;
}
//---------------------------------------------------------------------------------------------------------------------
void DialogNewport2936::on_btnZeroB_clicked()
{
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
    ui.group1->setEnabled(false);
    QMetaObject::invokeMethod(m_plugin, "zeroDevice", Q_ARG(int, 2), Q_ARG(ItomSharedSemaphore*, waitCond));
    observeInvocation(waitCond, msgLevelWarningAndError);
    ui.group1->setEnabled(true);
    waitCond->deleteSemaphore();
    waitCond = NULL;
}
//---------------------------------------------------------------------------------------------------------------------
void DialogNewport2936::enableDialog(bool enabled)
{
    //e.g.
    ui.group1->setEnabled(enabled);
}
