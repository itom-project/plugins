/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dialogThorlabsPowerMeter.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsPowerMeter::DialogThorlabsPowerMeter(ito::AddInBase *grabber) :
AbstractAddInConfigDialog(grabber),
m_plugin(grabber),
m_firstRun(true)
{
    ui.setupUi(this);
    enableDialog(false);

};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsPowerMeter::parametersChanged(QMap<QString, ito::Param> params)
{
    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        ui.dspinWavelength->setMinimum(params["wavelength"].getMin());
        ui.dspinWavelength->setMaximum(params["wavelength"].getMax());
        ui.comboMeasurementMode->addItem("relative");
        ui.comboMeasurementMode->addItem("absolute");
        ui.dspinReference->setMinimum(params["reference_power"].getMin()*1E3);
        ui.dspinReference->setMaximum(params["reference_power"].getMax()*1E3);
        ui.spinAverage->setMinimum(params["average_number"].getMin());
        ui.spinAverage->setMaximum(params["average_number"].getMax());
        ui.comboBandwidth->addItem("high");
        ui.comboBandwidth->addItem("low");
        ui.dspinAttenuation->setMinimum(params["attenuation"].getMin());
        ui.dspinAttenuation->setMaximum(params["attenuation"].getMax());
        ui.spinLineFrequency->setMaximum(params["line_frequency"].getMax());
        ui.spinLineFrequency->setMinimum(params["line_frequency"].getMin());
        ui.spinLineFrequency->setSingleStep(((ito::IntMeta*)params["line_frequency"].getMeta())->getStepSize());
        m_firstRun = false;

        //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
        enableDialog(true);
    }


    ui.dspinWavelength->setValue(params["wavelength"].getVal<double>());
    ui.spinAverage->setValue(params["average_number"].getVal<ito::int32>());
    ui.dspinAttenuation->setValue(params["attenuation"].getVal<double>());
    ui.spinLineFrequency->setValue(params["line_frequency"].getVal<ito::int32>());
    if (params["auto_range"].getVal<ito::int32>())
    {
        ui.checkBoxAutoRange->setCheckState(Qt::Checked);
        ui.sliderPowerRange->setEnabled(false);
    }
    else
    {
        ui.checkBoxAutoRange->setCheckState(Qt::Unchecked);
        ui.sliderPowerRange->setEnabled(true);
    }
    ui.comboBandwidth->setCurrentIndex(params["bandwidth"].getVal<ito::int32>());
    QString d(params["measurement_mode"].getVal<char*>());
    if (QByteArray(params["measurement_mode"].getVal<char*>()) == "absolute")
    {
        ui.comboMeasurementMode->setCurrentIndex(1);
    }
    else
    {
        ui.comboMeasurementMode->setCurrentIndex(0);
    }
    ui.dspinReference->setValue(params["reference_power"].getVal<double>()*1e3);
    //ui.labelOffset->setText("Offset: " + QString::number(params["dark_offset"].getVal<double>())+" [a.u.]");
    ui.sliderPowerRange->setMinimum(params["power_range"].getMin()*1E3);
    ui.sliderPowerRange->setMaximum(params["power_range"].getMax()*1E3);
    ui.sliderPowerRange->setValue(params["power_range"].getVal<double>()*1e3);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsPowerMeter::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;


    if (m_currentParameters["wavelength"].getVal<double>() != ui.dspinWavelength->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("wavelength", ito::ParamBase::Double, ui.dspinWavelength->value())));
    }
    QString mode(ui.comboMeasurementMode->itemText(ui.comboMeasurementMode->currentIndex()));
    if (m_currentParameters["measurement_mode"].getVal<char*>() != mode)
    {

            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("measurement_mode", ito::ParamBase::String, mode.toLatin1().data())));
    }
    if (qAbs(m_currentParameters["reference_power"].getVal<double>() - (ui.dspinReference->value()*1e-3)) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("reference_power", ito::ParamBase::Double, ui.dspinReference->value()*1e-3)));
    }
    if (m_currentParameters["average_number"].getVal<int>() != ui.spinAverage->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("average_number", ito::ParamBase::Int, ui.spinAverage->value())));
    }
    int intBandwidth = (ui.comboBandwidth->itemText(ui.comboBandwidth->currentIndex()) == "high") ? 0 : 1;
    if (intBandwidth != m_currentParameters["bandwidth"].getVal<ito::int32>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bandwidth", ito::ParamBase::Int, intBandwidth)));
    }
    if (qAbs(m_currentParameters["attenuation"].getVal<double>() - (ui.dspinAttenuation->value())) > std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("attenuation", ito::ParamBase::Double, ui.dspinAttenuation->value())));
    }
    if (m_currentParameters["line_frequency"].getVal<ito::int32>() != ui.spinLineFrequency->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("line_frequency", ito::ParamBase::Int, ui.spinLineFrequency->value())));
    }
    int autoRange = (ui.checkBoxAutoRange->checkState() == Qt::Unchecked) ? 0 : 1;
    if (m_currentParameters["auto_range"].getVal<ito::int32>() != autoRange)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("auto_range", ito::ParamBase::Int, autoRange)));
    }
    if (qAbs(m_currentParameters["power_range"].getVal<double>() - ui.sliderPowerRange->value()*1e-3) > std::numeric_limits<double>::epsilon())
    {
        if (!autoRange)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("power_range", ito::ParamBase::Double, ui.sliderPowerRange->value()*1e-3)));
        }
    }



    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}
//


//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsPowerMeter::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsPowerMeter::on_btnZero_clicked()
{
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
    ui.group2->setEnabled(false);
    QMetaObject::invokeMethod(m_plugin, "zeroDevice", Q_ARG(ItomSharedSemaphore*, waitCond));
    observeInvocation(waitCond, msgLevelWarningAndError);
    ui.group2->setEnabled(true);
    waitCond->deleteSemaphore();
    waitCond = NULL;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsPowerMeter::enableDialog(bool enabled)
{
    //e.g.
    ui.group2->setEnabled(enabled);
}
