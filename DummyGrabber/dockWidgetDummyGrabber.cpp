#include "dockWidgetDummyGrabber.h"

 DockWidgetDummyGrabber::DockWidgetDummyGrabber(QMap<QString, ito::Param> params, int uniqueID)
 {
    ui.setupUi(this); 
    
    ui.lblID->setText(QString::number(uniqueID));

    valuesChanged(params);
 }

 void DockWidgetDummyGrabber::valuesChanged(QMap<QString, ito::Param> params)
 {
    if(params.contains("bpp"))
    {
        ui.spinBpp->setValue(params["bpp"].getVal<int>());
    }

    if(params.contains("sizex"))
    {
        ui.spinWidth->setValue(params["sizex"].getVal<int>());
    }

    if(params.contains("sizey"))
    {
        ui.spinHeight->setValue(params["sizey"].getVal<int>());
    }

    if(params.contains("gain"))
    {
        if(!(params["gain"].getFlags() & ito::ParamBase::Readonly))
        {
            ui.spinBox_gain->setEnabled(true);
        }
        else
        {
            ui.spinBox_gain->setEnabled(false);
        }
        ui.spinBox_gain->setValue((int)(params["gain"].getVal<double>()*100.0+0.5));
    }
    else
        ui.spinBox_gain->setEnabled(false);

    if(params.contains("offset"))
    {
        if(!(params["offset"].getFlags() & ito::ParamBase::Readonly))
        {
            ui.spinBox_offset->setEnabled(true);
        }
        else
        {
            ui.spinBox_offset->setEnabled(false);
        }
        ui.spinBox_offset->setValue((int)(params["offset"].getVal<double>()*100.0+0.5));
    }
    else
        ui.spinBox_offset->setEnabled(false);

    if(params.contains("integration_time"))
    {
        if(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly))
        {
            ui.doubleSpinBox_integration_time->setEnabled(true);
        }
        else
        {
            ui.doubleSpinBox_integration_time->setEnabled(false);
        }
        ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax() *1000.0);
        ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin() *1000.0);
        ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>()*1000.0);
    }
    else
        ui.doubleSpinBox_integration_time->setEnabled(false);

 }

void DockWidgetDummyGrabber::on_spinBox_gain_valueChanged(int /*d*/)
{
    emit GainOffsetPropertiesChanged( ui.spinBox_gain->value()/100.0, ui.spinBox_offset->value()/100.0);
}

void DockWidgetDummyGrabber::on_spinBox_offset_valueChanged(int /*d*/)
{
    emit GainOffsetPropertiesChanged( ui.spinBox_gain->value()/100.0, ui.spinBox_offset->value()/100.0);
}

void DockWidgetDummyGrabber::on_doubleSpinBox_integration_time_valueChanged(double /*d*/)
{
    emit IntegrationPropertiesChanged( ui.doubleSpinBox_integration_time->value() / 1000.0);
}



