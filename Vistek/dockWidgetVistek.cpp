#include "dockWidgetVistek.h"

 DockWidgetVistek::DockWidgetVistek() : m_inEditing(false)
 {
     ui.setupUi(this); 
 }


 void DockWidgetVistek::valuesChanged(QMap<QString, ito::Param> params)
 {
    if (params.contains("sizex"))
    {
        ui.lblWidth->setText( QString("%1").arg( params["sizex"].getVal<int>()));
    }

    if (params.contains("sizey"))
    {
        ui.lblHeight->setText( QString("%1").arg( params["sizey"].getVal<int>()));
    }

    if (params.contains("bpp"))
    {
        ui.lblBitDepth->setText( QString("%1").arg( params["bpp"].getVal<int>()));
    }

    if (params.contains("cameraModel"))
    {
        ui.ModelLabel->setText( params["cameraModel"].getVal<char*>() );
    }
        
    if (params.contains("cameraSerialNo"))
    {
        ui.SerialLabel->setText( params["cameraSerialNo"].getVal<char*>() );
    }

    if (params.contains("cameraIP"))
    {
        ui.IPLabel->setText( params["cameraIP"].getVal<char*>() );
    }

    if (params.contains("cameraManufacturer"))
    {
        ui.ManufacturerLabel->setText( params["cameraManufacturer"].getVal<char*>() );
    }

    if (params.contains("exposure"))
    {
        ui.exposureSpinBox->setValue( params["exposure"].getVal<double>() );
    }

    if (params.contains("gain"))
    {
        ui.gainSpinBox->setValue( params["gain"].getVal<double>() );
    }
     
 }

void DockWidgetVistek::on_exposureSpinBox_editingFinished()
{
    emit ExposurePropertyChanged( ui.exposureSpinBox->value() );
}

void DockWidgetVistek::on_gainSpinBox_editingFinished()
{
    emit GainPropertyChanged( ui.gainSpinBox->value() );
}