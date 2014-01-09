#include "dockWidgetVistek.h"

 DockWidgetVistek::DockWidgetVistek(QMap<QString, ito::Param> params, int uniqueID)
 {
     ui.setupUi(this); 

     char *temp = params["name"].getVal<char*>(); //borrowed reference
//     ui.lblName->setText(temp);
     ui.lblID->setText(QString::number(uniqueID));

     valuesChanged(params);
 }

 void DockWidgetVistek::valuesChanged(QMap<QString, ito::Param> params)
 {
     char *temp = NULL;

     ui.WidthLCD->display(params["sizex"].getVal<int>());
     ui.HeightLCD->display(params["sizey"].getVal<int>());

     ui.exposureSpinBox->setValue(params["exposure"].getVal<double>());
     ui.gainSpinBox->setValue(params["gain"].getVal<double>());

     temp = params["CameraModel"].getVal<char*>(); //borrowed reference
     ui.ModelLabel->setText(temp);
     temp = params["CameraSerialNo"].getVal<char*>(); //borrowed reference
     ui.SerialLabel->setText(temp);
     temp = params["CameraIP"].getVal<char*>();
     ui.IPLabel->setText(temp);
     temp = params["CameraManufacturer"].getVal<char*>(); //borrowed reference
     ui.ManufacturerLabel->setText(temp);
 }

void DockWidgetVistek::on_exposureSpinBox_editingFinished()
{
    emit exposurePropertiesChanged(ui.exposureSpinBox->value());
}

void DockWidgetVistek::on_gainSpinBox_editingFinished()
{
    emit gainPropertiesChanged(ui.gainSpinBox->value());
}