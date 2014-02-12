/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
	Copyright (C) 2013, Institut für Technische Optik, Universität Stuttgart

    This file is part of a plugin for the measurement software itom.
  
    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dockWidgetPGRFlyCapture.h"

 DockWidgetPGRFlyCapture::DockWidgetPGRFlyCapture(QMap<QString, ito::Param> params, int uniqueID)
 {
    ui.setupUi(this); 
    
    char* temp = params["name"].getVal<char*>(); //borrowed reference
//    ui.lblName->setText(temp);
    ui.lblID->setText(QString::number(uniqueID));

    valuesChanged(params);
 }

 void DockWidgetPGRFlyCapture::valuesChanged(QMap<QString, ito::Param> params)
 {
    ui.spinBpp->setValue(params["bpp"].getVal<int>());
    ui.spinWidth->setValue(params["sizex"].getVal<int>());
    ui.spinHeight->setValue(params["sizey"].getVal<int>());


    if(!(params["gain"].getFlags() & ito::ParamBase::Readonly))
    {
        ui.spinBox_gain->setEnabled(true);
    }
    else
    {
        ui.spinBox_gain->setEnabled(false);
    }
    ui.spinBox_gain->setValue((int)(params["gain"].getVal<double>()*100.0+0.5));
    ui.horizontalSlider_gain->setValue(ui.spinBox_gain->value());

    if(!(params["offset"].getFlags() & ito::ParamBase::Readonly))
    {
        ui.spinBox_offset->setEnabled(true);
    }
    else
    {
        ui.spinBox_offset->setEnabled(false);
    }
    ui.spinBox_offset->setValue((int)(params["offset"].getVal<double>()*100.0+0.5));
    ui.horizontalSlider_offset->setValue(ui.spinBox_offset->value());

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

void DockWidgetPGRFlyCapture::on_spinBox_gain_editingFinished()
{
    ui.horizontalSlider_gain->setValue(ui.spinBox_gain->value());
    emit GainPropertiesChanged( ui.spinBox_gain->value()/100.0);
}

void DockWidgetPGRFlyCapture::on_horizontalSlider_gain_sliderMoved(int d)
{
    ui.spinBox_gain->setValue(d);
    emit GainPropertiesChanged(ui.spinBox_gain->value()/100.0);
}

void DockWidgetPGRFlyCapture::on_spinBox_offset_editingFinished()
{
    ui.horizontalSlider_offset->setValue(ui.spinBox_offset->value());
    emit OffsetPropertiesChanged(ui.spinBox_offset->value()/100.0);
}

void DockWidgetPGRFlyCapture::on_horizontalSlider_offset_sliderMoved(int d)
{
    ui.spinBox_offset->setValue(d);
    emit OffsetPropertiesChanged(ui.spinBox_offset->value()/100.0);
}

void DockWidgetPGRFlyCapture::on_doubleSpinBox_integration_time_editingFinished()
{
    emit IntegrationPropertiesChanged( ui.doubleSpinBox_integration_time->value() / 1000.0);
}



