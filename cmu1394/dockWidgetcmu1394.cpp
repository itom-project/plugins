/* ********************************************************************
    Plugin "cmu1394" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#include "dockWidgetCMU1394.h"

 DockWidgetCMU1394::DockWidgetCMU1394(QMap<QString, ito::Param> params, int uniqueID) :updating(0)
 {
    ui.setupUi(this); 
    
    char* temp = params["name"].getVal<char*>(); //borrowed reference
//    ui.lblName->setText(temp);
    ui.lblID->setText(QString::number(uniqueID));

    valuesChanged(params);
 }

 void DockWidgetCMU1394::valuesChanged(QMap<QString, ito::Param> params)
 {
    double offset = 1.0;
    double gain = 1.0;

    if(params.keys().contains("offset"))
        offset = params["offset"].getVal<double>();

    if(params.keys().contains("gain"))
        gain = params["gain"].getVal<double>();


    if (!updating)
    {
        updating = 1;
        ui.doubleSpinBoxOffset->setValue(offset);
        ui.doubleSpinBoxGain->setValue(gain);

        ui.horizontalSliderGain->setValue((int)(gain*100+0.5));
        ui.horizontalSliderOffset->setValue((int)(offset*100+0.5));
        updating = 0;
    }
 }

void DockWidgetCMU1394::on_doubleSpinBoxGain_editingFinished()
{
    double gain = ui.doubleSpinBoxGain->value();

//    ui.horizontalSliderGain->setValue((int)(ui.doubleSpinBoxGain->value()*100+0.5));
    
    QMap<QString, ito::ParamBase> paramsVals;
    ito::ParamBase param("gain", ito::ParamBase::Double, gain);
    paramsVals.insert(param.getName(), param);

    if (!updating)
    {
        updating = 1;
        ui.horizontalSliderGain->setValue(gain * 100.0);
        emit changeParameters(paramsVals);
        updating = 0;
    }
}

void DockWidgetCMU1394::on_doubleSpinBoxOffset_editingFinished()
{
    double offset = ui.doubleSpinBoxOffset->value();

//    ui.horizontalSliderOffset->setValue((int)(ui.doubleSpinBoxOffset->value()*100+0.5));

    QMap<QString, ito::ParamBase> paramsVals;
    ito::ParamBase param("offset", ito::ParamBase::Double, offset);
    paramsVals.insert(param.getName(), param);

    if (!updating)
    {
        updating = 1;
        ui.horizontalSliderOffset->setValue(offset * 100.0);
        emit changeParameters(paramsVals);
        updating = 0;
    }
}

void DockWidgetCMU1394::on_horizontalSliderGain_valueChanged(int Value)
{
    double gain = ui.horizontalSliderGain->value()/100.0;
    QMap<QString, ito::ParamBase> paramsVals;
    ito::ParamBase param("gain", ito::ParamBase::Double, gain);
    paramsVals.insert(param.getName(), param);

    if (!updating)
    {
        updating = 1;
        ui.doubleSpinBoxGain->setValue(gain);
        emit changeParameters(paramsVals);
        updating = 0;
    }
}

void DockWidgetCMU1394::on_horizontalSliderOffset_valueChanged(int Value)
{
    double offset = ui.horizontalSliderOffset->value()/100.0;
    QMap<QString, ito::ParamBase> paramsVals;
    ito::ParamBase param("offset", ito::ParamBase::Double, offset);
    ui.doubleSpinBoxOffset->setValue(offset);

    if (!updating)
    {
        updating = 1;
        paramsVals.insert(param.getName(), param);
        emit changeParameters(paramsVals);
        updating = 0;
    }
}