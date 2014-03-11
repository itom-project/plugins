/* ********************************************************************
    Plugin "Vistek" for itom software
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

#include "dockWidgetVistek.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetVistek::DockWidgetVistek() : m_inEditing(false), m_exposureStep(0.001)
{
    ui.setupUi(this); 

    ui.exposureSpinBox->setKeyboardTracking(false);
    ui.gainSpinBox->setKeyboardTracking(false);
    ui.offsetSpinBox->setKeyboardTracking(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::valuesChanged(QMap<QString, ito::Param> params)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
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
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["exposure"].getMeta());
            ui.exposureSpinBox->setMinimum( dm->getMin() );
            ui.exposureSpinBox->setMaximum( dm->getMax() );
            ui.exposureSpinBox->setSingleStep( (dm->getMax() - dm->getMin()) / 100 );
            ui.exposureSpinBox->setValue( params["exposure"].getVal<double>() );
        }

        if (params.contains("gain"))
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.gainSpinBox->setMinimum( dm->getMin() );
            ui.gainSpinBox->setMaximum( dm->getMax() );
            ui.gainSpinBox->setSingleStep( (dm->getMax() - dm->getMin()) / 100 );
            ui.gainSpinBox->setValue( params["gain"].getVal<double>() );
        }

        if (params.contains("offset")) //already from 0.0 to 1.0 (in vistek driver this is 0..255)
        {
            ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["offset"].getMeta());
            ui.offsetSpinBox->setValue( params["offset"].getVal<double>() );
        }
        m_inEditing = false;
    }
     
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::on_exposureSpinBox_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        double steps = (val-ui.exposureSpinBox->minimum()) / m_exposureStep;
        double val2 = ui.exposureSpinBox->minimum() + qRound(steps) * m_exposureStep;

        if (qAbs(val-val2) > 0.000001)
        {
            val = val2;
            ui.exposureSpinBox->setValue(val);
        }

        emit ExposurePropertyChanged( val );
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::on_gainSpinBox_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        //gain does not have specific increment steps

        emit GainPropertyChanged( val );
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::on_offsetSpinBox_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;

        //gain does not have specific increment steps

        emit OffsetPropertyChanged( val );
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVistek::propertiesChanged(float gainIncrement, float exposureIncrement, VistekFeatures features)
{
    ui.gainSpinBox->setEnabled(features.adjustGain);
    ui.offsetSpinBox->setEnabled(features.adjustOffset);
    ui.exposureSpinBox->setEnabled(features.adjustExposureTime);
    //ui.gainSpinBox->setSingleStep(gainIncrement);
    m_exposureStep = exposureIncrement;
}