/* ********************************************************************
    Plugin "PCOSensicam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut für Technische Optik (ITO),
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

#include "dockWidgetPCOSensicam.h"


#include <qmetaobject.h>
#include "cam_types.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetPCOSensicam::DockWidgetPCOSensicam(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPCOSensicam::parametersChanged(QMap<QString, ito::Param> params)
{

    ui.spinBpp->setValue(params["bpp"].getVal<int>());
    ui.spinWidth->setValue(params["sizex"].getVal<int>());
    ui.spinHeight->setValue(params["sizey"].getVal<int>());

    bool state = m_inEditing;
    m_inEditing = true;

    if (m_firstRun)
    {
        ui.comboGainMode->clear();
        ui.comboGainMode->addItem("normal analog gain", 0);
        ui.comboGainMode->addItem("extended analog gain", 1);
        if (params["gain_mode"].getMax() > 1)
        {
            ui.comboGainMode->addItem("low light mode", 3);
        }

        switch (params["cam_type"].getVal<int>())
        {
            case FASTEXP: //"Fast Exposure"
            case FASTEXPQE: //"Fast Exposure QE"
                ui.checkFastMode->setChecked(true);
                ui.checkFastMode->setEnabled(false);
                break;
            case LONGEXPQE: //"Long Exposure QE"
            case OEM:
            case LONGEXP: //"Long Exposure"
            case LONGEXPI: //"Long Exposure special"
                ui.checkFastMode->setChecked(false);
                ui.checkFastMode->setEnabled(true);
                break;
        }

        m_firstRun = false;
    }

    switch (params["cam_type"].getVal<int>())
    {
        case FASTEXP: //"Fast Exposure"
        case FASTEXPQE: //"Fast Exposure QE"
            exposureToSecFactor = 1e-6;
            ui.slider_delay->setSuffix(" \u00B5s");  // mu s
            ui.slider_delay->setMinimum(0.0);
            ui.slider_delay->setMaximum(1000.0);
            ui.slider_delay->setSingleStep(0.1);
            ui.slider_delay->setDecimals(1);
            ui.slider_delay->setValue(params["delay_time"].getVal<double>() / exposureToSecFactor);

            ui.slider_exposure->setSuffix(" \u00B5s");  // mu s
            ui.slider_exposure->setMinimum(0.0);
            ui.slider_exposure->setMaximum(1000.0);
            ui.slider_exposure->setSingleStep(0.1);
            ui.slider_exposure->setDecimals(1);
            ui.slider_exposure->setValue(params["integration_time"].getVal<double>() / exposureToSecFactor);
            break;
        case LONGEXPQE: //"Long Exposure QE"
            {
                if (params["fast_mode"].getVal<int>())
                {
                    exposureToSecFactor = 1e-6;
                    ui.slider_delay->setSuffix(" \u00B5s");  // mu s
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(50000.0);
                    ui.slider_delay->setSingleStep(0.1);
                    ui.slider_delay->setDecimals(1);
                    ui.slider_delay->setValue(params["delay_time"].getVal<double>() / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" \u00B5s");  // mu s
                    ui.slider_exposure->setMinimum(0.5);
                    ui.slider_exposure->setMaximum(10000.0);
                    ui.slider_exposure->setSingleStep(0.1);
                    ui.slider_exposure->setDecimals(1);
                    ui.slider_exposure->setValue(params["integration_time"].getVal<double>() / exposureToSecFactor);
                }
                else
                {
                    exposureToSecFactor = 1e-3;
                    ui.slider_delay->setSuffix(" ms");
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(1000000.0);
                    ui.slider_delay->setSingleStep(1);
                    ui.slider_delay->setDecimals(0);
                    ui.slider_delay->setValue(params["delay_time"].getVal<double>() / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" ms");
                    ui.slider_exposure->setMinimum(1.0);
                    ui.slider_exposure->setMaximum(1000000.0);
                    ui.slider_exposure->setSingleStep(1);
                    ui.slider_exposure->setDecimals(0);
                    ui.slider_exposure->setValue(params["integration_time"].getVal<double>() / exposureToSecFactor);
                }
                break;
            }
        case OEM:
        case LONGEXP: //"Long Exposure"
        case LONGEXPI: //"Long Exposure special"
            {
                if (params["fast_mode"].getVal<int>())
                {
                    exposureToSecFactor = 75.0 * (1e-6);
                    ui.slider_delay->setSuffix(" * 75 \u00B5s");  // mu s
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(200.0);
                    ui.slider_delay->setSingleStep(1.0);
                    ui.slider_delay->setDecimals(0);
                    ui.slider_delay->setValue(params["delay_time"].getVal<double>() / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" * 75 \u00B5s");  // mu s
                    ui.slider_exposure->setMinimum(1.0);
                    ui.slider_exposure->setMaximum(200.0);
                    ui.slider_exposure->setSingleStep(1.0);
                    ui.slider_exposure->setDecimals(0);
                    ui.slider_exposure->setValue(params["integration_time"].getVal<double>() / exposureToSecFactor);
                }
                else
                {
                    exposureToSecFactor = 1e-3;
                    ui.slider_delay->setSuffix(" ms");
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(1000000.0);
                    ui.slider_delay->setSingleStep(1);
                    ui.slider_delay->setDecimals(0);
                    ui.slider_delay->setValue(params["delay_time"].getVal<double>() / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" ms");
                    ui.slider_exposure->setMinimum(1.0);
                    ui.slider_exposure->setMaximum(1000000.0);
                    ui.slider_exposure->setSingleStep(1);
                    ui.slider_exposure->setDecimals(0);
                    ui.slider_exposure->setValue(params["integration_time"].getVal<double>() / exposureToSecFactor);
                }
                break;
            }
    }

    ui.checkFastMode->setChecked(params["fast_mode"].getVal<int>() > 0 ? true : false);

    for (int i = 0; i < ui.comboGainMode->count(); ++i)
    {
        if (ui.comboGainMode->itemData(i).toInt() == params["gain_mode"].getVal<int>())
        {
            ui.comboGainMode->setCurrentIndex(i);
            break;
        }
    }

    m_inEditing = state;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPCOSensicam::on_checkFastMode_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("fast_mode",ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPCOSensicam::on_comboGainMode_currentIndexChanged(int index)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain_mode",ito::ParamBase::Int, ui.comboGainMode->itemData(index).toInt()));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPCOSensicam::on_slider_exposure_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,d * exposureToSecFactor));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPCOSensicam::on_slider_delay_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("delay_time",ito::ParamBase::Double,d * exposureToSecFactor));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPCOSensicam::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}
