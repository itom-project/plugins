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


#include "DialogPCOSensicam.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

#include "cam_types.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogPCOSensicam::DialogPCOSensicam(ito::AddInBase *grabber, SC_Camera_Description &cameraDescription) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_camInfo(cameraDescription)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOSensicam::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());

        ui.comboGainMode->clear();
        ui.comboGainMode->addItem("normal analog gain", 0);
        ui.comboGainMode->addItem("extended analog gain", 1);
        if (params["gain_mode"].getMax() > 1)
        {
            ui.comboGainMode->addItem("low light mode", 3);
        }

        ui.comboTrigger->clear();
        ui.comboTrigger->addItem("Software (0)", 0);
        ui.comboTrigger->addItem("External Rising Edge (1)", 1);
        ui.comboTrigger->addItem("External Falling Edge (2)", 2);

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

        //set binning configurations
        ui.comboBinningX->clear();
        ui.comboBinningY->clear();

        ui.comboBinningX->addItem(QString::number(1), 1);
        ui.comboBinningY->addItem(QString::number(1), 1);

        m_firstRun = false;
    }

    //roi
    int *roi = params["roi"].getVal<int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    //fast mode
    ui.checkFastMode->setChecked(params["fast_mode"].getVal<int>() > 0 ? true : false);

    //gain_mode
    for (int i = 0; i < ui.comboGainMode->count(); ++i)
    {
        if (ui.comboGainMode->itemData(i).toInt() == params["gain_mode"].getVal<int>())
        {
            ui.comboGainMode->setCurrentIndex(i);
            break;
        }
    }

    //trigger
    for (int i = 0; i < ui.comboTrigger->count(); ++i)
    {
        if (ui.comboTrigger->itemData(i).toInt() == params["trigger"].getVal<int>())
        {
            ui.comboTrigger->setCurrentIndex(i);
            break;
        }
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

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPCOSensicam::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    bool changeX0 = false;
    bool changeX1 = false;
    bool changeY0 = false;
    bool changeY1 = false;

    if(ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
    {
        int x0, x1, y0, y1;
        ui.rangeX01->values(x0,x1);
        ui.rangeY01->values(y0,y1);
        int roi[] = {0,0,0,0};
        memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4*sizeof(int));

        if (roi[0] != x0 || roi[1] != y0 || roi[2] != (x1-x0+1) || roi[3] != (y1-y0+1))
        {
            roi[0] = x0;
            roi[1] = y0;
            roi[2] = x1-x0+1;
            roi[3] = y1-y0+1;
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
        }
    }

    int trigger = ui.comboTrigger->itemData(ui.comboTrigger->currentIndex()).toInt();
    if(m_currentParameters["trigger"].getVal<int>() != trigger)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger", ito::ParamBase::Int, trigger)));
    }

    int gain_mode = ui.comboGainMode->itemData(ui.comboGainMode->currentIndex()).toInt();
    if(m_currentParameters["gain_mode"].getVal<int>() != gain_mode)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain_mode", ito::ParamBase::Int, gain_mode)));
    }

    int fastmode = ui.checkFastMode->isChecked() ? 1 : 0;
    if(m_currentParameters["fast_mode"].getVal<int>() != fastmode)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("fast_mode", ito::ParamBase::Int, fastmode)));
    }

    double dval = ui.slider_delay->value() * exposureToSecFactor;
    if(qAbs(m_currentParameters["delay_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("delay_time", ito::ParamBase::Double, dval)));
    }

    dval = ui.slider_exposure->value() * exposureToSecFactor;
    if(qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPCOSensicam::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogPCOSensicam::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxAcquisition->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOSensicam::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOSensicam::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOSensicam::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOSensicam::on_checkFastMode_clicked(bool checked)
{
    double current_delay_sec = ui.slider_delay->value() * exposureToSecFactor;
    double current_exposure_sec = ui.slider_exposure->value() * exposureToSecFactor;

    switch (m_currentParameters["cam_type"].getVal<int>())
    {
        case FASTEXP: //"Fast Exposure"
        case FASTEXPQE: //"Fast Exposure QE"
            exposureToSecFactor = 1e-6;
            ui.slider_delay->setSuffix(" \u00B5s");  // mu s
            ui.slider_delay->setMinimum(0.0);
            ui.slider_delay->setMaximum(1000.0);
            ui.slider_delay->setSingleStep(0.1);
            ui.slider_delay->setDecimals(1);
            ui.slider_delay->setValue(current_delay_sec / exposureToSecFactor);

            ui.slider_exposure->setSuffix(" \u00B5s");  // mu s
            ui.slider_exposure->setMinimum(0.0);
            ui.slider_exposure->setMaximum(1000.0);
            ui.slider_exposure->setSingleStep(0.1);
            ui.slider_exposure->setDecimals(1);
            ui.slider_exposure->setValue(current_exposure_sec / exposureToSecFactor);
            break;
        case LONGEXPQE: //"Long Exposure QE"
            {
                if (checked)
                {
                    exposureToSecFactor = 1e-6;
                    ui.slider_delay->setSuffix(" \u00B5s");  // mu s
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(50000.0);
                    ui.slider_delay->setSingleStep(0.1);
                    ui.slider_delay->setDecimals(1);
                    ui.slider_delay->setValue(current_delay_sec / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" \u00B5s");  // mu s
                    ui.slider_exposure->setMinimum(0.5);
                    ui.slider_exposure->setMaximum(10000.0);
                    ui.slider_exposure->setSingleStep(0.1);
                    ui.slider_exposure->setDecimals(1);
                    ui.slider_exposure->setValue(current_exposure_sec / exposureToSecFactor);
                }
                else
                {
                    exposureToSecFactor = 1e-3;
                    ui.slider_delay->setSuffix(" ms");
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(1000000.0);
                    ui.slider_delay->setSingleStep(1);
                    ui.slider_delay->setDecimals(0);
                    ui.slider_delay->setValue(current_delay_sec / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" ms");
                    ui.slider_exposure->setMinimum(1.0);
                    ui.slider_exposure->setMaximum(1000000.0);
                    ui.slider_exposure->setSingleStep(1);
                    ui.slider_exposure->setDecimals(0);
                    ui.slider_exposure->setValue(current_exposure_sec / exposureToSecFactor);
                }
                break;
            }
        case OEM:
        case LONGEXP: //"Long Exposure"
        case LONGEXPI: //"Long Exposure special"
            {
                if (checked)
                {
                    exposureToSecFactor = 75.0 * (1e-6);
                    ui.slider_delay->setSuffix(" * 75 \u00B5s");  // mu s
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(200.0);
                    ui.slider_delay->setSingleStep(1.0);
                    ui.slider_delay->setDecimals(0);
                    ui.slider_delay->setValue(current_delay_sec / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" * 75 \u00B5s");  // mu s
                    ui.slider_exposure->setMinimum(1.0);
                    ui.slider_exposure->setMaximum(200.0);
                    ui.slider_exposure->setSingleStep(1.0);
                    ui.slider_exposure->setDecimals(0);
                    ui.slider_exposure->setValue(current_exposure_sec / exposureToSecFactor);
                }
                else
                {
                    exposureToSecFactor = 1e-3;
                    ui.slider_delay->setSuffix(" ms");
                    ui.slider_delay->setMinimum(0.0);
                    ui.slider_delay->setMaximum(1000000.0);
                    ui.slider_delay->setSingleStep(1);
                    ui.slider_delay->setDecimals(0);
                    ui.slider_delay->setValue(current_delay_sec / exposureToSecFactor);

                    ui.slider_exposure->setSuffix(" ms");
                    ui.slider_exposure->setMinimum(1.0);
                    ui.slider_exposure->setMaximum(1000000.0);
                    ui.slider_exposure->setSingleStep(1);
                    ui.slider_exposure->setDecimals(0);
                    ui.slider_exposure->setValue(current_exposure_sec / exposureToSecFactor);
                }
                break;
            }
    }
}
