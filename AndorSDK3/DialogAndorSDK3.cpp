/* ********************************************************************
    Plugin "AndorSDK3" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#include "DialogAndorSDK3.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogSDK3::DialogSDK3(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_currentSizeMaxX(-1),
    m_currentSizeMaxY(-1)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogSDK3::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;
    ParamMapIterator it;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<const char*>()) + " - " + tr("Configuration Dialog"));

        if (params["full_aoi_control"].getVal<int>() > 0)
        {
            ui.lblFullAoiControl->setText("full control of ROI is available");
        }
        else
        {
            ui.lblFullAoiControl->setText("camera does not support full control of ROI");
        }

        ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());
        ito::StringMeta *colorModeMeta = static_cast<ito::StringMeta*>(params["color_mode"].getMeta());
        ui.comboBpp->clear();

        for (int i = bppMeta->getMin(); i <= bppMeta->getMax(); i+= std::max(1, bppMeta->getStepSize()))
        {
            ui.comboBpp->addItem(QString("%1 bit").arg(i), i);
        }

        it = params.find("binning");
        ito::IntMeta *binMeta = static_cast<ito::IntMeta*>(it->getMeta());
        int binMin = it->getMin() / 100;
        int binMax = it->getMax() / 100;
        ui.comboBinning->clear();
        if (it->getFlags() & ito::ParamBase::Readonly)
        {
            ui.comboBinning->setEnabled(false);
        }
        else
        {
            ui.comboBinning->setEnabled(true);
            for (int i = binMin; i <= binMax; ++i)
            {
                if (i != 1 && i != 2 && i != 3 && i != 4 && i != 8) continue;
                ui.comboBinning->addItem(QString("%1x%1").arg(i), i);
            }
        }

        it = params.find("trigger_mode");
        ito::StringMeta *sm = static_cast<ito::StringMeta*>(it->getMeta());
        ui.comboTrigger->clear();
        if (it->getFlags() & ito::ParamBase::Readonly)
        {
            ui.comboTrigger->setEnabled(false);
        }
        else
        {
            ui.comboTrigger->setEnabled(true);
            for (int i = 0; i < sm->getLen(); ++i)
            {
                ui.comboTrigger->addItem(sm->getString(i));
            }
        }

        it = params.find("fan_speed");
        sm = static_cast<ito::StringMeta*>(it->getMeta());
        ui.comboFanSpeed->clear();
        if (it->getFlags() & ito::ParamBase::Readonly)
        {
            ui.comboFanSpeed->setEnabled(false);
        }
        else
        {
            ui.comboFanSpeed->setEnabled(true);
            for (int i = 0; i < sm->getLen(); ++i)
            {
                ui.comboFanSpeed->addItem(sm->getString(i));
            }
        }

        it = params.find("pixel_readout_rate");
        sm = static_cast<ito::StringMeta*>(it->getMeta());
        ui.comboPixelReadoutRate->clear();
        if (it->getFlags() & ito::ParamBase::Readonly)
        {
            ui.comboPixelReadoutRate->setEnabled(false);
        }
        else
        {
            ui.comboPixelReadoutRate->setEnabled(true);
            for (int i = 0; i < sm->getLen(); ++i)
            {
                ui.comboPixelReadoutRate->addItem(sm->getString(i));
            }
        }

        m_firstRun = false;
    }

    bool updateROIMeta = true;
    ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());

    if (m_currentSizeMaxY == rm->getHeightRangeMeta().getMax() &&
        m_currentSizeMaxX == rm->getWidthRangeMeta().getMax())
    {
        updateROIMeta = false;
    }

    if (updateROIMeta)
    {
        ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
        m_currentSizeMaxX = rm->getWidthRangeMeta().getMax();
        m_currentSizeMaxY = rm->getHeightRangeMeta().getMax();
    }

    bool updateSizeX = false;
    bool updateSizeY = false;

    const int *roi = params["roi"].getVal<const int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    ito::DoubleMeta *dm = static_cast<ito::DoubleMeta*>(params["integration_time"].getMeta());
    ui.sliderIntegrationTime->setMinimum(dm->getMin());
    ui.sliderIntegrationTime->setMaximum(dm->getMax());
    ui.sliderIntegrationTime->setSingleStep(dm->getStepSize());
    ui.sliderIntegrationTime->setValue(params["integration_time"].getVal<double>());
    ui.sliderIntegrationTime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

    ui.comboShutter->setCurrentIndex(params["electronic_shuttering_mode"].getVal<int>());

    it = params.find("timeout");
    ui.spinTimeout->setValue(it->getVal<double>());

    double dval = params["gain"].getVal<double>();
    ui.sliderGain->setValue(dval);
    ui.sliderGain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    int bpp = params["bpp"].getVal<int>();
    for (int i = 0; i < ui.comboBpp->count(); ++i)
    {
        if (ui.comboBpp->itemData(i, 32).toInt() == bpp)
        {
            ui.comboBpp->setCurrentIndex(i);
            break;
        }
    }

    int bin = params["binning"].getVal<int>() / 100;
    for (int i = 0; i < ui.comboBinning->count(); ++i)
    {
        if (ui.comboBinning->itemData(i, 32).toInt() == bin)
        {
            ui.comboBinning->setCurrentIndex(i);
            break;
        }
    }

    QString str = params["trigger_mode"].getVal<const char*>();
    for (int i = 0; i < ui.comboTrigger->count(); ++i)
    {
        if (ui.comboTrigger->itemText(i) == str)
        {
            ui.comboTrigger->setCurrentIndex(i);
            break;
        }
    }

    str = params["fan_speed"].getVal<const char*>();
    for (int i = 0; i < ui.comboFanSpeed->count(); ++i)
    {
        if (ui.comboFanSpeed->itemText(i) == str)
        {
            ui.comboFanSpeed->setCurrentIndex(i);
            break;
        }
    }

    str = params["pixel_readout_rate"].getVal<const char*>();
    for (int i = 0; i < ui.comboPixelReadoutRate->count(); ++i)
    {
        if (ui.comboPixelReadoutRate->itemText(i) == str)
        {
            ui.comboPixelReadoutRate->setCurrentIndex(i);
            break;
        }
    }

    ui.checkSensorCooling->setDisabled(params["sensor_cooling"].getFlags() & ito::ParamBase::Readonly);
    ui.checkSensorCooling->setChecked( params["sensor_cooling"].getVal<int>() > 0);

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogSDK3::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    if (ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
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

    if (ui.comboBinning->isEnabled())
    {
        int bin = ui.comboBinning->itemData(ui.comboBinning->currentIndex()).toInt() * 100 + ui.comboBinning->itemData(ui.comboBinning->currentIndex()).toInt();
        if (m_currentParameters["binning"].getVal<int>() != bin)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, bin)));
        }
    }

    if (ui.sliderGain->isEnabled())
    {
        double dval = ui.sliderGain->value();
        if (qAbs(m_currentParameters["gain"].getVal<double>() - dval) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.spinTimeout->isEnabled())
    {
        double dval = ui.spinTimeout->value();
        if (qAbs(m_currentParameters["timeout"].getVal<double>() - dval) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.sliderIntegrationTime->isEnabled())
    {
        double dval = ui.sliderIntegrationTime->value();
        if (qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= 0.00001) //the smallest range is 1musec, given by the number of decimals of the spin box. //std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.comboBpp->isEnabled())
    {
        int bpp = ui.comboBpp->itemData(ui.comboBpp->currentIndex()).toInt();

        if (m_currentParameters["bpp"].getVal<int>() !=  bpp)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
        }
    }

    if (ui.comboShutter->isEnabled())
    {
        if (m_currentParameters["electronic_shuttering_mode"].getVal<int>() != ui.comboShutter->currentIndex())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("electronic_shuttering_mode", ito::ParamBase::Int, ui.comboShutter->currentIndex())));
        }
    }

    if (ui.comboTrigger->isEnabled())
    {
        if (QString::compare(m_currentParameters["trigger_mode"].getVal<const char*>(),ui.comboTrigger->currentText()) != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::String, ui.comboTrigger->currentText().toLatin1().data())));
        }
    }

    if (ui.comboFanSpeed->isEnabled())
    {
        if (QString::compare(m_currentParameters["fan_speed"].getVal<const char*>(),ui.comboFanSpeed->currentText()) != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("fan_speed", ito::ParamBase::String, ui.comboFanSpeed->currentText().toLatin1().data())));
        }
    }

    if (ui.comboPixelReadoutRate->isEnabled())
    {
        if (QString::compare(m_currentParameters["pixel_readout_rate"].getVal<const char*>(),ui.comboPixelReadoutRate->currentText()) != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("pixel_readout_rate", ito::ParamBase::String, ui.comboPixelReadoutRate->currentText().toLatin1().data())));
        }
    }

    if (ui.checkSensorCooling->isEnabled())
    {
        if (m_currentParameters["sensor_cooling"].getVal<int>() == (ui.checkSensorCooling->isChecked() ? 1 : 0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sensor_cooling", ito::ParamBase::Int, (ui.checkSensorCooling->isChecked() ? 1 : 0))));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogSDK3::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogSDK3::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//------------------------------------------------------------------------------
void DialogSDK3::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
