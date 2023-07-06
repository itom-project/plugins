/* ********************************************************************
    Plugin "ThorlabsDCxCam" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2016, Institut fuer Technische Optik, Universitaet Stuttgart

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

#include "dialogThorlabsDCxCam.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsDCxCam::DialogThorlabsDCxCam(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsDCxCam::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        const ito::StringMeta *triggerMeta = static_cast<ito::StringMeta*>(params["trigger_mode"].getMeta());

        ui.comboTriggerMode->clear();
        for (int i = 0; i < triggerMeta->getLen(); ++i)
        {
            ui.comboTriggerMode->addItem(triggerMeta->getString(i));
        }


        ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());
        ito::StringMeta *colorModeMeta = static_cast<ito::StringMeta*>(params["color_mode"].getMeta());
        ui.comboBppMode->clear();

        for (int i = bppMeta->getMin(); i <= bppMeta->getMax(); i+=2)
        {
            ui.comboBppMode->addItem(tr("gray %1 bit").arg(i), i);
        }

        for (int i = 0; i < colorModeMeta->getLen(); ++i)
        {
            if (strcmp(colorModeMeta->getString(i), "color") == 0)
            {
                ui.comboBppMode->addItem("RGB, 8bit", 0);
                break;
            }
        }

        ito::IntMeta *binMeta = static_cast<ito::IntMeta*>(params["binning"].getMeta());
        ui.comboBinHor->clear();
        ui.comboBinVer->clear();
        if (params["binning"].getFlags() & ito::ParamBase::Readonly)
        {
            ui.comboBinHor->setEnabled(false);
            ui.comboBinVer->setEnabled(false);
        }
        else
        {
            int maxBin = params["binning"].getMax();
            int maxVert = maxBin % 100;
            int maxHorz = maxBin / 100;

            ui.comboBinHor->setEnabled(true);
            ui.comboBinVer->setEnabled(true);
            ui.comboBinHor->addItem("1x", 1);
            if (maxHorz >= 2) ui.comboBinHor->addItem("2x", 2);
            if (maxHorz >= 3) ui.comboBinHor->addItem("3x", 3);
            if (maxHorz >= 4) ui.comboBinHor->addItem("4x", 4);
            if (maxHorz >= 5) ui.comboBinHor->addItem("5x", 5);
            if (maxHorz >= 6) ui.comboBinHor->addItem("6x", 6);
            if (maxHorz >= 8) ui.comboBinHor->addItem("8x", 8);
            if (maxHorz >= 16) ui.comboBinHor->addItem("16x", 16);
            ui.comboBinVer->addItem("1x", 1);
            if (maxVert >= 2) ui.comboBinVer->addItem("2x", 2);
            if (maxVert >= 3) ui.comboBinVer->addItem("3x", 3);
            if (maxVert >= 4) ui.comboBinVer->addItem("4x", 4);
            if (maxVert >= 5) ui.comboBinVer->addItem("5x", 5);
            if (maxVert >= 6) ui.comboBinVer->addItem("6x", 6);
            if (maxVert >= 8) ui.comboBinVer->addItem("8x", 8);
            if (maxVert >= 16) ui.comboBinVer->addItem("16x", 16);
        }

        m_firstRun = false;
    }

    ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
    ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
    ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());

    int *roi = params["roi"].getVal<int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    ui.checkGainBoost->setChecked(params["gain_boost_enabled"].getVal<int>() > 0);
    ui.checkGainBoost->setDisabled(params["gain_boost_enabled"].getFlags() & ito::ParamBase::Readonly);

    ui.checkLongIntegrationTime->setChecked(params["long_integration_time_enabled"].getVal<int>() > 0);
    ui.checkLongIntegrationTime->setDisabled(params["long_integration_time_enabled"].getFlags() & ito::ParamBase::Readonly);

    ui.checkAutoBlacklevel->setChecked(params["auto_blacklevel_enabled"].getVal<int>() > 0);
    ui.checkAutoBlacklevel->setDisabled(params["auto_blacklevel_enabled"].getFlags() & ito::ParamBase::Readonly);

    ito::DoubleMeta *dm = static_cast<ito::DoubleMeta*>(params["integration_time"].getMeta());
    ui.sliderIntegrationTime->setMinimum(dm->getMin());
    ui.sliderIntegrationTime->setMaximum(dm->getMax());
    ui.sliderIntegrationTime->setSingleStep(dm->getStepSize());
    ui.sliderIntegrationTime->setValue(params["integration_time"].getVal<double>());
    ui.sliderIntegrationTime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

    dm = static_cast<ito::DoubleMeta*>(params["frame_rate"].getMeta());
    ui.sliderFrameRate->setMinimum(dm->getMin());
    ui.sliderFrameRate->setMaximum(dm->getMax());
    ui.sliderFrameRate->setSingleStep(std::max(0.1, dm->getStepSize()));
    ui.sliderFrameRate->setValue(params["frame_rate"].getVal<double>());
    ui.sliderFrameRate->setEnabled(!(params["frame_rate"].getFlags() & ito::ParamBase::Readonly));

    ito::IntMeta *im = static_cast<ito::IntMeta*>(params["pixel_clock"].getMeta());
    ui.sliderPixelClock->setMinimum(im->getMin());
    ui.sliderPixelClock->setMaximum(im->getMax());
    ui.sliderPixelClock->setSingleStep(im->getStepSize());
    ui.sliderPixelClock->setValue(params["pixel_clock"].getVal<int>());
    ui.sliderPixelClock->setEnabled(!(params["pixel_clock"].getFlags() & ito::ParamBase::Readonly));

    double dval = params["gain"].getVal<double>();
    ui.sliderGain->setValue(dval*100.0);
    ui.sliderGain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    double *gain_rgb = params["gain_rgb"].getVal<double*>();
    ui.sliderGainRed->setValue(gain_rgb[0]*100.0);
    ui.sliderGainRed->setEnabled(!(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly));

    ui.sliderGainGreen->setValue(gain_rgb[1]*100.0);
    ui.sliderGainGreen->setEnabled(!(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly));

    ui.sliderGainBlue->setValue(gain_rgb[2]*100.0);
    ui.sliderGainBlue->setEnabled(!(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly));

    dval = params["offset"].getVal<double>();
    ui.sliderOffset->setValue(dval*100.0);
    ui.sliderOffset->setEnabled(!(params["offset"].getFlags() & ito::ParamBase::Readonly));

    int userData = 0;
    if (strcmp(params["color_mode"].getVal<char*>(), "gray") == 0)
    {
        userData = params["bpp"].getVal<int>();
    }

    for (int i = 0; i < ui.comboBppMode->count(); ++i)
    {
        if (ui.comboBppMode->itemData(i, Qt::UserRole).toInt() == userData)
        {
            ui.comboBppMode->setCurrentIndex(i);
            break;
        }
    }

    for (int i = 0; i < ui.comboTriggerMode->count(); ++i)
    {
        if (ui.comboTriggerMode->itemText(i) == params["trigger_mode"].getVal<char*>())
        {
            ui.comboTriggerMode->setCurrentIndex(i);
            break;
        }
    }

    int binVer = params["binning"].getVal<int>() % 100;
    int binHor = (params["binning"].getVal<int>() - binVer) % 100;

    for (int i = 0; i < ui.comboBinHor->count(); ++i)
    {
        if (ui.comboBinHor->itemData(i, Qt::UserRole).toInt() == binHor)
        {
            ui.comboBinHor->setCurrentIndex(i);
            break;
        }
    }

    for (int i = 0; i < ui.comboBinVer->count(); ++i)
    {
        if (ui.comboBinVer->itemData(i, Qt::UserRole).toInt() == binVer)
        {
            ui.comboBinVer->setCurrentIndex(i);
            break;
        }
    }

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsDCxCam::applyParameters()
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

    if (ui.comboBinHor->isEnabled() && ui.comboBinVer->isEnabled())
    {
        int bin = ui.comboBinHor->itemData(ui.comboBinHor->currentIndex()).toInt() * 100 + ui.comboBinVer->itemData(ui.comboBinVer->currentIndex()).toInt();
        if (m_currentParameters["binning"].getVal<int>() != bin)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, bin)));
        }
    }

    if (ui.sliderPixelClock->isEnabled())
    {
        int clock = ui.sliderPixelClock->value();
        if (m_currentParameters["pixel_clock"].getVal<int>() != clock)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("pixel_clock", ito::ParamBase::Int, clock)));
        }
    }

    if (ui.checkGainBoost->isEnabled())
    {
        int ival = ui.checkGainBoost->isChecked() ? 1 : 0;
        if (m_currentParameters["gain_boost_enabled"].getVal<int>() != ival)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain_boost_enabled", ito::ParamBase::Int, ival)));
        }
    }

    if (ui.checkAutoBlacklevel->isEnabled())
    {
        int ival = ui.checkAutoBlacklevel->isChecked() ? 1 : 0;
        if (m_currentParameters["auto_blacklevel_enabled"].getVal<int>() != ival)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("auto_blacklevel_enabled", ito::ParamBase::Int, ival)));
        }
    }

    if (ui.sliderGain->isEnabled())
    {
        double dval = ui.sliderGain->value()/100.0;
        if (qAbs(m_currentParameters["gain"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.sliderGainRed->isEnabled())
    {
        double dval[] = {ui.sliderGainRed->value()/100.0, ui.sliderGainGreen->value()/100.0, ui.sliderGainBlue->value()/100.0};
        const double *curdval = m_currentParameters["gain_rgb"].getVal<double*>();
        if (qAbs(dval[0] - curdval[0]) >= std::numeric_limits<double>::epsilon() ||
           qAbs(dval[1] - curdval[1]) >= std::numeric_limits<double>::epsilon() ||
           qAbs(dval[2] - curdval[2]) >= std::numeric_limits<double>::epsilon() )
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain_rgb", ito::ParamBase::DoubleArray, 3, dval)));
        }
    }

    if (ui.comboTriggerMode->isEnabled())
    {
        QString mode = ui.comboTriggerMode->currentText();
        if (mode != m_currentParameters["trigger_mode"].getVal<char*>())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::String, mode.toLatin1().data())));
        }
    }

    if (ui.sliderFrameRate->isEnabled())
    {
        double dval = ui.sliderFrameRate->value();
        if (qAbs(m_currentParameters["frame_rate"].getVal<double>() - dval) >= 0.00001) //the smallest range is 1musec, given by the number of decimals of the spin box. //std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_rate", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.sliderOffset->isEnabled())
    {
        double dval = ui.sliderOffset->value()/100.0;
        if (qAbs(m_currentParameters["offset"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.checkLongIntegrationTime->isEnabled())
    {
        int ival = ui.checkLongIntegrationTime->isChecked() ? 1 : 0;
        if (m_currentParameters["long_integration_time_enabled"].getVal<int>() != ival)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("long_integration_time_enabled", ito::ParamBase::Int, ival)));
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

    if (ui.comboBppMode->isEnabled())
    {
        int i = ui.comboBppMode->itemData(ui.comboBppMode->currentIndex()).toInt();
        int bpp;
        bool color = false;
        if (i == 0)
        {
            color = true;
            bpp = 8;
        }
        else
        {
            bpp = i;
        }

        if (m_currentParameters["bpp"].getVal<int>() !=  bpp)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
        }

        if (color && strcmp(m_currentParameters["color_mode"].getVal<char*>(), "color") != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("color_mode", ito::ParamBase::String, "color")));
        }

        if (!color && strcmp(m_currentParameters["color_mode"].getVal<char*>(), "gray") != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("color_mode", ito::ParamBase::String, "gray")));
        }
    }



    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsDCxCam::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogThorlabsDCxCam::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsDCxCam::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsDCxCam::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
    ui.spinSizeY->setValue(maxValue - minValue + 1);
#else
    int min_ = minValue;
    int max_ = maxValue;
    int stepOffset = static_cast<ito::IntMeta*>( m_currentParameters["y0"].getMeta() )->getStepSize();
    int imageOffset = static_cast<ito::IntMeta*>( m_currentParameters["sizey"].getMeta() )->getStepSize();
    int maxHeight = static_cast<ito::IntMeta*>( m_currentParameters["y1"].getMeta() )->getMax() + 1;

    if ((min_ % stepOffset) != 0)
    {
        min_ = stepOffset * qRound((float)min_ / (float)stepOffset);
        if (min_ >= max_)
        {
            min_ = stepOffset * floor((float)min_ / (float)stepOffset);
        }
    }
    min_ = qBound<int>(0, min_, max_);

    if (((max_ - min_ + 1) % imageOffset) != 0)
    {
        max_ = min_ - 1 + imageOffset * qRound((float)(max_ - min_ + 1) / (float)imageOffset);
    }

    max_ = qBound<int>(0, max_, maxHeight - 1);

    if (min_ != minValue || max_ != maxValue)
    {
        ui.rangeY01->setValues(min_,max_);
    }
    else
    {
        ui.spinSizeY->setValue(maxValue - minValue + 1);
    }
#endif
}

//------------------------------------------------------------------------------
void DialogThorlabsDCxCam::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
