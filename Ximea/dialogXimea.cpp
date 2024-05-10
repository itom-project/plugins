/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2018, Institut für Technische Optik, Universität Stuttgart

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

#include "dialogXimea.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

#include "xiApi.h"

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogXimea::DialogXimea(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_inEditing(false),
    timing_mode_changed(false)
{
    ui.setupUi(this);
    ui.tabWidget->setCurrentIndex(0);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//---------------------------------------------------------------------------------------------------------------------
void DialogXimea::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}
//---------------------------------------------------------------------------------------------------------------------
void DialogXimea::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        enableDialog(true);
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.label_sensor->setText(params["sensor_type"].getVal<char*>());
        ui.label_serial->setText(params["serial_number"].getVal<char*>());

        int binning = params["binning"].getVal<int>();
        ito::IntMeta *binningMeta = static_cast<ito::IntMeta*>(params["binning"].getMeta());
        ui.combo_bin->clear();
        for(int i = binningMeta->getMin(); i <= binningMeta->getMax(); i += 101)
        {
            if (i != 303)
            {
                ui.combo_bin->addItem(QString("%1x%1").arg(i % 100), i);
            }
        }
        ui.combo_bin->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));

        int bpp = params["bpp"].getVal<int>();
        int sensor_bitdepth = params["max_sensor_bitdepth"].getVal<int>();
        ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());
        ui.combo_bpp->clear();
        for (int i = bppMeta->getMin(); i <= qMin(sensor_bitdepth, bppMeta->getMax()); i += qMax(1, bppMeta->getStepSize()))
        {
            ui.combo_bpp->addItem(QString("%1").arg(i), i);
        }

        if (bppMeta->getMax() == 32)
        {
            ui.combo_bpp->addItem(QString("32bit, rgba"), 32);
        }

        for (int i = 0; i < ui.combo_bpp->count(); ++i)
        {
            ui.combo_bpp->setCurrentIndex(i);
            if (ui.combo_bpp->currentText().toInt() == bpp)
            {
                ui.combo_bpp->setCurrentIndex(i);
                break;
            }
        }

        ui.btnFullROI->setEnabled(true);

        ui.checkTimingMode->setEnabled(!(params["timing_mode"].getFlags() & ito::ParamBase::Readonly));
        ui.comboTriggerMode->setEnabled(!(params["trigger_mode"].getFlags() & ito::ParamBase::Readonly));
        ui.comboTriggerSelector->setEnabled(!(params["trigger_selector"].getFlags() & ito::ParamBase::Readonly));
        int numGPIs = params["gpi_mode"].getLen();
        ui.comboGPI1->setEnabled(numGPIs >= 1);
        ui.comboGPI2->setEnabled(numGPIs >= 2);
        ui.comboGPI3->setEnabled(numGPIs >= 3);
        ui.comboGPI4->setEnabled(numGPIs >= 4);

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        ui.combo_bpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));

        ito::DoubleMeta *gain = static_cast<ito::DoubleMeta*>(params["gain"].getMeta());
        ui.sliderWidget_Gain->setMinimum(gain->getMin() * 100.0);
        ui.sliderWidget_Gain->setMaximum(gain->getMax() * 100.0);
        ui.sliderWidget_Gain->setValue(params["gain"].getVal<double>() * 100.0);
        ui.sliderWidget_Gain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

        ito::DoubleMeta *gamma = static_cast<ito::DoubleMeta*>(params["gamma"].getMeta());
        ui.sliderWidget_Gamma->setMinimum(gamma->getMin() * 100.0);
        ui.sliderWidget_Gamma->setMaximum(gamma->getMax() * 100.0);
        ui.sliderWidget_Gamma->setValue(params["gamma"].getVal<double>() * 100.0);
        ui.sliderWidget_Gamma->setEnabled(!(params["gamma"].getFlags() & ito::ParamBase::Readonly));

        ito::DoubleMeta *gammaColor = static_cast<ito::DoubleMeta*>(params["gammaColor"].getMeta());
        ui.sliderWidget_GammaColor->setMinimum(gammaColor->getMin() * 100.0);
        ui.sliderWidget_GammaColor->setMaximum(gammaColor->getMax() * 100.0);
        ui.sliderWidget_GammaColor->setValue(params["gammaColor"].getVal<double>() * 100.0);
        ui.sliderWidget_GammaColor->setEnabled(!(params["gammaColor"].getFlags() & ito::ParamBase::Readonly));

        ito::DoubleMeta *sharpness = static_cast<ito::DoubleMeta*>(params["sharpness"].getMeta());
        ui.sliderWidget_Sharpness->setMinimum(sharpness->getMin() * 100.0);
        ui.sliderWidget_Sharpness->setMaximum(sharpness->getMax() * 100.0);
        ui.sliderWidget_Sharpness->setValue(params["sharpness"].getVal<double>() * 100.0);
        ui.sliderWidget_Sharpness->setEnabled(!(params["sharpness"].getFlags() & ito::ParamBase::Readonly));

        ito::DoubleMeta *integrationtime = static_cast<ito::DoubleMeta*>(params["integration_time"].getMeta());
        ui.sliderWidget_integrationtime->setMinimum(secToMsec(integrationtime->getMin()));
        ui.sliderWidget_integrationtime->setMaximum(secToMsec(integrationtime->getMax()));
        ui.sliderWidget_integrationtime->setValue((secToMsec(params["integration_time"].getVal<double>())));
        ui.sliderWidget_integrationtime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

        int timing_mode = params["timing_mode"].getVal<int>();
        ui.checkTimingMode->setChecked(timing_mode == XI_ACQ_TIMING_MODE_FRAME_RATE);
        ui.sliderWidget_framerate->setEnabled(timing_mode == XI_ACQ_TIMING_MODE_FRAME_RATE);

        ito::DoubleMeta *framerate = static_cast<ito::DoubleMeta*>(params["framerate"].getMeta());
        ui.sliderWidget_framerate->setMinimum(framerate->getMin());
        ui.sliderWidget_framerate->setMaximum(framerate->getMax());
        ui.sliderWidget_framerate->setValue(params["framerate"].getVal<double>());
        ui.sliderWidget_framerate->setEnabled(!(params["framerate"].getFlags() & ito::ParamBase::Readonly));

        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
        ui.rangeY->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());
        ui.rangeX->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly) && params["sizex"].getMax() > 1);
        ui.rangeY->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly) && params["sizey"].getMax() > 1);

        int *roi = params["roi"].getVal<int*>();
        ui.rangeX->setValues(roi[0], roi[0] + roi[2] - 1);
        ui.rangeY->setValues(roi[1], roi[1] + roi[3] - 1);

        ui.spinSizeX->setValue(params["sizex"].getVal<int>());
        ui.spinSizeY->setValue(params["sizey"].getVal<int>());

        int bin = params["binning"].getVal<int>();
        for(int i = 0; i < ui.combo_bin->count(); ++i)
        {
            if(ui.combo_bin->itemData(i, Qt::UserRole).toInt() == bin)
            {
                ui.combo_bin->setCurrentIndex(i);
                break;
            }
        }

        int bpp = params["bpp"].getVal<int>();
        for(int i = 0; i < ui.combo_bpp->count(); ++i)
        {
            if(ui.combo_bpp->itemData(i, 32).toInt() == bpp)
            {
                ui.combo_bpp->setCurrentIndex(i);
                break;
            }
        }

        const int *gpi_mode = params["gpi_mode"].getVal<int*>();
        switch (std::min(4,params["gpi_mode"].getLen()))
        {
        case 4:
            ui.comboGPI4->setCurrentIndex(gpi_mode[3]);
        case 3:
            ui.comboGPI3->setCurrentIndex(gpi_mode[2]);
        case 2:
            ui.comboGPI2->setCurrentIndex(gpi_mode[1]);
        case 1:
            ui.comboGPI1->setCurrentIndex(gpi_mode[0]);
        }

        ui.comboTriggerSelector->setCurrentIndex(params["trigger_selector"].getVal<int>());
        ui.comboTriggerMode->setCurrentIndex(params["trigger_mode"].getVal<int>());

        if (params["frame_burst_count"].getMax() > 1 && params["trigger_mode"].getVal<int>() > XI_TRG_OFF && params["trigger_selector"].getVal<int>() == XI_TRG_SEL_FRAME_BURST_START)
        {
            ui.sliderFrameBurst->setEnabled(true);
            ui.sliderFrameBurst->setMaximum(params["frame_burst_count"].getMax());
            ui.sliderFrameBurst->setValue(params["frame_burst_count"].getVal<int>());
        }
        else
        {
            ui.sliderFrameBurst->setEnabled(false);
            ui.sliderFrameBurst->setMaximum(params["frame_burst_count"].getMax());
            ui.sliderFrameBurst->setValue(1);
        }

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogXimea::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;

    //only send parameters which are changed
    if(ui.rangeX->isEnabled() || ui.rangeY->isEnabled())
    {
        int x0, x1, y0, y1;
        ui.rangeX->values(x0,x1);
        ui.rangeY->values(y0,y1);
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

    if (ui.combo_bin->isEnabled())
    {
        int bin = ui.combo_bin->itemData(ui.combo_bin->currentIndex()).toInt();
        if (m_currentParameters["binning"].getVal<int>() != bin)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, bin)));
        }
    }

    if (ui.combo_bpp->isEnabled())
    {
        int bpp = ui.combo_bpp->itemData(ui.combo_bpp->currentIndex()).toInt();
        if (m_currentParameters["bpp"].getVal<int>() != bpp)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bpp)));
        }
    }

    if (ui.comboTriggerMode->isEnabled())
    {
        int trigger_mode = ui.comboTriggerMode->currentIndex();
        if (m_currentParameters["trigger_mode"].getVal<int>() != trigger_mode)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_mode", ito::ParamBase::Int, trigger_mode)));
        }
    }

    if (ui.comboTriggerSelector->isEnabled())
    {
        int trigger_selector = ui.comboTriggerSelector->currentIndex();
        if (m_currentParameters["trigger_selector"].getVal<int>() != trigger_selector)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("trigger_selector", ito::ParamBase::Int, trigger_selector)));
        }
    }

    int frame_burst_count = (int)ui.sliderFrameBurst->value();
    if (m_currentParameters["frame_burst_count"].getVal<int>() != frame_burst_count)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_burst_count", ito::ParamBase::Int, frame_burst_count)));
    }

    if (ui.comboGPI1->isEnabled())
    {
        int gpi[] = {XI_GPI_OFF,XI_GPI_OFF,XI_GPI_OFF,XI_GPI_OFF};
        if (ui.comboGPI1->isEnabled())
        {
            gpi[0] = ui.comboGPI1->currentIndex();
        }
        if (ui.comboGPI2->isEnabled())
        {
            gpi[1] = ui.comboGPI2->currentIndex();
        }
        if (ui.comboGPI3->isEnabled())
        {
            gpi[2] = ui.comboGPI3->currentIndex();
        }
        if (ui.comboGPI4->isEnabled())
        {
            gpi[3] = ui.comboGPI4->currentIndex();
        }

        if (memcmp(m_currentParameters["gpi_mode"].getVal<int*>(), gpi, std::min(4, m_currentParameters["gpi_mode"].getLen()) * sizeof(int)) != 0)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gpi_mode", ito::ParamBase::IntArray, m_currentParameters["gpi_mode"].getLen(), gpi)));
        }
    }

    if (ui.checkTimingMode->isEnabled())
    {
        int timing_mode = (ui.checkTimingMode->isChecked() ? XI_ACQ_TIMING_MODE_FRAME_RATE : XI_ACQ_TIMING_MODE_FREE_RUN);
        if (timing_mode != m_currentParameters["timing_mode"].getVal<int>())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timing_mode", ito::ParamBase::Int, timing_mode)));
        }

        if (timing_mode == XI_ACQ_TIMING_MODE_FRAME_RATE && ui.sliderWidget_framerate->isEnabled())
        {
            double framerate = ui.sliderWidget_framerate->value();
            if (qAbs(m_currentParameters["framerate"].getVal<double>() - framerate) > std::numeric_limits<double>::epsilon())
            {
                values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("framerate", ito::ParamBase::Double, framerate)));
            }
        }
    }

    if(ui.sliderWidget_Gain->isEnabled())
    {
        double gain = ui.sliderWidget_Gain->value() / 100.0;
        if(qAbs(m_currentParameters["gain"].getVal<double>() - gain) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, gain)));
        }
    }

    if (ui.sliderWidget_Gamma->isEnabled())
    {
        double gamma = ui.sliderWidget_Gamma->value() / 100.0;
        if (qAbs(m_currentParameters["gamma"].getVal<double>() - gamma) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gamma", ito::ParamBase::Double, gamma)));
        }
    }

    if (ui.sliderWidget_GammaColor->isEnabled())
    {
        double gammaColor = ui.sliderWidget_GammaColor->value() / 100.0;
        if (qAbs(m_currentParameters["gammaColor"].getVal<double>() - gammaColor) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gammaColor", ito::ParamBase::Double, gammaColor)));
        }
    }

    if (ui.sliderWidget_Sharpness->isEnabled())
    {
        double sharpness = ui.sliderWidget_Sharpness->value() / 100.0;
        if (qAbs(m_currentParameters["sharpness"].getVal<double>() - sharpness) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sharpness", ito::ParamBase::Double, sharpness)));
        }
    }

    if(ui.sliderWidget_integrationtime->isEnabled())
    {
        double integrationtime = msecToSec(ui.sliderWidget_integrationtime->value());
        if (qAbs(m_currentParameters["integration_time"].getVal<double>() - integrationtime) > std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, integrationtime)));
        }
    }

    //check further parameters...

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}


//------------------------------------------------------------------------------
void DialogXimea::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("roi"))
    {
        ito::RectMeta *rm = static_cast<ito::RectMeta*>(m_currentParameters["roi"].getMeta());
        const ito::RangeMeta &height = rm->getHeightRangeMeta();
        const ito::RangeMeta &width = rm->getWidthRangeMeta();
        ui.rangeX->setValues(width.getMin(), width.getMax());
        ui.rangeY->setValues(height.getMin(), height.getMax());
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogXimea::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        if (timing_mode_changed)
        {
            QMessageBox::information(this, tr("timing mode already updated"), tr("You changed the parameter timing_mode. The changed value is automatically applied"));
        }

        reject(); //close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else
    {
        timing_mode_changed = false;
        applyParameters(); //ApplyRole
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void DialogXimea::on_rangeX_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogXimea::on_rangeY_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogXimea::on_checkTimingMode_clicked(bool checked)
{
    QVector<QSharedPointer<ito::ParamBase> > values;

    int timing_mode = (checked ? XI_ACQ_TIMING_MODE_FRAME_RATE : XI_ACQ_TIMING_MODE_FREE_RUN);
    if (timing_mode != m_currentParameters["timing_mode"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timing_mode", ito::ParamBase::Int, timing_mode)));
    }

    timing_mode_changed = true;

    setPluginParameters(values, msgLevelWarningAndError);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogXimea::on_comboTriggerSelector_currentIndexChanged(int index)
{
    if (m_currentParameters["frame_burst_count"].getMax() > 1 && ui.comboTriggerMode->currentIndex() > XI_TRG_OFF && index == XI_TRG_SEL_FRAME_BURST_START)
    {
        ui.sliderFrameBurst->setEnabled(true);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
    }
    else
    {
        ui.sliderFrameBurst->setEnabled(false);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
        ui.sliderFrameBurst->setValue(1);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogXimea::on_comboTriggerMode_currentIndexChanged(int index)
{
    if (m_currentParameters["frame_burst_count"].getMax() > 1 && index > XI_TRG_OFF && ui.comboTriggerSelector->currentIndex() == XI_TRG_SEL_FRAME_BURST_START)
    {
        ui.sliderFrameBurst->setEnabled(true);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
    }
    else
    {
        ui.sliderFrameBurst->setEnabled(false);
        ui.sliderFrameBurst->setMaximum(m_currentParameters["frame_burst_count"].getMax());
        ui.sliderFrameBurst->setValue(1);
    }
}
