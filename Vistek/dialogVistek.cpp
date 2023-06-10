/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#include "dialogVistek.h"
#include "Vistek.h"
#include <qmessagebox.h>
#include <qdialogbuttonbox.h>
#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogVistek::DialogVistek(Vistek *grabber, const VistekFeatures *features) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    m_features = new VistekFeatures(*features);
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogVistek::~DialogVistek()
{
    delete m_features;
    m_features = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVistek::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        //file information
        ui.lblModel->setText(params["cameraModel"].getVal<char*>());
        ui.lblSerialNo->setText(params["cameraSerialNo"].getVal<char*>());
        ui.lblCameraIP->setText(params["cameraIP"].getVal<char*>());
        ui.lblManufacturer->setText(params["cameraManufacturer"].getVal<char*>());

        m_firstRun = false;
    }

    ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
    ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
    ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());

    int *roi = params["roi"].getVal<int*>();
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(!(params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    ui.combo_bpp->clear();
    ui.combo_bpp->setEnabled(false);

    if (m_features->has8bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("8bit", 8);
    }
    if (m_features->has10bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("10bit", 10);
    }
    if (m_features->has12bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("12bit", 12);
    }
    if (m_features->has16bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("16bit", 16);
    }

    if (params.contains("bpp"))
    {
        for (int i = 0; i < ui.combo_bpp->count(); ++i)
        {
            if (ui.combo_bpp->itemData(i).toInt() == params["bpp"].getVal<int>())
            {
                ui.combo_bpp->setCurrentIndex(i);
                break;
            }
        }
    }

    ui.combo_binning->setEnabled(m_features->adjustBinning);
    if (params.contains("binning"))
    {
        ui.combo_binning->setCurrentIndex(params["binning"].getVal<int>());
    }

    ui.doubleSpinBox_integration_time->setEnabled(m_features->adjustExposureTime);
    if (params.contains("integration_time"))
    {
        ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());

        ui.doubleSpinBox_integration_time->setMinimum(dm->getMin() * 1000.0);
        ui.doubleSpinBox_integration_time->setMaximum(dm->getMax() * 1000.0);
        ui.doubleSpinBox_integration_time->setSingleStep((dm->getMax() - dm->getMin()) * 10.0);
        ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>()  * 1000.0);
    }

    ui.spinBox_gain->setEnabled(m_features->adjustGain);
    if (params.contains("gain"))
    {
        ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());

        ui.spinBox_gain->setMinimum(dm->getMin());
        ui.spinBox_gain->setMaximum(dm->getMax());
        ui.spinBox_gain->setSingleStep((dm->getMax() - dm->getMin()) / 100.0);
        ui.spinBox_gain->setValue(params["gain"].getVal<double>());
    }

    ui.spinBox_offset->setEnabled(m_features->adjustOffset);
    ui.horizontalSlider_offset->setEnabled(m_features->adjustOffset);
    if (params.contains("offset")) //already from 0.0 to 1.0 (in vistek driver this is 0..255)
    {
        ui.spinBox_offset->setValue((int)(params["offset"].getVal<double>() * 100));
    }
    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogVistek::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    if (ui.rangeX01->isEnabled() || ui.rangeY01->isEnabled())
    {
        int x0, x1, y0, y1;
        ui.rangeX01->values(x0, x1);
        ui.rangeY01->values(y0, y1);
        int roi[] = { 0, 0, 0, 0 };
        memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4 * sizeof(int));

        if (roi[0] != x0 || roi[1] != y0 || roi[2] != (x1 - x0 + 1) || roi[3] != (y1 - y0 + 1))
        {
            roi[0] = x0;
            roi[1] = y0;
            roi[2] = x1 - x0 + 1;
            roi[3] = y1 - y0 + 1;
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
        }
    }

    //binning
    if (m_features->adjustBinning && ui.combo_binning->currentIndex() != m_currentParameters["binning"].getVal<int>() && ui.combo_binning->currentIndex() >= 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, ui.combo_binning->currentIndex())));
    }

    //bpp
    if (ui.combo_bpp->count() > 0 && ui.combo_bpp->itemData(ui.combo_bpp->currentIndex()).toInt() != m_currentParameters["bpp"].getVal<int>())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, ui.combo_bpp->itemData(ui.combo_bpp->currentIndex()).toInt())));
    }

    //offset
    if (m_features->adjustOffset && ui.spinBox_offset->value() != (int)(m_currentParameters["offset"].getVal<double>() * 100))
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, ui.spinBox_offset->value() / 100.0)));
    }

    //gain
    if (m_features->adjustGain && qAbs(ui.spinBox_gain->value() - m_currentParameters["gain"].getVal<double>()) > 0.0001)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, ui.spinBox_gain->value())));
    }

    //integration_time
    if (m_features->adjustExposureTime && qAbs(ui.doubleSpinBox_integration_time->value() - m_currentParameters["integration_time"].getVal<double>()  * 1000.0) > 0.0001)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, ui.doubleSpinBox_integration_time->value() / 1000.0)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}


//---------------------------------------------------------------------------------------------------------------------
void DialogVistek::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogVistek::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVistek::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVistek::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//------------------------------------------------------------------------------
void DialogVistek::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
