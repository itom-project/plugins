/* ********************************************************************
    Plugin "PcoPixelFly" for itom software
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

#include "dialogPCOPixelFly.h"

#include <qdialogbuttonbox.h>



//----------------------------------------------------------------------------------------------------------------------------------
DialogPCOPixelFly::DialogPCOPixelFly(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOPixelFly::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        m_firstRun = false;
    }

	ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
    ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());
    ui.rangeY01->setLimitsFromIntervalMeta(rm->getHeightRangeMeta());

    int *roi = params["roi"].getVal<int*>();
    qDebug() << roi[0] << roi[1] << roi[2] << roi[3];
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeY01->setValues(roi[1], roi[1] + roi[3] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));
    ui.rangeY01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    ui.sliderIntegrationTime->setMinimum(params["integration_time"].getMin()*1000);
    ui.sliderIntegrationTime->setMaximum(params["integration_time"].getMax()*1000);
    ui.sliderIntegrationTime->setValue(params["integration_time"].getVal<double>()*1000);
    ui.sliderIntegrationTime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

    ui.checkGain->setChecked(params["gain"].getVal<double>() > 0.5);
    ui.checkGain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    ui.comboBpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));

    if (params["bpp"].getVal<int>() == 8)
    {
        ui.comboBpp->setCurrentIndex(0);

        ui.comboBppShift->setEnabled(!(params["shift_bits"].getFlags() & ito::ParamBase::Readonly));
        ui.comboBppShift->setCurrentIndex(params["shift_bits"].getVal<int>());
    }
    else
    {
        ui.comboBpp->setCurrentIndex(1);
        ui.comboBppShift->setCurrentIndex(0);
        ui.comboBppShift->setEnabled(false);
    }

    int ival = params["binning"].getVal<int>();
    int ivalY = ival % 100;
    int ivalX = (ival - ivalY) / 100;
    ui.comboBinningX->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));
    ui.comboBinningY->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));

    ui.comboBinningX->setCurrentIndex(ivalX == 1 ? 0 : 1);
    ui.comboBinningY->setCurrentIndex(ivalY == 1 ? 0 : 1);

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPCOPixelFly::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

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

    if(ui.checkGain->isEnabled())
    {
        double dval = ui.checkGain->isChecked() ? 1.0 : 0.0;
        if(qAbs(m_currentParameters["gain"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.sliderIntegrationTime->isEnabled())
    {
        double dval = ui.sliderIntegrationTime->value()/1000.0;
        if(qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.comboBpp->isEnabled())
    {
        int bppNew = (ui.comboBpp->currentIndex() == 1) ? 12 : 8;

        if(m_currentParameters["bpp"].getVal<int>() !=  bppNew)
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bppNew)));
        }

        if (ui.comboBppShift->isEnabled() || bppNew == 8)
        {
            int bppShiftNew = (bppNew == 8) ? ui.comboBppShift->currentIndex() : 0; //0:1x, 1:2x, 2:4x, ... 5:32x (only 8 bit mode, else 1x)

            if(m_currentParameters["shift_bits"].getVal<int>() !=  bppShiftNew)
            {
                values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("shift_bits", ito::ParamBase::Int, bppShiftNew)));
            }
        }
    }

    if (ui.comboBinningX->isEnabled())
    {
        int binning = (ui.comboBinningX->currentIndex()+1) * 100 + (ui.comboBinningY->currentIndex()+1);
        if((m_currentParameters["binning"].getVal<int>() !=  binning))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, binning)));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPCOPixelFly::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogPCOPixelFly::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOPixelFly::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOPixelFly::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOPixelFly::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
