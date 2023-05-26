/* ********************************************************************
    Plugin "PCOCamera" for itom software
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


#include "dialogPCOCamera.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

#include "common/addInInterface.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogPCOCamera::DialogPCOCamera(ito::AddInBase *grabber, PCO_Description &cameraDescription) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true),
    m_camInfo(cameraDescription)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOCamera::parametersChanged(QMap<QString, ito::Param> params)
{
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));



        ito::IntMeta *bppMeta = static_cast<ito::IntMeta*>(params["bpp"].getMeta());

        ui.combo_bpp->clear();
        int count = 0;
        for (int i = bppMeta->getMin(); i <= bppMeta->getMax(); i += bppMeta->getStepSize())
        {
            ui.combo_bpp->addItem(QString::number(i));
            ui.combo_bpp->setItemData(count++, i, 32);
        }

        //set binning configurations
        ui.comboBinningX->clear();
        ui.comboBinningY->clear();

        ui.comboBinningX->addItem(QString::number(1), 1);
        ui.comboBinningY->addItem(QString::number(1), 1);

        if (m_camInfo.wBinHorzSteppingDESC > 0) //linear
        {
            for (int i = 2; i <= m_camInfo.wMaxBinHorzDESC; ++i)
            {
                ui.comboBinningX->addItem(QString::number(i), i);
            }
        }
        else
        {
            for (int i = 2; i <= m_camInfo.wMaxBinHorzDESC; )
            {
                ui.comboBinningX->addItem(QString::number(i), i);
                i <<= 1;
            }
        }

        if (m_camInfo.wBinVertSteppingDESC > 0) //linear
        {
            for (int i = 2; i <= m_camInfo.wMaxBinVertDESC; ++i)
            {
                ui.comboBinningY->addItem(QString::number(i), i);
            }
        }
        else
        {
            for (int i = 2; i <= m_camInfo.wMaxBinVertDESC; )
            {
                ui.comboBinningY->addItem(QString::number(i), i);
                i <<= 1;
            }
        }


        ui.comboPixelrate->clear();

        for (int i = 0; i < 4; ++i)
        {
            if (m_camInfo.dwPixelRateDESC[i] != 0)
            {
                ui.comboPixelrate->addItem(QString("%1 Mhz").arg(m_camInfo.dwPixelRateDESC[i] / 1e6), (int)(m_camInfo.dwPixelRateDESC[i] / 1e6));
            }
        }

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

    ui.doubleSpinBox_integration_time->setMinimum(params["integration_time"].getMin()*1000);
    ui.doubleSpinBox_integration_time->setMaximum(params["integration_time"].getMax()*1000);
    ui.doubleSpinBox_integration_time->setValue(params["integration_time"].getVal<double>()*1000);
    ui.doubleSpinBox_integration_time->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

    double dval = params["gain"].getVal<double>();
    ui.sliderGain->setValue(dval*100.0);
    ui.sliderGain->setEnabled(!(params["gain"].getFlags() & ito::ParamBase::Readonly));

    dval = params["offset"].getVal<double>();
    ui.sliderOffset->setValue(dval*100.0);
    ui.sliderOffset->setEnabled(!(params["offset"].getFlags() & ito::ParamBase::Readonly));

    ui.combo_bpp->setEnabled(!(params["bpp"].getFlags() & ito::ParamBase::Readonly));

    for (int i = 0; i < ui.combo_bpp->count(); ++i)
    {
        if (ui.combo_bpp->itemData(i, 32).toInt() == params["bpp"].getVal<int>())
        {
            ui.combo_bpp->setCurrentIndex(i);
            break;
        }
    }

    int ival = params["binning"].getVal<int>();
    int ivalY = ival % 100;
    int ivalX = (ival - ivalY) / 100;
    ui.comboBinningX->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));
    ui.comboBinningY->setEnabled(!(params["binning"].getFlags() & ito::ParamBase::Readonly));

    int idx = ui.comboBinningX->findData(ivalX, Qt::UserRole);
    if (idx >= 0)
    {
        ui.comboBinningX->setCurrentIndex(idx);
    }

    idx = ui.comboBinningY->findData(ivalY, Qt::UserRole);
    if (idx >= 0)
    {
        ui.comboBinningY->setCurrentIndex(idx);
    }

    ui.comboPixelrate->setEnabled(!(params["pixelrate"].getFlags() & ito::ParamBase::Readonly));
    idx = ui.comboPixelrate->findData(params["pixelrate"].getVal<int>(), Qt::UserRole);
    if (idx >= 0)
    {
        ui.comboPixelrate->setCurrentIndex(idx);
    }


    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogPCOCamera::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

#if defined(ITOM_ADDININTERFACE_VERSION) && ITOM_ADDININTERFACE_VERSION > 0x010300
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
#else

    if(ui.rangeX01->isEnabled())
    {
        int x0;
        int x1;
        ui.rangeX01->values(x0,x1);

        if((m_currentParameters["x0"].getVal<int>() !=  x0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, x0)));
        }
        if((m_currentParameters["x1"].getVal<int>() !=  x1))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, x1)));
        }
    }

    if(ui.rangeY01->isEnabled())
    {
        int y0;
        int y1;
        ui.rangeY01->values(y0, y1);

        if((m_currentParameters["y0"].getVal<int>() !=  y0))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, y0)));
        }
        if((m_currentParameters["y1"].getVal<int>() !=  y1))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, y1)));
        }
    }
#endif

    if(ui.sliderGain->isEnabled())
    {
        double dval = ui.sliderGain->value()/100.0;
        if(qAbs(m_currentParameters["gain"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.sliderOffset->isEnabled())
    {
        double dval = ui.sliderOffset->value()/100.0;
        if(qAbs(m_currentParameters["offset"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.doubleSpinBox_integration_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_integration_time->value()/1000.0;
        if(qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.combo_bpp->isEnabled())
    {
        QVariant qvar = ui.combo_bpp->currentIndex();
        bool ok;
        int bppNew = ui.combo_bpp->itemData(ui.combo_bpp->currentIndex(), 32).toInt(&ok);

        if(ok && (bppNew > 0) && (m_currentParameters["bpp"].getVal<int>() !=  bppNew))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bppNew)));
        }
    }

    if (ui.comboBinningX->isEnabled())
    {
        int binning = ui.comboBinningX->itemData(ui.comboBinningX->currentIndex(), Qt::UserRole).toInt() * 100 + ui.comboBinningY->itemData(ui.comboBinningY->currentIndex(), Qt::UserRole).toInt();
        if((m_currentParameters["binning"].getVal<int>() !=  binning))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, binning)));
        }
    }

    if (ui.comboPixelrate->isEnabled())
    {
        int pixelrate = ui.comboPixelrate->itemData(ui.comboPixelrate->currentIndex(), Qt::UserRole).toInt();
        if((m_currentParameters["pixelrate"].getVal<int>() !=  pixelrate))
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("pixelrate", ito::ParamBase::Int, pixelrate)));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogPCOCamera::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogPCOCamera::enableDialog(bool enabled)
{
    ui.groupBoxBinning->setEnabled(enabled);
    ui.groupBoxIntegration->setEnabled(enabled);
    ui.groupBoxSize->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOCamera::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOCamera::on_rangeY01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeY->setValue(maxValue - minValue + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogPCOCamera::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex") && m_currentParameters.contains("sizey"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
        ui.rangeY01->setValues(0, m_currentParameters["sizey"].getMax());
    }
}
