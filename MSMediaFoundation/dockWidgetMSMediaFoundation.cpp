/* ********************************************************************
    Plugin "MSMediaFoundation" for itom software
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

#include "dockWidgetMSMediaFoundation.h"


#include <qmetaobject.h>

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetMSMediaFoundation::DockWidgetMSMediaFoundation(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);

    ui.sW_Brightness->setTracking(false);
    ui.sW_Contrast->setTracking(false);
    ui.sW_Gain->setTracking(false);
    ui.sW_Saturation->setTracking(false);
    ui.sW_Sharpness->setTracking(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::parametersChanged(QMap<QString, ito::Param> params)
{
//qDebug() << "----------------- valuesChanged m_firstRun: " << m_firstRun << "; m_inEditing: " << m_inEditing;
    if (m_firstRun && !m_inEditing)
    {
        m_inEditing = true;

        int propCount = 0;
        if (params.contains("brightness"))
        {
            ui.lB_Brightness->setVisible(true);
            ui.cB_Brightness->setVisible(true);
            ui.sW_Brightness->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["brightness"].getMeta());
            ui.sW_Brightness->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Brightness->setVisible(false);
            ui.cB_Brightness->setVisible(false);
            ui.sW_Brightness->setVisible(false);
        }

        if (params.contains("contrast"))
        {
            ui.lB_Contrast->setVisible(true);
            ui.cB_Contrast->setVisible(true);
            ui.sW_Contrast->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["contrast"].getMeta());
            ui.sW_Contrast->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Contrast->setVisible(false);
            ui.cB_Contrast->setVisible(false);
            ui.sW_Contrast->setVisible(false);
        }

        if (params.contains("gain"))
        {
            ui.lB_Gain->setVisible(true);
            ui.cB_Gain->setVisible(true);
            ui.sW_Gain->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sW_Gain->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Gain->setVisible(false);
            ui.cB_Gain->setVisible(false);
            ui.sW_Gain->setVisible(false);
        }

        if (params.contains("saturation"))
        {
            ui.lB_Saturation->setVisible(true);
            ui.cB_Saturation->setVisible(true);
            ui.sW_Saturation->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["saturation"].getMeta());
            ui.sW_Saturation->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Saturation->setVisible(false);
            ui.cB_Saturation->setVisible(false);
            ui.sW_Saturation->setVisible(false);
        }

        if (params.contains("sharpness"))
        {
            ui.lB_Sharpness->setVisible(true);
            ui.cB_Sharpness->setVisible(true);
            ui.sW_Sharpness->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["sharpness"].getMeta());
            ui.sW_Sharpness->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Sharpness->setVisible(false);
            ui.cB_Sharpness->setVisible(false);
            ui.sW_Sharpness->setVisible(false);
        }

        if (params.contains("integrationTime"))
        {
            ui.lB_IntegrationTime->setVisible(true);
            ui.cB_IntegrationTime->setVisible(true);
            ui.combo_IntegrationTime->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["integrationTime"].getMeta());
            int minimum = qRound(log10(dm->getMin())/log10(2.0));
            int maximum = qRound(std::log10(dm->getMax())/std::log10(2.0));
            ui.combo_IntegrationTime->clear();
            for (int i = minimum; i <= maximum; ++i)
            {
                ui.combo_IntegrationTime->addItem(QString("%1 s").arg(pow(2.0f, i)), i);
            }
            ++propCount;
        }
        else
        {
            ui.lB_IntegrationTime->setVisible(false);
            ui.cB_IntegrationTime->setVisible(false);
            ui.combo_IntegrationTime->setVisible(false);
        }

        if (params.contains("sizex"))
        {
            ui.lblWidth->setText(QString("%1").arg(params["sizex"].getVal<int>()));
        }

        if (params.contains("sizey"))
        {
            ui.lblHeight->setText(QString("%1").arg(params["sizey"].getVal<int>()));
        }

        if (params.contains("bpp"))
        {
            ui.lblBitDepth->setText(QString("%1").arg(params["bpp"].getVal<int>()));
        }

        if (propCount == 0)
        {
            ui.groupBox_3->setVisible(false);
            setMinimumHeight(109);
        }
        else
        {
            setMinimumHeight((26 * propCount) + 129);
        }

        m_firstRun = false;
        m_inEditing = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        if (params.contains("brightness"))
        {
            ui.cB_Brightness->setChecked(params["brightnessAuto"].getVal<int>());
            ui.sW_Brightness->setValue(params["brightness"].getVal<double>());
            ui.sW_Brightness->setEnabled(params["brightnessAuto"].getVal<int>() == 0);
        }

        if (params.contains("contrast"))
        {
            ui.cB_Contrast->setChecked(params["contrastAuto"].getVal<int>());
            ui.sW_Contrast->setValue(params["contrast"].getVal<double>());
            ui.sW_Contrast->setEnabled(params["contrastAuto"].getVal<int>() == 0);
        }

        if (params.contains("gain"))
        {
            ui.cB_Gain->setChecked(params["gainAuto"].getVal<int>());
            ui.sW_Gain->setValue(params["gain"].getVal<double>());
            ui.sW_Gain->setEnabled(params["gainAuto"].getVal<int>() == 0);
        }

        if (params.contains("saturation"))
        {
            ui.cB_Saturation->setChecked(params["saturationAuto"].getVal<int>());
            ui.sW_Saturation->setValue(params["saturation"].getVal<double>());
            ui.sW_Saturation->setEnabled(params["saturationAuto"].getVal<int>() == 0);
        }

        if (params.contains("sharpness"))
        {
            ui.cB_Sharpness->setChecked(params["sharpnessAuto"].getVal<int>());
            ui.sW_Sharpness->setValue(params["sharpness"].getVal<double>());
            ui.sW_Sharpness->setEnabled(params["sharpnessAuto"].getVal<int>() == 0);
        }

        if (params.contains("integrationTime"))
        {
            ui.cB_IntegrationTime->setChecked(params["integrationTimeAuto"].getVal<int>());

            int index = qRound(log10(params["integrationTime"].getVal<double>())/log10(2.0));

            for (int i = 0; i < ui.combo_IntegrationTime->count(); ++i)
            {
                if (ui.combo_IntegrationTime->itemData(i).toInt() == index)
                {
                    ui.combo_IntegrationTime->setCurrentIndex(i);
                    break;
                }
            }
            ui.combo_IntegrationTime->setEnabled(params["integrationTimeAuto"].getVal<int>() == 0);
        }

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
double getStepValue(double value, double stepSize)
{
    int stepCount = (int)((value / stepSize) + .5);
    //qDebug() << "----------------- getStepValue stepCount: " << stepCount << "; alt: " << value / stepSize;  // getStepValue stepCount:  77.4818 ; alt:  77.4
    return stepSize * stepCount;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Brightness_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        d = getStepValue(d, ui.sW_Brightness->singleStep());
        ui.sW_Brightness->setValue(d);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("brightness",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Contrast_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        double d2 = getStepValue(d, ui.sW_Contrast->singleStep());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrast",ito::ParamBase::Double,d2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Gain_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        double d2 = getStepValue(d, ui.sW_Gain->singleStep());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,d2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Saturation_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        double d2 = getStepValue(d, ui.sW_Saturation->singleStep());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("saturation",ito::ParamBase::Double,d2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_sW_Sharpness_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        double d2 = getStepValue(d, ui.sW_Sharpness->singleStep());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("sharpness",ito::ParamBase::Double,d2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_combo_IntegrationTime_currentIndexChanged(int index)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integrationTime", ito::ParamBase::Double, std::pow(2.0, ui.combo_IntegrationTime->itemData(index).toDouble())));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Brightness_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Brightness->setEnabled(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("brightnessAuto",ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Contrast_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Contrast->setEnabled(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrastAuto",ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Gain_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Gain->setEnabled(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gainAuto",ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Saturation_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Saturation->setEnabled(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("saturationAuto",ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_Sharpness_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.sW_Sharpness->setEnabled(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("sharpnessAuto",ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::on_cB_IntegrationTime_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ui.combo_IntegrationTime->setEnabled(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integrationTimeAuto", ito::ParamBase::Int, checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetMSMediaFoundation::identifierChanged(const QString &identifier)
{
    ui.label_Identifier->setText(identifier);
}
