/* ********************************************************************
    Plugin "Xeneth" for itom software
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

//#include "DockWidgetIDS.h"
//
////----------------------------------------------------------------------------------------------------------------------------------
//DockWidgetIDS::DockWidgetIDS(ito::AddInDataIO *grabber) :
//    AbstractAddInDockWidget(grabber),
//    m_inEditing(false),
//    m_firstRun(true)
//{
//    ui.setupUi(this);
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::parametersChanged(QMap<QString, ito::Param> params)
//{
//    int inEditing = m_inEditing;
//    m_inEditing = true;
//
//    ui.lblWidth->setText(QString::number(params["sizex"].getVal<int>()));
//    ui.lblHeight->setText(QString::number(params["sizey"].getVal<int>()));
//
//    ui.lblGain->setVisible( !(params["gain"].getFlags() & ito::ParamBase::Readonly) );
//    ui.sliderGain->setVisible( !(params["gain"].getFlags() & ito::ParamBase::Readonly) );
//    ui.lblGainRed->setVisible( !(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly) );
//    ui.sliderGainRed->setVisible( !(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly) );
//    ui.lblGainGreen->setVisible( !(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly) );
//    ui.sliderGainGreen->setVisible( !(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly) );
//    ui.lblGainBlue->setVisible( !(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly) );
//    ui.sliderGainBlue->setVisible( !(params["gain_rgb"].getFlags() & ito::ParamBase::Readonly) );
//    ui.sliderOffset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );
//    ui.sliderExposure->setDisabled( params["exposure"].getFlags() & ito::ParamBase::Readonly );
//
//    if (m_firstRun)
//    {
//        if (params.contains("cam_model"))
//        {
//            ui.lblModel->setText(params["cam_model"].getVal<char*>());
//        }
//
//        if (params.contains("sensor_type"))
//        {
//            ui.lblSensor->setText(params["sensor_type"].getVal<char*>());
//        }
//
//        if (params.contains("cam_id"))
//        {
//            ui.lblID->setText(QString::number(params["cam_id"].getVal<int>()));
//        }
//
//        m_firstRun = false;
//    }
//
//    if (strcmp(params["color_mode"].getVal<char*>(),"gray") == 0)
//    {
//        ui.lblBitDepth->setText(QString("gray (%1 bit)").arg(params["bpp"].getVal<int>()));
//    }
//    else
//    {
//        ui.lblBitDepth->setText(QString("RGB (8 bit)"));
//    }
//
//    if (params.contains("integration_time"))
//    {
//        ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());
//        ui.sliderExposure->setMinimum(dm->getMin());
//        ui.sliderExposure->setMaximum(dm->getMax());
//        if (dm->getStepSize() != 0)
//        {
//            ui.sliderExposure->setSingleStep(std::max(dm->getStepSize(), 0.00001)); //0.00001 is the minimal step of the spin box
//        }
//        else
//        {
//            ui.sliderExposure->setSingleStep((dm->getMax() - dm->getMin()) / 100);
//        }
//        ui.sliderExposure->setValue(params["integration_time"].getVal<double>());
//    }
//
//    if (params.contains("gain"))
//    {
//        ui.sliderGain->setValue(params["gain"].getVal<double>());
//    }
//
//    if (params.contains("gain_rgb"))
//    {
//        const double *vals = params["gain_rgb"].getVal<const double*>();
//        ui.sliderGainRed->setValue(vals[0]);
//        ui.sliderGainGreen->setValue(vals[1]);
//        ui.sliderGainBlue->setValue(vals[2]);
//    }
//
//    if (params.contains("offset"))
//    {
//        ui.sliderOffset->setValue(params["offset"].getVal<double>());
//    }
//
//    m_inEditing = inEditing;
//
//    m_currentParams = params;
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::on_sliderExposure_valueChanged(double value)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,value));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::on_sliderGain_valueChanged(double value)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,value));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::on_sliderGainRed_valueChanged(double value)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double values[] = {value, ui.sliderGainGreen->value(), ui.sliderGainBlue->value()};
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain_rgb",ito::ParamBase::DoubleArray, 3, values));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::on_sliderGainGreen_valueChanged(double value)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double values[] = {ui.sliderGainRed->value(), value, ui.sliderGainBlue->value()};
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain_rgb",ito::ParamBase::DoubleArray, 3, values));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::on_sliderGainBlue_valueChanged(double value)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double values[] = {ui.sliderGainRed->value(), ui.sliderGainGreen->value(), value};
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain_rgb",ito::ParamBase::DoubleArray, 3, values));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetIDS::on_sliderOffset_valueChanged(double value)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,value));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
