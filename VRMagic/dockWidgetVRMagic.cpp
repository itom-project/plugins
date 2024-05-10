/* ********************************************************************
    Plugin "VRMagic" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2016, Institut für Technische Optik, Universität Stuttgart

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

#include "dockWidgetVRMagic.h"
#include "common/addInInterface.h"
#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetVRMagic::DockWidgetVRMagic(int uniqueID, ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
     ui.setupUi(this);
     identifierChanged(QString::number(uniqueID));
     enableWidget(true);
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVRMagic::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.sliderWidget_brightness->setDisabled(params["brightness"].getFlags() & ito::ParamBase::Readonly);
    ui.sliderWidget_brightness->setVisible(!(params["brightness"].getFlags() & ito::ParamBase::Readonly));
    ui.sliderWidget_contrast->setDisabled(params["contrast"].getFlags() & ito::ParamBase::Readonly);
    ui.sliderWidget_contrast->setVisible(!(params["contrast"].getFlags() & ito::ParamBase::Readonly));

    if (m_firstRun)
    {
        ui.label_sensor->setText(params["name"].getVal<char*>());
        ui.label_serial->setText(params["serial_number"].getVal<char*>());
        ui.label_width->setText(QString::number(params["sizex"].getVal<int>()));
        ui.label_height->setText(QString::number(params["sizey"].getVal<int>()));
        ui.label_bits->setText(QString::number(params["bpp"].getVal<int>()));
        //use params (identical to m_params of the plugin)
        //and initialize all widgets (e.g. min, max values, labels, enable some,...)

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        if (params.contains("brightness") & !(params["brightness"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::IntMeta *dm = (ito::IntMeta*)(params["brightness"].getMeta());
            ui.sliderWidget_brightness->setMinimum(dm->getMin());
            ui.sliderWidget_brightness->setMaximum(dm->getMax());
            ui.sliderWidget_brightness->setValue(params["brightness"].getVal<int>());
        }

        if (params.contains("contrast") & !(params["contrast"].getFlags() & ito::ParamBase::Readonly))
        {
            ito::IntMeta *dm = (ito::IntMeta*)(params["contrast"].getMeta());
            ui.sliderWidget_contrast->setMinimum(dm->getMin());
            ui.sliderWidget_contrast->setMaximum(dm->getMax());
            ui.sliderWidget_contrast->setValue(params["contrast"].getVal<int>());
        }
    }
    m_inEditing = false;

    m_currentParams = params;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVRMagic::on_sliderWidget_brightness_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("brightness",ito::ParamBase::Int,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVRMagic::on_sliderWidget_contrast_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrast",ito::ParamBase::Int,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVRMagic::enableWidget(bool enabled)
{
    ui.sliderWidget_brightness->setEnabled(enabled);
    ui.sliderWidget_contrast->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetVRMagic::identifierChanged(const QString &identifier)
{
    ui.label_ID->setText(identifier);
}
