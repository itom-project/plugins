/* ********************************************************************
    Plugin "GLDisplay" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut für Technische Optik (ITO),
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

#include "dockWidgetGLDisplay.h"
#include "glDisplay.h"

#include <qmessagebox.h>
//#include "common/addInInterface.h"

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetGLDisplay::DockWidgetGLDisplay(ito::AddInDataIO *GLDisplay) :
    AbstractAddInDockWidget(GLDisplay),
    m_inEditing(false),
    m_numTextures(0)
{
    ui.setupUi(this);
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGLDisplay::parametersChanged(QMap<QString, ito::Param> params)
{
    m_numTextures = params["numImages"].getVal<int>();

    if (!m_inEditing)
    {
        m_inEditing = true;
        //check the value of all given parameters and adjust your widgets according to them (value only should be enough)

        ui.lblIdx->setText( QString("%1 of %2").arg(params["currentIdx"].getVal<int>()).arg(m_numTextures-1));
        ui.sliderIdx->setMaximum(m_numTextures-1);
        ui.sliderIdx->setValue(params["currentIdx"].getVal<int>());

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGLDisplay::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetGLDisplay::on_sliderIdx_valueChanged(int index)
{
    ui.lblIdx->setText( QString("%1 of %2").arg(index).arg(m_numTextures-1));

    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("currentIdx",ito::ParamBase::Int,index));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
