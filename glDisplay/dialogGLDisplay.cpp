/* ********************************************************************
    Plugin "GLDisplay" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "dialogGLDisplay.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogGLDisplay::DialogGLDisplay(ito::AddInBase *plugin) :
    AbstractAddInConfigDialog(plugin),
    m_firstRun(true),
    m_inEditing(false)
{
    ui.setupUi(this);
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogGLDisplay::on_horizontalSlider_valueChanged(int value)
{
    if (!m_inEditing && !m_firstRun)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> v(new ito::ParamBase("currentIdx", ito::ParamBase::Int, value));

        if (m_currentParameters.contains("currentIdx") && m_currentParameters["currentIdx"].getVal<int>() != value)
        {
            setPluginParameter(v, msgLevelWarningAndError);
        }

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogGLDisplay::parametersChanged(QMap<QString, ito::Param> params)
{
    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        //this is the first time that parameters are sent to this dialog,
        //therefore you can add some initialization work here
        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;

        int numImages = params["numImages"].getVal<int>();
        ui.label_imgend->setText(QString::number(numImages - 1));

        ui.horizontalSlider->setMinimum(0);
        ui.horizontalSlider->setMaximum(numImages - 1);
        ui.horizontalSlider->setValue(params["currentIdx"].getVal<int>());

        //set the status of all widgets depending on the values of params
        ui.spinBox_x0->setValue(params["x0"].getVal<int>());
        ui.spinBox_y0->setValue(params["y0"].getVal<int>());
        ui.spinBox_xsize->setValue(params["xsize"].getVal<int>());
        ui.spinBox_ysize->setValue(params["ysize"].getVal<int>());

        ui.comboBox_color->setCurrentIndex(params["color"].getVal<int>());
        ui.checkBox_gamma->setChecked(params["gamma"].getVal<int>());

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogGLDisplay::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    ito::RetVal retval = ito::retOk;

    int x0 = ui.spinBox_x0->value();
    //x0
    if (m_currentParameters["x0"].getVal<int>() != ui.spinBox_x0->value())
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, ui.spinBox_x0->value())));
    }

    int y0 = ui.spinBox_y0->value();
    //y0
    if (m_currentParameters["y0"].getVal<int>() != ui.spinBox_y0->value())
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, ui.spinBox_y0->value())));
    }

    int xsize = ui.spinBox_xsize->value();
    //xsize
    if (m_currentParameters["xsize"].getVal<int>() != ui.spinBox_xsize->value())
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("xsize", ito::ParamBase::Int, ui.spinBox_xsize->value())));
    }

    int ysize = ui.spinBox_ysize->value();
    //ysize
    if (m_currentParameters["ysize"].getVal<int>() != ui.spinBox_ysize->value())
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("ysize", ito::ParamBase::Int, ui.spinBox_ysize->value())));
    }

    //color
    if (m_currentParameters["color"].getVal<int>() != ui.comboBox_color->currentIndex())
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("color", ito::ParamBase::Int, ui.comboBox_color->currentIndex())));
    }

    //gamma
    if ((m_currentParameters["gamma"].getVal<int>() > 0) != ui.checkBox_gamma->isChecked())
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("gamma", ito::ParamBase::Int, ui.checkBox_gamma->isChecked() ? 1 : 0)));
    }

    //currentIdx
    if ((ui.horizontalSlider->value() >= 0) && (m_currentParameters["currentIdx"].getVal<int>() != ui.horizontalSlider->value()))
    {
        values.append( QSharedPointer<ito::ParamBase>(new ito::ParamBase("currentIdx", ito::ParamBase::Int, ui.horizontalSlider->value())));
    }


    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogGLDisplay::on_buttonBox_clicked(QAbstractButton* btn)
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
