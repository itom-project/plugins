/* ********************************************************************
    Plugin "V4L2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "dialogV4L2.h"
#include "V4L2.h"

#include "common/addInInterface.h"

#include <qmetaobject.h>
#include <qdialogbuttonbox.h>
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogV4L2::DialogV4L2(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogV4L2::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ui.spinSizeX->setEnabled(false);  // readonly
        ui.spinSizeY->setEnabled(false);  // readonly

        ito::StringMeta* sm = (ito::StringMeta*)(params["colorMode"].getMeta());
        for (int x = 0; x < sm->getLen(); x++)
        {
            ui.comboColorMode->addItem(sm->getString(x));
        }
        m_firstRun = false;
    }

    ui.comboColorMode->setCurrentIndex(ui.comboColorMode->findText(params["colorMode"].getVal<char*>()));

    ui.spinX0->setMaximum(params["x1"].getMax());
    ui.spinX0->setValue(params["x0"].getVal<int>());

    ui.spinY0->setMaximum(params["y1"].getMax());
    ui.spinY0->setValue(params["y0"].getVal<int>());

    ui.spinX1->setMinimum(params["x0"].getVal<int>());
    ui.spinX1->setMaximum(params["x1"].getMax());
    ui.spinX1->setValue(params["x1"].getVal<int>());

    ui.spinY1->setMinimum(params["y0"].getVal<int>());
    ui.spinY1->setMaximum(params["y1"].getMax());
    ui.spinY1->setValue(params["y1"].getVal<int>());

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());
    ui.spinSizeY->setValue(params["sizey"].getVal<int>());

    //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
    enableDialog(true);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogV4L2::on_spinX0_valueChanged(int i)
{
    ui.spinX1->setMinimum(i);
    ui.spinSizeX->setValue(ui.spinX1->value() - i + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogV4L2::on_spinX1_valueChanged(int i)
{
    ui.spinX0->setMaximum(i);
    ui.spinSizeX->setValue(i - ui.spinX0->value() + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogV4L2::on_spinY0_valueChanged(int i)
{
    ui.spinY1->setMinimum(i);
    ui.spinSizeY->setValue(ui.spinY1->value() - i + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogV4L2::on_spinY1_valueChanged(int i)
{
    ui.spinY0->setMaximum(i);
    ui.spinSizeY->setValue(i - ui.spinY0->value() + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogV4L2::on_btnSetFullROI_clicked()
{
    ui.spinX0->setValue(0);
    ui.spinX0->setMaximum(ui.spinX1->maximum());
    ui.spinX1->setMinimum(0);
    ui.spinX1->setValue(ui.spinX1->maximum());
    ui.spinSizeX->setValue(ui.spinX1->value() + 1);

    ui.spinY0->setValue(0);
    ui.spinY0->setMaximum(ui.spinY1->maximum());
    ui.spinY1->setMinimum(0);
    ui.spinY1->setValue(ui.spinY1->maximum());
    ui.spinSizeY->setValue(ui.spinY1->value() + 1);
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogV4L2::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    //only send parameters which are changed

    if (QString::compare(m_currentParameters["colorMode"].getVal<char*>(), ui.comboColorMode->currentText()) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("colorMode", ito::ParamBase::String, ui.comboColorMode->currentText().toLatin1().data())));
    }

    int i = ui.spinX0->value();
    if (m_currentParameters["x0"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, i)));
    }

    i = ui.spinY0->value();
    if (m_currentParameters["y0"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, i)));
    }

    i = ui.spinX1->value();
    if (m_currentParameters["x1"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, i)));
    }

    i = ui.spinY1->value();
    if (m_currentParameters["y1"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, i)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogV4L2::on_buttonBox_clicked(QAbstractButton* btn)
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
void DialogV4L2::enableDialog(bool enabled)
{
    ui.groupColorMode->setEnabled(enabled);
    ui.groupROI->setEnabled(enabled);
}
