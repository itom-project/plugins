/* ********************************************************************
    Plugin "dispWindow" for itom software
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

#include "dialogDispWindow.h"
#include "projWindow.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qmessagebox.h>
#include <qsharedpointer.h>
#include <qvector.h>

//-------------------------------------------------------------------------------------
DialogDispWindow::DialogDispWindow(ito::AddInBase* grabber, PrjWindow* prjWindow) :
    AbstractAddInConfigDialog(grabber), m_pWindow(prjWindow), m_firstRun(true), m_inEditing(false)
{
    ui.setupUi(this);
};

//-------------------------------------------------------------------------------------
void DialogDispWindow::on_horizontalSlider_valueChanged(int value)
{
    if (!m_inEditing && !m_firstRun)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> v(new ito::ParamBase("numimg", ito::ParamBase::Int, value));

        if (m_currentParameters.contains("numimg"))
        {
            m_currentParameters["numimg"].setVal<int>(value);
        }

        setPluginParameter(v, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//-------------------------------------------------------------------------------------
void DialogDispWindow::parametersChanged(QMap<QString, ito::Param> params)
{
    // save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(
            QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        if (params.contains("period"))
        {
            ui.spinBox_period->setMinimum(params["period"].getMin());
            ui.spinBox_period->setMaximum(params["period"].getMax());
            ui.spinBox_period->setSingleStep(
                ((ito::IntMeta*)params["period"].getMeta())->getStepSize());
        }

        ui.spinBox_period->setMinimum(2);


        // this is the first time that parameters are sent to this dialog,
        // therefore you can add some initialization work here
        m_firstRun = false;
    }

    int numImages = m_pWindow->getNumImages();
    ui.label_imgend->setText(QString::number(numImages - 1));

    ui.horizontalSlider->setMinimum(0);
    ui.horizontalSlider->setMaximum(numImages - 1);
    ui.horizontalSlider->setValue(params["numimg"].getVal<int>());

    // set the status of all widgets depending on the values of params
    ui.spinBox_x0->setValue(params["x0"].getVal<int>());
    ui.spinBox_y0->setValue(params["y0"].getVal<int>());
    ui.spinBox_xsize->setValue(params["xsize"].getVal<int>());
    ui.spinBox_ysize->setValue(params["ysize"].getVal<int>());
    ui.spinBox_period->setValue(params["period"].getVal<int>());

    ui.comboBox_color->setCurrentIndex(params["color"].getVal<int>());
    ui.comboBox_orientation->setCurrentIndex(params["orientation"].getVal<int>());
    ui.checkBox_gamma->setChecked(params["gamma"].getVal<int>());

    switch (params["phaseshift"].getVal<int>())
    {
    case 3:
        ui.comboBox_phaseshifts->setCurrentIndex(0);
        break;

    default:
    case 4:
        ui.comboBox_phaseshifts->setCurrentIndex(1);
        break;

    case 5:
        ui.comboBox_phaseshifts->setCurrentIndex(2);
        break;

    case 8:
        ui.comboBox_phaseshifts->setCurrentIndex(3);
        break;
    }
}

//-------------------------------------------------------------------------------------
ito::RetVal DialogDispWindow::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase>> values;
    bool success = false;

    ito::RetVal retval = ito::retOk;

    int x0 = ui.spinBox_x0->value();
    // x0
    if (m_currentParameters["x0"].getVal<int>() != ui.spinBox_x0->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("x0", ito::ParamBase::Int, ui.spinBox_x0->value())));
    }

    int y0 = ui.spinBox_y0->value();
    // y0
    if (m_currentParameters["y0"].getVal<int>() != ui.spinBox_y0->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("y0", ito::ParamBase::Int, ui.spinBox_y0->value())));
    }

    int xsize = ui.spinBox_xsize->value();
    // xsize
    if (m_currentParameters["xsize"].getVal<int>() != ui.spinBox_xsize->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("xsize", ito::ParamBase::Int, ui.spinBox_xsize->value())));
    }

    int ysize = ui.spinBox_ysize->value();
    // ysize
    if (m_currentParameters["ysize"].getVal<int>() != ui.spinBox_ysize->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("ysize", ito::ParamBase::Int, ui.spinBox_ysize->value())));
    }

    int period = ui.spinBox_period->value();
    // period
    if (m_currentParameters["period"].getVal<int>() != ui.spinBox_period->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("period", ito::ParamBase::Int, ui.spinBox_period->value())));
    }

    int phaShift = 4;
    switch (ui.comboBox_phaseshifts->currentIndex())
    {
    case 0:
        phaShift = 3;
        break;
    default:
    case 1:
        phaShift = 4;
        break;
    case 2:
        phaShift = 5;
        break;
    case 3:
        phaShift = 8;
        break;
    }
    // phaseshift
    if (m_currentParameters["phaseshift"].getVal<int>() != phaShift)
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("phaseshift", ito::ParamBase::Int, phaShift)));
    }

    int orient = ui.comboBox_orientation->currentIndex();
    // orientation
    if (m_currentParameters["orientation"].getVal<int>() != orient)
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("orientation", ito::ParamBase::Int, orient)));
    }

    if (values.size() > 0)
    {
        // this is not necessary, however this dialog can directly change all parameters. Via
        // setParam, every parameter is changed step-by-step, and configProjectionFull recalculates
        // all textures after each change. Therefore send it here and call setParam multiple times
        // without the need to recalculate the textures.
        ito::RetVal retval = m_pWindow->configProjectionFull(
            x0, xsize, y0, ysize, period, phaShift, orient, nullptr);

        if (retval.containsError())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Error while configuring projection").toLatin1().data());
            if (retval.hasErrorMessage())
            {
                msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
            }
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.exec();
            return retValue;
        }
        else if (retval.containsWarning())
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Warning while configuring projection").toLatin1().data());
            if (retval.hasErrorMessage())
            {
                msgBox.setInformativeText(QLatin1String(retval.errorMessage()));
            }
            msgBox.setIcon(QMessageBox::Warning);
            msgBox.exec();
        }
    }

    // color
    if (m_currentParameters["color"].getVal<int>() != ui.comboBox_color->currentIndex())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("color", ito::ParamBase::Int, ui.comboBox_color->currentIndex())));
    }

    // gamma
    if ((m_currentParameters["gamma"].getVal<int>() > 0) != ui.checkBox_gamma->isChecked())
    {
        values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase(
            "gamma", ito::ParamBase::Int, ui.checkBox_gamma->isChecked() ? 1 : 0)));
    }

    // numimg
    if (m_currentParameters["numimg"].getVal<int>() != ui.horizontalSlider->value())
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("numimg", ito::ParamBase::Int, ui.horizontalSlider->value())));
    }


    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogDispWindow::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); // close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); // AcceptRole
    }
    else
    {
        applyParameters(); // ApplyRole
    }
}
