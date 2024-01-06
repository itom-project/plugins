/* ********************************************************************
    Plugin "NewportConexLDS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#include "dockWidgetNewportConexLDS.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetNewportConexLDS::DockWidgetNewportConexLDS(int uniqueID, ito::AddInDataIO* rawIO) :
    AbstractAddInDockWidget(rawIO), m_inEditing(false), m_firstRun(true), m_plugin(rawIO)
{
    ui.setupUi(this);
    identifierChanged(QString::number(uniqueID));
}

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetNewportConexLDS::~DockWidgetNewportConexLDS()
{
    killTimer(m_timerId);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewportConexLDS::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        m_inEditing = true;
        ui.lblVersion->setText(params["version"].getVal<char*>());
        QString config = params["configurationState"].getVal<char*>();
        ui.lblConfiguration->setText(config);

        if (config == "MEASURE")
        {
            ui.btnLaserPower->setChecked(true);
        }
        int range = params["range"].getVal<int>();
        ui.doubleSpinBoxXAxis->setMinimum(-range);
        ui.doubleSpinBoxXAxis->setMaximum(range);

        ui.doubleSpinBoxYAxis->setMinimum(-range);
        ui.doubleSpinBoxYAxis->setMaximum(range);


        ui.lblXAxis->setText(QString("x axis [%1]").arg(params["unit"].getVal<char*>()));
        ui.lblYAxis->setText(QString("y axis [%1]").arg(params["unit"].getVal<char*>()));

        m_firstRun = false;
        m_inEditing = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        // check the value of all given parameters and adjust your widgets according to them (value
        // only should be enough)

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewportConexLDS::timerEvent(QTimerEvent* event)
{
    ito::RetVal retval(ito::retOk);
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();

    QSharedPointer<ito::float64> values =
        QSharedPointer<ito::float64>(new ito::float64[3]{0.0, 0.0, 0.0});

    QMetaObject::invokeMethod(
        m_plugin,
        "autoGrabbing",
        Q_ARG(QSharedPointer<ito::float64>, values),
        Q_ARG(ItomSharedSemaphore*, waitCond));


    if (waitCond)
    {
        observeInvocation(waitCond, msgLevelWarningAndError);
    }

    if (values)
    {
        ui.doubleSpinBoxXAxis->setValue(values.data()[0]);
        ui.doubleSpinBoxYAxis->setValue(values.data()[1]);
        ui.sliderWidgetPowerLevel->setValue(values.data()[2]);
    }

    if (waitCond)
    {
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewportConexLDS::on_btnLaserPower_toggled(bool state)
{
    if (state)
    {
        m_timerId = startTimer(200);
        m_timerIsRunning = true;
        ui.btnLaserPower->setText("LASER ON");
    }
    else if (m_timerIsRunning)
    {
        killTimer(m_timerId);
        m_timerIsRunning = false;
        ui.btnLaserPower->setText("LASER OFF");
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewportConexLDS::identifierChanged(const QString& identifier)
{
    ui.lblIdentifier->setText(identifier);
}
