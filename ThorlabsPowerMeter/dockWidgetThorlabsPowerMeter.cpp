/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2018, Institut für Technische Optik (ITO),,
Universität Stuttgart, Germany

This file is part of itom and its software development toolkit (SDK).

itom is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

In addition, as a special exception, the Institut für Technische
Optik (ITO) gives you certain additional rights.
These rights are described in the ITO LGPL Exception version 1.0,
which can be found in the file LGPL_EXCEPTION.txt in this package.

itom is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dockWidgetThorlabsPowerMeter.h"
#include <cmath>
#include <QMessageBox>




//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsPowerMeter::DockWidgetThorlabsPowerMeter(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true),
    m_plugin(grabber),
    m_timerIsRunning(false)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::parametersChanged(QMap<QString, ito::Param> params)
{

    if (m_firstRun)
    {
        m_inEditing = true;
        ui.dspinWavelength->setMinimum(params["wavelength"].getMin());
        ui.dspinWavelength->setMaximum(params["wavelength"].getMax());
        ui.spinAverage->setMinimum(params["average_number"].getMin());
        ui.spinAverage->setMaximum(params["average_number"].getMax());
        ui.dspinAttenuation->setMinimum(params["attenuation"].getMin());
        ui.dspinAttenuation->setMaximum(params["attenuation"].getMax());
        ui.spinLineFrequency->setMaximum(params["line_frequency"].getMax());
        ui.spinLineFrequency->setMinimum(params["line_frequency"].getMin());
        ui.spinLineFrequency->setSingleStep(((ito::IntMeta*)params["line_frequency"].getMeta())->getStepSize());

        ui.comboBandwidth->addItem("High");
        ui.comboBandwidth->addItem("LOW");
        ui.lcdNumber->setPalette(Qt::red);
        m_inEditing = false;
        m_firstRun = false;
    }

    if (!m_inEditing)
    {

        m_inEditing = true;
        ui.dspinWavelength->setValue(params["wavelength"].getVal<double>());
        ui.spinAverage->setValue(params["average_number"].getVal<ito::int32>());
        ui.dspinAttenuation->setValue(params["attenuation"].getVal<double>());
        ui.spinLineFrequency->setValue(params["line_frequency"].getVal<ito::int32>());
        if (params["auto_range"].getVal<ito::int32>())
        {
            ui.checkBoxAutoRange->setCheckState(Qt::Checked);
            ui.sliderPowerRange->setEnabled(false);
        }
        else
        {
            ui.checkBoxAutoRange->setCheckState(Qt::Unchecked);
            ui.sliderPowerRange->setEnabled(true);
        }
        ui.comboBandwidth->setCurrentIndex(params["bandwidth"].getVal<ito::int32>());

        //check the value of all given parameters and adjust your widgets according to them (value only should be enough)

        m_inEditing = false;
    }
    // since the power range can be modified by the autorange and the attenuation it must be also updated if m_inEditing. To avoid a signal ring the signals are blocked
    ui.sliderPowerRange->blockSignals(true);
    ui.sliderPowerRange->setMinimum(params["power_range"].getMin()*1E3);
    ui.sliderPowerRange->setMaximum(params["power_range"].getMax()*1E3);
    ui.sliderPowerRange->setValue(params["power_range"].getVal<double>()*1e3);
    ui.sliderPowerRange->blockSignals(false);

}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_dspinWavelength_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("wavelength", ito::ParamBase::Double, val));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_spinAverage_valueChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("average_number", ito::ParamBase::Int, val));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;

    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_dspinAttenuation_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("attenuation", ito::ParamBase::Double, val));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;

    }

}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_spinLineFrequency_valueChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("line_frequency", ito::ParamBase::Int, val));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_sliderPowerRange_valueChanged(double val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("power_range", ito::ParamBase::Double, val*1e-3));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;

    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_checkBoxAutoRange_stateChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        if (val == 0)
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("auto_range", ito::ParamBase::Int, val));
            setPluginParameter(p, msgLevelWarningAndError);
            ui.sliderPowerRange->setEnabled(true);

        }
        else
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("auto_range", ito::ParamBase::Int, 1));
            setPluginParameter(p, msgLevelWarningAndError);
            ui.sliderPowerRange->setEnabled(false);
        }
        m_inEditing = false;
    }

}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_comboBandwidth_currentIndexChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("bandwidth", ito::ParamBase::Int, val));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_checkAutograbbing_stateChanged(int val)
{

    manageTimer(true);
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::on_btnZero_clicked()
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
        ui.groupBoxSettings->setEnabled(false);
        QMetaObject::invokeMethod(m_plugin, "zeroDevice", Q_ARG(ItomSharedSemaphore*, waitCond));
        observeInvocation(waitCond, msgLevelWarningAndError);
        ui.groupBoxSettings->setEnabled(true);
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
// void DockWidgetMyGrabber::on_contrast_valueChanged(int i)
// {
    // if (!m_inEditing)
    // {
        // m_inEditing = true;
        // QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrast",ito::ParamBase::Int,d));
        // setPluginParameter(p, msgLevelWarningAndError);
        // m_inEditing = false;
    // }
// }

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::timerEvent(QTimerEvent *event)
{

        ito::RetVal retval(ito::retOk);
        ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
        QSharedPointer<double> value = QSharedPointer<double>(new double);
        QPair<double, QString> result(0,"");
        QMetaObject::invokeMethod(m_plugin, "acquireAutograbbing", Q_ARG(QSharedPointer<double>, value), Q_ARG(ItomSharedSemaphore*, waitCond));
        if (waitCond->waitAndProcessEvents(10000))
        {
            retval += waitCond->returnValue;
            if (!retval.containsError())
            {
                calculateUnit(*value, result);
                ui.lcdNumber->display(result.first);
                ui.labelVal->setText(result.second);
            }
        }
        waitCond->deleteSemaphore();


}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::manageTimer(const bool &visible)
{
    if (visible && ui.checkAutograbbing->checkState() == Qt::Checked)
    {
        m_timerId=startTimer(100);
        m_timerIsRunning = true;
    }
    else if (m_timerIsRunning)
    {
       killTimer(m_timerId);
       m_timerIsRunning = false;
        ui.labelVal->setText(QString("").toLatin1().data());
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::calculateUnit(const ito::float64 &val, QPair<double,QString> &result)
{

    double exp(log10(abs(val)));
    if (exp >= 0.0)
    {
        result.first = val;
        result.second = "W";
    }
    else if (exp >= -3.0)
    {
        result.first = val*10e2;
        result.second = "mW";
    }
    else if (exp >= -6.0)
    {
        result.first = val*10e5;
        result.second = QString::fromLatin1("\u00B5W");
    }
    else if (exp >= -9.0)
    {
        result.first = val*10e8;
        result.second = "nW";
    }
    else if (exp >= -12.0)
    {
        result.first = val*10e11;
        result.second = "pW";
    }
    else
    {
        result.first = val;
        result.second = "unknown";
    }
}
