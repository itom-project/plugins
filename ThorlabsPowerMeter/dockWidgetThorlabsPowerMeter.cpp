/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2016, Institut für Technische Optik (ITO),
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





//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsPowerMeter::DockWidgetThorlabsPowerMeter(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true),
    m_plugin(grabber),
    m_currentVal(1, 1, ito::tFloat64)
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
        ui.dspinPowerRange->setMinimum(params["power_range"].getMin()*1E3);
        ui.dspinPowerRange->setMaximum(params["power_range"].getMax()*1E3);
        m_inEditing = false;
        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        
        m_inEditing = true;
        ui.dspinWavelength->setValue(params["wavelength"].getVal<double>());
        double a(params["wavelength"].getVal<double>());
        ui.spinAverage->setValue(params["average_number"].getVal<ito::int32>());
        ui.dspinAttenuation->setValue(params["attenuation"].getVal<double>());
        ui.spinLineFrequency->setValue(params["line_frequency"].getVal<ito::int32>());
        if (params["auto_range"].getVal<ito::int32>())
        {
            ui.checkBoxAutoRange->setCheckState(Qt::Checked);
            ui.dspinPowerRange->setEnabled(false);
        }
        else
        {
            ui.checkBoxAutoRange->setCheckState(Qt::Unchecked);
            ui.dspinPowerRange->setEnabled(true);
        }
        

        //check the value of all given parameters and adjust your widgets according to them (value only should be enough)

        m_inEditing = false;
    }

    ui.dspinPowerRange->blockSignals(true);
    ui.dspinPowerRange->setValue(params["power_range"].getVal<double>()*1e3);
    ui.dspinPowerRange->blockSignals(false);
    
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
void DockWidgetThorlabsPowerMeter::on_checkBoxAutoRange_stateChanged(int val)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        if (val == 0)
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("auto_range", ito::ParamBase::Int, val));
            setPluginParameter(p, msgLevelWarningAndError);
            ui.dspinPowerRange->setEnabled(true);
            
        }
        else
        {
            QSharedPointer<ito::ParamBase> p(new ito::ParamBase("auto_range", ito::ParamBase::Int, 1));
            setPluginParameter(p, msgLevelWarningAndError);
            ui.dspinPowerRange->setEnabled(false);
        }
        m_inEditing = false;
    }

}
void DockWidgetThorlabsPowerMeter::on_checkAutograbbing_stateChanged(int val)
{

    manageTimer(true);
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
    {
        ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
        QMetaObject::invokeMethod(m_plugin, "acquire", Q_ARG(int, 0), Q_ARG(ItomSharedSemaphore*, waitCond));
        waitCond->waitAndProcessEvents(10000);
        waitCond->deleteSemaphore();
    }
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
    QMetaObject::invokeMethod(m_plugin, "getVal", Q_ARG(void*, &m_currentVal), Q_ARG(ItomSharedSemaphore*, waitCond));
    waitCond->waitAndProcessEvents(100);
    waitCond->deleteSemaphore();
    ui.labelVal->setText(QString::number(m_currentVal.at<ito::float64>(0, 0)));
    
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::manageTimer(bool visible)
{
    if (visible && ui.checkAutograbbing->checkState() == Qt::Checked)
    {
        m_timerId=startTimer(100);
    }
    else
    {
        killTimer(m_timerId);
        ui.labelVal->setText(QString("").toLatin1().data());
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsPowerMeter::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}