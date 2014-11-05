/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dockWidgetAvtVimba.h"

#include <qmetaobject.h>


//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetAvtVimba::DockWidgetAvtVimba(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        ui.lblInterface->setText(params["interface"].getVal<char*>());
        m_firstRun = false;
    }

    


    if (!m_inEditing)
    {
        m_inEditing = true;
        
        ParamMapIterator it = params.find("gain");
        if (it != params.end())
        {
            ui.sW_Gain->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.sW_Gain->setMinimum(it->getMin());
            ui.sW_Gain->setMaximum(it->getMax());
            ui.sW_Gain->setValue(it->getVal<double>());
        }
        else
        {
            ui.lbl_Gain->setVisible(false);
            ui.sW_Gain->setVisible(false);
        }

        it = params.find("gain_auto");
        if (it != params.end())
        {
            ui.check_GainAuto->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.check_GainAuto->setChecked(it->getVal<int>() > 0);
            ui.sW_Gain->setEnabled(ui.sW_Gain->isEnabled() && (it->getVal<int>() == 0));
        }
        else
        {
            ui.check_GainAuto->setVisible(false);
        }

        it = params.find("offset");
        if (it != params.end())
        {
            ui.sW_Offset->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.sW_Offset->setMinimum(it->getMin());
            ui.sW_Offset->setMaximum(it->getMax());
            ui.sW_Offset->setValue(it->getVal<double>());
        }
        else
        {
            ui.lbl_Offset->setVisible(false);
            ui.sW_Offset->setVisible(false);
        }

        it = params.find("integration_time");
        if (it != params.end())
        {
            ui.sW_IntTime->setDisabled(it->getFlags() & ito::ParamBase::Readonly);
            ui.sW_IntTime->setMinimum(it->getMin());
            ui.sW_IntTime->setMaximum(it->getMax());
            ui.sW_IntTime->setValue(it->getVal<double>());
        }
        else
        {
            ui.lbl_IntTime->setVisible(false);
            ui.sW_IntTime->setVisible(false);
        }


        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

void DockWidgetAvtVimba::on_sW_Gain_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::on_sW_IntTime_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::on_sW_Offset_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::on_check_GainAuto_value_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain_auto",ito::ParamBase::Int, checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}