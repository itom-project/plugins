/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dockWidgetThorlabsCCS.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsCCS::DockWidgetThorlabsCCS(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsCCS::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.spinBpp->setValue(params["bpp"].getVal<int>());
    ui.spinWidth->setValue(params["sizex"].getVal<int>());
    //ui.spinHeight->setValue(params["sizey"].getVal<int>());

    if (m_firstRun)
    {
        //first time call
        //get all given parameters and adjust all widgets according to them (min, max, stepSize, values...)

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        //check the value of all given parameters and adjust your widgets according to them (value only should be enough)

        ui.spinIntegrationTime->setMaximum(params["integration_time"].getMax() *1000.0);
        ui.spinIntegrationTime->setMinimum(params["integration_time"].getMin() *1000.0);
        ui.spinIntegrationTime->setValue(params["integration_time"].getVal<double>()*1000.0);
        ui.spinIntegrationTime->setDisabled(params["integration_time"].getFlags() & ito::ParamBase::Readonly);

        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsCCS::on_spinIntegrationTime_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,d/1000.0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsCCS::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}
