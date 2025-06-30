/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dockWidgetSmarActMCS2.h"
#include "motorAxisController.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetSmarActMCS2::DockWidgetSmarActMCS2(ito::AddInActuator* actuator) :
    AbstractAddInDockWidget(actuator), m_inEditing(false), m_pActuator(actuator), 
    m_firstRun(true)
{
    ui.setupUi(this);

    enableWidgets(true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::parametersChanged(QMap<QString, ito::Param> params)
{
    ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());

    if (m_firstRun)
    {
        ui.axisController->setNumAxis(params["noOfChannels"].getVal<int>());
        for (int i = 0; i < params["noOfChannels"].getVal<int>(); i++)
        {
            switch (params["baseUnit"].getVal<int*>()[i])
            {
                case 1:
                ui.axisController->setAxisUnit(i, MotorAxisController::AxisUnit::UnitMm);
                    break;
                case 2:
                    ui.axisController->setAxisUnit(i, MotorAxisController::AxisUnit::UnitAU);
                    ui.axisController->setArbitraryUnit("deg");
                    break;
                default:
                    ui.axisController->setAxisUnit(i, MotorAxisController::AxisUnit::UnitAU);
                    break;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::enableWidgets(bool enabled)
{
    ui.axisController->setEnabled(enabled);
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::identifierChanged(const QString& identifier)
{
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetSmarActMCS2::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        // to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.axisController->setActuator(actuator);
        ui.axisController->setNumAxis(0);
        ui.axisController->setDefaultRelativeStepSize(0.001);
        ui.axisController->setDefaultDecimals(3);
    }
    else
    {
        ui.axisController->setActuator(QPointer<ito::AddInActuator>());
    }
}