#include "dockWidgetPiezosystemJena_NV40_1.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetPiezosystemJena_NV40_1::DockWidgetPiezosystemJena_NV40_1(ito::AddInActuator *actuator) : ito::AbstractAddInDockWidget(actuator), m_closedLoop(true), m_remote(true)
{
    ui.setupUi(this);

    enableWidget(true);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::parametersChanged(QMap<QString, ito::Param> params)
{
    if (params["remote"].getVal<int>() > 0)
    {
        ui.radioRemote->setChecked(true);
        ui.groupBox_2->setEnabled(true);
        ui.spinBoxTargetPos->setEnabled(true);
        ui.btnStart->setEnabled(true);
        m_remote = true;
    }
    else
    {
        ui.radioLocal->setChecked(true);
        ui.groupBox_2->setEnabled(false);
        ui.spinBoxTargetPos->setEnabled(false);
        ui.btnStart->setEnabled(false);
        m_remote = false;
    }

    QString unitMuM = QString(" %1m").arg(QChar(0x00, 0xB5)); // " \mum"
    ui.spinBoxStepSize->setSuffix(params["closedLoop"].getVal<int>() ? unitMuM : " V");  // \mu m
    ui.spinBoxActPos->setSuffix(params["closedLoop"].getVal<int>() ? unitMuM : " V");  // \mu m
    ui.spinBoxTargetPos->setSuffix(params["closedLoop"].getVal<int>() ? unitMuM : " V");  // \mu m
    m_closedLoop = params["closedLoop"].getVal<int>() > 0;
}

 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetPiezosystemJena_NV40_1::actuatorStatusChanged(QVector<int> status, QVector<double> actPosition) //!< slot to receive information about status and position changes.
 {
    ui.spinBoxTargetPos->setEnabled(status[0] & ito::actuatorEnabled);

    if (actPosition.size() > 0)
    {
        ui.spinBoxActPos->setValue(m_closedLoop ? actPosition[0] * 1000 : actPosition[0]);
    }

    bool running = false;
    QString style;

    if (status[0] & ito::actuatorMoving)
    {
        style = "background-color: yellow";
        running = true;
    }
    else if (status[0] & ito::actuatorInterrupted)
    {
        style = "background-color: red";
    }
    else if (status[0] & ito::actuatorUnknown)
    {
        style = "background-color: red";
    }
    else if (status[0] & ito::actuatorTimeout)
    {
        style = "background-color: #FFA3FD";
    }
    else
    {
        style = "background-color: ";
    }
    ui.spinBoxActPos->setStyleSheet(style);

    enableWidget(!running);
 }

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::targetChanged(QVector<double> targetPositions)
{
    if (targetPositions.size() > 0)
    {
        ui.spinBoxTargetPos->setValue(m_closedLoop ? targetPositions[0] * 1000 : targetPositions[0]);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::enableWidget(bool enabled)
{
    ui.spinBoxTargetPos->setEnabled(m_remote && enabled);
    ui.btnUp->setEnabled(enabled);
    ui.btnDown->setEnabled(enabled);
    ui.btnStart->setEnabled(m_remote && enabled);
    ui.groupBoxMode->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::on_radioLocal_clicked()
{
    QSharedPointer<ito::ParamBase> param;
    if (ui.radioLocal->isChecked())
    {
        param = QSharedPointer<ito::ParamBase>(new ito::ParamBase("remote",ito::ParamBase::Int,0));
    }
    else
    {
        param = QSharedPointer<ito::ParamBase>(new ito::ParamBase("remote",ito::ParamBase::Int,1));
    }

    setPluginParameter(param, ito::AbstractAddInDockWidget::msgLevelWarningAndError);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::on_btnUp_clicked()
{
    setActuatorPosition(0, m_closedLoop ? ui.spinBoxStepSize->value() / 1000.0 : ui.spinBoxStepSize->value(), true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::on_btnDown_clicked()
{
    setActuatorPosition(0, m_closedLoop ? -ui.spinBoxStepSize->value() / 1000.0 : -ui.spinBoxStepSize->value(), true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::on_btnStart_clicked()
{
    setActuatorPosition(0, m_closedLoop ? ui.spinBoxTargetPos->value() / 1000.0 : ui.spinBoxTargetPos->value(), false);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::on_btnRefresh_clicked()
{
    requestActuatorStatusAndPositions(true, true);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetPiezosystemJena_NV40_1::identifierChanged(const QString &identifier)
{
    ui.lblID->setText(identifier);
}
