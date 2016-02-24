#ifndef DOCKWIDGETUHL_H
#define DOCKWIDGETUHL_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "ui_dockWidgetUhl.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>


namespace ito
{
    class AddInActuator; //forward declaration
}

class DockWidgetUhl : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetUhl(int uniqueID, ito::AddInActuator *actuator);
        ~DockWidgetUhl() {};

    private:
        void enableWidget(bool enabled);
        void waitForDoneAndCheckRetVal(ItomSharedSemaphore *waitCond);

        ito::AddInActuator *m_pUhlMotor;

        Ui::DockWidgetUhl ui;

//        int m_numaxis;
//        ito::AddInActuator *m_actuator;
//        void CheckAxisNums(QMap<QString, ito::tParam> params);
//        void setMotorStatus(bool busy);

/*
    signals:
        void MoveRelative(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        void MoveAbsolute(QVector<int> axis,  QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        void MotorTriggerStatusRequest(bool sendActPosition, bool sendTargetPos);
*/
    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition); //!< slot to receive information about status and position changes.
        void targetChanged(QVector<double> targetPositions);
//        void MotorPosStatusChanged(bool MotorRunning);
//        void MotorPosChanged(QVector<int> axis, QVector<double> pos);
//        void MotorStatusChanged(int Status);

    private slots:
        void on_pushButton_xp_clicked();
        void on_pushButton_xm_clicked();
        void on_pushButton_yp_clicked();
        void on_pushButton_ym_clicked();
        void on_pushButton_down_clicked();
        void on_pushButton_up_clicked();
        void on_pushButton_refresh_clicked();
        void on_pushButton_start_clicked();
        void on_pushButton_stop_clicked();
        void on_checkBox_enablex_clicked();
        void on_checkBox_enabley_clicked();
        void on_checkBox_enablez_clicked();
        void on_checkBox_enablejoy_clicked();
};

#endif
