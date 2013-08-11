#ifndef DOCKWIDGETPIPIEZOCTRL_H
#define DOCKWIDGETPIPIEZOCTRL_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qsharedpointer.h>

#include "ui_dockWidgetPIPiezoCtrl.h"

namespace ito
{
    class AddInActuator; //forward declaration
}

class DockWidgetPIPiezoCtrl : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetPIPiezoCtrl(int uniqueID, ito::AddInActuator *actuator);
        ~DockWidgetPIPiezoCtrl() {};

    private:
        void enableWidget(bool enabled);
        void waitForDoneAndCheckRetVal(ItomSharedSemaphore *waitCond);

        ito::AddInActuator *m_pPlugin;

        Ui::DockWidgetPIPiezoCtrl ui;

	public slots:
		void valuesChanged(QMap<QString, ito::Param> params);
        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition); //!< slot to receive information about status and position changes.
        void targetChanged(QVector<double> targetPositions);

    private slots:
        void on_radioLocal_clicked();
        void on_radioRemote_clicked() { on_radioLocal_clicked(); }
        void on_btnUp_clicked();
        void on_btnDown_clicked();
        void on_btnStart_clicked();
        void on_btnRefresh_clicked();
};

#endif
