#ifndef DOCKWIDGETPIEZOSYSTEMJENA_NV40_1_H
#define DOCKWIDGETPIEZOSYSTEMJENA_NV40_1_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetPiezosystemJena_NV40_1.h"

namespace ito
{
    class AddInActuator; //forward declaration
}

class DockWidgetPiezosystemJena_NV40_1 : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetPiezosystemJena_NV40_1(ito::AddInActuator * myPlugin);
        ~DockWidgetPiezosystemJena_NV40_1() {};

    private:
        void enableWidget(bool enabled);
        bool m_closedLoop;
        bool m_remote;

        Ui::dockWidgetPiezosystemJena_NV40_1 ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
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
