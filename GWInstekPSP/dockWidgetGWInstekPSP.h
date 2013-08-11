#ifndef DOCKWIDGETGWINSTEKPSP_H
#define DOCKWIDGETGWINSTEKPSP_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qsharedpointer.h>
#include <qevent.h>

#include "ui_dockWidgetGWInstekPSP.h"

class DockWidgetGWInstekPSP : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetGWInstekPSP(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetGWInstekPSP() {};

    protected:
        void timerEvent(QTimerEvent *event);

    private:
        Ui::DockWidgetGWInstekPSP ui;
        int m_timerID;

    signals:
        void setParamVoltage(const double voltage);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_cbAuto_clicked();
        void on_pbStart_clicked();
};

#endif
