/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETMYACTUATOR_H
#define DOCKWIDGETMYACTUATOR_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qwidget.h>

#include "ui_dockWidgetFaulhaberMCS.h"

class DockWidgetFaulhaberMCS : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

public:
    DockWidgetFaulhaberMCS(ito::AddInActuator* actuator);
    ~DockWidgetFaulhaberMCS(){};

private:
    Ui::DockWidgetFaulhaberMCS ui; //! Handle to the ui
    bool m_inEditing;
    bool m_firstRun;

    void enableWidgets(bool enabled);

    QVector<QPushButton*> m_btnRelDec;
    QVector<QPushButton*> m_btnRelInc;
    QVector<QDoubleSpinBox*> m_spinCurrentPos;
    QVector<QDoubleSpinBox*> m_spinTargetPos;
    QVector<QLabel*> m_labels;


public slots:
    void parametersChanged(QMap<QString, ito::Param> params);
    void identifierChanged(const QString& identifier);

    void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
    void targetChanged(QVector<double> targetPositions);

private slots:

    void btnRelDecClicked(); // slot if any button for a relative, negative movement is clicked
    void btnRelIncClicked(); // slot if any button for a relative, positive movement is clicked

    void on_btnRefresh_clicked(); // slot if the refresh button is clicked
    void on_btnStart_clicked(); // slot if the start button is clicked
    void on_btnStop_clicked(); // slot if the stop button is clicked
};

#endif
