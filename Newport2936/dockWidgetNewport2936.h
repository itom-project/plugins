/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETNEWPORT2936_H
#define DOCKWIDGETNEWPORT2936_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetNewport2936.h"

class DockWidgetNewport2936 : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

public:
    DockWidgetNewport2936(ito::AddInDataIO *grabber);
    ~DockWidgetNewport2936() {};

private:
    Ui::DockWidgetNewport2936 ui;
    bool m_inEditing;
    bool m_firstRun;
    int m_timerId;
    bool m_timerIsRunning;
    int m_numChannel;
    ito::AddInDataIO *m_plugin;

    void calculateUnit(const double &val, QPair<double, QString> &result);
protected:
    void timerEvent(QTimerEvent *event);


    public slots:
    void identifierChanged(const QString &identifier);
    void parametersChanged(QMap<QString, ito::Param> params);
    void manageTimer(const bool &val);

    private slots:
    void on_checkAutograbbing_stateChanged(int val);




};

#endif
