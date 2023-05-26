/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETTHORLABSCCS_H
#define DOCKWIDGETTHORLABSCCS_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsCCS.h"

class DockWidgetThorlabsCCS : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsCCS(ito::AddInDataIO *grabber);
        ~DockWidgetThorlabsCCS() {};

    private:
        Ui::DockWidgetThorlabsCCS ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_spinIntegrationTime_valueChanged(double d);
};

#endif
