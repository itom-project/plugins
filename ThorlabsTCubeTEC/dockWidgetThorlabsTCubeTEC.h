/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#pragma once

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsTCubeTEC.h"

class DockWidgetThorlabsTCubeTEC : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsTCubeTEC(ito::AddInDataIO *plugin);
        ~DockWidgetThorlabsTCubeTEC() {};

    private:
        Ui::DockWidgetThorlabsTCubeTEC ui;
        bool m_inEditing;
        bool m_firstRun;
        QString m_unitSuffix;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        //add here slots connected to changes of any widget
        //example:
        void on_spinTarget_valueChanged(double value);
};

