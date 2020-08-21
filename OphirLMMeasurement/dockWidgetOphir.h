/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETOPHIR_H
#define DOCKWIDGETOPHIR_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetOphir.h"

class DockWidgetOphir : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetOphir(ito::AddInDataIO *ophirDevice);
        ~DockWidgetOphir() {};

    private:
        Ui::DockWidgetOphir ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        //add here slots connected to changes of any widget
        //example:
};

#endif
