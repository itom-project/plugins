/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETTHORLABSFF_H
#define DOCKWIDGETTHORLABSFF_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsFF.h"

class DockWidgetThorlabsFF : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsFF(ito::AddInDataIO *flipper);
        ~DockWidgetThorlabsFF() {};

    private:
        Ui::DockWidgetThorlabsFF ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        //add here slots connected to changes of any widget
        //example:
        void on_sBoxTransitTime_valueChanged(int i);
        void on_btnPos1_clicked();
        void on_btnPos2_clicked();
};

#endif
