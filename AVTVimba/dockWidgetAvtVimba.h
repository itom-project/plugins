/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETAVTVIMBA_H
#define DOCKWIDGETAVTVIMBA_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

//#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetAvtVimba.h"

class DockWidgetAvtVimba : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetAvtVimba(ito::AddInDataIO *grabber);
        ~DockWidgetAvtVimba() {};

    private:
        Ui::DockWidgetAvtVimba ui;
        bool m_inEditing;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

    private slots:
        void on_sW_Gain_valueChanged(double d);
        void on_sW_Offset_valueChanged(double d);
        void on_sW_IntTime_valueChanged(double d);
        void on_check_GainAuto_toggled(bool checked);
          
};

#endif
