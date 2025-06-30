/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETSMARACTMCS2_H
#define DOCKWIDGETSMARACTMCS2_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetSmarActMCS2.h"

class DockWidgetSmarActMCS2 : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetSmarActMCS2(ito::AddInActuator *actuator);
        ~DockWidgetSmarActMCS2() {};

    private:
        Ui::DockWidgetSmarActMCS2 ui; //! Handle to the ui
        bool m_inEditing;
        bool m_firstRun;

        void enableWidgets(bool enabled);

        ito::AddInActuator* m_pActuator;


    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString& identifier);
        void dockWidgetVisibilityChanged(bool visible);
};

#endif
