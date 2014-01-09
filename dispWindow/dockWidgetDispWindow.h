#ifndef DOCKWIDGETDISPWINDOW_H
#define DOCKWIDGETDISPWINDOW_H

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetDispWindow.h"
#include "projWindow.h"

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

namespace ito {
    class AddInActuator;
}

class DispWindow; //forward declaration

class DockWidgetDispWindow : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetDispWindow(const QString &identifier, PrjWindow *prjWindow, DispWindow *dispWindow);
        ~DockWidgetDispWindow() {}
        PrjWindow *m_pPrjWindow;
        DispWindow *m_pDispWindow;
        //QMap<QString, ito::Param> *m_pParams;

    private:
        Ui::DockWidgetDispWindow ui;

        int m_curNumPhaseShifts;
        int m_curNumGrayCodes;
        bool m_numimgChangeInProgress;

    signals:

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        
    private slots:
        void on_comboBox_currentIndexChanged(int index);
};

#endif
