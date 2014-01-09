#ifndef DOCKWIDGETCMU1394_H
#define DOCKWIDGETCMU1394_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetCMU1394.h"

class DockWidgetCMU1394 : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetCMU1394(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetCMU1394() {};

    private:
        Ui::DockWidgetCMU1394 ui;
        char updating;

    signals:
        void changeParameters(QMap<QString, ito::ParamBase> params);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_doubleSpinBoxGain_editingFinished();
        void on_doubleSpinBoxOffset_editingFinished();
        void on_horizontalSliderGain_valueChanged(int value);
        void on_horizontalSliderOffset_valueChanged(int value);
};

#endif
