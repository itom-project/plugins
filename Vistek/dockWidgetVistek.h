#ifndef DOCKWIDGETVISTEK_H
#define DOCKWIDGETVISTEK_H

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetVistek.h"
#include "common/sharedStructures.h"

class DockWidgetVistek : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetVistek(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetVistek() {};

    private:
        Ui::DockWidgetVistek ui;

    signals:
        void dataPropertiesChanged(int sizex, int sizey, int bpp);
        void gainPropertiesChanged(double gain);
        void exposurePropertiesChanged(double gain);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_gainSpinBox_editingFinished();
        void on_exposureSpinBox_editingFinished();
};

#endif
