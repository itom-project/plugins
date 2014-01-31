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
        DockWidgetVistek();
        ~DockWidgetVistek() {};

    private:
        Ui::DockWidgetVistek ui;
        bool m_inEditing;

    signals:
        void GainOffsetExposurePropertiesChanged(double gain, double offset, double exposure);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_exposureSpinBox_editingFinished();
        void on_gainSpinBox_editingFinished();
};

#endif
