#ifndef DOCKWIDGETVISTEK_H
#define DOCKWIDGETVISTEK_H

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetVistek.h"
#include "common/sharedStructures.h"

#include "Vistek.h"

class DockWidgetVistek : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetVistek();
        ~DockWidgetVistek() {};

    private:
        Ui::DockWidgetVistek ui;
        bool m_inEditing;

        float m_exposureStep;

    signals:
        void GainPropertyChanged(double gain);
        void OffsetPropertyChanged(double gain);
        void ExposurePropertyChanged(double gain);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void propertiesChanged(float gainIncrement, float exposureIncrement, Vistek::Features features);

    private slots:
        void on_exposureSpinBox_valueChanged(double val);
        void on_gainSpinBox_valueChanged(double val);
        void on_offsetSpinBox_valueChanged(double val);
};

#endif
