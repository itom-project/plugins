#ifndef DOCKWIDGETFIREGRABBER_H
#define DOCKWIDGETFIREGRABBER_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetFireGrabber.h"

class DockWidgetFireGrabber : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetFireGrabber();
        ~DockWidgetFireGrabber() {};

    private:
        Ui::DockWidgetFireGrabber ui;
        bool m_inEditing;

    signals:
        void GainOffsetPropertiesChanged(double gain, double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
        void setIdentifier(const QString &identifier);


    private slots:
        void on_spinBox_offset_valueChanged(int d);
        void on_spinBox_gain_valueChanged(int d);
        void on_doubleSpinBox_integration_time_valueChanged(double d);
};

#endif
