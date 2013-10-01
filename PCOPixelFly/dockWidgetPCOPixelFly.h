#ifndef DOCKWIDGETPCOPixelFly_H
#define DOCKWIDGETPCOPixelFly_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetPCOPixelFly.h"

class DockWidgetPCOPixelFly : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetPCOPixelFly(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetPCOPixelFly() {};

    private:
        Ui::DockWidgetPCOPixelFly ui;

    signals:
        void GainPropertiesChanged(double gain);
        void OffsetPropertiesChanged(double offset);
        void IntegrationPropertiesChanged(double integrationtime);

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_spinBox_offset_editingFinished();
        void on_horizontalSlider_offset_sliderMoved(int d);
        void on_spinBox_gain_editingFinished();
        void on_horizontalSlider_gain_sliderMoved(int d);
        void on_doubleSpinBox_integration_time_editingFinished();
};

#endif
