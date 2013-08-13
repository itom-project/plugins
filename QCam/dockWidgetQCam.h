#ifndef DOCKWIDGETQCAM_H
#define DOCKWIDGETQCAM_H

#include "common/sharedStructures.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetQCam.h"

class DockWidgetQCam : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetQCam(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetQCam() {};

    private:
        Ui::DockWidgetQCam ui;
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
