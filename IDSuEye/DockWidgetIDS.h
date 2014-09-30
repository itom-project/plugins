#ifndef DOCKWIDGETIDSUEYE_H
#define DOCKWIDGETIDSUEYE_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetIDS.h"

class DockWidgetIDS : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetIDS(ito::AddInDataIO *grabber);
        ~DockWidgetIDS() {};

    private:
        Ui::DockWidgetIDS ui;
        bool m_inEditing;
        bool m_firstRun;
        QMap<QString, ito::Param> m_currentParams;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) {};

    private slots:
        void on_sliderExposure_valueChanged(double value);   
        void on_sliderGain_valueChanged(double value);
        void on_sliderOffset_valueChanged(double value);
};

#endif


