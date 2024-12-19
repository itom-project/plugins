/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETTHORLABSDMH_H
#define DOCKWIDGETTHORLABSDMH_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qwidget.h>

#include "ThorlabsDMH.h"
#include "ui_dockWidgetThorlabsDMH.h"

class DockWidgetThorlabsDMH : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

public:
    DockWidgetThorlabsDMH(ThorlabsDMH* actuator); // IST ES RICHTIG, DASS NICHT ADDINACTUATOR HIER?
    ~DockWidgetThorlabsDMH() {};

private:
    Ui::DockWidgetThorlabsDMH ui; //! Handle to the ui
    bool m_inEditing;
    bool m_firstRun;

    QVector<SliderWidget*> m_sliderWidget;

    void enableWidgets(bool enabled);

    ThorlabsDMH* m_pActuator;

    QVector<QLabel*> m_labels;

    void setZernike(QVector<double> zernikeAmplitude);
    void updateSlider();


public slots:
    void parametersChanged(QMap<QString, ito::Param> params);
    void identifierChanged(const QString& identifier);

    void dockWidgetVisibilityChanged(bool visible);

private slots:

    void on_slider_valueChanged(double value, int sliderID); // if slider value changed
    void on_relaxMirror_clicked();
    void on_resetZernike_clicked();
};

#endif
