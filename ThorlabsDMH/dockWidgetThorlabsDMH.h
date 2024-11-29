/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DOCKWIDGETTHORLABSDMH_H
#define DOCKWIDGETTHORLABSDMH_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>

#include "ui_dockWidgetThorlabsDMH.h"
#include "ThorlabsDMH.h"

class DockWidgetThorlabsDMH : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetThorlabsDMH(ThorlabsDMH *actuator);//IST ES RICHTIG, DASS NICHT ADDINACTUATOR HIER?
        ~DockWidgetThorlabsDMH() {};

    private:
        Ui::DockWidgetThorlabsDMH ui; //! Handle to the ui
        bool m_inEditing;
        bool m_firstRun;

        QVector<SliderWidget*> m_sliderWidget;

        void enableWidgets(bool enabled);

        ThorlabsDMH* m_pActuator;

        /*QVector<QPushButton*> m_btnRelDec;
        QVector<QPushButton*> m_btnRelInc;*/
       /* QVector<QDoubleSpinBox*> m_spinCurrentPos;
        QVector<QDoubleSpinBox*> m_spinTargetPos;*/
        QVector<QLabel*> m_labels;

        void setZernike(QVector<double> zernikeAmplitude);
        void updateSlider();


    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier);

        void actuatorStatusChanged(QVector<int> status, QVector<double> actPosition);
        void targetChanged(QVector<double> targetPositions);

        void dockWidgetVisibilityChanged(bool visible);

    private slots:

        void btnRelDecClicked();    //slot if any button for a relative, negative movement is clicked
        void btnRelIncClicked();    //slot if any button for a relative, positive movement is clicked

        void on_btnRefresh_clicked();       //slot if the refresh button is clicked
        void on_btnStart_clicked(); //slot if the start button is clicked
        void on_btnStop_clicked();  //slot if the stop button is clicked

        void on_slider_valueChanged(double value, int sliderID); // if slider value changed
        void on_relaxMirror_clicked();
        void on_resetZernike_clicked();

};

#endif
