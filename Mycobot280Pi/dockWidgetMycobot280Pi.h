#ifndef DOCKWIDGETMYCOBOTCONTROL_H
#define DOCKWIDGETMYCOBOTCONTROL_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qpushbutton.h>
#include <qslider.h>

#include "ui_dockWidgetMycobotControl.h"

namespace ito {
    class AddInActuator; //forward declaration
};

//----------------------------------------------------------------------------------------------------------------------------------
class DockWidgetMycobotControl : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetMycobotControl(int uniqueID, ito::AddInActuator *myPlugin);    //!< Constructor called by MycobotControl::Constructor
        ~DockWidgetMycobotControl() {};

    private:
        Ui::DockWidgetMycobotControl ui;    //! Handle to the dialog
        int m_numaxis;  
        bool m_isVisible;
        bool m_inEditing;

        QVector<QSlider*> m_jointSliders;  //! List of sliders for controlling joint positions
        ito::AddInActuator *m_pActuator;

        void enableWidget(bool enabled);
        void sendJointData();  // Method to send joint data
        void sendStartCommand(); // Method to send start command

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) {};
        void dockWidgetVisibilityChanged(bool visible);
        void actuatorStatusChanged(QVector<int> status, QVector<double> positions);

    private slots:
        void onStartCommandClicked();  // Slot for "Send Command" button click
        void onStartRobotClicked();    // Slot for "Start Robot" button click
};

#endif // DOCKWIDGETMYCOBOTCONTROL_H
