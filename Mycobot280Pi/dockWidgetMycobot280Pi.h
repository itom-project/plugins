#ifndef DockWidgetMyCobot280Pi_H
#define DockWidgetMyCobot280Pi_H

#include "common/abstractAddInDockWidget.h"
#include "common/addInInterface.h"

#include <qmap.h>
#include <qstring.h>
#include <qpushbutton.h>
#include <qslider.h>

#include "ui_dockWidgetMyCobot280pi.h"

namespace ito {
    class AddInActuator; //forward declaration
};

//----------------------------------------------------------------------------------------------------------------------------------
class DockWidgetMyCobot280Pi : public ito::AbstractAddInDockWidget
{
    Q_OBJECT

    public:
        DockWidgetMyCobot280Pi(int uniqueID, ito::AddInActuator *myPlugin);    //!< Constructor called by MyCobot280Pi::Constructor
        ~DockWidgetMyCobot280Pi() {};

    private:
        Ui::DockWidgetMyCobot280Pi ui;    //! Handle to the dialog
        int m_numaxis;  
        bool m_isVisible;
        bool m_inEditing;

        QVector<QSlider*> m_jointSliders;  //! List of sliders for controlling joint positions
        ito::AddInActuator *m_pActuator;

        void enableWidget(bool enabled);
        void sendJointData();  // Method to send joint data
        void sendStartRobot();
        void sendStartArmcontrolCommand();
        void sendConnectArmcontrolCommand();
        void getCurrentJointStates();
        void connectSocketServer(); // Method to send start command

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void identifierChanged(const QString &identifier) {};
        void dockWidgetVisibilityChanged(bool visible);
        void actuatorStatusChanged(QVector<int> status, QVector<double> positions);

    private slots:
        void onStartCommandClicked();  // Slot for "Send Command" button click
        void onStartRobotClicked(); 
        void onStartArmControlClicked();
        void onGetJointStatesClicked();
        void onConnectSocketServer(); 
        void onConnectArmControlClicked();  // Slot for "Start Robot" button click
};

#endif // DockWidgetMyCobot280Pi_H
