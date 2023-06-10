#ifndef DIALOGUHL_H
#define DIALOGUHL_H

#include "common/addInInterface.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogUhl.h"
#include <QtGui>
#include <qdialog.h>


namespace ito
{
    class AddInActuator; //forward declaration
}

class dialogUhl : public QDialog
{
    Q_OBJECT

    public:
        dialogUhl(ito::AddInActuator *motor);
        ~dialogUhl() {};

    private:
        void enableDialog(bool enabled);
//        ito::RetVal checkParameters();
        ito::RetVal sendParameters();

        ito::AddInActuator *m_pUhlMotor;
        QMap<QString, ito::Param> m_actualParameters;

        Ui::dialogUhl ui;
        int m_invert[4];
        int m_enable[4];
        int m_numaxis;

        void setUhlAxisOrigin(int axis);

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_pushButtonCalib_clicked();
        void on_checkBox_InvertX_clicked();
        void on_checkBox_InvertY_clicked();
        void on_checkBox_InvertZ_clicked();
        void on_checkBox_EnableX_clicked();
        void on_checkBox_EnableY_clicked();
        void on_checkBox_EnableZ_clicked();
        void on_OriginXButton_clicked();
        void on_OriginYButton_clicked();
        void on_OriginZButton_clicked();
        void on_okButton_clicked();
};

#endif
