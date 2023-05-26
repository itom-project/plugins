#ifndef DIALOGPIHexapodCTRL_H
#define DIALOGPIHexapodCTRL_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogPIHexapodCtrl.h"

#include <qdialog.h>
#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>
#include <qvector.h>
#include <qsharedpointer.h>

namespace ito
{
    class AddInActuator; //forward declaration
}

class DialogPIHexapodCtrl : public QDialog
{
    Q_OBJECT

    public:
        DialogPIHexapodCtrl(ito::AddInActuator *motor);
        ~DialogPIHexapodCtrl() {};

    private:
        void enableDialog(bool enabled);
        ito::RetVal checkParameters();
        ito::RetVal sendParameters();

        ito::AddInActuator *m_pPIHexapod;
        QMap<QString, ito::Param> m_actualParameters;

        Ui::DialogPIHexapodCtrl ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);

};

#endif
