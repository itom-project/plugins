#ifndef DIALOGPIPIEZOCTRL_H
#define DIALOGPIPIEZOCTRL_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogPIPiezoCtrl.h"

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

class DialogPIPiezoCtrl : public QDialog 
{
    Q_OBJECT

    public:
        DialogPIPiezoCtrl(ito::AddInActuator *motor);
		~DialogPIPiezoCtrl() {};

    private:
        void enableDialog(bool enabled);
        ito::RetVal checkParameters();
        ito::RetVal sendParameters();

        ito::AddInActuator *m_pPIPiezo;
        QMap<QString, ito::Param> m_actualParameters;

        Ui::DialogPIPiezoCtrl ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);

};

#endif
