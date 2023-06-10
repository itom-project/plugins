#ifndef DIALOGPIEZOSYSTEMJENA_NV40_1_H
#define DIALOGPIEZOSYSTEMJENA_NV40_1_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogPiezosystemJena_NV40_1.h"

#include <qdialog.h>
#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>
#include <qvector.h>
#include <qsharedpointer.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogPiezosystemJena_NV40_1 : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogPiezosystemJena_NV40_1(ito::AddInBase *motor);
        ~DialogPiezosystemJena_NV40_1() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);

        Ui::dialogPiezosystemJena_NV40_1 ui;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);

};

#endif
