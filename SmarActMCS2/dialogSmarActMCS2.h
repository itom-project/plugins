/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGSMARACTMCS2_H
#define DIALOGSMARACTMCS2_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogSmarActMCS2.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogSmarActMCS2 : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogSmarActMCS2(ito::AddInBase *grabber);
        ~DialogSmarActMCS2() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        Ui::DialogSmarActMCS2 ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
