/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGNEWPORT2936_H
#define DIALOGNEWPORT2936_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogNewport2936.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogNewport2936 : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogNewport2936(ito::AddInBase *grabber);
        ~DialogNewport2936() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;
        ito::AddInBase* m_plugin;

        Ui::DialogNewport2936 ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_btnZeroA_clicked();
        void on_btnZeroB_clicked();
};

#endif
