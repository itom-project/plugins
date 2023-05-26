/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGMYGRABBER_H
#define DIALOGMYGRABBER_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogThorlabsPowerMeter.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogThorlabsPowerMeter : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogThorlabsPowerMeter(ito::AddInBase *grabber);
        ~DialogThorlabsPowerMeter() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;
        ito::AddInBase *m_plugin;

        Ui::DialogMyGrabber ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        void on_btnZero_clicked();

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
