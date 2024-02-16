/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGMYACTUATOR_H
#define DIALOGMYACTUATOR_H

#include "common/abstractAddInConfigDialog.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogFaulhaberMCS.h"

#include <qabstractbutton.h>
#include <qmap.h>
#include <qstring.h>

namespace ito {
class AddInBase; // forward declaration
}

class DialogFaulhaberMCS : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

public:
    DialogFaulhaberMCS(ito::AddInBase* grabber);
    ~DialogFaulhaberMCS(){};

    ito::RetVal applyParameters();

private:
    void enableDialog(bool enabled);
    bool m_firstRun;

    Ui::DialogFaulhaberMCS ui;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);

private slots:
    void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
