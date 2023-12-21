/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGMYGRABBER_H
#define DIALOGMYGRABBER_H

#include "common/abstractAddInConfigDialog.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogConexLDS.h"

#include <qabstractbutton.h>
#include <qmap.h>
#include <qstring.h>

namespace ito {
class AddInBase; // forward declaration
}

class DialogConexLDS : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

public:
    DialogConexLDS(ito::AddInBase* grabber);
    ~DialogConexLDS(){};

    ito::RetVal applyParameters();

private:
    void enableDialog(bool enabled);
    bool m_firstRun;

    Ui::DialogConexLDS ui;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);

private slots:
    void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
