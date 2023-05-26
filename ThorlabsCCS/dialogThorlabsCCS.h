/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGTHORLABSCCS_H
#define DIALOGTHORLABSCCS_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogThorlabsCCS.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogThorlabsCCS : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogThorlabsCCS(ito::AddInBase *grabber);
        ~DialogThorlabsCCS() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        Ui::DialogThorlabsCCS ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_rangeX01_valuesChanged(int minValue, int maxValue);
        void on_btnFullROI_clicked();
};

#endif
