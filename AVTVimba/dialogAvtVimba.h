/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef DIALOGAVTVIMBA_H
#define DIALOGAVTVIMBA_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogAvtVimba.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogAvtVimba : public ito::AbstractAddInConfigDialog 
{
    Q_OBJECT

    public:
        DialogAvtVimba(ito::AddInBase *grabber);
        ~DialogAvtVimba() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        Ui::DialogAvtVimba ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
		//void on_sB_PixFormat_valueChanged(int i);
};

#endif
