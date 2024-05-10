/* ********************************************************************
    Plugin "AndorSDK3" for itom software
    URL: https://github.com/itom-project/plugins
    Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef DIALOGANDORSDK3_H
#define DIALOGANDORSDK3_H

#include "common/param.h"
#include "common/retVal.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_DialogAndorSDK3.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>


namespace ito
{
    class AddInBase; //forward declaration
}

class DialogSDK3 : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogSDK3(ito::AddInBase *grabber);
        ~DialogSDK3() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;
        int m_currentSizeMaxX;
        int m_currentSizeMaxY;

        Ui::DialogAndorSDK3 ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_btnFullROI_clicked();
};

#endif //DIALOGANDORSDK3_H
