/* ********************************************************************
    Plugin "ThorlabsBP" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

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

#ifndef DIALOGTHORLABSBP_H
#define DIALOGTHORLABSBP_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"
#include "common/addInInterface.h"

#include "ui_dialogThorlabsBP.h"

#include <qdialog.h>
#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>
#include <qsharedpointer.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogThorlabsBP : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogThorlabsBP(ito::AddInActuator *actuator);
        ~DialogThorlabsBP();

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);

        Ui::DialogThorlabsBP ui;
        bool m_firstRun;
        ito::AddInActuator *m_pAia;
        QMap<QString, ito::Param> temporaryParams;
        int m_currentAxis;

        void currentAxisChanged(int newAxis);

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_btnCalib_clicked();
        void on_comboAxisSelector_currentIndexChanged(int index);

};

#endif
