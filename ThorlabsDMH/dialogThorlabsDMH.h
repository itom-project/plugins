/* ********************************************************************
    Plugin "ThorlabsDMH" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

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

#ifndef DIALOGTHORLABSDMH_H
#define DIALOGTHORLABSDMH_H

#include "common/abstractAddInConfigDialog.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "common/addInInterface.h"
#include "ui_dialogThorlabsDMH.h"

#include <qabstractbutton.h>
#include <qmap.h>
#include <qstring.h>

namespace ito {
class AddInBase; // forward declaration
}

class DialogThorlabsDMH : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

public:
    DialogThorlabsDMH(ito::AddInActuator* actuator);
    ~DialogThorlabsDMH() {};

    ito::RetVal applyParameters();

private:
    void enableDialog(bool enabled);
    bool m_firstRun;

    QPointer<ito::AddInBase> m_pluginPointer;
    Ui::DialogThorlabsDMH ui;

public slots:
    void parametersChanged(QMap<QString, ito::Param> params);

private slots:
    void on_buttonBox_clicked(QAbstractButton* btn);
};

#endif
