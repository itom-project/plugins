/* ********************************************************************
    Plugin "OphirSerialPlugin" for itom software
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

#ifndef DIALOGOPHIRSERIALPLUGIN_H
#define DIALOGOPHIRSERIALPLUGIN_H

#include "common/param.h"
#include "common/retVal.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogOphirSerialPlugin.h"

#include <qstring.h>
#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

namespace ito
{
    class AddInActuator; //forward declaration
}

class DialogOphirSerialPlugin : public ito::AbstractAddInConfigDialog 
{
    Q_OBJECT

    public:
        DialogOphirSerialPlugin (ito::AddInBase *motor);
        ~DialogOphirSerialPlugin () {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;
        bool m_inEditing;
        Ui::DialogOphirSerialPlugin ui; 

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        //auto-connected slot called if ok, apply or cancel is clicked
        void on_buttonBox_clicked(QAbstractButton* btn);

};

#endif
