/* ********************************************************************
    Plugin "Ximea" for itom software
    URL: http://www.twip-os.com
    Copyright (C) 2013, twip optical solutions GmbH
    Copyright (C) 2018, Institut für Technische Optik, Universität Stuttgart

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

#ifndef DIALOGXIMEA_H
#define DIALOGXIMEA_H

#include "common/param.h"
#include "common/retVal.h"
#include "common/addInGrabber.h"
#include "common/sharedStructures.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogXimea.h"
#include <qabstractbutton.h>
#include <qstring.h>
#include <qmap.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogXimea : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogXimea(ito::AddInBase *grabber);
        ~DialogXimea() {};
        ito::RetVal applyParameters();

    private:

        Ui::dialogXimea ui;
        QMap<QString, ito::Param> m_paramsVals;
        bool m_firstRun;
        bool m_inEditing;
        void enableDialog(bool enabled);

        inline double msecToSec(double musec) { return (double)musec * 1.0e-3; }
        inline double secToMsec(double sec) { return (double)(sec * 1.0e3); }
        bool timing_mode_changed;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:

        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_rangeX_valuesChanged(int minValue, int maxValue);
        void on_rangeY_valuesChanged(int minValue, int maxValue);
        void on_checkTimingMode_clicked(bool checked);
        void on_btnFullROI_clicked();
        void on_comboTriggerSelector_currentIndexChanged(int index);
        void on_comboTriggerMode_currentIndexChanged(int index);

};

#endif
