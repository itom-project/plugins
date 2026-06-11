/* ********************************************************************
Plugin "Roughness" for itom software
URL : http ://www.uni-stuttgart.de/ito
Copyright(C) 2016, Institut für Technische Optik (ITO),
Universität Stuttgart, Germany;
IPROM, TU Braunschweig, Germany

This file is part of a plugin for the measurement software itom.

This itom - plugin is free software; you can redistribute it and / or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or(at
your option) any later version.

itom and its plugins are distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom.If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef DIALOGAVTVIMBA_H
#define DIALOGAVTVIMBA_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"
#include "avtEnums.h"

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
        DialogAvtVimba(ito::AddInBase *grabber, const BppEnum *bppEnum/*, const TriggerSourceEnum *triggerSourceEnum, const TriggerActivationEnum *triggerActivationEnum*/);
        ~DialogAvtVimba() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        inline bool dblEq(double v1, double v2) { return qAbs(v1-v2) <= std::numeric_limits<double>::epsilon(); }

        Ui::DialogAvtVimba ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_rangeX01_valuesChanged(int minValue, int maxValue);
        void on_rangeY01_valuesChanged(int minValue, int maxValue);
        void on_btnFullROI_clicked();
};

#endif
