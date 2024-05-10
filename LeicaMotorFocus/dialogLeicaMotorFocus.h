/* ********************************************************************
    Plugin "LeicaMotorFocus" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#ifndef DIALOGLEICAMF_H
#define DIALOGLEICAMF_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogLeicaMotorFocus.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

class DialogLeicaMotorFocus : public QDialog
{
    Q_OBJECT

    private:
        Ui::dialogLeicaMotorFocus ui;
        QObject *m_pluginInstance;

        bool m_unappliedChanges;

        void enableDialog(bool enabled);

    public:
        DialogLeicaMotorFocus(QObject *pluginInstance);
        ~DialogLeicaMotorFocus() {};

        ito::RetVal applyParameters();

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_cmdHoming_clicked();
        void on_cmdOrigin_clicked();

        void on_spinBoxSpeed_valueChanged(double) { m_unappliedChanges = true; }
        void on_spinBoxRatio_valueChanged(int) { m_unappliedChanges = true; }
        void on_checkInvertAxis_stateChanged(int) { m_unappliedChanges = true; }
        void on_radioRefUpper_toggled(bool) { m_unappliedChanges = true; }
        void on_radioRefLower_toggled(bool) { m_unappliedChanges = true; }
        void on_cmdOk_clicked();



};

#endif
