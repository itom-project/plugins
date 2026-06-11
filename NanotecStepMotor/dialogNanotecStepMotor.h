/* ********************************************************************
    Plugin "Standa NanotecStepMotor" for itom software
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

#ifndef DIALOGNANOTECSTEPMOTOR_H
#define DIALOGNANOTECSTEPMOTOR_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogNanotecStepMotor.h"

#include <qdialog.h>
#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include <qcombobox.h>
#include <qsignalmapper.h>
#include <qcommandlinkbutton.h>
#include <qspinbox.h>

namespace ito
{
    class AddInBase; //forward declaration
}

class DialogNanotecStepMotor : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogNanotecStepMotor(ito::AddInBase *actuator);
        ~DialogNanotecStepMotor();

        ito::RetVal applyParameters();

    private:
        Ui::DialogNanotecStepMotor ui;
        bool freshStarted;
        bool m_firstRun;
        int m_numAxis;
        bool m_isChanging;

        QVector<int> m_microStepsInitialStatus;
        QVector<int> m_decelInitialStatus;
        QVector<int> m_speedInitialStatus;
        QVector<int> m_accelInitialStatus;

        QVector<bool> m_axisToInitialize;

        QList<QFrame*> m_pListElements;
        QVector<QSpinBox*> m_pSpeedSpin;
        QVector<QSpinBox*> m_pAccelSpin;
        QVector<QSpinBox*> m_pMicroStepsSpin;
        QVector<QSpinBox*> m_pDecelSpin;
        QVector<QPushButton*> m_pResetBtn;

        void enableDialog(bool enabled);
        void createUiListEntry(const int i);

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void resetButtonClicked();
        void spinboxChanged(int value);
};

#endif
