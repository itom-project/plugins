/* ********************************************************************
    Plugin "Newport SMC100" for itom software
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

#ifndef DIALOGSMC100_H
#define DIALOGSMC100_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogSMC100.h"

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

class DialogSMC100 : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogSMC100(ito::AddInBase *actuator);
        ~DialogSMC100();

        ito::RetVal applyParameters();

    private:
        ito::AddInBase *m_pPlugin;
        bool freshStarted;
        int m_numAxis;

        // Just the list for naming
        QStringList m_calibStatusNames;

        QVector<int> m_calibInitialStatus;
        QVector<int> m_InitialStatus;
        QVector<double> m_speedInitialStatus;
        QVector<double> m_accelInitialStatus;

        QVector<bool> m_axisToInitialize;

        QList<QFrame*> m_pListElements;
        QVector<QDoubleSpinBox*> m_pSpeedSpin;
        QVector<QDoubleSpinBox*> m_pAccelSpin;
        QVector<QComboBox*> m_pComboBoxes;
        QVector<QPushButton*> m_pResetBtn;


        void enableDialog(bool enabled);
        //ito::RetVal checkParameters();
        //ito::RetVal sendParameters();

        //ito::AddInActuator *m_pSMCPiezo;
        //QMap<QString, ito::Param> m_actualParameters;
        void createUiListEntry(const int i);

        Ui::DialogSMC100 ui;
        bool m_firstRun;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_calibrateBtn_clicked();
        void resetButtonClicked();
        void comboBoxChanged(int itemIdx);
        void spinboxChanged(double value);

};

#endif
