/* ********************************************************************
    Plugin "AerotechEnsemble" for itom software
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

#ifndef DIALOGAEROTECHENSEMBLE_H
#define DIALOGAEROTECHENSEMBLE_H

/**\file dialogAerotechEnsemble.h
* \brief In this file the class of the modal dialog for the AerotechEnsemble are specified
*
*\sa dialogAerotechEnsemble, AerotechEnsemble
*/

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "common/addInInterface.h"

#include "ui_dialogAerotechEnsemble.h"    //! Header-file generated by Qt-Gui-Editor which has to be called

#include <QtGui>
#include <qdialog.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class dialogAerotechEnsemble
*   @brief Config dialog functionality of AerotechEnsemble
*
*   This class is used for the modal configuration dialog. It is used for parameter setup and calibration.
*    ui_dialogAerotechEnsemble.h is generated by the Gui-Editor.
*
*\sa AerotechEnsemble
*/
class dialogAerotechEnsemble : public QDialog
{
    Q_OBJECT

    private:
        Ui::dialogAerotechEnsemble ui;    //! Handle to the dialog
        ito::AddInActuator *m_pAerotechEnsemble;    //! Handle to the attached motor to invoke calib command
        int m_numaxis;    //!    Number of axis of this device
        QVector<QDoubleSpinBox*> m_pDialogSpeed;
        QVector<QLabel*> m_pDialogSpeedLabel;
        QVector<QCheckBox*> m_pDialogEnabled;

    public:
        dialogAerotechEnsemble(ito::AddInActuator *motor, QStringList axisNames);
        ~dialogAerotechEnsemble() {};
        int setVals(QMap<QString, ito::Param> *paramVals); //!< Function called by AerotechEnsemble::showConfDialog to set parameters values at dialog startup
        int getVals(QMap<QString, ito::Param> *paramVals);    //!< Function called by AerotechEnsemble::showConfDialog to get back the changed parameters

    public slots:

    private slots:
        void on_pushButtonCalib_clicked();    //!< If the Button invokes a AerotechEnsemble::Calib of enabled Axis
};

#endif
