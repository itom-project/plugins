/* ********************************************************************
    Plugin "QCam" for itom software
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

#ifndef DIALOGQCAM_H
#define DIALOGQCAM_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogQCam.h"
#include "common/sharedStructures.h"
#include "common/addInInterface.h"
 /**
  *\class    DialogQCam
  *\brief    configuration dialog for QCam cameras
  *
  *         This dialog can be used to setup the most important parameters for the camera.
  *            During setup, itom is blocked
  *
  *
  */
class DialogQCam : public QDialog
{
    Q_OBJECT

    public:
        DialogQCam(ito::AddInDataIO *grabber);    //!< Constructor
        ~DialogQCam() {};//! Destructor

        int getVals(); //!< Writeback called before closing

    private:
        Ui::dialogQCam ui;    //! The QT-Design-GUI
        ito::AddInDataIO *m_grabber;

        bool m_gainChanged;
        bool m_offsetChanged;

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
        void on_pushButton_setSizeXMax_clicked();    //!< Set x-size to maximum valid value
        void on_pushButton_setSizeYMax_clicked();    //!< Set y-sizes to maximum valid value

        void on_doubleSpinBox_offset_valueChanged(double /*val*/) { m_offsetChanged = true; }
        void on_doubleSpinBox_gain_valueChanged(double /*val*/) { m_gainChanged = true; }
};

#endif //DIALOGQCAM_H
