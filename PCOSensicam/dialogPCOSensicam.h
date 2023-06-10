/* ********************************************************************
    Plugin "PCOSensicam" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
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

#ifndef DIALOGPCOSENSICAM_H
#define DIALOGPCOSENSICAM_H

#include "common/param.h"
#include "common/retVal.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogPCOSensicam.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

#include <Windows.h>
#define PCO_ERRT_H_CREATE_OBJECT
#include "SC_SDKStructures.h"


namespace ito
{
    class AddInBase; //forward declaration
}

class DialogPCOSensicam : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogPCOSensicam(ito::AddInBase *grabber, SC_Camera_Description &cameraDescription);
        ~DialogPCOSensicam() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        Ui::DialogPCOSensicam ui;
        SC_Camera_Description m_camInfo;
        double exposureToSecFactor;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_rangeX01_valuesChanged(int minValue, int maxValue);
        void on_rangeY01_valuesChanged(int minValue, int maxValue);
        void on_btnFullROI_clicked();
        void on_checkFastMode_clicked(bool checked);
};

#endif
