/* ********************************************************************
    Plugin "PCOCamera" for itom software
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

#ifndef DIALOGPCOCAMERA_H
#define DIALOGPCOCAMERA_H

#include "common/param.h"
#include "common/retVal.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "ui_dialogPCOCamera.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

#include <Windows.h>
#define PCO_ERRT_H_CREATE_OBJECT
#include "sc2_SDKStructures.h"


namespace ito
{
    class AddInBase; //forward declaration
}

class DialogPCOCamera : public ito::AbstractAddInConfigDialog
{
    Q_OBJECT

    public:
        DialogPCOCamera(ito::AddInBase *grabber, PCO_Description &cameraDescription);
        ~DialogPCOCamera() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        Ui::DialogPCOCamera ui;
        PCO_Description m_camInfo;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_rangeX01_valuesChanged(int minValue, int maxValue);
        void on_rangeY01_valuesChanged(int minValue, int maxValue);
        void on_btnFullROI_clicked();
};

#endif
