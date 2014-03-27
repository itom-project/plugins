/* ********************************************************************
    Plugin "PIPiezoControl" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#ifndef DIALOGMSMEDIAFOUNDATION_H
#define DIALOGMSMEDIAFOUNDATION_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogMSMediaFoundation.h"

#include <qdialog.h>
#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>
#include <qvector.h>
#include <qsharedpointer.h>

namespace ito
{
    class AddInGrabber; //forward declaration
}

class DialogMSMediaFoundation : public QDialog 
{
    Q_OBJECT

    public:
        DialogMSMediaFoundation(ito::AddInGrabber *grabber);
        ~DialogMSMediaFoundation() {};

        int sendParameters(void);

    private:
        void enableDialog(bool enabled);
        bool m_firstRun;

        ito::AddInGrabber *m_pMSMediaFoundation;
        QMap<QString, ito::Param> m_actualParameters;

        Ui::DialogMSMediaFoundation ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_buttonBox_clicked(QAbstractButton* btn);
        void on_spinX0_valueChanged(int i);
        void on_spinX1_valueChanged(int i);
        void on_spinY0_valueChanged(int i);
        void on_spinY1_valueChanged(int i);
        void on_btnSetFullROI_clicked();
};

#endif
