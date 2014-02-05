/* ********************************************************************
    Plugin "DummyGrabber" for itom software
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

#ifndef DIALOGDUMMYGRABBER_H
#define DIALOGDUMMYGRABBER_H

#include "common/addInGrabber.h"
//#include "common/sharedStructures.h"

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogDummyGrabber.h"

class dialogDummyGrabber : public QDialog
{
    Q_OBJECT

    public:
        dialogDummyGrabber(ito::AddInGrabber *grabber):m_Grabber(grabber){ m_paramsVals.clear(); ui.setupUi(this); };
        ~dialogDummyGrabber() {m_paramsVals.clear();};
        int getVals(QMap<QString, ito::Param> *paramVals);
        int sendVals(void);

    private:
        ito::AddInGrabber *m_Grabber;

        Ui::dialogDummyGrabber ui;
        QMap<QString, ito::Param> m_paramsVals;

    signals:

    public slots:

        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_btnResetROI_clicked();
        void on_applyButton_clicked();    //!< Write the current settings to the internal paramsVals and sent them to the grabber

        void on_spinBox_x0_valueChanged(int value);
        void on_spinBox_x1_valueChanged(int value);
        void on_spinBox_y0_valueChanged(int value);
        void on_spinBox_y1_valueChanged(int value);
        void on_spinBox_binX_valueChanged(int value);
        void on_spinBox_binY_valueChanged(int value);

};

#endif
