/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut fuer Technische Optik (ITO),
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

#ifndef DIALOGVISTEK_H
#define DIALOGVISTEK_H

#include "common/addInGrabber.h"

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogVistek.h"

class Vistek;
class VistekFeatures;

class DialogVistek : public QDialog
{
    Q_OBJECT

    public:
        DialogVistek(Vistek *grabber, const VistekFeatures *features);
        ~DialogVistek();

        int sendParameters(void);

    private:
        Vistek *m_Grabber;

        Ui::dialogVistek ui;
        const VistekFeatures *m_features;

        int m_currentBinning;
        int m_currentBpp;
        int m_currentOffset;
        double m_currentGain;
        double m_currentExposure;

    signals:

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_applyButton_clicked();    //!< Write the current settings to the internal paramsVals and sent them to the grabber

};

#endif

