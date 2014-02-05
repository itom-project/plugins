/* ********************************************************************
    Plugin "OpenCV-Grabber" for itom software
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

#ifndef DIALOGOPENCVGRABBER_H
#define DIALOGOPENCVGRABBER_H

#include "common/sharedStructures.h"
#include "common/addInGrabber.h"

#include "ui_dialogOpenCVGrabber.h"

#include <qdialog.h>
#include <qrect.h>

class DialogOpenCVGrabber : public QDialog 
{
    Q_OBJECT

    public:
        DialogOpenCVGrabber(ito::AddInGrabber *grabber, bool colorCam, int camWidth, int camHeight);    //!< Constructor
        ~DialogOpenCVGrabber() {};//! Destructor
        int sendVals(void);

    private:
        ito::AddInGrabber *m_grabber;

        Ui::dialogOpenCVGrabber ui;
        QMap<QString, ito::Param> m_paramsVals;
        bool m_colorCam;
        QRect m_camSize;

        bool sizeXChanged;
        bool sizeYChanged;
        bool colorModeChanged;

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        
        void on_applyButton_clicked();    //!< Write the current settings to the internal paramsVals and sent them to the grabber

        void on_btnSetFullROI_clicked();    //!< Set x and y-sizes to maximum valid value

        void on_spinX0_valueChanged(int value);
        void on_spinX1_valueChanged(int value);
        void on_spinSizeX_valueChanged(int value);

        void on_spinY0_valueChanged(int value);
        void on_spinY1_valueChanged(int value);
        void on_spinSizeY_valueChanged(int value);

        void on_comboColorMode_currentIndexChanged(int index) { colorModeChanged = true; }

};

#endif
