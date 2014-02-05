/* ********************************************************************
    Plugin "dispWindow" for itom software
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

#ifndef DIALOGSERIALIO_H
#define DIALOGSERIALIO_H

#include "common/sharedStructures.h"

#include "ui_dialogDispWindow.h"
#include <QtGui>
#include <qdialog.h>

class dialogDispWindow : public QDialog 
{
    Q_OBJECT

    private:
        Ui::dialogDispWindow ui;
        void *m_pWindow;

    public:
        dialogDispWindow(const void *prjWindow);
        ~dialogDispWindow();
        int setVals(QMap<QString, ito::Param> *params, const int numImages);
        int getVals(QMap<QString, ito::Param> *params);

    public slots:
        void on_pushButtonSet_clicked(void);
//        void on_slider_image_valueChanged();
        void on_horizontalSlider_valueChanged();
        void on_comboBox_orientation_currentIndexChanged(int index);
        void on_comboBox_color_currentIndexChanged(int index);
        void on_checkBox_gamma_stateChanged(int state);
};

#endif
