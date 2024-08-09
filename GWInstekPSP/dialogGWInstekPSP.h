/* ********************************************************************
    Plugin "GWInstekPSP" for itom software
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

#ifndef DIALOGGWINSTEKPSP_H
#define DIALOGGWINSTEKPSP_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogGWInstekPSP.h"

#include <QtGui>
#include <qdialog.h>
#include <qstring.h>

class dialogGWInstekPSP : public QDialog
{
    Q_OBJECT

    private:
        Ui::dialogGWInstekPSP ui;
        void *m_dataIO;

    public:
        dialogGWInstekPSP(void *dataIO);
        ~dialogGWInstekPSP() {};
        int setVals(QMap<QString, ito::Param> *paramVals);
        int getVals(QMap<QString, ito::Param> *paramVals);

    public slots:


    private slots:

};

#endif
