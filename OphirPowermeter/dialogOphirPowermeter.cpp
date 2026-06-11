/* ********************************************************************
    Plugin "OphirPowermeter" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut für Technische Optik (ITO),
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

#include "dialogOphirPowermeter.h"
//#include "OphirSerialPlugin.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>

//#include "common/addInInterface.h"

//#include <qmetaobject.h>
//#include <qdialogbuttonbox.h>
//#include <qmessagebox.h>

//#include "ui_dialogOphirSerialPlugin.h"

//#include <qdialog.h>
//#include <qstring.h>
//#include <qmap.h>
//#include <qabstractbutton.h>
//#include <qvector.h>
//#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogOphirPowermeter::DialogOphirPowermeter(ito::AddInBase *motor) :
    AbstractAddInConfigDialog(motor),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOphirPowermeter::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {

        m_firstRun = false;
    }

    //__________________________________________________________________________________________________________ Editing
    if (!m_inEditing)
    {
        m_inEditing = true;


        m_inEditing = false;
    }

    m_currentParameters = params;
}

//---------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogOphirPowermeter::applyParameters()
{
    ito::RetVal retValue(ito::retOk);


    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogOphirPowermeter::on_buttonBox_clicked(QAbstractButton* btn)
{

}

//---------------------------------------------------------------------------------------------------------------------
void DialogOphirPowermeter::enableDialog(bool enabled)
{

}
