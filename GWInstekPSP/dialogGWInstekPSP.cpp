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

#include "dialogGWInstekPSP.h"
#include "GWInstekPSP.h"

#ifdef WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif

//----------------------------------------------------------------------------------------------------------------------------------
dialogGWInstekPSP::dialogGWInstekPSP(void *dataIO) : m_dataIO(dataIO)
{
    ui.setupUi(this);
};
//----------------------------------------------------------------------------------------------------------------------------------
int dialogGWInstekPSP::setVals(QMap<QString, ito::Param> *paramVals)
{
    QMap<QString, ito::Param>::const_iterator paramIt;

    paramIt = (*paramVals).constFind("name");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    }
    // added by itobiege, Mar. 2013, but not tested!

    paramIt = (*paramVals).constFind("voltage_limit");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.dbVoltageLimit->setMaximum((*paramVals)["voltage_limit"].getMax());
        ui.dbVoltageLimit->setValue((*paramVals)["voltage_limit"].getVal<double>());
    }

    paramIt = (*paramVals).constFind("current_limit");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.dbCurrentLimit->setMaximum((*paramVals)["current_limit"].getMax());
        ui.dbCurrentLimit->setValue((*paramVals)["current_limit"].getVal<double>());
    }

    paramIt = (*paramVals).constFind("load_limit");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.dbLoadLimit->setMaximum((*paramVals)["load_limit"].getMax());
        ui.dbLoadLimit->setValue((*paramVals)["load_limit"].getVal<double>());
    }

    paramIt = (*paramVals).constFind("save");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbSave->setChecked((*paramVals)["save"].getVal<int>());
    }

    paramIt = (*paramVals).constFind("relay");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbRelayOn->setChecked((*paramVals)["relay"].getVal<int>());
    }

    paramIt = (*paramVals).constFind("temperature");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbTemperature->setChecked((*paramVals)["temperature"].getVal<int>());
    }

    paramIt = (*paramVals).constFind("wheel");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbWheelFine->setChecked((*paramVals)["wheel"].getVal<int>());
    }

    paramIt = (*paramVals).constFind("wheel_lock");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbWheelUnlock->setChecked((*paramVals)["wheel_lock"].getVal<int>());
    }

    paramIt = (*paramVals).constFind("remote");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbRemote->setChecked((*paramVals)["remote"].getVal<int>());
    }

    paramIt = (*paramVals).constFind("lock");    // To check if this parameter exist
    if (paramIt != ((*paramVals).constEnd()))
    {
        ui.cbUnlock->setChecked((*paramVals)["lock"].getVal<int>());
    }

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int dialogGWInstekPSP::getVals(QMap<QString, ito::Param> * /*paramVals*/)
{
    double dtemp = 0.0;
    int itemp = 0;
//    int len = 0;

    GWInstekPSP *GWI = (GWInstekPSP*) m_dataIO;

    dtemp = ui.dbVoltageLimit->value();
    GWI->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("voltage_limit", ito::ParamBase::Int, dtemp)), NULL);

    dtemp = ui.dbCurrentLimit->value();
    GWI->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("current_limit", ito::ParamBase::Int, dtemp)), NULL);

    dtemp = ui.dbLoadLimit->value();
    GWI->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("load_limit", ito::ParamBase::Int, dtemp)), NULL);

    // Relay on is doing curious things!!!
    //itemp = (int)ui.cbRelayOn->isChecked();
    //GWI->setParam( QSharedPointer<ito::tParam>(new ito::tParam("relay", ito::ParamBase::Int, itemp, itemp, itemp)), NULL);

    itemp = (int)ui.cbSave->isChecked();
    GWI->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("save", ito::ParamBase::Int, itemp)), NULL);

    itemp = (int)ui.cbWheelFine->isChecked();
    GWI->setParam( QSharedPointer<ito::ParamBase>(new ito::ParamBase("wheel", ito::ParamBase::Int, itemp)), NULL);

    return 0;
}

//---------------------------------------------------------------------------------------------------------------------
