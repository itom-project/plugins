/* ********************************************************************
    Plugin "OphirSerialPlugin" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2020, Institut fuer Technische Optik (ITO),
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

#include "dockWidgetOphirSerialPlugin.h"

#include "common/addInInterface.h"

#include <qmessagebox.h>
#include <qmetaobject.h>

//-------------------------------------------------------------------------------------------------------------------------------------------------
DockWidgetOphirSerialPlugin::DockWidgetOphirSerialPlugin(int uniqueID, ito::AddInBase *actuator) :  
    AbstractAddInDockWidget(actuator),  
    m_inEditing(false),  
    m_firstRun(true)
{
     ui.setupUi(this); 

     identifierChanged(QString::number(uniqueID));

     enableWidget(true);
}

 //-------------------------------------------------------------------------------------------------------------------------------------------------
 void DockWidgetOphirSerialPlugin::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {

        ui.lblComPort->setText(QString::number(params["comPort"].getVal<int>()));
        ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());
        ui.lblROM->setText(params["ROMVersion"].getVal<char*>());
        ui.lblDeviceType->setText(params["deviceType"].getVal<char*>());
        ui.lblHead->setText(params["headType"].getVal<char*>());

		ito::IntMeta *intmeta = static_cast<ito::IntMeta*>(params["range"].getMeta());
		ui.comboBoxRange->clear();
		int count = 0;
		for (int i = intmeta->getMin(); i <= intmeta->getMax(); i += intmeta->getStepSize())
		{
			ui.comboBoxRange->addItem(QString::number(i));
		}
		int index = ui.comboBoxRange->findText(QString::number(params["range"].getVal<int>()));
		ui.comboBoxRange->setCurrentIndex(index);

		ito::StringMeta* sm = (ito::StringMeta*)(params["measurementType"].getMeta());
		for (int x = 0; x < sm->getLen(); x++)
		{
			ui.comboBoxMeasurementType->addItem(sm->getString(x));
			int index = ui.comboBoxMeasurementType->findText(params["measurementType"].getVal<char*>());
			ui.comboBoxMeasurementType->setCurrentIndex(index);
		}

		char* set = params["wavelengthSet"].getVal<char*>();
		ui.lblWavelengthSet->setText(set);

		if (QString::fromLatin1(set).compare("CONTINUOUS") == 0)
		{
			ui.stackedWidgetWavelengthSet->setCurrentIndex(0);
		}
		else
		{
			ui.stackedWidgetWavelengthSet->setCurrentIndex(1);

			ito::StringMeta* sm = (ito::StringMeta*)(params["wavelength"].getMeta());
			for (int x = 0; x < sm->getLen(); x++)
			{
				ui.comboBoxWavelength->addItem(sm->getString(x));
			}
			int index = ui.comboBoxWavelength->findText(params["wavelength"].getVal<char*>());
			ui.comboBoxWavelength->setCurrentIndex(index);
		}

		ui.lblUnit->setText(params["unit"].getVal<char*>());

        m_firstRun = false;
    }

    //__________________________________________________________________________________________________________ Editing
    if (!m_inEditing)
    {
        m_inEditing = true;

        
        m_inEditing = false;
    }

}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirSerialPlugin::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetOphirSerialPlugin::enableWidget(bool enabled)
{
    
}
