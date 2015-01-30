/* ********************************************************************
    Plugin "IDSuEye" for itom software
    URL: http://www.bitbucket.org/itom/plugins
    Copyright (C) 2014, Pulsar Photonics GmbH, Aachen
	Copyright (C) 2014, Institut für Technische Optik, Universität Stuttgart

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

#ifndef DIALOGIDS_H
#define DIALOGIDS_H

#include "common/param.h"
#include "common/retVal.h"
#include "common/sharedStructuresQt.h"
#include "common/abstractAddInConfigDialog.h"

#include "dialogNI-DAQmx.h"

#include "ui_dialogNI-DAQmx.h"

#include <qstring.h>
#include <qmap.h>
#include <qabstractbutton.h>

#include "NI-DAQmx.h"


namespace ito
{
    class AddInBase; //forward declaration
}

class DialogNiDAQmx : public ito::AbstractAddInConfigDialog 
{
    Q_OBJECT

    public:
		DialogNiDAQmx(ito::AddInBase *grabber, void *plugin);
        ~DialogNiDAQmx() {};

        ito::RetVal applyParameters();

    private:
        void enableDialog(bool enabled);
        
		void *m_pPlugin;

		bool m_firstRun;
		QMap<QString, ito::Param> m_params;
		

		Ui::niDAQmx ui;

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:
		void on_taskApplyButton_clicked(bool checked = false);
		void on_taskCombo_currentIndexChanged(int index);

		void on_aiApplyButton_clicked(bool checked = false);
		void on_aiChannelCombo_currentIndexChanged(int index);
		
		void on_aoApplyButton_clicked(bool checked = false);
		void on_aoChannelCombo_currentIndexChanged(int index);

		void on_dioApplyButton_clicked(bool checked = false);
		void on_dioChannelCombo_currentIndexChanged(int index);

		void on_cioApplyButton_clicked(bool checked = false);
		void on_cioChannelCombo_currentIndexChanged(int index);

		void on_aiBitCombo_currentIndexChanged(int index);
		void on_aiRangeCombo_currentIndexChanged(int index);

		void calculateResolution();

};

#endif
