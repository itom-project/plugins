/* ********************************************************************
    Plugin "ThorlabsKCubePA" for itom software
    Copyright (C) 2018, TRUMPF Laser- & Systemtechnik GmbH, Ditzingen

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

#ifndef THORLABSKCUBEPA_H
#define THORLABSKCUBEPA_H

#include "common/addInInterface.h"
#include "common/addInGrabber.h"

#include "dialogThorlabsKCubePA.h"
#include "dockWidgetThorlabsKCubePA.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qlist.h>
#include <qpair.h>
#include <qbytearray.h>
#include <qlibrary.h>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class ThorlabsKCubePAInterface
*/
class ThorlabsKCubePAInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        ThorlabsKCubePAInterface();
        ~ThorlabsKCubePAInterface() {};
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

        ito::RetVal loadDLL();
        ito::RetVal unloadDLL();

        QLibrary mLib;

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class ThorlabsKCubePA
*/
class ThorlabsKCubePA : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ThorlabsKCubePA();
        ~ThorlabsKCubePA() {}

    public:
        friend class ThorlabsKCubePAInterface;

        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 0; } //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_async;
        bool m_opened;
        char m_serialNo[16];
		ito::DataObject m_data;
		bool m_isgrabbing;
        bool m_includeSumSignal;

        static QList<QByteArray> openedDevices;

		ito::RetVal checkError(short value, const char* message);
		ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
		ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    public slots:
		//!< Get Camera-Parameter
		ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
		//!< Set Camera-Parameter
		ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
		//!< Initialise board, load dll, allocate buffer
		ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
		//!< Free buffer, delete board, unload dll
		ito::RetVal close(ItomSharedSemaphore *waitCond);

		//!< Start the camera to enable acquire-commands
		ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
		//!< Stop the camera to disable acquire-commands
		ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
		//!< Softwaretrigger for the camera
		ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
		//!< Wait for acquired picture, copy the picture to dObj of right type and size
		ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);

		ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ThorlabsKCubePA_H
