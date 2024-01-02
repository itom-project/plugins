/*/* ********************************************************************
    Plugin "NewportConexLDS" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2024, Institut für Technische Optik (ITO),
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

#ifndef NEWPORTCONEXLDS_H
#define NEWPORTCONEXLDS_H

#include "common/addInGrabber.h"
#include "dialogNewportConexLDS.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/**
 *\class    NewportConexLDSInterface
 *
 *\brief    Interface-Class for NewportConexLDS-Class
 *
 *    \sa    AddInDataIO, NewportConexLDS
 *
 */
class NewportConexLDSInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

public:
    NewportConexLDSInterface();
    ~NewportConexLDSInterface();
    ito::RetVal getAddInInst(ito::AddInBase** addInInst);

private:
    ito::RetVal closeThisInst(ito::AddInBase** addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
/**
 *\class    NewportConexLDS

 */
class NewportConexLDS : public ito::AddInGrabber
{
    Q_OBJECT

protected:
    //! Destructor
    ~NewportConexLDS();
    //! Constructor
    NewportConexLDS();

    ito::RetVal retrieveData(
        ito::DataObject* externalDataObject = NULL); /*!< Wait for acquired picture */

public:
    friend class NewportConexLDSInterface;
    const ito::RetVal showConfDialog(void);
    int hasConfDialog(void)
    {
        return 1;
    }; //!< indicates that this plugin has got a configuration dialog

    char* bufferPtr; // this can be a pointer holding the image array from the camera. This buffer
                     // is then copied to the dataObject m_data (defined in AddInGrabber)

private:
    bool m_isgrabbing; /*!< Check if acquire was executed */


public slots:
    //!< Get Camera-Parameter
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond);
    //!< Set Camera-Parameter
    ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond);
    //!< Initialise board, load dll, allocate buffer
    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = NULL);
    //!< Free buffer, delete board, unload dll
    ito::RetVal close(ItomSharedSemaphore* waitCond);

    //!< Start the camera to enable acquire-commands
    ito::RetVal startDevice(ItomSharedSemaphore* waitCond);
    //!< Stop the camera to disable acquire-commands
    ito::RetVal stopDevice(ItomSharedSemaphore* waitCond);
    //!< Softwaretrigger for the camera
    ito::RetVal acquire(const int trigger, ItomSharedSemaphore* waitCond = NULL);
    //!< Wait for acquired picture, copy the picture to dObj of right type and size
    ito::RetVal getVal(void* vpdObj, ItomSharedSemaphore* waitCond);

    ito::RetVal copyVal(void* vpdObj, ItomSharedSemaphore* waitCond);

    // checkData usually need not to be overwritten (see comments in source code)
    // ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};

#endif // NEWPORTCONEXLDS_H
