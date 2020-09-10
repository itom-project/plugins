/* ********************************************************************
itom software
URL: http://www.uni-stuttgart.de/ito
Copyright (C) 2020, Institut fuer Technische Optik (ITO),,
Universität Stuttgart, Germany

This file is part of itom and its software development toolkit (SDK).

itom is free software; you can redistribute it and/or modify it
under the terms of the GNU Library General Public Licence as published by
the Free Software Foundation; either version 2 of the Licence, or (at
your option) any later version.

In addition, as a special exception, the Institut für Technische
Optik (ITO) gives you certain additional rights.
These rights are described in the ITO LGPL Exception version 1.0,
which can be found in the file LGPL_EXCEPTION.txt in this package.

itom is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
General Public Licence for more details.

You should have received a copy of the GNU Library General Public License
along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#ifndef OPHIRSERIALPlUGIN_H
#define OPHIRSERIALPlUGIN_H

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include "dialogOphirSerialPlugin.h"
#include "dockWidgetOphirSerialPlugin.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

#include "DataObject/dataobj.h"


//----------------------------------------------------------------------------------------------------------------------------------
class OphirSerialPlugin : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        //! Destructor
        ~OphirSerialPlugin() {}
        //! Constructor
        OphirSerialPlugin();
        
        //ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        
    public:
        friend class OphirSerialPluginInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
        
        ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private:
        bool m_isgrabbing; /*!< Check if acquire was executed */
        ito::AddInDataIO *m_pSer;
        int m_delayAfterSendCommandMS;
        ito::DataObject m_data;

        DockWidgetOphirSerialPlugin *m_dockWidget;
        ito::RetVal SerialSendCommand(QByteArray command);
        ito::RetVal readString(QByteArray &result, int &len, int timeoutMS);
        ito::RetVal SendQuestionWithAnswerInt(QByteArray questionCommand, int &answer, int timeoutMS);
        ito::RetVal SendQuestionWithAnswerDouble(QByteArray questionCommand, double &answer, int timeoutMS);
        ito::RetVal SendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS);

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
        void dockWidgetVisibilityChanged(bool visible);
    signals:
        void visibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
class OphirSerialPluginInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
        Q_INTERFACES(ito::AddInInterfaceBase)
        PLUGIN_ITOM_API

protected:

public:
    OphirSerialPluginInterface();
    ~OphirSerialPluginInterface() {};
    ito::RetVal getAddInInst(ito::AddInBase **addInInst);

private:
    ito::RetVal closeThisInst(ito::AddInBase **addInInst);

signals:


    public slots :
};

#endif // OPHIRSERIALPLUGIN_H
