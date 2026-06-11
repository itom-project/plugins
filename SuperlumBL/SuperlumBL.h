/* ********************************************************************
    Plugin "SuperlumBL" for itom software
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

#ifndef SUPERLUMBL_H
#define SUPERLUMBL_h

#include "common/addInInterface.h"
#include "common/abstractAddInDockWidget.h"

#include "dialogSuperlumBL.h"
#include "dockWidgetSuperlumBL.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qpair.h>
#include <qbytearray.h>

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    SuperlumBL
  *\brief    ToDo
  *
  *    \sa    AddInDataIO, SuperlumBL,
  *    \date    31.08.2015
  *    \author    J. Krauter, T. Boettcher
  * \warning    NA
  *
  */
class SuperlumBL : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~SuperlumBL() {}
        SuperlumBL();

    public:
        friend class SuperlumBLInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }        //!< indicates that this plugin has got a configuration dialog

    private:

        ito::AddInDataIO *m_pSer;
        int m_delayAfterSendCommandMS;

        enum DeviceType {S_840_B_I_20, Unknown};

        DeviceType m_deviceType;

        DockWidgetSuperlumBL *m_dockWidget;
        //ito::RetVal SerialDummyRead(void); /*!< reads buffer of serial port without delay in order to clear it */
        ito::RetVal SerialSendCommand(QByteArray command);
        ito::RetVal readString(QByteArray &command, QByteArray &result, int &len, int timeoutMS);
        ito::RetVal SendQuestionWithAnswerString(QByteArray questionCommand, QByteArray &answer, int timeoutMS);
        //ito::RetVal SetPos(const int axis, const double posMM, bool relNotAbs, ItomSharedSemaphore *waitCond = NULL);    /*!< Set a position (absolute or relative) */
        //ito::RetVal CheckStatus(void);
        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);
        ito::RetVal IdentifyAndInitializeSystem();

    public slots:

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------
 /**
  *\class    SuperlumBLInterface
  *
  *\brief    Interface-Class for SuperlumBLInterface-Class
  *
  *    \sa    AddInDataIO, SuperlumBL
  *    \date    31.08.2015
  *    \author    J. Krauter, T.Boettcher
  * \warning    NA
  *
  */
class SuperlumBLInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        SuperlumBLInterface();
        ~SuperlumBLInterface() {};
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:


    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // SUPERLUMBL_H
