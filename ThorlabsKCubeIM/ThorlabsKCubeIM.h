/* ********************************************************************
    Plugin "ThorlabsISM" for itom software
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

#ifndef THORLABSKCUBEIM_H
#define THORLABSKCUBEIM_H

#define NOMINMAX // https://stackoverflow.com/questions/22744262/cant-call-stdmax-because-minwindef-h-defines-max

#include "common/addInInterface.h"

#include "dialogThorlabsKCubeIM.h"
#include "dockWidgetThorlabsKCubeIM.h"

#include <qsharedpointer.h>
#include <qvector.h>
#include <qlist.h>
#include <qpair.h>
#include <qbytearray.h>
#include <qlibrary.h>

#include "Thorlabs.MotionControl.KCube.InertialMotor.h"

//----------------------------------------------------------------------------------------------------------------------------------
/** @class ThorlabsKCubeIMInterface
*/
class ThorlabsKCubeIMInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        ThorlabsKCubeIMInterface();
        ~ThorlabsKCubeIMInterface() {};
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

        QLibrary mLib;

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class ThorlabsKCubeIM
*/
class ThorlabsKCubeIM : public ito::AddInActuator
{
    Q_OBJECT

    protected:
        ThorlabsKCubeIM();
        ~ThorlabsKCubeIM() {}

    public:
        friend class ThorlabsKCubeIMInterface;

        const ito::RetVal showConfDialog(void);    /*!<shows the configuration dialog*/
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog

    private:
        bool m_async;
        bool m_opened;
        int m_numaxis;
        char m_serialNo[16];
        int m_pollingInterval = 150;

        static QList<QByteArray> openedDevices;

        enum MoveType { Absolute = 0, Relative = 1 };

        ito::RetVal waitForDone(const int timeoutMS = -1, const int axis = -1, const int flags = -1 /*for your use*/);
        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = -1 /*for your use*/);
        ito::RetVal checkError(short value, const char *message);

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //! Starts calibration for a single axis
        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = NULL);
        //! Starts calibration for all axis -> calls single axis version
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);
        //! Not implelemted yet
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = NULL);
        //! Not implelemted yet
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = NULL);
        //! Reads out status request answer and gives back ito::retOk or ito::retError
        ito::RetVal getStatus(QSharedPointer<QVector<int>> status, ItomSharedSemaphore *waitCond);
        //! Get the position of a single axis
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        //! Get the position of a all axis -> calls single axis version
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);
        //! Set an absolut position and go thier. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set an absolut position and go thier. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls SMCSetPos of axis=0 else ito::retError
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = NULL);
        //! Set a relativ offset of current position and go thier. Waits if m_async=0. Calls SMCSetPos of axis[0]=0 && axis.size()=1 else ito::retError
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = NULL);

        //! Emits status and position if triggered. Used form the dockingwidget
        ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);

    private slots:
        void dockWidgetVisibilityChanged( bool visible );
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // ThorlabsKCubeIM_H
