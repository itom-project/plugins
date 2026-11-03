/* ********************************************************************
    Plugin "DummyMultiChannelGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2023, Institut fuer Technische Optik (ITO),
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

#pragma once

#include "common/addInMultiChannelGrabber.h"
#include "common/typeDefs.h"
#include "dialogDummyMultiChannelGrabber.h"
#include "cameraEmulator.h"

#include <qsharedpointer.h>
#include <qtimer.h>


//-------------------------------------------------------------------------------------
class DummyMultiChannelGrabber : public ito::AddInMultiChannelGrabber
{
    Q_OBJECT

protected:
    ~DummyMultiChannelGrabber();
    DummyMultiChannelGrabber();

    ito::RetVal getParameter(
        QSharedPointer<ito::Param> val,
        const ParamMapIterator& it,
        const QString& key,
        const QString& suffix,
        int index,
        bool hasIndex,
        bool& ok);

    ito::RetVal setParameter(
        QSharedPointer<ito::ParamBase>& val,
        const ParamMapIterator& it,
        const QString& suffix,
        const QString& key,
        int index,
        bool hasIndex,
        bool& ok,
        QStringList& pendingUpdate);

    ito::RetVal retrieveData(const QStringList& channels = QStringList());

public:
    friend class DummyMultiChannelGrabberInterface;

    const ito::RetVal showConfDialog(void);

    //!< indicates that this plugin has got a configuration dialog
    int hasConfDialog(void) { return 1; };

private:
    CameraEmulator m_camEmulator;

signals:

public slots:

    ito::RetVal init(QVector<ito::ParamBase>* paramsMand, QVector<ito::ParamBase>* paramsOpt, ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal close(ItomSharedSemaphore* waitCond);

    ito::RetVal startDevice(ItomSharedSemaphore* waitCond);
    ito::RetVal stopDevice(ItomSharedSemaphore* waitCond);
    ito::RetVal acquire(const int trigger, ItomSharedSemaphore* waitCond = nullptr);


private slots:
    void dockWidgetVisibilityChanged(bool visible);
};
