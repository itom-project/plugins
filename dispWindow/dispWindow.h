/* ********************************************************************
    Plugin "dispWindow" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2022, Institut fuer Technische Optik (ITO),
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

#include "DataObject/dataobj.h"
#include "common/addInInterface.h"
#include "dialogDispWindow.h"
#include "dockWidgetDispWindow.h"
#include "projWindow.h"

//-------------------------------------------------------------------------------------
class DispWindowInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

protected:
public:
    DispWindowInterface();
    ~DispWindowInterface();
    ito::RetVal getAddInInst(ito::AddInBase** addInInst);

private:
    ito::RetVal closeThisInst(ito::AddInBase** addInInst);

signals:

public slots:
};

//-------------------------------------------------------------------------------------
class DispWindow : public ito::AddInDataIO
{
    Q_OBJECT

private:
    PrjWindow* m_pWindow;

    ito::RetVal constructionResult;

protected:
    ~DispWindow();
    DispWindow();

    ito::RetVal interpolateLUT(QVector<double>& grayvalues, QVector<unsigned char>& lut);

public:
    friend class DispWindowInterface;
    const ito::RetVal showConfDialog(void);
    int hasConfDialog(void)
    {
        return 1;
    }; //!< indicates that this plugin has got a configuration dialog
    void* getPrjWindow(void)
    {
        return (void*)m_pWindow;
    }

signals:

public slots:
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal setParam(
        QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond = nullptr);

    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = nullptr);
    ito::RetVal close(ItomSharedSemaphore* waitConde = nullptr);

    ito::RetVal getVal(void* data, ItomSharedSemaphore* waitCond);
    ito::RetVal setVal(const void* data, ItomSharedSemaphore* waitCond);
    void numberOfImagesChanged(
        int numImg, int numGray, int numCos); /** Slot to archieve data from the child-GL-window*/

    ito::RetVal execFunc(
        const QString funcName,
        QSharedPointer<QVector<ito::ParamBase>> paramsMand,
        QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
        QSharedPointer<QVector<ito::ParamBase>> paramsOut,
        ItomSharedSemaphore* waitCond = nullptr);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};
