/*# ********************************************************************
    Plugin "QuantumComposer" for itom software
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


#ifndef QUANTUMCOMPOSER_H
#define QUANTUMCOMPOSER_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
class QuantumComposerInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

public:
    QuantumComposerInterface();
    ~QuantumComposerInterface();
    ito::RetVal getAddInInst(ito::AddInBase** addInInst);

private:
    ito::RetVal closeThisInst(ito::AddInBase** addInInst);
};


//----------------------------------------------------------------------------------------------------------------------------------
class QuantumComposer : public ito::AddInDataIO
{
    Q_OBJECT

protected:
    //! Destructor
    ~QuantumComposer();
    //! Constructor
    QuantumComposer();

public:
    friend class QuantumComposerInterface;
    const ito::RetVal showConfDialog(void);
    int hasConfDialog(void)
    {
        return 1;
    };

private:
    ito::AddInDataIO* m_pSer;
    int m_delayAfterSendCommandMS;
    int m_requestTimeOutMS;
    int m_numChannels = 8;

    ito::RetVal SendCommand(const QByteArray& command);
    ito::RetVal ReadString(QByteArray& result, int& len, const int timeoutMS);
    ito::RetVal SendQuestionWithAnswerString(
        const QByteArray& questionCommand, QByteArray& answer, const int timeoutMS);
    ito::RetVal SendQuestionWithAnswerDouble(
        const QByteArray& questionCommand, double& answer, const int timeoutMS);
    ito::RetVal SendQuestionWithAnswerInteger(
        const QByteArray& questionCommand, int& answer, const int timeoutMS);

    // exec functions
    ito::RetVal setChannelOutputState(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelWidths(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelDelays(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelSyncs(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelMuxs(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelPolarities(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelOutputModes(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelAdjustableAmplitude(
        ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelModes(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelBurstCounter(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelPulseCounter(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelOffCounter(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelWaitCounter(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelGatesModes(ito::ParamBase& channelIndicesList, ito::ParamBase& valList);
    ito::RetVal setChannelGatesLogicLevel(
        ito::ParamBase& channelIndicesList, ito::ParamBase& valList);

public slots:
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond);
    ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond);
    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = NULL);
    //!< Free buffer, delete board, unload dll
    ito::RetVal close(ItomSharedSemaphore* waitCond);

    ito::RetVal execFunc(
        const QString funcName,
        QSharedPointer<QVector<ito::ParamBase>> paramsMand,
        QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
        QSharedPointer<QVector<ito::ParamBase>> paramsOut,
        ItomSharedSemaphore* waitCond = NULL);

private slots:
     void dockWidgetVisibilityChanged(bool visible);
};

#endif // QUANTUMCOMPOSER_H
