/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef QUANTUMCOMPOSER_H
#define QUANTUMCOMPOSER_H

#include "common/addInGrabber.h"
#include <qsharedpointer.h>

//----------------------------------------------------------------------------------------------------------------------------------
/**
 *\class    MyGrabberInterface
 *
 *\brief    Interface-Class for MyGrabber-Class
 *
 *    \sa    AddInDataIO, MyGrabber
 *
 */
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
/**
 *\class    QuantumComposer

 */
class QuantumComposer : public ito::AddInDataIO
{
    Q_OBJECT

protected:
    //! Destructor
    ~QuantumComposer();
    //! Constructor
    QuantumComposer();

    // ito::RetVal retrieveData(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired
    // picture */

public:
    friend class QuantumComposerInterface;
    // const ito::RetVal showConfDialog(void);
    int hasConfDialog(void)
    {
        return 1;
    }; //!< indicates that this plugin has got a configuration dialog

    char* bufferPtr; // this can be a pointer holding the image array from the camera. This buffer
                     // is then copied to the dataObject m_data (defined in AddInGrabber)

private:
    ito::AddInDataIO* m_pSer;
    int m_delayAfterSendCommandMS;
    int m_requestTimeOutMS;

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

    ito::RetVal execFunc(
        const QString funcName,
        QSharedPointer<QVector<ito::ParamBase>> paramsMand,
        QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
        QSharedPointer<QVector<ito::ParamBase>> paramsOut,
        ItomSharedSemaphore* waitCond = NULL);

private slots:
    // void dockWidgetVisibilityChanged(bool visible);
};

#endif // QUANTUMCOMPOSER_H
