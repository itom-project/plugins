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
class NewportConexLDS : public ito::AddInDataIO
{
    Q_OBJECT

protected:
    //! Destructor
    ~NewportConexLDS();
    //! Constructor
    NewportConexLDS();

public:
    friend class NewportConexLDSInterface;
    const ito::RetVal showConfDialog(void);
    int hasConfDialog(void)
    {
        return 1;
    }; //!< indicates that this plugin has got a configuration dialog

private:
    ito::AddInDataIO* m_pSerialIO;
    int m_delayAfterSendCommandMS;
    int m_requestTimeOutMS;
    int m_controllerAddress = 1;

    enum ConfigurationState
    {
        MEASURE = 28,
        READY = 32,
        CONFIGURATION = 14
    };

    // SeralIO functions
    ito::RetVal sendCommand(const QByteArray& command);
    ito::RetVal readString(QByteArray& result, int& len);
    ito::RetVal sendQuestionWithAnswerString(const QByteArray& questionCommand, QByteArray& answer);
    ito::RetVal sendQuestionWithAnswerDouble(const QByteArray& questionCommand, double& answer);
    ito::RetVal sendQuestionWithAnswerDoubleArray(
        const QByteArray& questionCommand, double* answer, const int number);
    ito::RetVal sendQuestionWithAnswerInteger(const QByteArray& questionCommand, int& answer);
    void filterCommand(const QByteArray& questionCommand, QByteArray& answer);

    // Conex get functions
    ito::RetVal getVersion(QString& version, QString& deviceName);
    ito::RetVal getLaserPowerState(int& state);
    ito::RetVal getFactoryCalibrationState(QString& state);
    ito::RetVal getGain(ito::float64* gain);
    ito::RetVal getOffset(ito::float64* offset);
    ito::RetVal getFrequency(ito::float64& frequency);
    ito::RetVal getPositionAndLaserPower(ito::float64* values);
    ito::RetVal getCalibrationCoefficients(ito::float64* calibrationCoefficients);
    ito::RetVal getRange(int& range);
    ito::RetVal getLowLevelPowerThreshold(int& level);
    ito::RetVal getHighLevelPowerThreshold(int& level);
    ito::RetVal getUnit(QString& unit);
    ito::RetVal getError(QString& error);
    ito::RetVal getConfigurationState(ConfigurationState& state);

    // Conex set functions
    ito::RetVal setLaserPowerState(const int state);
    ito::RetVal setGain(const ito::float64* gain);
    ito::RetVal setOffset(const ito::float64* offset);
    ito::RetVal setFrequency(const ito::float64 frequency);
    ito::RetVal setCalibrationCoefficients(const ito::float64* calibrationCoefficients);
    ito::RetVal setRange(const int& range);
    ito::RetVal setLowLevelPowerThreshold(const int& level);
    ito::RetVal setHighLevelPowerThreshold(const int& level);
    ito::RetVal setUnit(const QString& unit);
    ito::RetVal setConfigurationState(const int& state);

    // exec functions
    ito::RetVal execGetPositionAndPower(
        ito::ParamBase& positionAndPower, ito::ParamBase& timeStamp);
    ito::RetVal execGetPositionAndPowerArray(
        ito::DataObject& data, ito::ParamBase& timeStamps, const int& interval);

    QString configurationEnumToString(const ConfigurationState& state);

public slots:
    ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore* waitCond);
    ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore* waitCond);
    ito::RetVal init(
        QVector<ito::ParamBase>* paramsMand,
        QVector<ito::ParamBase>* paramsOpt,
        ItomSharedSemaphore* waitCond = NULL);
    ito::RetVal close(ItomSharedSemaphore* waitCond);

    ito::RetVal execFunc(
        const QString funcName,
        QSharedPointer<QVector<ito::ParamBase>> paramsMand,
        QSharedPointer<QVector<ito::ParamBase>> paramsOpt,
        QSharedPointer<QVector<ito::ParamBase>> paramsOut,
        ItomSharedSemaphore* waitCond = NULL);

    ito::RetVal autoGrabbing(QSharedPointer<ito::float64> values, ItomSharedSemaphore* waitCond);

private slots:
    void dockWidgetVisibilityChanged(bool visible);
};

#endif // NEWPORTCONEXLDS_H
