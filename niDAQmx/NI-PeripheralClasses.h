/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
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

#ifndef NIPeriphralClasses_H
#define NIPeriphralClasses_H

#include <common/retVal.h>

#include <qstring.h>
#include <qvector.h>
#include <qlist.h>

// This is required, since NIDAQmx.h (in the NI library) for linux and apple systems defines
// int64 as long long int. This is incompatible with open_cv, which defines it as int64_t

#if defined(__linux__) || defined(__APPLE__)
    #ifndef _NI_int64_DEFINED_
    #define _NI_int64_DEFINED_   
        typedef int64_t      int64;
    #endif
#endif

#include "NIDAQmx.h" // This is the NI C Library .h file
#include <qobject.h>
#include <qstandarditemmodel.h>
#include "NI-DAQmxError.h"

//---------------------------------------------------------------------------
class NiBaseChannel
{
    // enums
    public:
        enum NiChannelIoType 
        {
            ChIoOutput = 0, 
            ChIoInput = 1 
        };

        enum NiChannelType 
        {
            ChTypeAnalog = 0, 
            ChTypeDigital = 1, 
            ChTypeCounter = 2
        };

        NiBaseChannel(const QString &physicalName, NiChannelType channelType, NiChannelIoType ioType);
        ~NiBaseChannel();

        QString physicalName() const { return m_physicalName; }
        NiChannelType channelType() const { return m_chType; }
        NiChannelIoType ioType() const { return m_chIo; }

        // This routines must be overwritten by derived classes to create a channel and return the current parameter string
        virtual ito::RetVal addChannelToTask(TaskHandle taskHandle) = 0;
        virtual QString getConfigurationString() const = 0;

    protected:
        QString m_physicalName;                // physical name of channel
        NiChannelType m_chType;                // analog, digital, counter
        NiChannelIoType m_chIo;                // 0 = output, 1 = input
};

//---------------------------------------------------------------------------
class NiAnalogInputChannel : public NiBaseChannel
{
    public:
        
        enum NiAITerminalConfig 
        {
            NiTerminalConfDefault = 0, 
            NiTerminalConfDifferential = 1, 
            NiTerminalConfRSE = 2, 
            NiTerminalConfNRSE = 3, 
            NiTerminalConfPseudoDiff = 4,
            NiTerminalConfEndValue = 5 //never used, but this number is always the end of the enumeration in order to check for valid input.
        };

        NiAnalogInputChannel(const QString &physicalName);
        ~NiAnalogInputChannel();
        
        static NiBaseChannel* fromConfigurationString(const QString &configString, ito::RetVal &retValue);

        double getMaxInputLim() { return m_maxInputLim; }
        void setMaxInputLim(const double max) { m_maxInputLim = max; }

        double getMinInputLim() { return m_minInputLim; }
        void setMinInputLim(const double min) { m_minInputLim = min; }

        NiAITerminalConfig getTerminalConfig() { return m_terminalConfig; }
        void setTerminalConfig(const NiAITerminalConfig conf) { m_terminalConfig = conf; }

        virtual ito::RetVal addChannelToTask(TaskHandle taskHandle);
        virtual QString getConfigurationString() const;

    private:
        NiAITerminalConfig m_terminalConfig;
        double m_maxInputLim;
        double m_minInputLim;
        bool m_analogInParamsInitialized = false;

};

//---------------------------------------------------------------------------
class NiAnalogOutputChannel : public NiBaseChannel
{
    public:
        enum NiAnalogOutputMode 
        {
            NiAnOutModeVoltageDC = 0, 
            NiAnOutModeSinewaveGen = 1
        };

        enum NiAnalogOutputTransferMechanism 
        {
            niAnOutTransMechDMA = 0, 
            niAnOutTransMechInterrupts = 1, 
            niAnOutTransMechProgrammedIO = 2, 
            niAnOutTransMechUSBBulk = 3, 
            niAnOutTransMechDefault = 4
        };

        NiAnalogOutputChannel(const QString &physicalName);
        ~NiAnalogOutputChannel();

        static NiBaseChannel* fromConfigurationString(const QString &configString, ito::RetVal &retValue);

        double getMaxOutputLim() { return m_maxOutputLim; };
        void setMaxOutputLim(const double max) { m_maxOutputLim = max; };

        double getMinOutputLim() { return m_minOutputLim; };
        void setMinOutputLim(const double min) { m_minOutputLim = min; };

        virtual ito::RetVal addChannelToTask(TaskHandle taskHandle);
        virtual QString getConfigurationString() const;

    private:
        double m_maxOutputLim;
        double m_minOutputLim;
};

//---------------------------------------------------------------------------
class NiDigitalInputChannel : public NiBaseChannel
{
    public:
        NiDigitalInputChannel(const QString &physicalName);
        ~NiDigitalInputChannel();

        static NiBaseChannel* fromConfigurationString(const QString &configString, ito::RetVal &retValue);

        virtual ito::RetVal addChannelToTask(TaskHandle taskHandle);
        virtual QString getConfigurationString() const;

    private:
        bool m_digitalInParamsInitialized = false;
};

//---------------------------------------------------------------------------
class NiDigitalOutputChannel : public NiBaseChannel
{
    public:
        NiDigitalOutputChannel(const QString &physicalName);
        ~NiDigitalOutputChannel();

        static NiBaseChannel* fromConfigurationString(const QString &configString, ito::RetVal &retValue);

        virtual ito::RetVal addChannelToTask(TaskHandle taskHandle);
        virtual QString getConfigurationString() const;

    private:
        bool m_digitalOutParamsInitialized = false;

};


#endif //#define NI-PeriphralClasses_h


