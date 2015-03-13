/* ********************************************************************
    Plugin "NI-DAQmx" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
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

#ifndef NIPeriphralClasses_H
#define NIPeriphralClasses_H

#include <qstring.h>
#include <qvector.h>
#include <common\retVal.h>
#include "NIDAQmx-LibHeader.h"		// include NI-DAQmx Library functions
#include <qobject.h>
#include <qstandarditemmodel.h>

class niTask
{
	public:
		enum niTaskMode {niTaskModeFinite = 0, niTaskModeContinuous = 1, niTaskModeOnDemand = 2};

		niTask(QString name);
		~niTask();

		int getRateHz(){return m_rateHz;};
		void setRateHz(const int rate){m_rateHz = rate;};

		int getSamplesToRW(){return m_samplesToRW;};
		void setSamplesToRW(const int sTRW){m_samplesToRW = sTRW;};

		int getMode(){return m_mode;};
		void setMode(const int mode){m_mode = mode;};

		QString getTriggerPort(){return m_triggerPort;};
		void setTriggerPort(const QString port){m_triggerPort = port;};

		int getTriggerEdge(){return m_triggerEdge;};
		void setTriggerEdge(const int edge){m_triggerEdge = edge;};

		QString getName(){return m_name;};
		void setName(const QString s){m_name = s;};

		ito::RetVal resetTaskHandle();
		ito::RetVal run();
		bool isDone();
		ito::RetVal stop();
		ito::RetVal free();
		bool isInitialized();
		//niChannelList getChannelPointer();
		TaskHandle* getTaskHandle(){return &m_task;};
		uInt32 getChCount();
		QStringList getChList();
		void channelAdded(const QString name);

		ito::RetVal applyParameters();

	private:
		int m_rateHz;
		int m_samplesToRW;
		int m_mode;
		int m_triggerEdge;
		//niChannelList m_channel;
		QString m_triggerPort;
		QString m_name;
		uInt32 m_chCount;
		TaskHandle m_task;
		QStringList m_chList;
};

class niBaseChannel
{
	// enums
    public:
		enum niChIoType {chIoOutput = 0, chIoInput = 1 };
		enum niChType {chTypeAnalog = 0, chTypeDigital = 1, chTypeCounter = 2};
		//niChannel(const char *virtualChannelStr);
		niBaseChannel(const QString virtChannelName = "");
		~niBaseChannel();

		QString getDevID() {return m_devId;};
		void setDevID(const QString id) {m_devId = id;};

		QString getChID() {return m_chId;};
		void setChID(const QString id) {m_chId = id;};

		QString getName() {return m_chName;};
		void setName(const QString name) {m_chName = name;};

		niChType getChType() {return m_chType;};
		void setChType(const niChType type) {m_chType = type;};

		niChIoType getIoType() {return m_chIo;};
		void setIoType(const niChIoType type) {m_chIo = type;};

		bool errorOccured() {return m_chErrorOcc;};
        //ito::RetVal retValue(void);

		QVector<QString> getDeviceList(); // DAQmxGetDevChassisModuleDevNames

		// This routine must be overwritten by derived classes to Create a Channel
		virtual ito::RetVal applyParameters(niTask *task) {return ito::retError;};
		virtual QStringList getParameters() {return QStringList(0);};
		QVector<QString> getChannelsOfDevice(const QString dev);

    protected:
        QString m_devId;                // device number
        QString m_chId;                 // channel number
		QString m_chName;				// Virtual name of the channel
		niChType m_chType;				// analog, digital, counter
		niChIoType m_chIo;				// 0 = output, 1 = input
        bool m_chErrorOcc;              // 1 = error occurred
        ito::RetVal m_retValue;			// return value by channel creation
};

class niAnalogInputChannel : public niBaseChannel
{
	public:
		
		enum niAnalogInputConfig {niAnInConfDefault = 0, niAnInConfDifferential = 1, niAnInConfRSE = 2, niAnInConfNRSE = 3, niAnInConfPseudoDiff = 4};
		
		niAnalogInputChannel();
		~niAnalogInputChannel();

		double getInputLim(){return m_inputLim;};
		void setInputLim(const double max){m_inputLim = max;};

		int getAnalogInputMode(){return m_analogInputMode;};
		void setAnalogInputMode(const int mode){m_analogInputMode = mode;};

		int getAnalogInputConfig(){return m_analogInputConfig;};
		void setAnalogInputConfig(const int conf){m_analogInputConfig = conf;};

		ito::RetVal applyParameters(niTask *task);
		QStringList getParameters();

	private:
		int m_analogInputConfig;
		double m_inputLim;
		int m_analogInputMode;

};

class niAnalogOutputChannel : public niBaseChannel
{
	public:
		enum niAnalogOutputMode {niAnOutModeVoltageDC = 0, niAnOutModeSinewaveGen = 1};
		enum niAnalogOutputTransferMechanism {niAnOutTransMechDMA = 0, niAnOutTransMechInterrupts = 1, niAnOutTransMechProgrammedIO = 2, niAnOutTransMechUSBBulk = 3, niAnOutTransMechDefault = 4};

		niAnalogOutputChannel();
		~niAnalogOutputChannel();

		int getMaxOutputLim(){return m_maxOutputLim;};
		void setMaxOutputLim(const int max){m_maxOutputLim = max;};

		int getMinOutputLim(){return m_minOutputLim;};
		void setMinOutputLim(const int min){m_minOutputLim = min;};

		ito::RetVal applyParameters(niTask *task);
		QStringList getParameters();

	private:
		int m_maxOutputLim;
		int m_minOutputLim;
};

class niDigitalInputChannel : public niBaseChannel
{
	public:
		niDigitalInputChannel();
		~niDigitalInputChannel();

		ito::RetVal applyParameters(niTask *task);
		QStringList getParameters();

};

class niDigitalOutputChannel : public niBaseChannel
{
	public:
		niDigitalOutputChannel();
		~niDigitalOutputChannel();

		ito::RetVal applyParameters(niTask *task);
		QStringList getParameters();

	private:

};

class niCounterChannel : public niBaseChannel
{
	public:
		niCounterChannel();
		~niCounterChannel();
		
		ito::RetVal applyParameters(niTask *task);
		QStringList getParameters();

	private:
		QString m_edgeSource;
		int m_frequency;
		int m_dutyCycle;
};

class niChannelList : public QMap<QString, niBaseChannel*>
{
	public:
		niChannelList(QString device = "Dev1"); // TODO: Wie kann man hier den default weg lassen? muss dafür die "getchannelsofdevice" static werden?
		~niChannelList();

		void getChannelsOfDevice(const QString dev); // DAQmxGetDevAIPhysicalChans
		
		QVector<niBaseChannel*> getAllChannelOfType(niBaseChannel::niChType chType);
		int getNrOfChannels(niBaseChannel::niChType chType);
		int getNrOfOutputs();
		int getNrOfInputs();
		QStringList getAllChannelAsString();
		QStringList getAllChParameters(niBaseChannel::niChType type,  niBaseChannel::niChIoType io);
		//void setChannelType(QString hardwareCh, niBaseChannel::niChIoType io, niBaseChannel::niChType type);
		//void setMultipleChannelType(int idx, niBaseChannel::niChIoType io, niBaseChannel::niChType type);
		//QStandardItemModel updateModel();
		//QMap<QString, niBaseChannel*> m_channelList;
		
};


#endif //#define NI-PeriphralClasses_h


