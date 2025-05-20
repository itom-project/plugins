/* ********************************************************************
    Plugin "SerialIO" for itom software
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

#ifndef SERIALIO_H
#define SERIALIO_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include "dialogSerialIO.h"

#include <qsharedpointer.h>
#include <qbytearray.h>

#ifdef WIN32
#include <windows.h>
#endif

//----------------------------------------------------------------------------------------------------------------------------------
class SerialPort
{
    private:
        struct serParams {
            serParams() :
                port(0),
                baud(9600),
                bits(8),
                parity(0),
                stopbits(1),
                flow(0),
//                debug(0),
//                debugIgnoreEmpty(0),
                sendDelay(0),
                timeout(4000) { endline[0] = '\n'; endline[1] = 0; endline[2] = 0; }
            char port;
            int baud;
            char bits;
            char parity;
            char stopbits;
            char flow;
            char endline[3];
//            char debug;
//            char debugIgnoreEmpty;
            int sendDelay;
            int timeout;
        };
        serParams m_serParams;
        char *m_pDevice;

#ifdef WIN32
        HANDLE m_dev;
#else
        int m_dev;
#endif

    public:
        enum PortType { COM, TTYS, TTYUSB, TTYACM }; //COM is for windows, TTYS is a serial port on linux, TTYUSB is a usb-serial port on linux
        SerialPort() : m_pDevice(0), m_dev(0) {}
        const ito::RetVal sopen(const int port, const int baud, const char* endline, const int bits, const int stopbits, const int parity, const int flow, const int sendDelay, const int timeout, PortType &portType);
        const ito::RetVal sclose(void);
        const ito::RetVal sread(char *buf, int *len, const int sendDelay);
        int sreadable(void) const;
        const ito::RetVal swrite(const char c) const;
        const ito::RetVal swrite(const char *buf, const int len, const int sendDelay) const;
        const ito::RetVal setparams(const serParams &params);
        const ito::RetVal setparams(const int baud, const char* endline, const int bits = 8, const int stopbits = 0, const int parity = 0, const int flow = 0, const int sendDelay = 0, const int timeout = 4000);
        int isOpen() { return m_dev != 0 ? 1 : 0; }
        const ito::RetVal sclearbuffer(int BufferType);
        const ito::RetVal getendline(char *eline);
        const bool isValidBaudRate(const int baud);

        static int baudRates[];
        int m_baudRatesSize;
};

//----------------------------------------------------------------------------------------------------------------------------------
class SerialIO : public ito::AddInDataIO //, public DummyGrabberInterface
{
    Q_OBJECT

    protected:
        virtual ~SerialIO();
        SerialIO();

    public:
        friend class SerialIOInterface;
//        friend class SerialPort;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; } //!< indicates that this plugin has got a configuration dialog
        int isOpen() { return m_serport.isOpen(); }

    private:
        SerialPort m_serport;
        bool m_debugMode;   /*! Enables / Disables live connection to dockingwidge-protocol */
        bool m_debugIgnoreEmpty;   /*! Enables / Disables to ignore empty messages */
        static int m_instCounter;
        QByteArray m_preBuf;

    signals:
        void serialLog(QByteArray data, QByteArray endline, bool incomingData);
        void uniqueIDChanged(const int);
        //void parametersChanged(QMap<QString, ito::tParam>); (defined in AddInBase)

    public slots:
/*
        ito::RetVal getParam(const char *name, QSharedPointer<double> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getParam(const char *name, QSharedPointer<char> val, QSharedPointer<int> len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const char *val, const int len, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(const char *name, const double val, ItomSharedSemaphore *waitCond = NULL);
*/
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitConde = NULL);

        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
class SerialIOInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        SerialIOInterface();
        ~SerialIOInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // SERIALIO_H
