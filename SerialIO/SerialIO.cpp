/* ********************************************************************
    Plugin "SerialIO" for itom software
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

/********** Serial Port *****************
#   1         5
#   /---------\      1 DCD    6 DSR 
#  | . . . . . |     2 RxD    7 RTS
#   \ . . . . /      3 TxD    8 CTS
#    \       /       4 DTR    9 RI
#     -------        5 GND
#     6     9
*/

#define ITOM_IMPORT_API
#define ITOM_IMPORT_PLOTAPI

#include "SerialIO.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>
#include <qapplication.h>
#include <qelapsedtimer.h>

#include "pluginVersion.h"
#include "gitVersion.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

#ifdef WIN32
    #include <Windows.h>
//#define CCTS_OFLOW      0x00010000      /* CTS flow control of output */
//#define CRTSCTS         (CCTS_OFLOW | CRTS_IFLOW)
//#define CRTS_IFLOW      0x00020000      /* RTS flow control of input */
#else
    #include <unistd.h>
//    #include <stdio.h>
    #include <termios.h>
//    #include <sys/stat.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
#endif

#include "dockWidgetSerialIO.h"

/*static*/ int SerialPort::baudRates[] = 
{
    50,
    75,
    110,
    134,
    150,
    200,
    300,
    600,
    1200,
    1800,
    2400,
    4800,
    9600,
    19200,
    38400,
    57600,
    76800,
    115200,
    230400,
    460800,
    500000,
    576000,
    921600,
    1000000,
    1152000,
    1500000,
    2000000,
    2500000,
    3000000,
    3500000,
    4000000
};

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const bool SerialPort::isValidBaudRate(const int baud)
{
    int i = 0;
    while (i < m_baudRatesSize && baudRates[i] != baud)
    {
        ++i;
    }

    return i < m_baudRatesSize;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::setparams(const SerialPort::serParams &params)
{
    ito::RetVal ret(ito::retOk);
#ifndef WIN32
    struct termios options;      // Structure with the device's options

    // Set parameters
    tcgetattr(m_dev, &options);    // Get the current options of the port

    // set up other port settings
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= (~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG));
    options.c_iflag &= (~(INPCK | IGNPAR | PARMRK | ISTRIP | ICRNL | IXANY));
    options.c_oflag &= (~OPOST);
    options.c_cc[VMIN] = 0;
#ifdef _POSIX_VDISABLE
    // Is a disable character available on this system?
    // Some systems allow for per-device disable-characters, so get the
    //  proper value for the configured device
    const long vdisable = fpathconf(m_dev, _PC_VDISABLE);
    options.c_cc[VINTR] = vdisable;
    options.c_cc[VQUIT] = vdisable;
    options.c_cc[VSTART] = vdisable;
    options.c_cc[VSTOP] = vdisable;
    options.c_cc[VSUSP] = vdisable;
#endif //_POSIX_VDISABLE

    switch (params.baud)        // Set the speed (Bauds)
    {
        case 50:
            cfsetispeed(&options, B50);
            cfsetospeed(&options, B50);
        break;
        case 75:
            cfsetispeed(&options, B75);
            cfsetospeed(&options, B75);
        break;
        case 110:
            cfsetispeed(&options, B110);
            cfsetospeed(&options, B110);
        break;
        case 134:
            cfsetispeed(&options, B134);
            cfsetospeed(&options, B134);
        break;
        case 150:
            cfsetispeed(&options, B150);
            cfsetospeed(&options, B150);
        break;
        case 200:
            cfsetispeed(&options, B200);
            cfsetospeed(&options, B200);
        break;
        case 300:
            cfsetispeed(&options, B300);
            cfsetospeed(&options, B300);
        break;
        case 600:
            cfsetispeed(&options, B600);
            cfsetospeed(&options, B600);
        break;
        case 1200:
            cfsetispeed(&options, B1200);
            cfsetospeed(&options, B1200);
        break;
        case 1800:
            cfsetispeed(&options, B1800);
            cfsetospeed(&options, B1800);
        break;
        case 2400:
            cfsetispeed(&options, B2400);
            cfsetospeed(&options, B2400);
        break;
        case 4800:
            cfsetispeed(&options, B4800);
            cfsetospeed(&options, B4800);
        break;
        case 9600:
            cfsetispeed(&options, B9600);
            cfsetospeed(&options, B9600);
        break;
        case 19200:
            cfsetispeed(&options, B19200);
            cfsetospeed(&options, B19200);
        break;
        case 38400:
            cfsetispeed(&options, B38400);
            cfsetospeed(&options, B38400);
        break;
        case 57600:
            cfsetispeed(&options, B57600);
            cfsetospeed(&options, B57600);
        break;
        case 115200:
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
        break;
        case 230400:
            cfsetispeed(&options, B230400);
            cfsetospeed(&options, B230400);
        break;
#ifndef __APPLE__
        case 460800:
            cfsetispeed(&options, B460800);
            cfsetospeed(&options, B460800);
        break;
        case 500000:
            cfsetispeed(&options, B500000);
            cfsetospeed(&options, B500000);
        break;
        case 576000:
            cfsetispeed(&options, B576000);
            cfsetospeed(&options, B576000);
        break;
        case 921600:
            cfsetispeed(&options, B921600);
            cfsetospeed(&options, B921600);
        break;
        case 1000000:
            cfsetispeed(&options, B1000000);
            cfsetospeed(&options, B1000000);
        break;
        case 1152000:
            cfsetispeed(&options, B1152000);
            cfsetospeed(&options, B1152000);
        break;
        case 1500000:
            cfsetispeed(&options, B1500000);
            cfsetospeed(&options, B1500000);
        break;
        case 2000000:
            cfsetispeed(&options, B2000000);
            cfsetospeed(&options, B2000000);
        break;
        case 2500000:
            cfsetispeed(&options, B2500000);
            cfsetospeed(&options, B2500000);
        break;
        case 3000000:
            cfsetispeed(&options, B3000000);
            cfsetospeed(&options, B3000000);
        break;
        case 3500000:
            cfsetispeed(&options, B3500000);
            cfsetospeed(&options, B3500000);
        break;
        case 4000000:
            cfsetispeed(&options, B4000000);
            cfsetospeed(&options, B4000000);
            break;
#endif // __APPLE__
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid baud rate").toLatin1().data());
    }
    m_serParams.baud = params.baud;

    switch (params.bits)
    {
        case 5:
            options.c_cflag &= (~CSIZE);
            options.c_cflag |= CS5;
        break;
        case 6:
            options.c_cflag &= (~CSIZE);
            options.c_cflag |= CS6;
        break;
        case 7:
            options.c_cflag &= (~CSIZE);
            options.c_cflag |= CS7;
        break;
        case 8:
            options.c_cflag &= (~CSIZE);
            options.c_cflag |= CS8;
        break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of bits").toLatin1().data());
    }
    m_serParams.bits = params.bits;

    switch (params.stopbits)
    {
        case 1:
            options.c_cflag &= (~CSTOPB);
        break;
        case 2:
            options.c_cflag |= (CSTOPB);
        break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of stopbits").toLatin1().data());
    }
    m_serParams.stopbits = params.stopbits;

    switch (params.parity)
    {
        // no parity
        case 0:
            options.c_cflag &= (~PARENB);
        break;
        // odd parity
        case 1:
            options.c_cflag |= (PARENB|PARODD);
        break;
        // even parity
        case 2:
            options.c_cflag &= (~PARODD);
            options.c_cflag |= PARENB;
        break;
        // mark parity
        case 3:
            options.c_cflag |= PARENB | CMSPAR | PARODD;
        break;
        // space parity
        case 4:
            options.c_cflag |= PARENB | CMSPAR;
            options.c_cflag &= ~PARODD;
        break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid parity").toLatin1().data());
    }
    m_serParams.parity = params.parity;

// http://svn.codeskin.com/pub/rxtx/tags/rxtx-2.1-7r2-DTR-patch/termios.c
// http://pronix.linuxdelta.de/C/Linuxprogrammierung/Linuxsystemprogrammieren_C_Kurs_Kapitel6.shtml
// http://www.koders.com/cpp/fidD25BB0849B79991C7049EB1A3E2583EBB9CF6034.aspx

    // xon/xoff - software flow controll
    if (params.flow & 1)
    {
        options.c_iflag |= (IXON|IXOFF|IXANY);
    }
    else
    {
        options.c_iflag &= (~(IXON|IXOFF|IXANY));
    }

    // rts
    options.c_cflag &= (~CRTSCTS);
    if ((params.flow & 6) == 0)
    {
        options.c_iflag &= (~0x00020000);
    }
    else if ((params.flow & 6) == 2)
    {
        options.c_iflag |= 0x00020000;
        options.c_iflag |= CRTSCTS;
    }
    else
    {
        options.c_iflag |= (0x00020000);
    }

    // cts
    if (params.flow & 8)
    {
        options.c_iflag |= (0x00010000);
    }
    else
    {
        options.c_iflag &= (~0x00010000);
    }

    // dtr
    if ((params.flow & 48) == 0)
    {
        options.c_iflag &= (~0x00040000);
    }
    else if ((params.flow & 48) == 16)
    {
        options.c_iflag |= (0x00040000);
        options.c_iflag |= CRTSCTS;
    }
    else
    {
        options.c_iflag |= (0x00040000);
    }

    // dsr
    if (params.flow & 64)
    {
        options.c_iflag |= (0x00080000);
    }
    else
    {
        options.c_iflag &= (~0x00080000);
    }

    m_serParams.flow = params.flow;

    tcsetattr(m_dev, TCSAFLUSH, &options);
    options.c_cc[VTIME] = params.timeout;    // Set timeout in [ms]
//    options.c_cc[VTIME] = 0;    // Set timeout
    options.c_cc[VMIN] = 0;                  // At least on character before satisfy reading
    tcsetattr(m_dev, TCSANOW, &options);     // Activate the settings
#else
    // port setup
    DCB dcbSerialParams = {0};                                          // Structure for the port parameters
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(m_dev, &dcbSerialParams))                         // Get the port parameters
    {
        return -2;                                                      // Error while getting port parameters
    }

    if (SerialPort::isValidBaudRate(params.baud))
    {
        // Set the speed (Bauds)
        dcbSerialParams.BaudRate = params.baud;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("invalid baud rate").toLatin1().data());
    }
    m_serParams.baud = params.baud;

    if (params.bits > 4 && params.bits < 9)
    {
        dcbSerialParams.ByteSize = params.bits;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of bits").toLatin1().data());
    }
    m_serParams.bits = params.bits;

    switch (params.stopbits)
    {
        case 1: dcbSerialParams.StopBits = ONESTOPBIT;      break;
        case 2: dcbSerialParams.StopBits = TWOSTOPBITS;     break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of stopbits").toLatin1().data());
        // this is windows specific so we will do 1 and 2 as intuitiv standard
        //case 3: dcbSerialParams.StopBits = ONE5STOPBITS;    break;
    }
    m_serParams.stopbits = params.stopbits;

    switch (params.parity)
    {
        case 0: dcbSerialParams.Parity = NOPARITY;      break;
        case 1: dcbSerialParams.Parity = ODDPARITY;     break;
        case 2: dcbSerialParams.Parity = EVENPARITY;    break;
        case 3: dcbSerialParams.Parity = MARKPARITY;    break;
        case 4: dcbSerialParams.Parity = SPACEPARITY;   break;
        // space parity and mark parity not supported
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid parity").toLatin1().data());
    }
    m_serParams.parity = params.parity;

    // xon/xoff - software flow controll
    if (params.flow & 1)
    {
        dcbSerialParams.fInX = TRUE;
        dcbSerialParams.fOutX = TRUE;
    }
    else
    {
        dcbSerialParams.fInX = FALSE;
        dcbSerialParams.fOutX = FALSE;
    }

    // rts
    if ((params.flow & 6) == 0)
    {
        dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;
    }
    else if ((params.flow & 6) == 2)
    {
        dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;
    }
    else
    {
        dcbSerialParams.fRtsControl = RTS_CONTROL_HANDSHAKE;
    }

    // cts
    if (params.flow & 8)
    {
        dcbSerialParams.fOutxCtsFlow = TRUE;
    }
    else
    {
        dcbSerialParams.fOutxCtsFlow = FALSE;
    }

    // dtr
    if ((params.flow & 48) == 0)
    {
        dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
    }
    else if ((params.flow & 48) == 16)
    {
        dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
    }
    else
    {
        dcbSerialParams.fDtrControl = DTR_CONTROL_HANDSHAKE;
    }

    // dsr
    if (params.flow & 64)
    {
        dcbSerialParams.fOutxDsrFlow = TRUE;
    }
    else
    {
        dcbSerialParams.fOutxDsrFlow = FALSE;
    }

    m_serParams.flow = params.flow;

    if (!SetCommState(m_dev, &dcbSerialParams))      // Write the parameters
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error setting parameters").toLatin1().data());   // Ea=dataIO("SerialIO",0,9600,"\r")rror while writing
    }

    // Set TimeOut
    COMMTIMEOUTS timeouts;
//    timeouts.ReadIntervalTimeout = 0;                   // Set the Timeout parameters
    timeouts.ReadIntervalTimeout = params.timeout;      // Set the Timeout parameters [ms]
//    timeouts.ReadTotalTimeoutConstant = MAXDWORD;       // No TimeOut
    timeouts.ReadTotalTimeoutConstant = params.timeout;
    //timeouts.ReadTotalTimeoutConstant = 10000;        // No TimeOut
//    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutMultiplier = params.timeout;
//    timeouts.WriteTotalTimeoutConstant = MAXDWORD;
    timeouts.WriteTotalTimeoutConstant = params.timeout;
    //timeouts.WriteTotalTimeoutConstant = 10000;
//    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutMultiplier = params.timeout; // Set timeout in [ms]
    if (!SetCommTimeouts(m_dev, &timeouts))              // Write the parameters
        return ito::RetVal(ito::retError, 0, QObject::tr("error setting timeout").toLatin1().data());   // Error while writting the parameters
#endif

    const char testBuf[3] = {0, 0, 0};

    int len = strlen(params.endline)+1;//add one for endline
    strcpy(m_serParams.endline, params.endline);
    while (3 - len > 0)
    {
        m_serParams.endline[(len++)] = 0;
    }

    m_serParams.sendDelay = params.sendDelay;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This function sets the parameters of this serial port

    \param[in] baud            Baudrate in bits / s
    \param[in] endline         Endline character
    \param[in] bits            Number of bits in line before stopbits
    \param[in] stopbits        Number of stop bits after every n bits
    \param[in] parity          Toggle parity check options
    \param[in] flow            Flow control bitmask
    \param[in] sendDelay       Write every character seperated with delay or (=0) complete buffer at once
    \param[in] timeout         Time to wait until timeout in [ms]

    \return retOk
*/
const ito::RetVal SerialPort::setparams(const int baud, const char* endline, const int bits,
            const int stopbits, const int parity, const int flow, const int sendDelay, const int timeout)
{
    SerialPort::serParams params;
    params.baud = baud;
    params.bits = bits;
    params.stopbits = stopbits;
    params.parity = parity;
    params.flow = flow;
    params.sendDelay = sendDelay;
    params.timeout = timeout; // Set timeout in [ms]
    strcpy(params.endline, endline);
    return setparams(params);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This function opens the serial port

    \param[in] port            Number of serial port
    \param[in] baud            Baudrate in bits / s
    \param[in] endline         Endline character
    \param[in] bits            Number of bits in line before stopbits
    \param[in] stopbits        Number of stop bits after every n bits
    \param[in] parity          Toggle parity check options
    \param[in] flow            Flow control bitmask
    \param[in] sendDelay       Write every character seperated with delay or (=0) complete buffer at once
    \param[in] timeout         Time to wait until timeout in [ms]

    \return retOk
*/
const ito::RetVal SerialPort::sopen(const int port, const int baud, const char* endline, const int bits,
            const int stopbits, const int parity, const int flow, const int sendDelay, const int timeout, PortType &portType)
{
    char device[50];

    m_baudRatesSize = (int)(sizeof(baudRates)/sizeof(int));

#ifndef WIN32
    if (port < 1000) // ttyS of if not found try ttyUSB
    {
        _snprintf(device, 50, "/dev/ttyS%d", port);
        // Open device
        m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (m_dev < 0)
        {
            _snprintf(device, 50, "/dev/ttyUSB%d", port);
            m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
            if (m_dev < 0)
            {
                _snprintf(device, 50, "/dev/ttyACM%d", port);
                m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
                if (m_dev < 0)
                {
                    return ito::RetVal(ito::retError, 0, QObject::tr("could not open device").toLatin1().data());      // Device not found
                }
                else
                {
                    portType = TTYACM;
                }
            }
            else
            {
                portType = TTYUSB;
            }
        }
        else
        {
            portType = TTYS;
        }
        m_serParams.port = port;
        fcntl(m_dev, F_SETFL, FNDELAY);     // set nonblocking mode
    }
    else if ((port > 999) && (port < 2000))
    {
        _snprintf(device, 50, "/dev/ttyUSB%d", port-1000);
        m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (m_dev < 0)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("could not open device").toLatin1().data());      // Device not found
        }
        else
        {
            portType = TTYUSB;
        }
        m_serParams.port = port;
        fcntl(m_dev, F_SETFL, FNDELAY);     // set nonblocking mode
    }
    else if ((port > 1999) && (port < 3000))
    {
        _snprintf(device, 50, "/dev/ttyACM%d", port-2000);
        m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (m_dev < 0)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("could not open device").toLatin1().data());      // Device not found
        }
        else
        {
            portType = TTYACM;
        }
        m_serParams.port = port;
        fcntl(m_dev, F_SETFL, FNDELAY);     // set nonblocking mode
    }

    return setparams(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout);
#else
    _snprintf(device, 50, "\\\\.\\COM%d", port);
    // Open device
    m_dev = CreateFileA(device, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    if (m_dev == INVALID_HANDLE_VALUE)
    {
         m_dev = 0;
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("could not open device").toLatin1().data());      // Device not found
        }
        else
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("unknown error opening com port").toLatin1().data());      // other generic error
        }
    }
    else
    {
        portType = COM;
    }

    m_serParams.port = port;

    return setparams(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout);
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::sclose(void)
{
#ifndef WIN32
    if (m_dev)
    {
        close(m_dev);
    }
    m_dev = 0;
#else
    if (m_dev && INVALID_HANDLE_VALUE != m_dev)
    {
        CloseHandle(m_dev);
    }
    m_dev = 0;
#endif
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
int SerialPort::sreadable(void) const
{
#ifndef WIN32
    int bytes;

    ioctl(m_dev, FIONREAD, &bytes);
    return bytes;
#else
    DWORD errors;
    COMSTAT comstat;

    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return -1;
    }

    memset(&comstat, 0, sizeof(comstat));
    if (!ClearCommError(m_dev, &errors, &comstat))
    {
        return 0;
    }
    return comstat.cbInQue;
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::sread(char *buf, int *len, const int sendDelay)
{    
    int ret = 0;
#ifndef WIN32
    unsigned int numread = 0;
#else
    DWORD numread = 0;
#endif

#ifndef WIN32
    if (!m_dev)
#else
    if (!m_dev || m_dev == INVALID_HANDLE_VALUE)
#endif
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

    int bytesToRead = sreadable();
    bytesToRead = MIN(bytesToRead, *len);
    *len = 0;
    if (bytesToRead)
    {
        if (sendDelay)
        {
            while (*len < bytesToRead)
            {
#ifndef WIN32
                ret = read(m_dev, buf, 1);
                numread = ret;
#else
                ret = ReadFile(m_dev, buf, 1, &numread, NULL);
#endif
                if ((ret <= 0) || (numread != 1))
                {
                    return ito::RetVal(ito::retError, 0, QObject::tr("error reading from com port").toLatin1().data());
                }
                buf++;
                *len++;
                Sleep(sendDelay);
            }
        }
        else
        {
#ifndef WIN32
            ret = read(m_dev, buf, bytesToRead);
            *len = ret;
#else
            ret = ReadFile(m_dev, buf, bytesToRead, &numread, NULL);
            *len = numread;
#endif
            if ((ret <= 0) || (*len != bytesToRead))
            {
                return ito::RetVal(ito::retError, 0, QObject::tr("error reading from com port").toLatin1().data());
            }
        }
    }

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::swrite(const char c) const
{
    char outbuf[4];

    int length = 1;

    if (m_serParams.endline[0] != 0)
    {
        sprintf(outbuf, "%c%s", c, m_serParams.endline);
        length += (int)strlen(m_serParams.endline);
    }

#ifndef WIN32
    if (m_dev == 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

    if (write(m_dev, &outbuf, length) != length)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
    }
#else
    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

//std::cout << "serial::swrite: " << c << "\n" << std::endl;

    DWORD bytesWritten = 0;
    if (!WriteFile(m_dev, outbuf, length, &bytesWritten, NULL))
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
    }
    if (bytesWritten != length)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
    }
#endif
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::swrite(const char *buf, const int len, const int sendDelay) const
{
//    char *outbuf = (char*)calloc(strlen(buf) + 3, sizeof(char));
//    sprintf(outbuf, "%s%s", buf, m_serParams.endline);
//    int length = strlen(outbuf);

    int endlinelen = 0;
    int length = len;

    if (m_serParams.endline[0] != 0)
    {
        endlinelen = (int)strlen(m_serParams.endline);
        length += endlinelen;
    }

    char *outbuf = (char*)malloc(length * sizeof(char));

//    memset(outbuf,0,length);
    memcpy(outbuf, buf, len); // Copy buf with length of thens to the output buffer

    if (m_serParams.endline[0] != 0)
    {
        memcpy(&(outbuf[len]), &m_serParams.endline, endlinelen); // Attend to outbuffer, after buf,  endline
    }

#ifndef WIN32
    if (m_dev == 0)
    {
        free(outbuf);
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

    if (sendDelay)
    {
        for (int n = 0; n < length; n++)
        {
            // maybe error here it maybe was &outbuf???
            if (write(m_dev, outbuf + n, 1) != 1)
            {
                free(outbuf);
                return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
            }
            Sleep(sendDelay);
        }
    }
    else
    {
        // maybe error here it maybe was &outbuf???
        int a;
        if ((a = write(m_dev, outbuf, length)) != length)
        {
            free(outbuf);
            return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
        }
    }
#else
    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        free(outbuf);
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

//std::cout << "serial::swrite: " << buf << "\n" << std::endl;
    DWORD bytesWritten = 0;
    if (sendDelay)
    {
        for (int n = 0; n < length; n++)
        {
            if (!WriteFile(m_dev, outbuf + n, 1, &bytesWritten, NULL))
            {
                free(outbuf);
                return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
            }
            if (bytesWritten != 1)
            {
                free(outbuf);
                return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
            }
            Sleep(sendDelay);
        }
    }
    else
    {
        if (!WriteFile(m_dev, outbuf, length, &bytesWritten, NULL))
        {
            free(outbuf);
            return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
        }
        if (bytesWritten != length)
        {
            free(outbuf);
            return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toLatin1().data());
        }
    }
#endif
    free(outbuf);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This function clears the input or output buffer of the serial port

    \param[in] port            Number of buffer type (0 - input, 1 - output)

    \return retOk
*/
const ito::RetVal SerialPort::sclearbuffer(int BufferType)
{
#ifndef WIN32
    int errorCode;
    if (m_dev == 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

    switch (BufferType)
    {
        case 0: errorCode = tcflush(m_dev, TCIFLUSH); break;
        case 1: errorCode = tcflush(m_dev, TCOFLUSH); break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of buffer type (0: input, 1: output)").toLatin1().data());
    }

    if (errorCode != 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("Unable to clear buffer").toLatin1().data());
    }
#else
    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toLatin1().data());
    }

    DWORD dwFlags;
    switch (BufferType)
    {
        case 0: dwFlags = PURGE_RXCLEAR; break;
        case 1: dwFlags = PURGE_TXCLEAR; break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of buffer type (0: input, 1: output)").toLatin1().data());
    }

    if (PurgeComm(m_dev, dwFlags) == 0)
    {
//        return ito::RetVal(ito::retError, 0, QObject::tr("Unable to clear buffer! Error message: %1").arg(SysErrorMessage(GetLastError())).toLatin1().data());
        return ito::RetVal(ito::retError, 0, QObject::tr("Unable to clear buffer").toLatin1().data());
    }
#endif
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::getendline(char *eline)
{
    strcpy(eline, m_serParams.endline);
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIOInterface::getAddInInst(ito::AddInBase **addInInst)
{
    NEW_PLUGININSTANCE(SerialIO)
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIOInterface::closeThisInst(ito::AddInBase **addInInst)
{
   REMOVE_PLUGININSTANCE(SerialIO)
   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIOInterface::SerialIOInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("SerialIO");

    m_description = tr("itom-plugin for a serial port communication");

    //for the docstring, please don't set any spaces at the beginning of the line.
/*    char docstring[] = \
"SerialIO is a itom-Plugin which gives direct access to serial ports.\nIt is used by different plugins for communication, (e.g. 'PIPiezoCtrl', 'UhlActuator', 'LeicaMotorFocus').\n\
The plugin is implemented for Windows or Linux; the possible baudrates depend on the possibilites of the operating system. \n\
\n\
flow bitmask \n\
-------------- \n\
\n\
The flow bitmask is an OR combination of the following possible values: \n\
Xon/Xoff - default: Xoff, Xon=1 (1. bit) \n\
rts control - default: disabled, enabled=2, handshake=4 or (4+2) (2. and 3. bit) \n\
cts control - default: disabled, enabled=8 (4. bit) \n\
dtr control - default: disabled, enabled = 16, handshake = 32 or (32+16) (5. and 6. bit) \n\
dsr control - default: disabled, enabled = 64 \n\
\n\
If an endline character is given, this is automatically appended to each sequence that is send using the setVal-command. \n\
On the other side, any obtained value from the serial port is scanned for this endline character and automatically split. \n\
Use an empty endline character if you want to organize all this by yourself. \n\
\n\
Example \n\
-------- \n\
\n\
.. \n\
    \n\
    s = dataIO(\"SerialIO\",port=1,baud=9600,endline=\"\",bits=8,stopbits=1,parity=0,flow=16) \n\
    \n\
    #send command \n\
    sendString = bytearray(b\"POS?\") #or bytearray([80,79,83,63]); \n\
    s.setVal(sendString) \n\
    \n\
    #get result \n\
    answer = bytearray(9) #supposed length is 9 characters \n\
    num = s.getVal(answer) #if ok, num contains the number of received characters(max: length of answer), immediately returns ";
    m_detaildescription = tr(docstring);*/
    m_detaildescription = tr("SerialIO is a itom-Plugin which gives direct access to serial ports.\nIt is used by different plugins for communication, (e.g. 'PIPiezoCtrl', 'UhlActuator', 'LeicaMotorFocus').\n\
The plugin is implemented for Windows or Linux; the possible baudrates depend on the possibilites of the operating system. \n\
\n\
flow bitmask \n\
-------------- \n\
\n\
The flow bitmask is an OR combination of the following possible values:\n\
Xon/Xoff - default: Xoff, Xon=1 (1. bit)\n\
rts control - default: disabled, enabled=2, handshake=4 or (4+2) (2. and 3. bit)\n\
cts control - default: disabled, enabled=8 (4. bit)\n\
dtr control - default: disabled, enabled = 16, handshake = 32 or (32+16) (5. and 6. bit) \n\
dsr control - default: disabled, enabled = 64 \n\
\n\
If an endline character is given, this is automatically appended to each sequence that is send using the setVal-command.\n\
On the other side, any obtained value from the serial port is scanned for 'endlineRead' character and automatically split.\n\
Use an empty endline character if you want to organize all this by yourself.\n\
\n\
Example\n\
--------\n\
\n\
..\n\
    \n\
    s = dataIO(\"SerialIO\",port=1,baud=9600,endline=\"\",bits=8,stopbits=1,parity=0,flow=16)\n\
    \n\
    #send command\n\
    sendString = bytearray(b\"POS?\") #or bytearray([80,79,83,63]);\n\
    s.setVal(sendString)\n\
    \n\
    #get result\n\
    answer = bytearray(9) #supposed length is 9 characters\n\
    num = s.getVal(answer) #if ok, num contains the number of received characters(max: length of answer), immediately returns");

    m_author = "H. Bieger, C. Kohler, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr(GITVERSION);
#ifndef WIN32
    ito::Param paramVal("port", ito::ParamBase::Int, 0, 4095, 1, tr("The number of the serial port, [0 999] = ttyS, [1000 1999] = ttyUSB, [2000 2999] = ttyACM").toLatin1().data());
    m_initParamsMand.append(paramVal);
#else
    ito::Param paramVal("port", ito::ParamBase::Int, 0, 255, 1, tr("The number of the serial port, starting with 1 (linux 0)").toLatin1().data());
    m_initParamsMand.append(paramVal);
#endif
    paramVal = ito::Param("baud", ito::ParamBase::Int, 50, 4000000, 9600, tr("The baudrate of the port").toLatin1().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("endline", ito::ParamBase::String, "\n", tr("The endline character, which is added automatically after every setVal()").toLatin1().data());
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("bits", ito::ParamBase::Int, 5, 8, 8, tr("Number of bits to be written in line").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("stopbits", ito::ParamBase::Int, 1, 2, 1, tr("Stop bits after every n bits").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::Int, 0, 4, 0, tr("Parity: 0 -> none, 1 -> odd parity, 2 -> even parity, 3 -> mark, 4 -> space").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("flow", ito::ParamBase::Int, 0, 127, 0, tr("Bitmask for flow control (see docstring for more information)").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("sendDelay", ito::ParamBase::Int, 0, 65000, 0, tr("0: write output buffer as block, else: single characters with delay (1..65000 ms)").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("enableDebug", ito::ParamBase::Int, 0, 1, 0, tr("Initialised 'debug'-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget").toLatin1().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("debugIgnoreEmpty", ito::ParamBase::Int, 0, 1, 1, tr("If debug-param is true, all out and inputs are written to dockingWidget. If debugIgnoreEmpty is true, empty messages will be ignored").toLatin1().data());
    m_initParamsOpt.append(paramVal);
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIOInterface::~SerialIOInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialIO::showConfDialog(void)
{
    if (qobject_cast<QApplication*>(QCoreApplication::instance()))
        return apiShowConfigurationDialog(this, new dialogSerialIO(this, (void*)this, m_identifier, (int)(sizeof(SerialPort::baudRates) / sizeof(int))));
    else
        return ito::retOk;
/*    dialogSerialIO *confDialog = new dialogSerialIO((void*)this, m_identifier);
    QVariant qvar = m_params["port"].getVal<double>();
    confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
//        confDialog->getVals(&m_paramNames);
    }
    delete confDialog;

    return ito::retOk;*/
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIO::SerialIO() : AddInDataIO(), m_debugMode(false), m_debugIgnoreEmpty(false)
{
    m_preBuf.clear();

    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, "SerialIO", NULL);
    m_params.insert(paramVal.getName(), paramVal);

#ifndef WIN32
    paramVal = ito::Param("port", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 4095, 0, tr("The number of the serial port, [0 999] = ttyS, [1000 1999] = ttyUSB, [2000 2999] = ttyACM").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
#else
    paramVal = ito::Param("port", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 255, 0, tr("Serial port number of this device").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
#endif
    paramVal = ito::Param("baud", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 50, 4000000, 9600, tr("Current baudrate in bits/s").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bits", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 5, 8, 8, tr("Number of bits to be written in line").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stopbits", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 1, 2, 1, tr("Stop bits after every n bits").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 4, 0, tr("Parity: 0 -> none, 1 -> odd parity, 2 -> even parity, 3 -> mark, 4 -> space").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("flow", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 127, 0, tr("Bitmask for flow control as integer").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endline", ito::ParamBase::String | ito::ParamBase::NoAutosave, "\n", tr("Endline character, will be added automatically during setVal").toLatin1().data());
    ito::StringMeta stringMeta(ito::StringMeta::RegExp, "^.{0,2}$");
    paramVal.setMeta(&stringMeta, false);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endlineRead", ito::ParamBase::String | ito::ParamBase::NoAutosave, "\n", tr("Endline character, will be looking for during getVal by using readline").toLatin1().data());
    paramVal.setMeta(&stringMeta, false);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("readline", ito::ParamBase::Int, 0, 1, 0, tr("If true, reading next line terminated by endlineRead.").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("sendDelay", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 65000, 0, tr("0: write output buffer as block, else: single characters with delay (1..65000 ms)").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::NoAutosave, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 1, 0, tr("If true, all outputs and inputs are written to the toolbox").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debugIgnoreEmpty", ito::ParamBase::Int, 0, 1, 0, tr("If debug-param is true, all outputs and inputs are written to the toolbox. If debugIgnoreEmpty is true, empty messages will be ignored").toLatin1().data());
    m_params.insert(paramVal.getName(), paramVal);

    //register exec functions
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    registerExecFunc("clearInputBuffer", pMand, pOpt, pOut, tr("Clears the input buffer of serial port"));
    registerExecFunc("clearOutputBuffer", pMand, pOpt, pOut, tr("Clears the output buffer of serial port"));

    pMand << ito::Param("bufferType", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Clears input (0) or output (1) buffer").toLatin1().data());
    registerExecFunc("clearBuffer", pMand, pOpt, pOut, tr("Clears the input or output buffer of serial port"));

/*    //now create dock widget for this plugin
    DockWidgetSerialIO *SerialIOWidget = new DockWidgetSerialIO(m_params, m_uniqueID);
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), SerialIOWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(this, SIGNAL(uniqueIDChanged(const int)), SerialIOWidget, SLOT(uniqueIDChanged(const int)));
    connect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), SerialIOWidget, SLOT(serialLog(QByteArray, QByteArray, const char)));*/

    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    //now create dock widget for this plugin
    if (hasGuiSupport())
    {
        DockWidgetSerialIO *dw = new DockWidgetSerialIO(this);
        Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
        QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
        createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIO::~SerialIO()
{
   m_params.clear();
}

//----------------------------------------------------------------------------------------------------------------------------------
//! returns parameter of m_params with key name.
/*!
    This method copies the string of the corresponding parameter to val with a maximum length of maxLen.

    \param [in] name is the key name of the parameter
    \param [in,out] val is a shared-pointer of type char*.
    \param [in] maxLen is the maximum length which is allowed for copying to val
    \param [in] waitCond is the semaphore (default: NULL), which is released if this method has been terminated
    \return retOk in case that everything is ok, else retError
    \sa ito::tParam, ItomSharedSemaphore
*/
ito::RetVal SerialIO::getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;
    QString key;
    bool hasIndex = false;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName(val->getName(), key, hasIndex, index, suffix);

    if(retValue == ito::retOk)
    {
        //gets the parameter key from m_params map (read-only is allowed, since we only want to get the value).
        retValue += apiGetParamFromMapByKey(m_params, key, it, false);
    }

    if(!retValue.containsError())
    {
        //put your switch-case.. for getting the right value here

        //finally, save the desired value in the argument val (this is a shared pointer!)
        //if the requested parameter name has an index, e.g. roi[0], then the sub-value of the
        //array is split and returned using the api-function apiGetParam
        if (hasIndex)
        {
            *val = apiGetParam(*it, hasIndex, index, retValue);
        }
        else
        {
            *val = *it;
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue(ito::retOk);
    QString key;
    bool hasIndex;
    int index;
    QString suffix;
    ParamMapIterator it;

    //parse the given parameter-name (if you support indexed or suffix-based parameters)
    retValue += apiParseParamName( val->getName(), key, hasIndex, index, suffix );

    if(!retValue.containsError())
    {
        //gets the parameter key from m_params map (read-only is not allowed and leads to ito::retError).
        retValue += apiGetParamFromMapByKey(m_params, key, it, true);
    }

    if(!retValue.containsError())
    {
        retValue += apiValidateAndCastParam(*it, *val, false, true, true);
    }

    if(!retValue.containsError())
    {
        //all parameters that don't need further checks can simply be assigned
        //to the value in m_params (the rest is already checked above)
        retValue += it->copyValueFrom( &(*val) );
    }

    if (!retValue.containsError())
    {
        int baud = m_params["baud"].getVal<int>();
        int bits = m_params["bits"].getVal<int>();
        int stopbits = m_params["stopbits"].getVal<int>();
        int parity = m_params["parity"].getVal<int>();
        int flow = m_params["flow"].getVal<int>();
        char *endline = m_params["endline"].getVal<char*>(); //borrowed reference
        int sendDelay = m_params["sendDelay"].getVal<int>();
        int timeout = (int)(m_params["timeout"].getVal<double>() * 1000.0 + 0.5);
        m_debugMode = (bool)(m_params["debug"].getVal<int>());
        m_debugIgnoreEmpty = (bool)(m_params["debugIgnoreEmpty"].getVal<int>());
        retValue += m_serport.setparams(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout);
    }

    if(!retValue.containsError())
    {
        emit parametersChanged(m_params); //send changed parameters to any connected dialogs or dock-widgets
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond)
{
    // for lazy initialisation we expect at least port, baudrate and endline everything else will be
    // set to default values
    // parameters MUST be in this order:
    // 1. port [0 .. 99]
    // 2. baud [300 .. 115200]
    // 3. endline [\n, \r, \r\n]
    // 4. bits [5 .. 8]
    // 5. stopbits [1 .. 2]
    // 6. parity [0 .. 4: none, odd, even, mark, space]
    // 7. flow control [off, hardware, xoff]
    // 8. sendDelay [0 .. 65000]
    // 9. timeout [0 .. 30000]

    ItomSharedSemaphoreLocker locker(waitCond);

    ito::RetVal retval;
    int port = 0;
    int baud = 9600;
    char endline[3] = "\n";
    int bits = 8;
    int stopbits = 1;
    int parity = 0;
    int flow = 0;
    int sendDelay = 0;
    int timeout = 4000;
    char * tendline = NULL;

    // mandatory parameters
    if (paramsMand == NULL)
    {
        retval = ito::RetVal(ito::retError, 0, tr("Mandatory paramers are NULL").toLatin1().data());
        goto end;
    }

    retval += m_params["port"].copyValueFrom(&((*paramsMand)[0]));
    port = m_params["port"].getVal<int>();

    retval += m_params["baud"].copyValueFrom(&((*paramsMand)[1]));
    baud = m_params["baud"].getVal<int>();

    retval += m_params["endline"].copyValueFrom(&((*paramsMand)[2]));
    retval += m_params["endlineRead"].copyValueFrom(&((*paramsMand)[2]));
    
    tendline = m_params["endline"].getVal<char *>(); //borrowed reference
    strncpy(endline, tendline, 3);
//    sprintf(endline, "%s", tendline);

    // optional parameters
    if (paramsOpt == NULL)
    {
        retval = ito::RetVal(ito::retError, 0, tr("Optinal paramers are NULL").toLatin1().data());
        goto end;
    }

    retval += m_params["bits"].copyValueFrom(&((*paramsOpt)[0]));
    bits = m_params["bits"].getVal<int>();

    retval += m_params["stopbits"].copyValueFrom(&((*paramsOpt)[1]));
    stopbits = m_params["stopbits"].getVal<int>();

    retval += m_params["parity"].copyValueFrom(&((*paramsOpt)[2]));
    parity = m_params["parity"].getVal<int>();

    retval += m_params["flow"].copyValueFrom(&((*paramsOpt)[3]));
    flow = m_params["flow"].getVal<int>();

    retval += m_params["sendDelay"].copyValueFrom(&((*paramsOpt)[4]));
    sendDelay = m_params["sendDelay"].getVal<int>();

    retval += m_params["timeout"].copyValueFrom(&((*paramsOpt)[5]));
    timeout = (int)(m_params["timeout"].getVal<double>() * 1000 + 0.5);

    SerialPort::PortType portType;
    retval = m_serport.sopen(port, baud, endline, bits, stopbits, parity, flow, sendDelay, timeout, portType);

    if (!retval.containsError())
    {
        switch (portType)
        {
        case SerialPort::COM:
            m_identifier = QString("COM %1").arg( port );
            break;
        case SerialPort::TTYS:
            m_identifier = QString("/dev/ttyS%1").arg(port);
            break;
        case SerialPort::TTYUSB:
            m_identifier = QString("/dev/ttyUSB%1").arg(port);
            break;
        }

        setIdentifier(m_identifier);
    }

    if (!retval.containsError())
    {
        retval += m_params["debug"].copyValueFrom(&((*paramsOpt)[6]));
        m_debugMode = (bool)(m_params["debug"].getVal<int>());
    }

    if (!retval.containsError())
    {
        retval += m_params["debugIgnoreEmpty"].copyValueFrom(&((*paramsOpt)[7]));
        m_debugIgnoreEmpty = (bool)(m_params["debugIgnoreEmpty"].getVal<int>());    
    }

    emit parametersChanged(m_params);

end:

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }
    setInitialized(true); //init method has been finished (independent on retval)
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::close(ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    retval = m_serport.sclose();

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::startDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StartDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StopDevice not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("Acquire not necessary").toLatin1().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
//    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retValue;

    if (m_params["readline"].getVal<int>() == 0)
    {
        if (m_preBuf.isEmpty())
        {
            retValue = m_serport.sread(data.data(), length.data(), m_params["sendDelay"].getVal<int>());
        }
        else
        {
            //prepend prebuf to data
            int numCharactersForPrebuf = std::min((int)m_preBuf.size(), *length);
            int remainingCharactersInData = *length - numCharactersForPrebuf;
            memcpy(data.data(), m_preBuf.data(), sizeof(char) * numCharactersForPrebuf);
            m_preBuf.remove(0, numCharactersForPrebuf); //shorten prebuf after copying

            if (remainingCharactersInData > 0)
            {
                retValue = m_serport.sread(&(data.data()[numCharactersForPrebuf]), &remainingCharactersInData, m_params["sendDelay"].getVal<int>());
                *length = remainingCharactersInData + numCharactersForPrebuf;
            }
        }
    }
    else
    {
        char *endlineChar = m_params["endlineRead"].getVal<char *>();
        int endlineLen = endlineChar[0] == 0 ? 0 : (endlineChar[1] == 0 ? 1 : (endlineChar[2] == 0 ? 2 : 3));
        QByteArray endlineRead = QByteArray::fromRawData(endlineChar, endlineLen);
        int numCharactersForPrebuf = 0;

        if (!m_preBuf.isEmpty())
        {
            //prepend prebuf to data
            numCharactersForPrebuf = std::min((int)m_preBuf.size(), *length);
            memcpy(data.data(), m_preBuf.data(), sizeof(char) * numCharactersForPrebuf);
            m_preBuf.remove(0, numCharactersForPrebuf); //shorten prebuf after copying
        }

        if (*length == numCharactersForPrebuf)
        {
            // prebuf data longer than *length
            QByteArray temp = QByteArray(data.data(), *length);
            int endlinePos = temp.indexOf(endlineRead);

            if (endlinePos >= 0) //endlineRead found
            {
                m_preBuf = temp.mid(endlinePos + endlineLen);
                *length = endlinePos;
            }
        }
        else
        {
            // prebuf data shorter than *length
            QElapsedTimer timer;
            int timeoutMS = (int)(m_params["timeout"].getVal<double>() * 1000);
            bool done = false;
            int pos = numCharactersForPrebuf; 
            int len = 0;

            timer.start();

            while (!done && !retValue.containsError())
            {
                len = *length - pos;
                retValue = m_serport.sread(&(data.data()[pos]), &len, m_params["sendDelay"].getVal<int>());

                if (!retValue.containsError())
                {
                    QByteArray temp = QByteArray(data.data(), pos + len);
                    int endlinePos = temp.indexOf(endlineRead);

                    if (endlinePos >= 0) //endlineRead found
                    {
                        m_preBuf = temp.mid(endlinePos + endlineLen);
                        *length = endlinePos;
                        done = true;
                    }
                    else
                    {
                        pos += len;
                        done = (pos == *length);
                    }
                }
                else
                {
                    len = 0;
                }

                if (!done && timer.elapsed() > timeoutMS && timeoutMS >= 0)
                {
                    *length = pos + len;
                    retValue += ito::RetVal(ito::retError, 256, tr("timeout while reading from serial port.").toLatin1().data());
                }
                else
                {
                    setAlive();
                }
            }
        }
    }

    if (m_debugMode)
    {
        emit serialLog(QByteArray(data.data(),*length), "", '<');
    }

    if (waitCond)
    {
        waitCond->returnValue = retValue;
        waitCond->release();
    }

    return retValue;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::setVal(const char *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    const char *buf = data;
    char endline[3] = {0, 0, 0};
    ito::RetVal retval(ito::retOk);

    m_serport.getendline(endline);
    if (m_debugMode)
    {
        emit serialLog(QByteArray(buf,datalength), QByteArray(endline, (int)strlen(endline)), '>');
    }
    retval = m_serport.swrite(buf, datalength, m_params["sendDelay"].getVal<int>());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond /*= NULL*/)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;

    if (funcName == "clearInputBuffer")
    {
        retval = m_serport.sclearbuffer(0);
        m_preBuf.clear();
    }
    else if (funcName == "clearOutputBuffer")
    {
        retval = m_serport.sclearbuffer(1);
    }
    else if (funcName == "clearBuffer")
    {
        int bufferType = paramsMand->at(0).getVal<int>();
        retval = m_serport.sclearbuffer(bufferType);
        if (bufferType == 0)
        {
            m_preBuf.clear();
        }
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
void SerialIO::dockWidgetVisibilityChanged(bool visible)
{
    if (qobject_cast<QApplication*>(QCoreApplication::instance()) && getDockWidget())
    {
        DockWidgetSerialIO *dw = qobject_cast<DockWidgetSerialIO*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), dw, SLOT(serialLog(QByteArray, QByteArray, const char)));
            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(parametersChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), dw, SLOT(serialLog(QByteArray, QByteArray, const char)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
