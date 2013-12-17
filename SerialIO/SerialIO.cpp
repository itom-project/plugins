/********** Serial Port *****************
#   1         5
#   /---------\      1 DCD    6 DSR 
#  | . . . . . |     2 RxD    7 RTS
#   \ . . . . /      3 TxD    8 CTS
#    \       /       4 DTR    9 RI
#     -------        5 GND
#     6     9
*/

#include "SerialIO.h"

#define _USE_MATH_DEFINES  // needs to be defined to enable standard declartions of PI constant
#include "math.h"

#include <qstring.h>
#include <qbytearray.h>
#include <qstringlist.h>
#include <QtCore/QtPlugin>

#include "pluginVersion.h"

//#include <qdebug.h>
//#include <qmessagebox.h>

#ifndef linux
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

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::setparams(const SerialPort::serParams &params)
{
    ito::RetVal ret(ito::retOk);
#ifdef __linux__
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
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid baud rate").toAscii().data());
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
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of bits").toAscii().data());
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
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of stopbits").toAscii().data());
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
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid parity").toAscii().data());
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

//    return 0;
#else
    // port setup
    DCB dcbSerialParams = {0};                                          // Structure for the port parameters
    dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
    if (!GetCommState(m_dev, &dcbSerialParams))                         // Get the port parameters
    {
        return -2;                                                      // Error while getting port parameters
    }

/*    switch (params.baud)                                                // Set the speed (Bauds)
    {
        case 110:       dcbSerialParams.BaudRate = CBR_110;     break;
        case 300:       dcbSerialParams.BaudRate = CBR_300;     break;
        case 600:       dcbSerialParams.BaudRate = CBR_600;     break;
        case 1200:      dcbSerialParams.BaudRate = CBR_1200;    break;
        case 2400:      dcbSerialParams.BaudRate = CBR_2400;    break;
        case 4800:      dcbSerialParams.BaudRate = CBR_4800;    break;
        case 9600:      dcbSerialParams.BaudRate = CBR_9600;    break;
//        case 14400:     dcbSerialParams.BaudRate = CBR_14400;   break;
        case 19200:     dcbSerialParams.BaudRate = CBR_19200;   break;
        case 38400:     dcbSerialParams.BaudRate = CBR_38400;   break;
//        case 56000:     dcbSerialParams.BaudRate = CBR_56000;   break;
        case 57600:     dcbSerialParams.BaudRate = CBR_57600;   break;
        case 115200:    dcbSerialParams.BaudRate = CBR_115200;  break;
//        case 128000:    dcbSerialParams.BaudRate = CBR_128000;  break;
//        case 256000:    dcbSerialParams.BaudRate = CBR_256000;  break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid baud rate").toAscii().data());
    }*/
    if (params.baud == 50      || params.baud == 75      || params.baud == 134     || params.baud == 150     || params.baud == 200     || params.baud == 300     ||
        params.baud == 600     || params.baud == 1200    || params.baud == 1800    || params.baud == 2400    || params.baud == 4800    || params.baud == 9600    ||
        params.baud == 19200   || params.baud == 38400   || params.baud == 57600   || params.baud == 115200  || params.baud == 230400  || params.baud == 460800  ||
        params.baud == 500000  || params.baud == 576000  || params.baud == 921600  || params.baud == 1000000 || params.baud == 1152000 || params.baud == 1500000 || 
        params.baud == 2000000 || params.baud == 2500000 || params.baud == 3000000 || params.baud == 3500000 || params.baud == 4000000)
    {
        // Set the speed (Bauds)
        dcbSerialParams.BaudRate = params.baud;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("invalid baud rate").toAscii().data());
    }
    m_serParams.baud = params.baud;

/*    switch (params.bits)
    {
        case 5: dcbSerialParams.ByteSize = 5;   break;
        case 6: dcbSerialParams.ByteSize = 6;   break;
        case 7: dcbSerialParams.ByteSize = 7;   break;
        case 8: dcbSerialParams.ByteSize = 8;   break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of bits").toAscii().data());
    }*/
    if (params.bits > 4 && params.bits < 9)
    {
        dcbSerialParams.ByteSize = params.bits;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of bits").toAscii().data());
    }
    m_serParams.bits = params.bits;

    switch (params.stopbits)
    {
        case 1: dcbSerialParams.StopBits = ONESTOPBIT;      break;
        case 2: dcbSerialParams.StopBits = TWOSTOPBITS;     break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of stopbits").toAscii().data());
        // this is windows specific so we will do 1 and 2 as intuitiv standard
        //case 3: dcbSerialParams.StopBits = ONE5STOPBITS;    break;
    }
    m_serParams.stopbits = params.stopbits;

    switch (params.parity)
    {
        case 0: dcbSerialParams.Parity = NOPARITY;      break;
        case 1: dcbSerialParams.Parity = ODDPARITY;     break;
        case 2: dcbSerialParams.Parity = EVENPARITY;    break;
        // space parity and mark parity not supported
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid parity").toAscii().data());
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
        return ito::RetVal(ito::retError, 0, QObject::tr("error setting parameters").toAscii().data());   // Ea=dataIO("SerialIO",0,9600,"\r")rror while writing
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
        return ito::RetVal(ito::retError, 0, QObject::tr("error setting timeout").toAscii().data());   // Error while writting the parameters
#endif

    const char testBuf[3] = {0, 0, 0};

    if (strcmp(params.endline, "\n") == 0)
    {
        m_serParams.endline[0] = '\n';
        m_serParams.endline[1] = 0;
        m_serParams.endline[2] = 0;
    }
    else if (strcmp(params.endline, "\r") == 0)
    {
        m_serParams.endline[0] = '\r';
        m_serParams.endline[1] = 0;
        m_serParams.endline[2] = 0;
    }
    else if (strcmp(params.endline, "\r\n") == 0)
    {
        m_serParams.endline[0] = '\r';
        m_serParams.endline[1] = '\n';
        m_serParams.endline[2] = 0;
    }
    else if (strcmp(params.endline, testBuf) == 0)
    {
        m_serParams.endline[0] = 0;
        m_serParams.endline[1] = 0;
        m_serParams.endline[2] = 0;
    }
    else
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("invalid endline character").toAscii().data());
    }

    m_serParams.singlechar = params.singlechar;

    return ito::retOk;
}
//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This function sets the parameters of this serial port

    \param[in] baud		    Baudrate in bits / s
    \param[in] endline		Endline character
    \param[in] bits		    Number of bits in line before stopbits
    \param[in] stopbits		Number of stop bits after every n bits
    \param[in] parity		Toggle parity check options
    \param[in] flow 		Flow control bitmask
    \param[in] singlechar	Write every character seperated or complete buffer at once
    \param[in] timeout		Time to wait until timeout in [ms]

    \return retOk
*/
const ito::RetVal SerialPort::setparams(const int baud, const char* endline, const int bits,
            const int stopbits, const int parity, const int flow, const int singlechar, const int timeout)
{
    SerialPort::serParams params;
    params.baud = baud;
    params.bits = bits;
    params.stopbits = stopbits;
    params.parity = parity;
    params.flow = flow;
    params.singlechar = singlechar;
    params.timeout = timeout; // Set timeout in [ms]
    strcpy(params.endline, endline);
    return setparams(params);
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This function opens the serial port

    \param[in] port		    Number of serial port
    \param[in] baud		    Baudrate in bits / s
    \param[in] endline		Endline character
    \param[in] bits		    Number of bits in line before stopbits
    \param[in] stopbits		Number of stop bits after every n bits
    \param[in] parity		Toggle parity check options
    \param[in] flow 		Flow control bitmask
    \param[in] singlechar	Write every character seperated or complete buffer at once
    \param[in] timeout		Time to wait until timeout in [ms]

    \return retOk
*/
const ito::RetVal SerialPort::sopen(const int port, const int baud, const char* endline, const int bits,
            const int stopbits, const int parity, const int flow, const int singlechar, const int timeout)
{
    char device[50];
#ifdef __linux__
//    _snprintf(device, 50, "/dev/ttyS%d", port);
    _snprintf(device, 50, "/dev/ttyUSB%d", port);
    // Open device
    m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_dev < 0)
    {
//        _snprintf(device, 50, "/dev/ttyUSB%d", port);
        _snprintf(device, 50, "/dev/ttyS%d", port);
        m_dev = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if (m_dev < 0)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("could not open device").toAscii().data());      // Device not found
        }
    }
    m_serParams.port = port;
    fcntl(m_dev, F_SETFL, FNDELAY);     // set nonblocking mode

    return setparams(baud, endline, bits, stopbits, parity, flow, singlechar, timeout);
#else
    _snprintf(device, 50, "\\\\.\\COM%d", port);
    // Open device
    m_dev = CreateFileA(device, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    if (m_dev == INVALID_HANDLE_VALUE)
    {
         m_dev = 0;
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("could not open device").toAscii().data());      // Device not found
        }
        else
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("unknown error opening com port").toAscii().data());      // other generic error
        }
    }
    m_serParams.port = port;

    return setparams(baud, endline, bits, stopbits, parity, flow, singlechar, timeout);
#endif
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::sclose(void)
{
#ifdef __linux__
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
#ifdef __linux__
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
const ito::RetVal SerialPort::sread(char *buf, int *len, const int singlechar)
{
#ifdef __linux__
    int ret = 0;

    if (!m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

    int readable = sreadable();
    if (readable)
    {
        *len = *len > readable ? *len : readable;
        if (singlechar)
        {
            while (readable)
            {
                ret = read(m_dev, buf, 1);
                buf++;
                readable--;
            }
        }
        else
        {
            ret = read(m_dev, buf, *len);
        }
    }
    else
    {
        *buf = 0;
    }
    *len = readable;
//    if (!ret)
//		return ito::RetVal(ito::retError, 0, QObjcet::tr("error reading from com port"));

#else
    DWORD numread = 0;
    int ret;

    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

    int readable = sreadable();
    *len = readable < *len ? readable : *len;
    if (readable)
    {
        if (singlechar)
        {
            while (readable)
            {
                ret = ReadFile(m_dev, buf, 1, &numread, NULL);
                if ((numread != 1) || !ret)
                {
                    return ito::RetVal(ito::retError, 0, QObject::tr("error reading from com port").toAscii().data());
                }
                readable++;
                buf--;
            }
        }
        else
        {
            ret = ReadFile(m_dev, buf, readable > *len ? *len : readable, &numread, NULL);
            *len = numread;
            if ((!ret) || (!numread))
            {
                return ito::RetVal(ito::retError, 0, QObject::tr("error reading from com port").toAscii().data());
            }
        }
    }
    else
    {
        *buf = 0;
    }

//		return ERRORFROMWIN(GetLastError());

#endif
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

#ifdef __linux__
    if (m_dev == 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

    if (write(m_dev, &outbuf, length) != length)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
    }
#else
    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

//std::cout << "serial::swrite: " << c << "\n" << std::endl;

    DWORD bytesWritten = 0;
    if (!WriteFile(m_dev, outbuf, length, &bytesWritten, NULL))
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
    }
    if (bytesWritten != length)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
    }
#endif
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialPort::swrite(const char *buf, const int len, const int singlechar) const
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

#ifdef __linux__
    if (m_dev == 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

    if (singlechar)
    {
        for (int n = 0; n < length; n++)
        {
            // maybe error here it maybe was &outbuf???
            if (write(m_dev, outbuf + n, 1) != 1)
            {
                return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
            }
        }
    }
    else
    {
        // maybe error here it maybe was &outbuf???
        int a;
        if ((a = write(m_dev, outbuf, length)) != length)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
        }
    }
#else
    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

//std::cout << "serial::swrite: " << buf << "\n" << std::endl;
    DWORD bytesWritten = 0;
    if (singlechar)
    {
        for (int n = 0; n < length; n++)
        {
            if (!WriteFile(m_dev, outbuf + n, 1, &bytesWritten, NULL))
            {
                return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
            }
            if (bytesWritten != 1)
            {
                return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
            }
        }
    }
    else
    {
        if (!WriteFile(m_dev, outbuf, length, &bytesWritten, NULL))
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
        }
        if (bytesWritten != length)
        {
            return ito::RetVal(ito::retError, 0, QObject::tr("error writing to com port").toAscii().data());
        }
    }
#endif
    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*!
    \detail This function clears the input or output buffer of the serial port

    \param[in] port		    Number of buffer type (0 - input, 1 - output)

    \return retOk
*/
const ito::RetVal SerialPort::sclearbuffer(int BufferType)
{
#ifdef __linux__
    int errorCode;
    if (m_dev == 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

    switch (BufferType)
    {
        case 0: errorCode = tcflush(m_dev, TCIFLUSH); break;
        case 1: errorCode = tcflush(m_dev, TCOFLUSH); break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of buffer type (0: input, 1: output)").toAscii().data());
    }

    if (errorCode != 0)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("Unable to clear buffer").toAscii().data());
    }
#else
    if (!m_dev || INVALID_HANDLE_VALUE == m_dev)
    {
        return ito::RetVal(ito::retError, 0, QObject::tr("com port not open").toAscii().data());
    }

    DWORD dwFlags;
    switch (BufferType)
    {
        case 0: dwFlags = PURGE_RXCLEAR; break;
        case 1: dwFlags = PURGE_TXCLEAR; break;
        default:
            return ito::RetVal(ito::retError, 0, QObject::tr("invalid number of buffer type (0: input, 1: output)").toAscii().data());
    }

    if (PurgeComm(m_dev, dwFlags) == 0)
    {
//        return ito::RetVal(ito::retError, 0, QObject::tr("Unable to clear buffer! Error message: %1").arg(SysErrorMessage(GetLastError())).toAscii().data());
        return ito::RetVal(ito::retError, 0, QObject::tr("Unable to clear buffer").toAscii().data());
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
    SerialIO* newInst = new SerialIO();
    newInst->setBasePlugin(this);
    *addInInst = qobject_cast<ito::AddInBase*>(newInst);

    m_InstList.append(*addInInst);

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIOInterface::closeThisInst(ito::AddInBase **addInInst)
{
   if (*addInInst)
   {
        delete ((SerialIO *)*addInInst);
        int idx = m_InstList.indexOf(*addInInst);
        m_InstList.removeAt(idx);
   }

   return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIOInterface::SerialIOInterface()
{
    m_type = ito::typeDataIO | ito::typeRawIO;
    setObjectName("SerialIO");

    m_description = tr("itom-plugin for a serial port communication");

    //for the docstring, please don't set any spaces at the beginning of the line.
    char docstring[] = \
"SerialIO is a itom-Plugin which gives direct access to serial ports.\nIt is used by different plugins for communication, (e.g. 'PIPiezoCtrl', 'UhlActuator', 'LeicaMotorFocus').\n\
The plugin is implemented for Windows or Linux; the possible baudrates depend on the possibilites of the operating system. \n\
\n\
flow bitmask \n\
-------------- \n\
\n\
The flow bitmask is an OR combination of the following possible values: \n\
Bit 1: Xon/Xoff enabled, if not set disabled \n\
Bit 2/4: not set -> no rts control, 2 only -> rts control on, 4 set, 2 arbitrary -> rts control handshake \n\
Bit 8: cts enabled, if not set disabled \n\
Bit 16/32: not set -> dtr disabled, 16 only -> dtr enabled, 32 set, 16 arbitrary -> dtr handshake \n\
Bit 64: dsr enabled, if not set dsr disabled";

    m_detaildescription = tr(docstring);
    m_author = "H. Bieger, C. Kohler, ITO, University Stuttgart";
    m_version = (PLUGIN_VERSION_MAJOR << 16) + (PLUGIN_VERSION_MINOR << 8) + PLUGIN_VERSION_PATCH;
    m_minItomVer = MINVERSION;
    m_maxItomVer = MAXVERSION;
    m_license = QObject::tr("licensed under LGPL");
    m_aboutThis = QObject::tr("N.A.");  

    ito::Param paramVal("port", ito::ParamBase::Int, 1, 255, 1, tr("The number of the serial port, starting with 1").toAscii().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("baud", ito::ParamBase::Int, 50, 4000000, 9600, tr("The baudrate of the port").toAscii().data());
    m_initParamsMand.append(paramVal);
    paramVal = ito::Param("endline", ito::ParamBase::String, "\n", tr("The endline character, which is added automatically after every setVal()").toAscii().data());
    m_initParamsMand.append(paramVal);

    paramVal = ito::Param("bits", ito::ParamBase::Int, 5, 8, 8, tr("Number of bits to be written in line").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("stopbits", ito::ParamBase::Int, 1, 2, 1, tr("Stop bits after every n bits").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::Int, 0, 2, 0, tr("Parity: 0 -> no parity, 1 -> odd parity, 2 -> even parity").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("flow", ito::ParamBase::Int, 0, 127, 0, tr("Bitmask for flow control (see docstring for more information)").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("singlechar", ito::ParamBase::Int, 0, 1, 0, tr("Toggle: write output buffer as block or single characters").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toAscii().data());
    m_initParamsOpt.append(paramVal);
    paramVal = ito::Param("enableDebug", ito::ParamBase::Int, 0, 1, 0, tr("Initialised 'debug'-parameter with given value. If debug-param is true, all out and inputs are written to dockingWidget").toAscii().data());
    m_initParamsOpt.append(paramVal);

    return;
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIOInterface::~SerialIOInterface()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
Q_EXPORT_PLUGIN2(SerialIOinterface, SerialIOInterface)

//----------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------
const ito::RetVal SerialIO::showConfDialog(void)
{
    dialogSerialIO *confDialog = new dialogSerialIO((void*)this);
    QVariant qvar = m_params["port"].getVal<double>();
    confDialog->setVals(&m_params);
    if (confDialog->exec())
    {
//        confDialog->getVals(&m_paramNames);
    }
    delete confDialog;

    return ito::retOk;
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIO::SerialIO() : AddInDataIO(), m_debugMode(false)
{
    ito::Param paramVal("name", ito::ParamBase::String | ito::ParamBase::NoAutosave, "SerialIO", NULL);
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("port", ito::ParamBase::Int | ito::ParamBase::Readonly | ito::ParamBase::NoAutosave, 0, 255, 0, tr("Serial port number of this device").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("baud", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 50, 4000000, 9600, tr("Current baudrate in bits/s").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("bits", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 5, 8, 8, tr("Number of bits to be written in line").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("stopbits", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 1, 2, 1, tr("Stop bits after every n bits").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("parity", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 2, 0, tr("Toggle parity check").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("flow", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 127, 0, tr("Bitmask for flow control as integer").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("endline", ito::ParamBase::String | ito::ParamBase::NoAutosave, "\n", tr("Endline character, will be added automatically during setVal").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("singlechar", ito::ParamBase::Int | ito::ParamBase::NoAutosave, 0, 1, 0, tr("Toggle: write output buffer as block @ once or single characters").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("timeout", ito::ParamBase::Double | ito::ParamBase::NoAutosave, 0.0, 65.0, 4.0, tr("Timeout for reading commands in [s]").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);
    paramVal = ito::Param("debug", ito::ParamBase::Int, 0, 1, 0, tr("If true, all out and inputs are written to dockingWidget").toAscii().data());
    m_params.insert(paramVal.getName(), paramVal);

    //register exec functions
    QVector<ito::Param> pMand;
    QVector<ito::Param> pOpt;
    QVector<ito::Param> pOut;
    registerExecFunc("clearInputBuffer", pMand, pOpt, pOut, tr("Clears the input buffer of serial port"));
    registerExecFunc("clearOutputBuffer", pMand, pOpt, pOut, tr("Clears the output buffer of serial port"));

    pMand << ito::Param("bufferType", ito::ParamBase::Int | ito::ParamBase::In, 0, 1, 0, tr("Clears input (0) or output (1) buffer").toAscii().data());
    registerExecFunc("clearBuffer", pMand, pOpt, pOut, tr("Clears the input or output buffer of serial port"));

/*    //now create dock widget for this plugin
    DockWidgetSerialIO *SerialIOWidget = new DockWidgetSerialIO(m_params, m_uniqueID);
    connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), SerialIOWidget, SLOT(valuesChanged(QMap<QString, ito::Param>)));
    connect(this, SIGNAL(uniqueIDChanged(const int)), SerialIOWidget, SLOT(uniqueIDChanged(const int)));
    connect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), SerialIOWidget, SLOT(serialLog(QByteArray, QByteArray, const char)));*/

    qRegisterMetaType<QMap<QString, ito::Param> >("QMap<QString, ito::Param>");

    //now create dock widget for this plugin
    DockWidgetSerialIO *dw = new DockWidgetSerialIO(m_params, getID() );
    Qt::DockWidgetAreas areas = Qt::AllDockWidgetAreas;
    QDockWidget::DockWidgetFeatures features = QDockWidget::DockWidgetClosable | QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable;
    createDockWidget(QString(m_params["name"].getVal<char *>()), features, areas, dw);
}

//----------------------------------------------------------------------------------------------------------------------------------
SerialIO::~SerialIO()
{
   m_pThread->quit();
   m_pThread->wait(5000);
   delete m_pThread;
   m_pThread = NULL;

   m_params.clear();

   return;
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
    ito::RetVal retValue(ito::retOk);
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of requested parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::const_iterator paramIt = m_params.constFind(key);
        if (paramIt != m_params.constEnd())
        {
            *val = paramIt.value();
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("parameter not found in m_params.").toAscii().data());
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
    QString key = val->getName();

    if (key == "")
    {
        retValue += ito::RetVal(ito::retError, 0, tr("name of given parameter is empty.").toAscii().data());
    }
    else
    {
        QMap<QString, ito::Param>::iterator paramIt = m_params.find(key);
        if (paramIt != m_params.end())
        {
            int baud = 0;
            int bits = 0;
            int stopbits = 0;
            int parity = 0;
            int flow = 0;
            char *endline = NULL;
            int singlechar = 0;
            int timeout = 0;

            if (paramIt->getFlags() & ito::ParamBase::Readonly)	//check read-only
            {
                retValue += ito::RetVal(ito::retWarning, 0, tr("Parameter is read only, input ignored").toAscii().data());
                goto end;
            }
            else if (val->isNumeric() && paramIt->isNumeric())
            {
                double curval = val->getVal<double>();
                if (curval > paramIt->getMax())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is larger than parameter range, input ignored").toAscii().data());
                    goto end;
                }
                else if (curval < paramIt->getMin())
                {
                    retValue += ito::RetVal(ito::retError, 0, tr("New value is smaller than parameter range, input ignored").toAscii().data());
                    goto end;
                }
                else
                {
                    paramIt.value().setVal<double>(curval);
                }
            }
            else if (paramIt->getType() == val->getType())
            {
                retValue += paramIt.value().copyValueFrom(&(*val));
            }
            else
            {
                retValue += ito::RetVal(ito::retError, 0, tr("Parameter type conflict").toAscii().data());
                goto end;
            }

            baud = m_params["baud"].getVal<int>();
            bits = m_params["bits"].getVal<int>();
            stopbits = m_params["stopbits"].getVal<int>();
            parity = m_params["parity"].getVal<int>();
            flow = m_params["flow"].getVal<int>();
            endline = m_params["endline"].getVal<char*>(); //borrowed reference
            singlechar = m_params["singlechar"].getVal<int>();
            timeout = (int)(m_params["timeout"].getVal<double>() * 1000.0 + 0.5);
            m_debugMode = (bool)(m_params["debug"].getVal<int>());
            retValue += m_serport.setparams(baud, endline, bits, stopbits, parity, flow, singlechar, timeout);
        }
        else
        {
            retValue += ito::RetVal(ito::retError, 0, tr("Parameter not found").toAscii().data());
        }
    }
    emit parametersChanged(m_params);

end:
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
    // 6. parity [0 .. 2: none, odd, even]
    // 7. flow control [off, hardware, xoff]
    // 8. singlechar [0 .. 1]
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
    int singlechar = 0;
    int timeout = 4000;
    char * tendline = NULL;

    // mandatory parameters
    if (paramsMand == NULL)
    {
        retval = ito::retError;
        goto end;
    }

    retval += m_params["port"].copyValueFrom(&((*paramsMand)[0]));
    port = m_params["port"].getVal<int>();
    m_identifier = QString("COM %1").arg( port );

    retval += m_params["baud"].copyValueFrom(&((*paramsMand)[1]));
    baud = m_params["baud"].getVal<int>();

    retval += m_params["endline"].copyValueFrom(&((*paramsMand)[2]));
    tendline = m_params["endline"].getVal<char *>(); //borrowed reference
    strncpy(endline, tendline, 3);
//    sprintf(endline, "%s", tendline);

    // optional parameters
    if (paramsOpt == NULL)
    {
        retval = ito::retError;
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

    retval += m_params["singlechar"].copyValueFrom(&((*paramsOpt)[4]));
    singlechar = m_params["singlechar"].getVal<int>();

    retval += m_params["timeout"].copyValueFrom(&((*paramsOpt)[5]));
    timeout = (int)(m_params["timeout"].getVal<double>() * 1000 + 0.5);

    retval = m_serport.sopen(port, baud, endline, bits, stopbits, parity, flow, singlechar, timeout);

    retval += m_params["debug"].copyValueFrom(&((*paramsOpt)[6]));
    m_debugMode = (bool)(m_params["debug"].getVal<int>());

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
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StartDevice not necessary").toAscii().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::stopDevice(ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("StopDevice not necessary").toAscii().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::acquire(const int /*trigger*/, ItomSharedSemaphore *waitCond)
{
    ito::RetVal retval = ito::RetVal(ito::retWarning, 0, tr("Acquire not necessary").toAscii().data());

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
        waitCond->deleteSemaphore();
        waitCond = NULL;
    }
    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::getVal(QSharedPointer<char> data, QSharedPointer<int> length, ItomSharedSemaphore *waitCond)
{
//    ito::DataObject *dObj = reinterpret_cast<ito::DataObject *>(vpdObj);
    ItomSharedSemaphoreLocker locker(waitCond);
    ito::RetVal retval;
    retval = m_serport.sread(data.data(), length.data(), 0);

    if (m_debugMode)
    {
        emit serialLog(QByteArray(data.data(),*length), "", '<');
    }

    if (waitCond)
    {
        waitCond->returnValue = retval;
        waitCond->release();
    }

    return retval;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal SerialIO::setVal(const void *data, const int datalength, ItomSharedSemaphore *waitCond)
{
    ItomSharedSemaphoreLocker locker(waitCond);
    const char *buf = (const char*)data;
    char endline[3] = {0, 0, 0};
    ito::RetVal retval(ito::retOk);

    m_serport.getendline(endline);
    if (m_debugMode)
    {
        emit serialLog(QByteArray(buf,datalength), QByteArray(endline, (int)strlen(endline)), '>');
    }
    retval = m_serport.swrite(buf, datalength, m_params["singlechar"].getVal<int>());

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
    }
    else if (funcName == "clearOutputBuffer")
    {
        retval = m_serport.sclearbuffer(1);
    }
    else if (funcName == "clearBuffer")
    {
        ito::ParamBase *bufferType = NULL;
        bufferType = &((*paramsMand)[0]);
        if (!retval.containsError())
        {
            retval = m_serport.sclearbuffer(static_cast<bool>(bufferType->getVal<int>()));
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
    if (getDockWidget())
    {
        DockWidgetSerialIO *dw = qobject_cast<DockWidgetSerialIO*>(getDockWidget()->widget());
        if (visible)
        {
            connect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            connect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), dw, SLOT(serialLog(QByteArray, QByteArray, const char)));

            emit parametersChanged(m_params);
        }
        else
        {
            disconnect(this, SIGNAL(parametersChanged(QMap<QString, ito::Param>)), dw, SLOT(valuesChanged(QMap<QString, ito::Param>)));
            disconnect(this, SIGNAL(serialLog(QByteArray, QByteArray, const char)), dw, SLOT(serialLog(QByteArray, QByteArray, const char)));
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
