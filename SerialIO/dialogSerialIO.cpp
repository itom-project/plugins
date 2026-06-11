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

#include "dialogSerialIO.h"
#include "SerialIO.h"

#include <QMessageBox>

#ifndef WIN32
#include <string.h>
#include <unistd.h>
#else
#include <WinBase.h>
#endif
//----------------------------------------------------------------------------------------------------------------------------------
QString dialogSerialIO::interpretAnswer(const char* temp, const int templen)
{
    QString ret = "";

    if (templen > 0)
    {
        ret = "< ";
        if (ui.RBASCII->isChecked())
        {
            for (int n = 0; n < (templen > 255 ? 255 : templen); n++)
            {
                if (temp[n] == '\r')
                {
                    ret.append("\\r");
                }
                else if (temp[n] == '\n')
                {
                    ret.append("\\n");
                }
                else if (temp[n] == '\t')
                {
                    ret.append("\\t");
                }
                else if (((uchar)temp[n] < 32) || ((uchar)temp[n] > 127))
                {
                    QString s;
                    s.setNum((uchar)temp[n]);
                    ret.append("$(" + s + ")");
                }
                else
                {
                    ret.append(temp[n]);
                }
            }
        }
        else
        {
            int base = 0;
            int len = 0;
            if (ui.RBHex->isChecked())
            {
                ret.append("0x ");
                base = 16;
                len = 2;
            }
            else if (ui.RBBin->isChecked())
            {
                base = 2;
                len = 8;
            }
            else
            {
                base = 10;
                len = 3;
            }

            for (int n = 0; n < (templen > 255 ? 255 : templen); n++)
            {
                if (n > 0)
                {
                    ret.append(" ");
                }

                ret.append(QString("%1")
                               .arg(QString::number(temp[n], base), len, QLatin1Char('0'))
                               .toUpper());
            }
        }
        ret.append(QString(" [%1]").arg(templen));
    }
    else
    {
        ret = "< ---";
    }
    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*int dialogSerialIO::setVals(QMap<QString, ito::Param> *params)
{
    QString BaudStr = QString::number((*params)["baud"].getVal<int>());
    int i = 0;
    while (i < ui.combo_baud->count() && ui.combo_baud->itemText(i) != BaudStr)
    {
        i++;
    }
    if (i == ui.combo_baud->count())
    {
        i = 12;
    }
    ui.combo_baud->setCurrentIndex(i);

    if ((*params).keys().contains("debug"))
    {
        ui.checkBox_debug->setChecked((*params)["debug"].getVal<int>());
    }

    char *endline = (*params)["endline"].getVal<char *>(); //borrowed reference
    if (strcmp(endline, "\r") == 0)
    {
        ui.combo_endline->setCurrentIndex(0);
    }
    else if (strcmp(endline, "\n") == 0)
    {
        ui.combo_endline->setCurrentIndex(1);
    }
    else if (strcmp(endline, "\r\n") == 0)
    {
        ui.combo_endline->setCurrentIndex(2);
    }
    else
    {
        ui.combo_endline->setCurrentIndex(3);
    }

    ui.combo_bits->setCurrentIndex((*params)["bits"].getVal<int>() - 5);
    ui.combo_stopbits->setCurrentIndex((*params)["stopbits"].getVal<int>() - 1);
    ui.combo_parity->setCurrentIndex((*params)["parity"].getVal<int>());

    unsigned int flow = (unsigned int)((*params)["flow"].getVal<int>());
    ui.combo_flow_xonxoff->setCurrentIndex(flow & 1);
    ui.combo_flow_rts->setCurrentIndex((flow >> 1) & 3);
    ui.combo_flow_cts->setCurrentIndex((flow >> 3) & 1);
    ui.combo_flow_dtr->setCurrentIndex((flow >> 4) & 3);
    ui.combo_flow_dsr->setCurrentIndex((flow >> 6) & 1);

    int timeout = (int)((*params)["timeout"].getVal<double>() * 1000.0 + 0.5);
    ui.spinBox_timeout->setValue(timeout);
    int timeoutmax = (int)((*params)["timeout"].getMax() * 1000.0 + 0.5);
    ui.spinBox_timeout->setMaximum(timeoutmax);

    int sendDelay = (*params)["sendDelay"].getVal<int>();
    ui.spinBox_sendDelay->setValue(sendDelay);

    return 0;
}*/

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::parametersChanged(QMap<QString, ito::Param> params)
{
    QString BaudStr = QString::number((params)["baud"].getVal<int>());
    int i = 0;
    while (i < ui.combo_baud->count() && ui.combo_baud->itemText(i) != BaudStr)
    {
        i++;
    }
    if (i == ui.combo_baud->count())
    {
        i = 12;
    }
    ui.combo_baud->setCurrentIndex(i);

    if ((params).keys().contains("debug"))
    {
        ui.checkBox_debug->setChecked((params)["debug"].getVal<int>());
    }

    char* endline = (params)["endline"].getVal<char*>(); // borrowed reference
    if (strcmp(endline, "\r") == 0)
    {
        ui.combo_endline->setCurrentIndex(0);
    }
    else if (strcmp(endline, "\n") == 0)
    {
        ui.combo_endline->setCurrentIndex(1);
    }
    else if (strcmp(endline, "\r\n") == 0)
    {
        ui.combo_endline->setCurrentIndex(2);
    }
    else
    {
        QByteArray ba = endline;
        QByteArray ba1 = ba.toHex();
        QString text = ba1;
        if (text.size() > 0)
        {
            text = "0x" + text;
        }

        ui.combo_endline->addItem(text, ba);
        ui.combo_endline->setCurrentIndex(4);
    }

    endline = (params)["endlineRead"].getVal<char*>(); // borrowed reference
    if (strcmp(endline, "\r") == 0)
    {
        ui.combo_endlineRead->setCurrentIndex(0);
    }
    else if (strcmp(endline, "\n") == 0)
    {
        ui.combo_endlineRead->setCurrentIndex(1);
    }
    else if (strcmp(endline, "\r\n") == 0)
    {
        ui.combo_endlineRead->setCurrentIndex(2);
    }
    else
    {
        QByteArray ba = endline;
        QByteArray ba1 = ba.toHex();
        QString text = ba1;
        if (text.size() > 0)
        {
            text = "0x" + text;
        }

        ui.combo_endlineRead->addItem(text, ba);
        ui.combo_endlineRead->setCurrentIndex(4);
    }

    if ((params).keys().contains("readline"))
    {
        ui.checkBox_readline->setChecked((params)["readline"].getVal<int>());
    }

    ui.combo_bits->setCurrentIndex((params)["bits"].getVal<int>() - 5);
    ui.combo_stopbits->setCurrentIndex((params)["stopbits"].getVal<int>() - 1);
    ui.combo_parity->setCurrentIndex((params)["parity"].getVal<int>());

    unsigned int flow = (unsigned int)((params)["flow"].getVal<int>());
    ui.combo_flow_xonxoff->setCurrentIndex(flow & 1);
    ui.combo_flow_rts->setCurrentIndex((flow >> 1) & 3);
    ui.combo_flow_cts->setCurrentIndex((flow >> 3) & 1);
    ui.combo_flow_dtr->setCurrentIndex((flow >> 4) & 3);
    ui.combo_flow_dsr->setCurrentIndex((flow >> 6) & 1);

    int timeout = (int)((params)["timeout"].getVal<double>() * 1000.0 + 0.5);
    ui.spinBox_timeout->setValue(timeout);
    int timeoutmax = (int)((params)["timeout"].getMax() * 1000.0 + 0.5);
    ui.spinBox_timeout->setMaximum(timeoutmax);

    int sendDelay = (params)["sendDelay"].getVal<int>();
    ui.spinBox_sendDelay->setValue(sendDelay);

    m_currentParameters = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal dialogSerialIO::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase>> values;
    bool success = false;

    // only send parameters which are changed

    int i = ui.combo_baud->itemText((int)ui.combo_baud->currentIndex()).toInt();
    if (m_currentParameters["baud"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, i)));
    }

    i = ui.checkBox_debug->isChecked() ? 1 : 0;
    if (m_currentParameters["debug"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("debug", ito::ParamBase::Int, i)));
    }

    i = ui.checkBox_readline->isChecked() ? 1 : 0;
    if (m_currentParameters["readline"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("readline", ito::ParamBase::Int, i)));
    }

    char endline[3] = {0, 0, 0};
    switch (ui.combo_endline->currentIndex())
    {
    case 0:
        endline[0] = '\r';
        break;
    case 1:
        endline[0] = '\n';
        break;
    case 2:
        endline[0] = '\r';
        endline[1] = '\n';
        break;
    case 3:
        endline[0] = 0;
        break;
    case 4:
#if QT_VERSION < 0x050200
        QByteArray var = ui.combo_endline->itemData(ui.combo_endline->currentIndex()).toByteArray();
#else
        QByteArray var = ui.combo_endline->currentData().toByteArray();
#endif
        memcpy(endline, var.data(), std::min(3, (int)var.size()) * sizeof(char));
        break;
    }
    if (strcmp(m_currentParameters["endline"].getVal<char*>(), endline) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("endline", ito::ParamBase::String, endline)));
    }

    endline[0] = 0;
    endline[1] = 0;
    endline[2] = 0;
    switch (ui.combo_endlineRead->currentIndex())
    {
    case 0:
        endline[0] = '\r';
        break;
    case 1:
        endline[0] = '\n';
        break;
    case 2:
        endline[0] = '\r';
        endline[1] = '\n';
        break;
    case 3:
        endline[0] = 0;
        break;
    case 4:
#if QT_VERSION < 0x050200
        QByteArray var =
            ui.combo_endlineRead->itemData(ui.combo_endlineRead->currentIndex()).toByteArray();
#else
        QByteArray var = ui.combo_endlineRead->currentData().toByteArray();
#endif
        memcpy(endline, var.data(), std::min(3, (int)var.size()) * sizeof(char));
        break;
    }
    if (strcmp(m_currentParameters["endlineRead"].getVal<char*>(), endline) != 0)
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("endlineRead", ito::ParamBase::String, endline)));
    }

    i = ui.combo_bits->currentIndex() + 5;
    if (m_currentParameters["bits"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, i)));
    }

    i = ui.combo_stopbits->currentIndex() + 1;
    if (m_currentParameters["stopbits"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, i)));
    }

    i = ui.combo_parity->currentIndex();
    if (m_currentParameters["parity"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Int, i)));
    }

    i = ui.combo_flow_xonxoff->currentIndex() + ui.combo_flow_rts->currentIndex() * 2 +
        ui.combo_flow_cts->currentIndex() * 8 + ui.combo_flow_dtr->currentIndex() * 16 +
        ui.combo_flow_dsr->currentIndex() * 64;
    if (m_currentParameters["flow"].getVal<int>() != i)
    {
        values.append(
            QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, i)));
    }

    double v = (double)ui.spinBox_timeout->value() / 1000.0;
    if (m_currentParameters["timeout"].getVal<double>() != v)
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("timeout", ito::ParamBase::Double, v)));
    }

    i = ui.spinBox_sendDelay->value();
    if (m_currentParameters["sendDelay"].getVal<int>() != i)
    {
        values.append(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("sendDelay", ito::ParamBase::Int, i)));
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
/*int dialogSerialIO::getVals(int &baud, QString &endline, int &bits, int &stopbits, int &parity,
unsigned int &flow, int &sendDelay, double &timeout, bool &debug)
{
    baud = ui.combo_baud->itemText((int)ui.combo_baud->currentIndex()).toInt();

    debug = ui.checkBox_debug->isChecked();

    endline = ui.combo_endline->currentText();
    if (endline.size() > 4)
    {
        endline = "";
    }

    bits = ui.combo_bits->currentIndex() + 5;

    stopbits = ui.combo_stopbits->currentIndex() + 1;

    parity = ui.combo_parity->currentIndex();

    flow = ui.combo_flow_xonxoff->currentIndex() +
        ui.combo_flow_rts->currentIndex() * 2 +
        ui.combo_flow_cts->currentIndex() * 8 +
        ui.combo_flow_dtr->currentIndex() * 16 +
        ui.combo_flow_dsr->currentIndex() * 64;

    timeout = (double)ui.spinBox_timeout->value() / 1000.0;

    sendDelay = ui.spinBox_sendDelay->value();

    return 0;
}*/

//----------------------------------------------------------------------------------------------------------------------------------
// dialogSerialIO::dialogSerialIO(void *sport, QString identifier) :
//    m_psport(sport)
dialogSerialIO::dialogSerialIO(
    ito::AddInBase* dataIO, void* sport, QString identifier, int baudRatesSize) :
    AbstractAddInConfigDialog(dataIO),
    m_psport(sport)
{
    ito::RetVal ret;

    memset(m_endline, 0, 3 * sizeof(char));
    ui.setupUi(this);

    //    ui.combo_baud->clear();
    for (int i = 0; i < baudRatesSize; i++)
    {
        ui.combo_baud->addItem(QString::number(SerialPort::baudRates[i]));
    }

    ui.lineEditSend->installEventFilter(this);

    SerialIO* sio = (SerialIO*)m_psport;

    QMap<QString, ito::Param>* paramList = NULL;

    sio->getParamList(&paramList);
    setWindowTitle(
        QString((*paramList)["name"].getVal<char*>()) + ": " + identifier + " - " +
        tr("Configuration Dialog"));
    m_baud = (*paramList)["baud"].getVal<int>();
    m_bits = (*paramList)["bits"].getVal<int>();
    m_stopbits = (*paramList)["stopbits"].getVal<int>();
    m_parity = (*paramList)["parity"].getVal<int>();
    m_flow = (unsigned int)(*paramList)["flow"].getVal<int>();
    m_timeout = (*paramList)["timeout"].getVal<double>();
    char* buf = (*paramList)["endline"].getVal<char*>(); // borrowed reference
    sprintf(m_endline, "%s", buf);
}

//----------------------------------------------------------------------------------------------------------------------------------
dialogSerialIO::~dialogSerialIO()
{
}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal dialogSerialIO::parseOutString(char* buf, int* length)
{
    ito::RetVal ret = ito::retOk;
    int len = (int)strlen(buf);
    char* buf1 = NULL;
    char* buf2 = NULL;
    char* out = NULL;
    char* outbuf = (char*)calloc(len + 1, sizeof(char));
    out = outbuf;

    buf1 = buf;
    buf2 = strstr(buf1, "$");

    if (ui.checkAsciiParsing->isChecked())
    {
        while (buf2 && ((buf2 - buf) < (len - 1)))
        {
            if (*(buf2 + 1) == '$')
            {
                strncpy(outbuf, buf1, buf2 - buf1); // copy preceding string
                outbuf += strlen(outbuf);
                buf2++;
                buf2++; // eat the $$
                buf1 = buf2;
                buf2 = strstr(buf2, "$"); // search for next
            }
            else if (*(buf2 + 1) == '(')
            {
                strncpy(outbuf, buf1, buf2 - buf1); // copy preceding string
                outbuf += strlen(outbuf);
                // try to read number
                buf2++;
                buf2++; // eat the $(
                int charnum = 0;
                char charbuf[4] = {0, 0, 0, 0};

                // read character number until either the character token is closed, three numbers
                // are read or the end of the string is reached
                while ((charnum < 3) && (*buf2 != ')') && ((buf2 - buf) < len - 1))
                {
                    charbuf[charnum] = *buf2;
                    charnum++;
                    buf2++;
                }
                if ((*buf2 != ')') ||
                    (atoi(charbuf) >
                     255)) // char token not closed correctly or number to big -> end with error
                {
                    free(out);
                    return ito::RetVal(
                        ito::retError,
                        0,
                        QObject::tr("Char token not closed correctly or number to big.")
                            .toLatin1()
                            .data());
                    // return ito::retError;
                }

                *outbuf = atoi(charbuf);
                outbuf++;
                buf2++; // eat the closing ')'
                buf1 = buf2;
                buf2 = strstr(buf2, "$"); // search for next $
            }
            else
            {
                free(out);
                return ito::RetVal(
                    ito::retError, 0, QObject::tr("Undefined error.").toLatin1().data());
            }
        }
    }

    *length = outbuf - out + len - (buf1 - buf);
    // Check if buf is not empty before accessing its elements
    if (*length > 0)
    {
        memcpy(outbuf, buf1, len - (buf1 - buf));
        memcpy(buf, out, *length);
        buf[*length] = 0;
    }
    else
    {
        // If buf is empty, set length to 0 and return without further processing
        *length = 0;
    }
    free(out);

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------
/*
 bool dialogSerialIO::on_lineEditSend_eventFilter(QObject *obj, QEvent *event)
 {
     if (event->type() == QEvent::KeyPress)
     {
         QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
         ui.lineEditSend->setText("Ate key press " + keyEvent->key());
//         qDebug("Ate key press %d", keyEvent->key());
         return true;
     }
     else
     {
         // standard event processing
         return QObject::eventFilter(obj, event);
     }
 }
*/
//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_lineEditSend_returnPressed()
{
    ito::RetVal ret;
    QString qstr, qout;
    char endline[3] = {0, 0, 0};
    int length = 0;
    int maxlen = 255;
    QSharedPointer<char> readBuf(new char[maxlen]);
    QSharedPointer<int> len(new int);
    *len = maxlen;
    QByteArray qb;
    char* tmpChar;

    qstr = ui.lineEditSend->text();
    ui.lineEditSend->setText("");

    if (!qstr.isEmpty())
    {
        for (int i = 0; i < m_historyList.count(); ++i)
        {
            if (qstr.compare(m_historyList.at(i), Qt::CaseInsensitive) == 0)
            {
                m_historyList.removeAt(i);
                break;
            }
        }

        m_historyList.append(qstr);
        m_historyListPointer = m_historyList.count();
    }
    qout = "> ";

    SerialIO* sio = (SerialIO*)m_psport;
    QMap<QString, ito::Param>* paramList = NULL;
    ret += sio->getParamList(&paramList);
    char* buf = (*paramList)["endline"].getVal<char*>(); // borrowed reference
    sprintf(endline, "%s", buf);
    bool test = false;
    test = strcmp(endline, "\0");

    if (ui.RBASCII->isChecked())
    {
        qout.append(qstr);
        qb = qstr.toLatin1();
        tmpChar = qb.data();
        if (parseOutString(tmpChar, &length) == ito::retError)
        {
            ui.text_transfer->append(tr("Error: malformed command string - not send"));
            return;
        }

        if (strcmp(endline, "\n") == 0)
        {
            qout.append("\\n");
        }
        else if (strcmp(endline, "\r") == 0)
        {
            qout.append("\\r");
        }
        else if (strcmp(endline, "\r\n") == 0)
        {
            qout.append("\\r\\n");
        }
        else if (strcmp(endline, ""))
        {
            bool isprintable = true;
            for (int i = 0; i < 3; ++i)
            {
                if ((endline[i] <= 32 || endline[i] > 126) && endline[i] != 0)
                {
                    if (isprintable)
                    {
                        qout.append("<font color=red>'0x");
                    }
                    qout.append(QString::number(endline[i], 16).rightJustified(2, '0'));
                    isprintable = false;
                }
                else
                {
                    if (!isprintable)
                    {
                        qout.append("'</font>");
                        isprintable = true;
                    }
                    qout.append(endline[i]);
                }
            }
            if (!isprintable)
            {
                qout.append("'</font>");
            }
        }
    }
    else
    {
        int base = 0;
        int len = 0;
        if (ui.RBHex->isChecked())
        {
            qout.append("0x ");
            base = 16;
            len = 2;
        }
        else if (ui.RBBin->isChecked())
        {
            base = 2;
            len = 8;
        }
        else
        {
            base = 10;
            len = 3;
        }

        int tmpInt;
        QString err = "";
        bool ok = true;
        QStringList list = qstr.split(" ");
        length = list.count();
        for (int i = 0; i < length; i++)
        {
            if (list[i].length() > 0)
            {
                tmpInt = list[i].toInt(&ok, base);
                if (err == "")
                {
                    if (!ok || tmpInt > 255)
                    {
                        err = list[i];
                    }
                    else
                    {
                        qb.append(char(tmpInt));
                    }
                }

                if (i > 0)
                {
                    qout.append(" ");
                }

                if (ok)
                {
                    qout.append(QString("%1")
                                    .arg(QString::number(tmpInt, base), len, QLatin1Char('0'))
                                    .toUpper());
                }
                else
                {
                    qout.append(list[i]);
                }
            }
        }

        if (err != "")
        {
            ui.text_transfer->append(qout);
            ui.text_transfer->append(
                tr("Error: '%1' could not be interpreted - not send").arg(err));
            return;
        }

        if (length > 0)
        {
            qout.append(" ");
        }

        if (strcmp(endline, "\n") == 0)
        {
            qout.append(
                QString("%1").arg(QString::number(10, base), len, QLatin1Char('0')).toUpper());
        }
        else if (strcmp(endline, "\r") == 0)
        {
            qout.append(
                QString("%1").arg(QString::number(13, base), len, QLatin1Char('0')).toUpper());
        }
        else if (strcmp(endline, "\r\n") == 0)
        {
            qout.append(
                QString("%1").arg(QString::number(10, base), len, QLatin1Char('0')).toUpper());
            qout.append(" ");
            qout.append(
                QString("%1").arg(QString::number(13, base), len, QLatin1Char('0')).toUpper());
        }

        tmpChar = qb.data();
    }
    // ui.text_transfer->insertHtml(qout);
    ui.text_transfer->append(qout);

    // ItomSharedSemaphore *waitCond = new ItomSharedSemaphore();
    // QMetaObject::invokeMethod(sio, "setVal", Q_ARG(ito::RetVal*, &ret), Q_ARG(const void*, (const
    // void*)tmpChar), Q_ARG(ItomSharedSemaphore *, waitCond)); waitCond->wait(15000);
    // ItomSharedSemaphore::deleteSemaphore(waitCond);
    ret += sio->setVal(tmpChar, length, 0);

    if (ui.checkReadAfterSend->isChecked())
    {
        if (!ret.containsError())
        {
            Sleep(ui.spinBox_readdelay->value());

            // waitCond = new ItomSharedSemaphore();
            // QMetaObject::invokeMethod(sio, "getVal", Q_ARG(ito::RetVal*, &ret), Q_ARG(char*,
            // readBuf), Q_ARG(int *, &len), Q_ARG(ItomSharedSemaphore *, waitCond));
            // waitCond->wait(15000);
            // ItomSharedSemaphore::deleteSemaphore(waitCond);
            ret += sio->getVal(readBuf, len, 0);
        }

        if (!ret.containsError())
        {
            ui.text_transfer->append(interpretAnswer(readBuf.data(), *len));
        }
    }

    if (ret.containsError())
    {
        QMessageBox msgBox;
        msgBox.setText(QLatin1String(ret.errorMessage()));
        msgBox.setInformativeText("");
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        ret = msgBox.exec();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_pushButtonSet_clicked()
{
    applyParameters();
    /*    int baud;
        int bits;
        int stopbits;
        int parity;
        unsigned int flow;
        int sendDelay;
        double timeout;
        bool enableDebug;
        ito::RetVal ret;
        char endline[3] = {0, 0, 0};

        SerialIO *sio = (SerialIO *)m_psport;
        getVals(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout, enableDebug);

        ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline",
       ito::ParamBase::String, endline))); ret += sio->setParam(QSharedPointer<ito::ParamBase>(new
       ito::ParamBase("baud", ito::ParamBase::Int, baud))); ret +=
       sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int,
       bits))); ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits",
       ito::ParamBase::Int, stopbits))); ret += sio->setParam(QSharedPointer<ito::ParamBase>(new
       ito::ParamBase("parity", ito::ParamBase::Int, parity))); ret +=
       sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int,
       (int)flow))); ret += sio->setParam(QSharedPointer<ito::ParamBase>(new
       ito::ParamBase("sendDelay", ito::ParamBase::Int, sendDelay))); ret +=
       sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout",
       ito::ParamBase::Double, timeout))); ret += sio->setParam(QSharedPointer<ito::ParamBase>(new
       ito::ParamBase("debug", ito::ParamBase::Int, enableDebug)));*/
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_pushButtonCreateCommand_clicked()
{
    char txt[150];
    ito::RetVal ret;

    SerialIO* sio = (SerialIO*)m_psport;
    QMap<QString, ito::Param>* paramList = NULL;
    sio->getParamList(&paramList);

    int baud = ui.combo_baud->itemText((int)ui.combo_baud->currentIndex()).toInt();

    QString endline = ui.combo_endline->currentText();
    if (endline.size() > 4)
    {
        endline = "";
    }


    int bits = ui.combo_bits->currentIndex() + 5;

    int stopbits = ui.combo_stopbits->currentIndex() + 1;

    int parity = ui.combo_parity->currentIndex();

    unsigned int flow = ui.combo_flow_xonxoff->currentIndex() +
        ui.combo_flow_rts->currentIndex() * 2 + ui.combo_flow_cts->currentIndex() * 8 +
        ui.combo_flow_dtr->currentIndex() * 16 + ui.combo_flow_dsr->currentIndex() * 64;

    double timeout = (double)ui.spinBox_timeout->value() / 1000.0;

    int sendDelay = ui.spinBox_sendDelay->value();

    char* deviceName = (*paramList)["name"].getVal<char*>(); // borrowed reference
    if (endline.compare("\r") || endline.compare("\r\n") || endline.compare("\n") ||
        endline.compare(
            "")) // all those are standard endline characters all other will be displayed as hex
    {
        sprintf(
            txt,
            "dataIO(\"%s\",%d,%d,\"%s\",%d,%d,%d,%d,%d,%.3f)",
            deviceName,
            (*paramList)["port"].getVal<int>(),
            baud,
            endline.toLatin1().data(),
            bits,
            stopbits,
            parity,
            flow,
            sendDelay,
            timeout);
    }
    else
    {
        sprintf(
            txt,
            "dataIO(\"%s\",%d,%d,chr(%s),%d,%d,%d,%d,%d,%.3f)",
            deviceName,
            (*paramList)["port"].getVal<int>(),
            baud,
            endline.toLatin1().data(),
            bits,
            stopbits,
            parity,
            flow,
            sendDelay,
            timeout);
    }
    ui.lineEditCommandStr->setText(txt);

    //    QClipboard *clipboard = QApplication::clipboard();
    //    clipboard->setText(txt);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_pushSendButton_clicked()
{
    on_lineEditSend_returnPressed();
    ui.lineEditSend->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_pushReadButton_clicked()
{
    ito::RetVal ret;
    int maxlen = 255;
    QSharedPointer<char> readBuf(new char[maxlen]);
    QSharedPointer<int> len(new int);
    *len = maxlen;

    SerialIO* sio = (SerialIO*)m_psport;
    ret += sio->getVal(readBuf, len, 0);

    if (!ret.containsError())
    {
        ui.text_transfer->append(interpretAnswer(readBuf.data(), *len));
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText(QLatin1String(ret.errorMessage()));
        msgBox.setInformativeText("");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        ret = msgBox.exec();
    }
    ui.lineEditSend->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_cancelButton_clicked()
{
    ito::RetVal ret;
    SerialIO* sio = (SerialIO*)m_psport;

    if (m_endline)
    {
        ret += sio->setParam(QSharedPointer<ito::ParamBase>(
            new ito::ParamBase("endline", ito::ParamBase::String, m_endline)));
    }
    ret += sio->setParam(
        QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, m_baud)));
    ret += sio->setParam(
        QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, m_bits)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(
        new ito::ParamBase("stopbits", ito::ParamBase::Int, m_stopbits)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(
        new ito::ParamBase("parity", ito::ParamBase::Int, m_parity)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(
        new ito::ParamBase("flow", ito::ParamBase::Int, (int)m_flow)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(
        new ito::ParamBase("timeout", ito::ParamBase::Double, m_timeout)));
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_okButton_clicked()
{
    on_pushButtonSet_clicked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogSerialIO::on_pushButton_clear_clicked()
{
    ui.text_transfer->clear();
    ui.lineEditSend->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool dialogSerialIO::eventFilter(QObject* obj, QEvent* event)
{
    if (obj == ui.lineEditSend)
    {
        if (event->type() == QEvent::KeyPress && !m_historyList.empty())
        {
            QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
            if (keyEvent->key() == Qt::Key_Up)
            {
                if (m_historyListPointer > 0)
                {
                    m_historyListPointer--;
                    ui.lineEditSend->setText(m_historyList.value(m_historyListPointer));
                }
                return true;
            }
            else if (keyEvent->key() == Qt::Key_Down)
            {
                if (m_historyListPointer < m_historyList.count())
                {
                    m_historyListPointer++;
                    if (m_historyListPointer == m_historyList.count())
                    {
                        ui.lineEditSend->setText("");
                    }
                    else
                    {
                        ui.lineEditSend->setText(m_historyList.value(m_historyListPointer));
                    }
                }
                return true;
            }
        }
        else
        {
            // standard event processing
            return QObject::eventFilter(obj, event);
        }
    }

    return QObject::eventFilter(obj, event);
}

//----------------------------------------------------------------------------------------------------------------------------------
