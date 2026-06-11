/* ********************************************************************
    Plugin "LibModBus" for itom software
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

#include "dialogLibModBus.h"
#include "LibModBus.h"

#include <QMessageBox>

#ifdef __linux__
    #include <unistd.h>
    #include <string.h>
#else
    #include <WinBase.h>
#endif

/*//----------------------------------------------------------------------------------------------------------------------------------
QString dialogLibModBus::interpretAnswer(const char* temp, const int templen)
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
                    ret.append("$(");
                    ret.append((uchar)temp[n]);
                    ret.append(")");
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

                ret.append(QString("%1").arg(QString::number(temp[n], base), len, QLatin1Char('0')).toUpper());
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
int dialogLibModBus::setVals(QMap<QString, ito::Param> *params)
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
}

//----------------------------------------------------------------------------------------------------------------------------------
int dialogLibModBus::getVals(int &baud, char *endline, int &bits, int &stopbits, int &parity, unsigned int &flow, int &sendDelay, double &timeout, bool &debug)
{
    QVariant qvar;

    baud = ui.combo_baud->itemText((int)ui.combo_baud->currentIndex()).toInt();

    debug = ui.checkBox_debug->isChecked();

    qvar = ui.combo_endline->currentIndex();
    switch (qvar.toInt())
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
}

//----------------------------------------------------------------------------------------------------------------------------------
dialogLibModBus::dialogLibModBus(void *sport) :
    m_psport(sport)
{
    ito::RetVal ret;

    memset(m_endline, 0, 3 * sizeof(char));
    ui.setupUi(this);

    ui.lineEditSend->installEventFilter(this);

    LibModBus *sio = (LibModBus *)m_psport;

    QMap<QString, ito::Param> *paramList = NULL;

    sio->getParamList(&paramList);
    setWindowTitle(QString((*paramList)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    m_baud = (*paramList)["baud"].getVal<int>();
    m_bits = (*paramList)["bits"].getVal<int>();
    m_stopbits = (*paramList)["stopbits"].getVal<int>();
    m_parity = (*paramList)["parity"].getVal<int>();
    m_flow = (unsigned int)(*paramList)["flow"].getVal<int>();
    m_timeout = (*paramList)["timeout"].getVal<double>();
    char *buf = (*paramList)["endline"].getVal<char*>();  //borrowed reference
    sprintf(m_endline, "%s", buf);
}

//----------------------------------------------------------------------------------------------------------------------------------
dialogLibModBus::~dialogLibModBus()
{ }

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal parseOutString(char *buf, int *length)
{
    ito::RetVal ret = ito::retOk;
    int len = (int)strlen(buf);
    char *buf1 = NULL;
    char *buf2 = NULL;
    char *out = NULL;
    char *outbuf = (char*)calloc(len + 1, sizeof(char));
    out = outbuf;

    buf1 = buf;
    buf2 = strstr(buf1, "$");
    while (buf2 && ((buf2 - buf) < (len - 1)))
    {
        if (*(buf2 + 1) == '$')
        {
            strncpy(outbuf, buf1, buf2 - buf1);     // copy preceding string
            outbuf += strlen(outbuf);
            buf2++;
            buf2++;                                 // eat the $$
            buf1 = buf2;
            buf2 = strstr(buf2, "$");               // search for next
        }
        else if (*(buf2 + 1) == '(')
        {
            strncpy(outbuf, buf1, buf2 - buf1);     // copy preceding string
            outbuf += strlen(outbuf);
            // try to read number
            buf2++;
            buf2++;                                 // eat the $(
            int charnum = 0;
            char charbuf[4] = {0, 0, 0, 0};

            // read character number until either the character token is closed, three numbers are read
            // or the end of the string is reached
            while ((charnum < 3) && (*buf2 != ')') && ((buf2 - buf) < len - 1))
            {
                charbuf[charnum] = *buf2;
                charnum++;
                buf2++;
            }
            if ((*buf2 != ')') || (atoi(charbuf) > 255))    // char token not closed correctly or number to big -> end with error
            {
                free(out);
                return ito::RetVal(ito::retError, 0, QObject::tr("Char token not closed correctly or number to big.").toLatin1().data());
                //return ito::retError;
            }

            *outbuf = atoi(charbuf);
            outbuf++;
            buf2++;                                 // eat the closing ')'
            buf1 = buf2;
            buf2 = strstr(buf2, "$");               // search for next $
        }
        else
        {
            free(out);
            return ito::RetVal(ito::retError, 0, QObject::tr("Undefined error.").toLatin1().data());
        }
    }
    *length = outbuf - out + len - (buf1 - buf);
    memcpy(outbuf, buf1, len - (buf1 - buf));
//    strncpy(outbuf, buf1, len - (buf1 - buf));  // copy remaining string
    memcpy(buf, out, *length);
//    memcpy(buf, out, strlen(out));
    buf[*length] = 0;
//    buf[strlen(out)] = 0;
    free(out);

    return ret;
}

//----------------------------------------------------------------------------------------------------------------------------------

 bool dialogLibModBus::on_lineEditSend_eventFilter(QObject *obj, QEvent *event)
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

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_lineEditSend_returnPressed()
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
    char *tmpChar;

    qstr = ui.lineEditSend->text();
    ui.lineEditSend->setText("");
    if (!m_historyList.filter(qstr).empty())
    {
        m_historyList.removeAt(m_historyList.indexOf(qstr));
    }
    m_historyList.append(qstr);
    m_historyListPointer = m_historyList.count();
    qout = "> ";

    LibModBus *sio = (LibModBus *)m_psport;
    QMap<QString, ito::Param> *paramList = NULL;
    ret += sio->getParamList(&paramList);
    char *buf = (*paramList)["endline"].getVal<char*>(); //borrowed reference
    sprintf(endline, "%s", buf);

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
                        qb.append(char (tmpInt));
                    }
                }

                if (i > 0)
                {
                    qout.append(" ");
                }

                if (ok)
                {
                    qout.append(QString("%1").arg(QString::number(tmpInt, base), len, QLatin1Char('0')).toUpper());
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
            ui.text_transfer->append(tr("Error: '%1' could not be interpreted - not send").arg(err));
            return;
        }

        if (length > 0)
        {
            qout.append(" ");
        }

        if (strcmp(endline, "\n") == 0)
        {
            qout.append(QString("%1").arg(QString::number(10, base), len, QLatin1Char('0')).toUpper());
        }
        else if (strcmp(endline, "\r") == 0)
        {
            qout.append(QString("%1").arg(QString::number(13, base), len, QLatin1Char('0')).toUpper());
        }
        else if (strcmp(endline, "\r\n") == 0)
        {
            qout.append(QString("%1").arg(QString::number(10, base), len, QLatin1Char('0')).toUpper());
            qout.append(" ");
            qout.append(QString("%1").arg(QString::number(13, base), len, QLatin1Char('0')).toUpper());
        }

        tmpChar = qb.data();
    }

    ui.text_transfer->append(qout);

    //ItomSharedSemaphore *waitCond = new ItomSharedSemaphore();
    //QMetaObject::invokeMethod(sio, "setVal", Q_ARG(ito::RetVal*, &ret), Q_ARG(const void*, (const void*)tmpChar), Q_ARG(ItomSharedSemaphore *, waitCond));
    //waitCond->wait(15000);
    //ItomSharedSemaphore::deleteSemaphore(waitCond);
    ret += sio->setVal(tmpChar, length, 0);
    if (ret != ito::retError)
    {
        Sleep(ui.spinBox_readdelay->value());

        //waitCond = new ItomSharedSemaphore();
        //QMetaObject::invokeMethod(sio, "getVal", Q_ARG(ito::RetVal*, &ret), Q_ARG(char*, readBuf), Q_ARG(int *, &len), Q_ARG(ItomSharedSemaphore *, waitCond));
        //waitCond->wait(15000);
        //ItomSharedSemaphore::deleteSemaphore(waitCond);
        ret += sio->getVal(readBuf, len, 0);
    }

    if (!ret.containsError())
    {
        ui.text_transfer->append(interpretAnswer(readBuf.data(), *len));
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText(ret.errorMessage());
        msgBox.setInformativeText("");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        ret = msgBox.exec();
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_pushButtonSet_clicked()
{
    int baud;
    int bits;
    int stopbits;
    int parity;
    unsigned int flow;
    int sendDelay;
    double timeout;
    bool enableDebug;
    ito::RetVal ret;
    char endline[3] = {0, 0, 0};

    LibModBus *sio = (LibModBus *)m_psport;
    getVals(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout, enableDebug);

    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, endline)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, baud)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, bits)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, stopbits)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Int, parity)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, (int)flow)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("sendDelay", ito::ParamBase::Int, sendDelay)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, timeout)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("debug", ito::ParamBase::Int, enableDebug)));
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_pushButtonCreateCommand_clicked()
{
    char txt[100];
    int baud;
    int bits;
    int stopbits;
    int parity;
    unsigned int flow;
    int sendDelay;
    double timeout;
    bool enableDebug;
    ito::RetVal ret;
    char endline[3] = {0, 0, 0};

    LibModBus *sio = (LibModBus *)m_psport;
    QMap<QString, ito::Param> *paramList = NULL;
    sio->getParamList(&paramList);

    getVals(baud, endline, bits, stopbits, parity, flow, sendDelay, timeout, enableDebug);

    char *deviceName = (*paramList)["name"].getVal<char*>(); //borrowed reference
    sprintf(txt,
            "dataIO(\"%s\",%d,%d,\"%s\",%d,%d,%d,%d,%d,%f)",
            deviceName,
            (*paramList)["port"].getVal<int>(),
            baud,
            endline,
            bits,
            stopbits,
            parity,
            flow,
            sendDelay,
            timeout);
    ui.lineEditCommandStr->setText(txt);
*/
/*    char txt[100];
    char endline[5] = { 0, 0, 0, 0, 0 };
    LibModBus *sio = (LibModBus *)m_psport;
    QMap<QString, ito::Param> *paramList = NULL;

    sio->getParamList(&paramList);

    endline[0] = '\\';
    char *temp = (*paramList)["endline"].getVal<char*>(); //borrowed reference
    if (strcmp(temp, "\r\n") == 0)
    {
        endline[1] = 'r';
        endline[2] = '\\';
        endline[3] = 'n';
    }
    else if (strcmp(temp, "\n") == 0)
    {
        endline[1] = 'n';
    }
    else if (strcmp(temp, "\r") == 0)
    {
        endline[1] = 'r';
    }
    else
    {
        endline[0] = 0;
    }

    char *deviceName = (*paramList)["name"].getVal<char*>(); //borrowed reference
    sprintf(txt,
            "dataIO(\"%s\",%d,%d,\"%s\",%d,%d,%d,%d,%d,%f)",
            deviceName,
            (*paramList)["port"].getVal<int>(),
            (*paramList)["baud"].getVal<int>(),
            endline,
            (*paramList)["bits"].getVal<int>(),
            (*paramList)["stopbits"].getVal<int>(),
            (*paramList)["parity"].getVal<int>(),
            (*paramList)["flow"].getVal<int>(),
            (*paramList)["sendDelay"].getVal<int>(),
            (*paramList)["timeout"].getVal<double>());
    ui.lineEditCommandStr->setText(txt);

//    QClipboard *clipboard = QApplication::clipboard();
//    clipboard->setText(txt);*/
/*}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_pushSendButton_clicked()
{
    on_lineEditSend_returnPressed();
    ui.lineEditSend->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_pushReadButton_clicked()
{
    ito::RetVal ret;
    int maxlen = 255;
    QSharedPointer<char> readBuf(new char[maxlen]);
    QSharedPointer<int> len(new int);
    *len = maxlen;

    LibModBus *sio = (LibModBus *)m_psport;
    ret += sio->getVal(readBuf, len, 0);

    if (!ret.containsError())
    {
        ui.text_transfer->append(interpretAnswer(readBuf.data(), *len));
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText(ret.errorMessage());
        msgBox.setInformativeText("");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        ret = msgBox.exec();
    }
    ui.lineEditSend->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_cancelButton_clicked()
{
    ito::RetVal ret;
    LibModBus *sio = (LibModBus *)m_psport;

    if (m_endline)
    {
        ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("endline", ito::ParamBase::String, m_endline)));
    }
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("baud", ito::ParamBase::Int, m_baud)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bits", ito::ParamBase::Int, m_bits)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("stopbits", ito::ParamBase::Int, m_stopbits)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("parity", ito::ParamBase::Int, m_parity)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("flow", ito::ParamBase::Int, (int)m_flow)));
    ret += sio->setParam(QSharedPointer<ito::ParamBase>(new ito::ParamBase("timeout", ito::ParamBase::Double, m_timeout)));
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_okButton_clicked()
{
    on_pushButtonSet_clicked();
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogLibModBus::on_pushButton_clear_clicked()
{
    ui.text_transfer->clear();
    ui.lineEditSend->setFocus();
}

//----------------------------------------------------------------------------------------------------------------------------------
bool dialogLibModBus::eventFilter(QObject *obj, QEvent *event)
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
*/
