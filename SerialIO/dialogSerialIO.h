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

#ifndef DIALOGSERIALIO_H
#define DIALOGSERIALIO_H

#include "common/sharedStructures.h"
#include "ui_dialogSerialIO.h"
#include "common/abstractAddInConfigDialog.h"

#include <QtGui>
#include <qdialog.h>
#include <QStringList>

namespace ito
{
    class AddInBase; //forward declaration
}

class dialogSerialIO : public ito::AbstractAddInConfigDialog 
{
    Q_OBJECT

    private:
        Ui::dialogSerialIO ui;
        void *m_psport;
//        double m_readDelay;
        int m_baud;
        int m_bits;
        int m_stopbits;
        int m_parity;
        double m_timeout;
        unsigned int m_flow;
        char m_endline[3];
        QStringList m_historyList;
        int m_historyListPointer;

        ito::RetVal parseOutString(char *buf, int *length);

    protected:
        bool eventFilter(QObject *obj, QEvent *event);

    public:
//        dialogSerialIO(void *sport, QString identifier);
        dialogSerialIO(ito::AddInBase *dataIO, void *sport, QString identifier, int baudRatesSize);
        ~dialogSerialIO();
//        int setVals(QMap<QString, ito::Param> *params);
//        int getVals(int &baud, QString &endline, int &bits, int &stopbits, int &parity, unsigned int &flow, int &sendDelay, double &timeout, bool &debug);
        ito::RetVal applyParameters();

    public slots:
        QString interpretAnswer(const char* temp, const int len);
        void on_pushButtonSet_clicked();
        void on_pushButtonCreateCommand_clicked();
        void on_pushSendButton_clicked();
        void on_pushReadButton_clicked();
        void on_lineEditSend_returnPressed();
//        bool on_lineEditSend_eventFilter(QObject *obj, QEvent *event);
        void on_cancelButton_clicked();
        void on_okButton_clicked();
        void on_pushButton_clear_clicked();
        void parametersChanged(QMap<QString, ito::Param> params);

    private slots:

};

#endif
