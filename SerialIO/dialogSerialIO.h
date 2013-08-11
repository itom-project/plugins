#ifndef DIALOGSERIALIO_H
#define DIALOGSERIALIO_H

#include "common/sharedStructures.h"
#include "ui_dialogSerialIO.h"

#include <QtGui>
#include <qdialog.h>
#include <QStringList>

class dialogSerialIO : public QDialog 
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

    protected:
        bool eventFilter(QObject *obj, QEvent *event);

    public:
        dialogSerialIO(void *sport);
//        dialogSerialIO() : m_psport(NULL), readDelay(1000) { ui.setupUi(this); };
        ~dialogSerialIO();
        int setVals(QMap<QString, ito::Param> *params);
        int getVals(int &baud, char *endline, int &bits, int &stopbits, int &parity, unsigned int &flow, int &singlechar, double &timeout, bool &debug);

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

    private slots:

};

#endif
