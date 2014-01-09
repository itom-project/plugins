#ifndef DIALOGGWINSTEKPSP_H
#define DIALOGGWINSTEKPSP_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

#include "ui_dialogGWInstekPSP.h"

#include <QtGui>
#include <qdialog.h>
#include <qstring.h>

class dialogGWInstekPSP : public QDialog 
{
    Q_OBJECT

    private:
        Ui::dialogGWInstekPSP ui;
        void *m_dataIO;

    public:
        dialogGWInstekPSP(void *dataIO);
        ~dialogGWInstekPSP() {};
        int setVals(QMap<QString, ito::Param> *paramVals);
        int getVals(QMap<QString, ito::Param> *paramVals);

    public slots:
        

    private slots:

};

#endif
