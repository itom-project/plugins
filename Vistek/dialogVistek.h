#ifndef DIALOGVISTEK_H
#define DIALOGVISTEK_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogVistek.h"
#include "common/sharedStructures.h"
#include "common/addInGrabber.h"

class DialogVistek : public QDialog 
{
    Q_OBJECT

    public:
        DialogVistek() { ui.setupUi(this); };
        ~DialogVistek() {};
		void sendVals(ito::AddInGrabber *receiverGrabber);

    private:
        Ui::dialogVistek ui;
		QMap<QString, ito::Param> m_params;

    public slots:
		void valuesChanged(QMap<QString, ito::Param> params);
};

#endif
