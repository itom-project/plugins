#ifndef DIALOGDUMMYGRABBER_H
#define DIALOGDUMMYGRABBER_H

#include "common/addInGrabber.h"
//#include "common/sharedStructures.h"

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogDummyGrabber.h"

class dialogDummyGrabber : public QDialog
{
    Q_OBJECT

    public:
        dialogDummyGrabber(ito::AddInGrabber *grabber):m_Grabber(grabber){ m_paramsVals.clear(); ui.setupUi(this); };
        ~dialogDummyGrabber() {m_paramsVals.clear();};
        int getVals(QMap<QString, ito::Param> *paramVals);
        int sendVals(void);

    private:
        ito::AddInGrabber *m_Grabber;

        Ui::dialogDummyGrabber ui;
        QMap<QString, ito::Param> m_paramsVals;

    signals:

    public slots:

        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_btnResetROI_clicked();
        void on_applyButton_clicked();	//!< Write the current settings to the internal paramsVals and sent them to the grabber

        void on_spinBox_x0_valueChanged(int value);
        void on_spinBox_x1_valueChanged(int value);
        void on_spinBox_y0_valueChanged(int value);
        void on_spinBox_y1_valueChanged(int value);
        void on_spinBox_binX_valueChanged(int value);
        void on_spinBox_binY_valueChanged(int value);

};

#endif
