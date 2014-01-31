#ifndef DIALOGVISTEK_H
#define DIALOGVISTEK_H

#include "common/addInGrabber.h"

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogVistek.h"

class Vistek;
class VistekFeatures;

class DialogVistek : public QDialog
{
    Q_OBJECT

    public:
        DialogVistek(Vistek *grabber, const VistekFeatures *features);
        ~DialogVistek();

        int sendParameters(void);

    private:
        Vistek *m_Grabber;

        Ui::dialogVistek ui;
        const VistekFeatures *m_features;

        int m_currentBinning;
        int m_currentBpp;
        int m_currentOffset;
        double m_currentGain;
        double m_currentExposure;

    signals:

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_applyButton_clicked();    //!< Write the current settings to the internal paramsVals and sent them to the grabber

};

#endif

