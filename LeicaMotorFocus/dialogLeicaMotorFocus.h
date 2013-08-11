#ifndef DIALOGLEICAMF_H
#define DIALOGLEICAMF_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogLeicaMotorFocus.h"
#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"

class DialogLeicaMotorFocus : public QDialog 
{
    Q_OBJECT

    private:
        Ui::dialogLeicaMotorFocus ui;
		QObject *m_pluginInstance;

        bool m_unappliedChanges;

        void enableDialog(bool enabled);

    public:
        DialogLeicaMotorFocus(QObject *pluginInstance);
		~DialogLeicaMotorFocus() {};

        ito::RetVal applyParameters();

    public slots:
        void parametersChanged(QMap<QString, ito::Param> params);
        

    private slots:
        void on_cmdHoming_clicked();
        void on_cmdOrigin_clicked();

        void on_spinBoxSpeed_valueChanged(double) { m_unappliedChanges = true; }
        void on_spinBoxRatio_valueChanged(int) { m_unappliedChanges = true; }
        void on_checkInvertAxis_stateChanged(int) { m_unappliedChanges = true; }
        void on_radioRefUpper_toggled(bool) { m_unappliedChanges = true; }
        void on_radioRefLower_toggled(bool) { m_unappliedChanges = true; }
		void on_cmdOk_clicked();



};

#endif
