#ifndef DIALOGQCAM_H
#define DIALOGQCAM_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogqcam.h"
#include "common/sharedStructures.h"
#include "common/addInInterface.h"
 /**
  *\class	DialogQCam 
  *\brief	configuration dialog for QCam cameras
  *
  *         This dialog can be used to setup the most important parameters for the camera.
  *			During setup, itom is blocked
  *
  *
  */
class DialogQCam : public QDialog 
{
    Q_OBJECT

    public:
        DialogQCam(ito::AddInDataIO *grabber);	//!< Constructor
        ~DialogQCam() {};//! Destructor

        int getVals(); //!< Writeback called before closing

    private:
        Ui::dialogQCam ui;	//! The QT-Design-GUI
        ito::AddInDataIO *m_grabber;

		bool m_gainChanged;
		bool m_offsetChanged;
		
	public slots:
		void valuesChanged(QMap<QString, ito::Param> params);


    private slots:
		void on_pushButton_setSizeXMax_clicked();	//!< Set x-size to maximum valid value
		void on_pushButton_setSizeYMax_clicked();	//!< Set y-sizes to maximum valid value

		void on_doubleSpinBox_offset_valueChanged(double /*val*/) { m_offsetChanged = true; }
		void on_doubleSpinBox_gain_valueChanged(double /*val*/) { m_gainChanged = true; }
};

#endif //DIALOGQCAM_H
