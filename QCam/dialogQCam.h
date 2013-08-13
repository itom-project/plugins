#ifndef DIALOGQCAM_H
#define DIALOGQCAM_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogqcam.h"
#include "common/sharedStructures.h"
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
        DialogQCam();	//!< Constructor
        ~DialogQCam() {};//! Destructor
        int setVals(QMap<QString, ito::Param> *paramVals);	//!< Setup called during creation
        int getVals(QMap<QString, ito::Param> *paramVals); //!< Writeback called before closing

    private:
        Ui::dialogQCam ui;	//! The QT-Design-GUI

	signals:

		void changeParameters(QMap<QString, ito::ParamBase> params);
		
	public slots:

		void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
		void on_pushButton_setSizeXMax_clicked();	//!< Set x-size to maximum valid value
		void on_pushButton_setSizeYMax_clicked();	//!< Set y-sizes to maximum valid value
};

#endif //DIALOGQCAM_H
