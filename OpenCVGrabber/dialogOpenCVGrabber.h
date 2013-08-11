#ifndef DIALOGOPENCVGRABBER_H
#define DIALOGOPENCVGRABBER_H

#include "common/sharedStructures.h"

#include "ui_dialogOpenCVGrabber.h"

#include <QtGui>
#include <qdialog.h>

 /**
  *\class	dialogOpenCVGrabber 
  *\brief	Configurationdiablog for the PCO Pixelfly QE CCD
  *
  *         This dialog can be used to setup the most important parameters for the camera.
  *			During setup, the ITOm is blocked
  *
  *	\sa	OpenCVGrabber
  *	\date	11.10.2010
  *	\author	Wolfram Lyda
  * \warning	NA
  *
  */
class dialogOpenCVGrabber : public QDialog 
{
    Q_OBJECT

    public:
        dialogOpenCVGrabber();	//!< Constructor
        ~dialogOpenCVGrabber() {};//! Destructor
        int setVals(QMap<QString, ito::Param> *paramVals);	//!< Setup called during creation
        int getVals(QMap<QString, ito::Param> *paramVals); //!< Writeback called before closing

    private:
        Ui::dialogOpenCVGrabber ui;	//! The QT-Design-GUI

	signals:

		void changeParameters(QMap<QString, ito::ParamBase> params);
		
	public slots:

		void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
		void on_pushButton_setSizeXMax_clicked();	//!< Set x-size to maximum valid value
		void on_pushButton_setSizeYMax_clicked();	//!< Set y-sizes to maximum valid value
		void on_applyButton_clicked();	//!< Set binning and grab depth
};

#endif