#ifndef DIALOGCMU1394_H
#define DIALOGCMU1394_H

#include <QtGui>
#include <qdialog.h>

#include "ui_dialogcmu1394.h"
#include "common/sharedStructures.h"
 /**
  *\class    dialogCMU1394 
  *\brief    Configurationdiablog for the CMU-driver
  *
  *         This dialog can be used to setup the most important parameters for the camera.
  *            During setup, the ITOm is blocked
  *
  *    \sa    CMU1394
  *    \date    11.10.2010
  *    \author    Wolfram Lyda
  * \warning    NA
  *
  */
class dialogCMU1394 : public QDialog 
{
    Q_OBJECT

    public:
        dialogCMU1394();    //!< Constructor
        ~dialogCMU1394() {};//! Destructor
        int setVals(QMap<QString, ito::Param> *paramVals);    //!< Setup called during creation
        int getVals(QMap<QString, ito::Param> *paramVals); //!< Writeback called before closing

    private:
        Ui::dialogCMU1394 ui;    //! The QT-Design-GUI

    signals:

        void changeParameters(QMap<QString, ito::ParamBase> params);
        
    public slots:

        void valuesChanged(QMap<QString, ito::Param> params);

    private slots:
        void on_pushButton_setSizeXMax_clicked();    //!< Set x-size to maximum valid value
        void on_pushButton_setSizeYMax_clicked();    //!< Set y-sizes to maximum valid value
};

#endif //DIALOGCMU1394_H
