/* ********************************************************************
    Plugin "Vistek" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2013, Institut für Technische Optik (ITO),
    Universität Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.
  
    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dialogVistek.h"
#include "Vistek.h"
#include <qmessagebox.h>

//----------------------------------------------------------------------------------------------------------------------------------
DialogVistek::DialogVistek(Vistek *grabber, const VistekFeatures *features) : 
    m_Grabber(grabber), 
    m_currentBinning(-1),
    m_currentBpp(-1),
    m_currentOffset(-1),
    m_currentGain(-1.0),
    m_currentExposure(-1.0)
{ 
    m_features = new VistekFeatures(*features);
    ui.setupUi(this); 
};

//----------------------------------------------------------------------------------------------------------------------------------
DialogVistek::~DialogVistek()
{
    delete m_features;
    m_features = NULL;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogVistek::valuesChanged(QMap<QString, ito::Param> params)
{
    setWindowTitle(QString(params["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

    //file information

    if (params.contains("cameraModel"))
    {
        ui.lblModel->setText( params["cameraModel"].getVal<char*>() );
    }
        
    if (params.contains("cameraSerialNo"))
    {
        ui.lblSerialNo->setText( params["cameraSerialNo"].getVal<char*>() );
    }

    if (params.contains("cameraIP"))
    {
        ui.lblCameraIP->setText( params["cameraIP"].getVal<char*>() );
    }

    if (params.contains("cameraManufacturer"))
    {
        ui.lblManufacturer->setText( params["cameraManufacturer"].getVal<char*>() );
    }

    if (params.contains("sizex"))
    {
        ui.lblWidth->setText( QString("%1").arg( params["sizex"].getVal<int>()));
    }

    if (params.contains("sizey"))
    {
        ui.lblHeight->setText( QString("%1").arg( params["sizey"].getVal<int>()));
    }

    ui.combo_bpp->clear();
    ui.combo_bpp->setEnabled(false);

    if (m_features->has8bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("8bit",8);
    }
    if (m_features->has10bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("10bit",10);
    }
    if (m_features->has12bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("12bit",12);
    }
    if (m_features->has16bit)
    {
        ui.combo_bpp->setEnabled(true);
        ui.combo_bpp->addItem("16bit",16);
    }

    if (params.contains("bpp"))
    {
        m_currentBpp = params["bpp"].getVal<int>();
        for (int i = 0; i < ui.combo_bpp->count(); ++i)
        {
            if (ui.combo_bpp->itemData(i).toInt() == m_currentBpp)
            {
                ui.combo_bpp->setCurrentIndex(i);
                break;
            }
        }
    }

    ui.combo_binning->setEnabled( m_features->adjustBinning );
    if (params.contains("binning"))
    {
        m_currentBinning = params["binning"].getVal<int>();
        ui.combo_binning->setCurrentIndex( m_currentBinning );
    }

    ui.doubleSpinBox_integration_time->setEnabled( m_features->adjustExposureTime );
    if (params.contains("exposure"))
    {
        m_currentExposure = params["exposure"].getVal<double>()  * 1000.0; //ms
        ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["exposure"].getMeta());

        ui.doubleSpinBox_integration_time->setMinimum( dm->getMin() * 1000.0 );
        ui.doubleSpinBox_integration_time->setMaximum( dm->getMax() * 1000.0 );
        ui.doubleSpinBox_integration_time->setSingleStep( (dm->getMax() - dm->getMin()) * 10.0);
        ui.doubleSpinBox_integration_time->setValue( m_currentExposure );
    }

    ui.spinBox_gain->setEnabled( m_features->adjustGain );
    if (params.contains("gain"))
    {
        m_currentGain = params["gain"].getVal<double>();

        ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["gain"].getMeta());

        ui.spinBox_gain->setMinimum( dm->getMin() );
        ui.spinBox_gain->setMaximum( dm->getMax() );
        ui.spinBox_gain->setSingleStep( (dm->getMax() - dm->getMin())/100.0);
        ui.spinBox_gain->setValue( m_currentGain );
    }

    ui.spinBox_offset->setEnabled( m_features->adjustOffset );
    ui.horizontalSlider_offset->setEnabled(m_features->adjustOffset);
    if (params.contains("offset")) //already from 0.0 to 1.0 (in vistek driver this is 0..255)
    {
        m_currentOffset = (int)(params["offset"].getVal<double>() * 100);
        ui.spinBox_offset->setValue( m_currentOffset );
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
int DialogVistek::sendParameters(void)
{
    QVector<QSharedPointer<ito::ParamBase> > outVector;

    //binning
    if (m_features->adjustBinning && ui.combo_binning->currentIndex() != m_currentBinning && ui.combo_binning->currentIndex() >= 0)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("binning", ito::ParamBase::Int, ui.combo_binning->currentIndex())));
    }

    //bpp
    if (ui.combo_bpp->count() > 0 && ui.combo_bpp->itemData( ui.combo_bpp->currentIndex() ).toInt() != m_currentBpp)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, ui.combo_bpp->itemData( ui.combo_bpp->currentIndex() ).toInt())));
    }

    //offset
    if (m_features->adjustOffset && ui.spinBox_offset->value() != m_currentOffset)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, ui.spinBox_offset->value() / 100.0)));
    }

    //gain
    if (m_features->adjustGain && qAbs(ui.spinBox_gain->value() - m_currentGain) > 0.0001)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, ui.spinBox_gain->value())));
    }

    //exposure
    if (m_features->adjustExposureTime && qAbs(ui.doubleSpinBox_integration_time->value() - m_currentExposure) > 0.0001)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("exposure", ito::ParamBase::Double, ui.doubleSpinBox_integration_time->value() / 1000.0)));
    }

    if(m_Grabber)   // Grabber exists
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        bool alive = true;
        QMetaObject::invokeMethod(m_Grabber, "setParamVector", Q_ARG(const QVector<QSharedPointer<ito::ParamBase> >, outVector), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        while (!locker.getSemaphore()->wait(5000))
        {
            if (!m_Grabber->isAlive())
            {
                alive = false;
                break;
            }
        }

        if (!alive)
        {
            QMessageBox msgBox;
            msgBox.setText("Error while setting parameters");
            msgBox.setInformativeText("The plugin does not react any more.");
            msgBox.setIcon(QMessageBox::Critical);
            msgBox.exec();
        }
        else
        {
            ito::RetVal retval = locker.getSemaphore()->returnValue;

            if(retval.containsError())
            {
                QString msg = "<unknown error>";
                if (retval.hasErrorMessage()) msg = QLatin1String(retval.errorMessage());
                QMessageBox::critical(this,tr("error"),tr("Error while setting parameters (%1)").arg(msg));
            }
            else if(retval.containsWarning())
            {
                QString msg = "<unknown warning>";
                if (retval.hasErrorMessage()) msg = QLatin1String(retval.errorMessage());
                QMessageBox::warning(this,tr("warning"),tr("Warning while setting parameters (%1)").arg(msg));
            }
        }
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------


//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the applyButton is clicked, the bpp and the binning of the attached camera is changed!
 *  Changes of parameters lead to a reload of all camera parameters. Other unapplied values are lost!
*/
void DialogVistek::on_applyButton_clicked()
{
    sendParameters();
}

