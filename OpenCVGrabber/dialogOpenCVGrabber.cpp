/* ********************************************************************
    Plugin "OpenCV-Grabber" for itom software
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

#include "dialogOpenCVGrabber.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogOpenCVGrabber::DialogOpenCVGrabber(ito::AddInGrabber *grabber, bool colorCam, int camWidth, int camHeight)
    : m_grabber(grabber), m_colorCam(colorCam)
{
    ui.setupUi(this);

    m_camSize = QRect(0,0,camWidth,camHeight);

    ui.comboColorMode->clear();
    if (colorCam)
    {
        ui.comboColorMode->addItem("auto");
        ui.comboColorMode->addItem("color");
        ui.comboColorMode->addItem("red");
        ui.comboColorMode->addItem("green");
        ui.comboColorMode->addItem("blue");
        ui.comboColorMode->addItem("gray");
    }
    else
    {
        ui.comboColorMode->addItem("auto");
        ui.comboColorMode->addItem("gray");
    }

    ui.groupSettings->setVisible(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::valuesChanged(QMap<QString, ito::Param> params)
{
    int x0 = params["x0"].getVal<int>();
    int x1 = params["x1"].getVal<int>();
    int y0 = params["y0"].getVal<int>();
    int y1 = params["y1"].getVal<int>();
    ui.spinX0->setMaximum(m_camSize.width() - 2);
    ui.spinX0->setValue(x0);
    ui.spinY0->setMaximum(m_camSize.height() - 2);
    ui.spinY0->setValue(y0);
    ui.spinX1->setMaximum(m_camSize.width()-1);
    ui.spinX1->setMinimum(x0+1);
    ui.spinX1->setValue(x1);
    ui.spinY1->setMaximum(m_camSize.height()-1);
    ui.spinY1->setMinimum(y0+1);
    ui.spinY1->setValue(y1);
    ui.spinSizeX->setMaximum(m_camSize.width()-x0);
    ui.spinSizeX->setValue(x1-x0+1);
    ui.spinSizeY->setMaximum(m_camSize.height()-y0);
    ui.spinSizeY->setValue(y1-y0+1);

    sizeXChanged = false;
    sizeYChanged = false;


    ui.spinBpp->setValue( params["bpp"].getVal<int>() );

    if (params.contains("colorMode"))
    {
        QString mode = params["colorMode"].getVal<char*>();
        mode = mode.toLower();
        int idx;
        if (m_colorCam)
        {
            if (mode == "auto")
            {
                idx = 0;
            }
            else if (mode == "color")
            {
                idx = 1;
            }
            else if (mode == "red")
            {
                idx = 2;
            }
            else if (mode == "green")
            {
                idx = 3;
            }
            else if (mode == "blue")
            {
                idx = 4;
            }
            else
            {
                idx = 5;
            }
        }
        else
        {
            if (mode == "auto")
            {
                idx = 0;
            }
            else if (mode == "gray")
            {
                idx = 1;
            }
            else
            {
                idx = 0;
            }
        }

        ui.comboColorMode->setCurrentIndex(idx);

        colorModeChanged = false;
    }

}


//----------------------------------------------------------------------------------------------------------------------------------
int DialogOpenCVGrabber::sendVals()
{
    QVector<QSharedPointer<ito::ParamBase> > outVector;

    if (sizeXChanged)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, ui.spinX0->value())));
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, ui.spinX1->value())));
    }
    
    if (sizeYChanged)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, ui.spinY0->value())));
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, ui.spinY1->value())));
    }

    if (colorModeChanged)
    {
        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("colorMode", ito::ParamBase::String, ui.comboColorMode->currentText().toLatin1().data())));
    }


    if (m_grabber)   // Grabber exists
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_grabber, "setParamVector", Q_ARG(const QVector<QSharedPointer<ito::ParamBase> >, outVector), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        while (!locker.getSemaphore()->wait(5000))
        {
            if (!m_grabber->isAlive())
            {
                break;
            }
        }

        sizeXChanged = false;
        sizeYChanged = false;
        colorModeChanged = false;
    }
    return 0;
}



//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_applyButton_clicked()
{
    sendVals();
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_spinX0_valueChanged(int value)
{
    ui.spinX1->setMinimum(value+1);
    ui.spinSizeX->setMaximum( 1 + ui.spinX1->maximum() - value );
    ui.spinSizeX->setValue( 1 + ui.spinX1->value() - value );
    sizeXChanged = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_spinX1_valueChanged(int value)
{
    ui.spinX0->setMaximum( value - 1 );
    ui.spinSizeX->setMaximum( 1 + ui.spinX1->maximum() - ui.spinX0->value() );
    ui.spinSizeX->setValue( 1 + value - ui.spinX0->value() );
    sizeXChanged = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_spinSizeX_valueChanged(int value)
{
    ui.spinX1->setValue( ui.spinX0->value() + value - 1 );
    on_spinX1_valueChanged( ui.spinX1->value() );
    sizeXChanged = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_spinY0_valueChanged(int value)
{
    ui.spinY1->setMinimum(value+1);
    ui.spinSizeY->setMaximum( 1 + ui.spinY1->maximum() - value );
    ui.spinSizeY->setValue( 1 + ui.spinY1->value() - value );
    sizeYChanged = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_spinY1_valueChanged(int value)
{
    ui.spinY0->setMaximum( value - 1 );
    ui.spinSizeY->setMaximum( 1 + ui.spinY1->maximum() - ui.spinY0->value() );
    ui.spinSizeY->setValue( 1 + value - ui.spinY0->value() );
    sizeYChanged = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_spinSizeY_valueChanged(int value)
{
    ui.spinY1->setValue( ui.spinY0->value() + value - 1 );
    on_spinY1_valueChanged( ui.spinY1->value() );
    sizeYChanged = true;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogOpenCVGrabber::on_btnSetFullROI_clicked()
{
    ui.spinX0->setMinimum(0);
    ui.spinX0->setMaximum(m_camSize.width() - 2);
    ui.spinX0->setValue(0);

    ui.spinY0->setMinimum(0);
    ui.spinY0->setMaximum(m_camSize.height() - 2);
    ui.spinY0->setValue(0);

    ui.spinX1->setMinimum(1);
    ui.spinX1->setMaximum(m_camSize.width() - 1);
    ui.spinX1->setValue(ui.spinX1->maximum());

    ui.spinY1->setMinimum(1);
    ui.spinY1->setMaximum(m_camSize.height() - 1);
    ui.spinY1->setValue(ui.spinY1->maximum());


    ui.spinSizeX->setMaximum(ui.spinX1->maximum());
    ui.spinSizeY->setMaximum(ui.spinY1->maximum());

    ui.spinSizeX->setValue(ui.spinX1->maximum());
    ui.spinSizeY->setValue(ui.spinY1->maximum());

    sizeXChanged = true;
    sizeYChanged = true;
}
