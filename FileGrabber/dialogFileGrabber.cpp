/* ********************************************************************
    Plugin "FileGrabber" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut für Technische Optik (ITO),
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

#include "dialogFileGrabber.h"

//----------------------------------------------------------------------------------------------------------------------------------
int dialogFileGrabber::getVals(QMap<QString, ito::Param> *paramVals)
{
    setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    // added by itobiege, Mar. 2013, but not tested!

    foreach(const ito::Param &param, *paramVals)
    {
        if ((m_paramsVals).contains(param.getName()))
        {
            m_paramsVals[param.getName()] = param;
        }
        else // Or create a new parameter
        {
            m_paramsVals.insert(param.getName(), param);
        }

        if (!strcmp(param.getName(), "x0"))
        {
            int ival = param.getVal<int>();

            ui.spinBox_x0->setValue(ival);
            ui.spinBox_x1->setMinimum(ival);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.spinBox_x0->setEnabled(true);
            }
            else
            {
                ui.spinBox_x0->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(), "x1"))
        {
            int ival = (int)param.getMax();
            ui.spinBox_x1->setMaximum(ival);

            ival = param.getVal<int>();
            ui.spinBox_x1->setValue(ival);
            ui.spinBox_x0->setMaximum(ival);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.pushButton_setSizeXMax->setEnabled(true);
                ui.spinBox_x1->setEnabled(true);
            }
            else
            {
                ui.spinBox_x1->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(), "y0"))
        {
            int ival = param.getVal<int>();

            ui.spinBox_y0->setValue(ival);
            ui.spinBox_y1->setMinimum(ival);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.spinBox_y0->setEnabled(true);
            }
            else
            {
                ui.spinBox_y0->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(), "y1"))
        {
            int ival = (int)param.getMax();
            ui.spinBox_y1->setMaximum(ival);

            ival = param.getVal<int>();
            ui.spinBox_y1->setValue(ival);
            ui.spinBox_y0->setMaximum(ival);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.pushButton_setSizeYMax->setEnabled(true);
                ui.spinBox_y1->setEnabled(true);
            }
            else
            {
                ui.spinBox_y1->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(), "sizex"))
        {
            int ival = param.getVal<int>();
            ui.spinBox_xsize->setValue(ival);
        }

        if (!strcmp(param.getName(), "sizey"))
        {
            int ival = param.getVal<int>();
            ui.spinBox_ysize->setValue(ival);
        }

        if (!strcmp(param.getName(),"gain"))
        {
            double dval = param.getVal<double>();
            ui.spinBox_gain->setValue((int)(dval*100.0+0.5));
            ui.horizontalSlider_gain->setValue((dval*100 + 0.5));

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.spinBox_gain->setEnabled(true);
                ui.horizontalSlider_gain->setEnabled(true);
            }
            else
            {
                ui.spinBox_gain->setEnabled(false);
                ui.horizontalSlider_gain->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(),"offset"))
        {
            double dval = param.getVal<double>();
            ui.spinBox_offset->setValue((int)(dval*100.0+0.5));
            ui.horizontalSlider_offset->setValue((dval*100 + 0.5));

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.spinBox_offset->setEnabled(true);
                ui.horizontalSlider_offset->setEnabled(true);
            }
            else
            {
                ui.spinBox_offset->setEnabled(false);
                ui.horizontalSlider_offset->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(),"frame_time"))
        {
            double dval = param.getMin()*1000;
            ui.doubleSpinBox_frame_time->setMinimum(dval);

            dval = param.getMax()*1000;
            ui.doubleSpinBox_frame_time->setMaximum(dval);

            dval = param.getVal<double>()*1000;
            ui.doubleSpinBox_frame_time->setValue(dval);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.doubleSpinBox_frame_time->setEnabled(true);
            }
            else
            {
                ui.doubleSpinBox_frame_time->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(),"integration_time"))
        {
            double dval = param.getMin()*1000;
            ui.doubleSpinBox_integration_time->setMinimum(dval);

            dval = param.getMax()*1000;
            ui.doubleSpinBox_integration_time->setMaximum(dval);

            dval = param.getVal<double>()*1000;
            ui.doubleSpinBox_integration_time->setValue(dval);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.doubleSpinBox_integration_time->setEnabled(true);
            }
            else
            {
                ui.doubleSpinBox_integration_time->setEnabled(false);
            }
        }

        if (!strcmp(param.getName(), "binning"))
        {
            int ival = param.getMin();
            int ivalX = (int)(ival/100);
            int ivalY = ival - ivalX * 100;

            ui.spinBox_binX->setMinimum(ivalX);
            ui.spinBox_binY->setMinimum(ivalY);

            ival = param.getMax();
            ivalX = (int)(ival/100);
            ivalY = ival - ivalX * 100;

            ui.spinBox_binX->setMaximum(ivalX);
            ui.spinBox_binY->setMaximum(ivalY);

            ival = param.getVal<int>();
            ivalX = (int)(ival/100);
            ivalY = ival - ivalX * 100;

            ui.spinBox_binX->setValue(ivalX);
            ui.spinBox_binY->setValue(ivalY);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.spinBox_binX->setEnabled(true);
                ui.spinBox_binY->setEnabled(true);
            }
            else
            {
                ui.spinBox_binX->setEnabled(false);
                ui.spinBox_binY->setEnabled(false);
            }
        }


        if (!strcmp(param.getName(), "bpp"))
        {
            int ival = param.getVal<int>();
            ui.spinBox_bpp->setValue(ival);

            if (!(param.getFlags() & ito::ParamBase::Readonly))
            {
                ui.combo_bpp->setEnabled(true);
            }
            else
            {
                ui.combo_bpp->setEnabled(false);
            }


            switch (ival)
            {
                case 8:
                    ui.combo_bpp->setCurrentIndex(0);
                break;
                case 10:
                    ui.combo_bpp->setCurrentIndex(1);
                break;
                case 12:
                    ui.combo_bpp->setCurrentIndex(2);
                break;
                case 14:
                    ui.combo_bpp->setCurrentIndex(3);
                break;
                case 16:
                    ui.combo_bpp->setCurrentIndex(4);
                break;
                case 24:
                    ui.combo_bpp->setCurrentIndex(5);
                break;
                case 30:
                    ui.combo_bpp->setCurrentIndex(6);
                break;
                case 32:
                    ui.combo_bpp->setCurrentIndex(7);
                break;
                default:
                    ui.combo_bpp->setEnabled(false);
                    m_paramsVals["bpp"].setFlags(ito::ParamBase::Readonly);
                break;
            }
        }
    }
//    ui.groupBoxIntegration->setEnabled(true);
//    ui.groupBoxBinning->setEnabled(true);
//    ui.groupBoxSize->setEnabled(true);

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int dialogFileGrabber::sendVals()
{
    bool binning_changed = false;

    if (m_paramsVals.size() < 1)
    {
        return 0;
    }

    QVector<QSharedPointer<ito::ParamBase> > outVector;


    if ((ui.spinBox_binX->isEnabled() || ui.spinBox_binY->isEnabled()))
    {
        int ival = ui.spinBox_binX->value() *100 + ui.spinBox_binY->value();
        if ((m_paramsVals["binning"].getVal<int>() !=  ival))
        {
            outVector.append(QSharedPointer<ito::ParamBase>( new ito::ParamBase("binning", ito::ParamBase::Int, ival) ));
            binning_changed = true;
        }
    }

    if (!binning_changed)
    {
        int ivalFirst, ivalLast;
        bool changeX0 = false;
        bool changeX1 = false;
        bool changeY0 = false;
        bool changeY1 = false;

        if (ui.spinBox_x0->isEnabled())
        {
            ivalFirst = ui.spinBox_x0->value();
            if (m_paramsVals["x0"].getVal<int>() !=  ivalFirst)
            {
                changeX0 = true;
            }
        }

        if (ui.spinBox_x1->isEnabled())
        {
            ivalLast = ui.spinBox_x1->value();
            if (m_paramsVals["x1"].getVal<int>() !=  ivalLast)
            {
                changeX1 = true;
            }
        }

        if (changeX0 && changeX1)
        {
            if (ivalFirst > m_paramsVals["x1"].getVal<int>())
            {
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, ivalLast)));
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, ivalFirst)));
            }
            else
            {
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, ivalFirst)));
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, ivalLast)));
            }
        }
        else if (changeX0)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, ivalFirst)));
        }
        else if (changeX1)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, ivalLast)));
        }

        if (ui.spinBox_y0->isEnabled())
        {
            ivalFirst = ui.spinBox_y0->value();
            if (m_paramsVals["y0"].getVal<int>() !=  ivalFirst)
            {
                changeY0 = true;
            }
        }

        if (ui.spinBox_y1->isEnabled())
        {
            ivalLast = ui.spinBox_y1->value();
            if (m_paramsVals["y1"].getVal<int>() !=  ivalLast)
            {
                changeY1 = true;
            }
        }

        if (changeY0 && changeY1)
        {
            if (ivalFirst > m_paramsVals["y1"].getVal<int>())
            {
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, ivalLast)));
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, ivalFirst)));
            }
            else
            {
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, ivalFirst)));
                outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, ivalLast)));
            }
        }
        else if (changeY0)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, ivalFirst)));
        }
        else if (changeY1)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, ivalLast)));
        }

    }

    if (ui.spinBox_gain->isEnabled())
    {
        double dval = ui.spinBox_gain->value()/100.0;
        if (m_paramsVals["gain"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.spinBox_offset->isEnabled())
    {
        double dval = ui.spinBox_offset->value()/100.0;
        if (m_paramsVals["offset"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.doubleSpinBox_integration_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_integration_time->value()/1000.0;
        if (m_paramsVals["integration_time"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.doubleSpinBox_frame_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_frame_time->value()/1000.0;
        if (m_paramsVals["frame_time"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_time", ito::ParamBase::Double, dval)));
        }
    }

    if (ui.combo_bpp->isEnabled())
    {
        QVariant qvar = ui.combo_bpp->currentIndex();
        int bppNew = -1;
        switch (qvar.toInt())
        {
            case 0:
                bppNew = 8;
            break;
            case 1:
                bppNew = 10;
            break;
            case 2:
                bppNew = 12;
            break;
            case 3:
                bppNew = 14;
            break;
            case 4:
                bppNew = 16;
            break;
            case 5:
                bppNew = 24;
            break;
            case 6:
                bppNew = 30;
            break;
            case 7:
                bppNew = 32;
            break;
        }
        if ((bppNew > 0) && (m_paramsVals["bpp"].getVal<int>() !=  bppNew))
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bppNew)));
        }
    }

    if (m_Grabber)   // Grabber exists
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(m_Grabber, "setParamVector", Q_ARG(const QVector<QSharedPointer<ito::ParamBase> >, outVector), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));
        while (!locker.getSemaphore()->wait(5000))
        {
            if (!m_Grabber->isAlive())
            {
                break;
            }
        }
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------------------------
void dialogFileGrabber::valuesChanged(QMap<QString, ito::Param> params)
{
    getVals(&params);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function resets the x-size of the ROI to the maximum value!
 *
 * \date    Oct.2011
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_pushButton_setSizeXMax_clicked()
{
    int inttemp = 0;

    inttemp = ui.spinBox_x0->minimum();
    ui.spinBox_x0->setValue(inttemp);

    inttemp = ui.spinBox_x1->maximum();
    ui.spinBox_x1->setValue(inttemp);

    ui.spinBox_xsize->update();

}

//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function resets the y-size of the ROI to the maximum value!
 *
 * \date    Oct.2011
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_pushButton_setSizeYMax_clicked()
{
    int inttemp = 0;

    inttemp = ui.spinBox_y1->maximum();
    ui.spinBox_y1->setValue(inttemp);

    inttemp = ui.spinBox_y0->minimum();
    ui.spinBox_y0->setValue(inttemp);

    ui.spinBox_ysize->update();
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the applyButton is clicked, the bpp and the binning of the attached camera is changed!
 *  Changes of parameters lead to a reload of all camera parameters. Other unapplied values are lost!
 *
 * \date    Oct.2011
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_applyButton_clicked()
{
    ui.groupBoxSize->setTitle("Size");
    ui.groupBoxSize->setEnabled(true);
    this->sendVals();
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the spinBox for x0 changes its value, the size in X and the minimal x1 changes also
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_spinBox_x0_valueChanged(int value)
{
    if (ui.spinBox_x1->value() < value)
    {
        ui.spinBox_x1->setValue(value);
    }
    ui.spinBox_x1->setMinimum(value);
    ui.spinBox_xsize->setValue(ui.spinBox_x1->value() - value + 1);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the spinBox for x1 changes its value, the size in X and the maximal x0 changes also
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_spinBox_x1_valueChanged(int value)
{
    if (ui.spinBox_x0->value() > value)
    {
        ui.spinBox_x0->setValue(value);
    }
    ui.spinBox_x0->setMaximum(value);
    ui.spinBox_xsize->setValue(value - ui.spinBox_x0->value() + 1);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the spinBox for y0 changes its value, the size in Y and the minimal y1 changes also
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_spinBox_y0_valueChanged(int value)
{
    if (ui.spinBox_y1->value() < value)
    {
        ui.spinBox_y1->setValue(value);
    }
    ui.spinBox_y1->setMinimum(value);
    ui.spinBox_ysize->setValue(ui.spinBox_y1->value() - value + 1);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the spinBox for y0 changes its value, the size in Y and the maximal y0 changes also
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_spinBox_y1_valueChanged(int value)
{
    if (ui.spinBox_y0->value() > value)
    {
        ui.spinBox_y0->setValue(value);
    }
    ui.spinBox_y0->setMaximum(value);
    ui.spinBox_ysize->setValue(value - ui.spinBox_y0->value() + 1);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the binning is activated, further settings of size will be disabled until apply is pressed
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_spinBox_binX_valueChanged(int /*value*/)
{
    ui.groupBoxSize->setTitle("Size (Binning changed, press apply or save)");
    ui.groupBoxSize->setEnabled(false);

    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the binning is activated, further settings of size will be disabled until apply is pressed
 *
 * \date    Jun.2012
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogFileGrabber::on_spinBox_binY_valueChanged(int /*value*/)
{
    ui.groupBoxSize->setTitle("Size (Binning changed, press apply or save)");
    ui.groupBoxSize->setEnabled(false);
    return;
}
//----------------------------------------------------------------------------------------------------------------------------------
