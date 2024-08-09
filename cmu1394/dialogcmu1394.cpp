/* ********************************************************************
    Plugin "cmu1394" for itom software
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

#include "dialogCMU1394.h"
dialogCMU1394::dialogCMU1394()
{
    ui.setupUi(this);
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function sets the values of the different GUI-Elements during startup of the window
 *
 * \sa CMU1394::showConfDialog(void)
 * \date    Oct.2011
 * \author    Wolfram Lyda
 * \warning    NA
*/
int dialogCMU1394::setVals(QMap<QString, ito::Param> *paramVals)
{
    return 0;
    QVariant qvar;

    double dgain = 0.0;
    int inttemp =0;
    double dtemp = 0.0;

    if(paramVals->keys().contains("name"))
    {
        setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
        // added by itobiege, Mar. 2013, but not tested!
    }

    if(paramVals->keys().contains("x0"))
    {
        inttemp = ((*paramVals)["x0"]).getVal<int>();
        ui.spinBox_x0->setValue(inttemp);
        inttemp = (int)((*paramVals)["x0"]).getMax();
        ui.spinBox_x0->setMaximum(inttemp);
        inttemp = (int)((*paramVals)["x0"]).getMin();
        ui.spinBox_x0->setMinimum(inttemp);
    }

    if(paramVals->keys().contains("sizex"))
    {
        inttemp = ((*paramVals)["sizex"]).getVal<int>();
        ui.spinBox_xsize->setValue(inttemp);
        inttemp = (int)((*paramVals)["sizex"]).getMax();
        ui.spinBox_xsize->setMaximum(inttemp);
        inttemp = (int)((*paramVals)["sizex"]).getMin();
        ui.spinBox_xsize->setMinimum(inttemp);
    }

    if(paramVals->keys().contains("y0"))
    {
        inttemp = ((*paramVals)["y0"]).getVal<int>();
        ui.spinBox_y0->setValue(inttemp);
        inttemp = (int)((*paramVals)["y0"]).getMax();
        ui.spinBox_y0->setMaximum(inttemp);
        inttemp = (int)((*paramVals)["y0"]).getMin();
        ui.spinBox_y0->setMinimum(inttemp);
    }

    if(paramVals->keys().contains("sizey"))
    {
        inttemp = ((*paramVals)["sizey"]).getVal<int>();
        ui.spinBox_ysize->setValue(inttemp);
        inttemp = (int)((*paramVals)["sizey"]).getMax();
        ui.spinBox_ysize->setMaximum(inttemp);
        inttemp = (int)((*paramVals)["sizey"]).getMin();
        ui.spinBox_ysize->setMinimum(inttemp);
    }

    if(paramVals->keys().contains("offset"))
    {
        dtemp = ((*paramVals)["offset"]).getVal<double>();
        ui.doubleSpinBox_offset->setValue(dtemp);
    }

    if(paramVals->keys().contains("gain"))
    {
        dgain = ((*paramVals)["gain"]).getVal<double>();
        ui.doubleSpinBox_gain->setValue(dgain);
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function writes back the value of the different GUI-Elements to the 1394 before the dialog is deleted. Only the grab depth and the binning will be separated, because they need a realloc of memory.
 *
 * \sa RetVal CMU1394::showConfDialog(void)
 * \date    Oct.2011
 * \author    Wolfram Lyda
 * \warning    NA
*/
int dialogCMU1394::getVals(QMap<QString, ito::Param> *paramVals)
{
    QVariant qvar;

    int inttemp = 0;
    double dtemp = 0.0;

    if(paramVals->keys().contains("x0"))
    {
        inttemp = ui.spinBox_x0->value();
        ((*paramVals)["x0"]).setVal<int>(inttemp);
    }

    if(paramVals->keys().contains("sizex"))
    {
        inttemp = ui.spinBox_xsize->value();
        ((*paramVals)["sizex"]).setVal<int>(inttemp);
    }

    if(paramVals->keys().contains("y0"))
    {
        inttemp = ui.spinBox_y0->value();
        ((*paramVals)["y0"]).setVal<int>(inttemp);
    }
    if(paramVals->keys().contains("sizey"))
    {
        inttemp = ui.spinBox_ysize->value();
        ((*paramVals)["sizey"]).setVal<int>(inttemp);
    }

    if(paramVals->keys().contains("offset"))
    {
        dtemp = ui.doubleSpinBox_offset->value();
        ((*paramVals)["offset"]).setVal<double>(dtemp);
    }
    if(paramVals->keys().contains("gain"))
    {
        dtemp = ui.doubleSpinBox_gain->value();
        ((*paramVals)["gain"]).setVal<double>(dtemp);
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------
void dialogCMU1394::valuesChanged(QMap<QString, ito::Param> params)
{
    setVals(&params);
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
void dialogCMU1394::on_pushButton_setSizeXMax_clicked()
{
    int inttemp = 0;

    inttemp = ui.spinBox_x0->minimum();
    ui.spinBox_x0->setValue(inttemp);

    inttemp = ui.spinBox_xsize->maximum();
    ui.spinBox_xsize->setValue(inttemp);
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function resets the y-size of the ROI to the maximum value!
 *
 * \date    Oct.2011
 * \author    Wolfram Lyda
 * \warning    NA
*/
void dialogCMU1394::on_pushButton_setSizeYMax_clicked()
{
    int inttemp = 0;

    inttemp = ui.spinBox_ysize->maximum();
    ui.spinBox_ysize->setValue(inttemp);

    inttemp = ui.spinBox_y0->minimum();
    ui.spinBox_y0->setValue(inttemp);
}


//----------------------------------------------------------------------------------------------------------------------------------
