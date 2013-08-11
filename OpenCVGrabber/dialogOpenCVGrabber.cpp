#include "dialogOpenCVGrabber.h"

dialogOpenCVGrabber::dialogOpenCVGrabber() 
{ 
	ui.setupUi(this); 
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function sets the values of the different GUI-Elements during startup of the window
 *
 * \sa OpenCVGrabber::showConfDialog(void)
 * \date	Oct.2011
 * \author	Wolfram Lyda
 * \warning	NA
*/
int dialogOpenCVGrabber::setVals(QMap<QString, ito::Param> *paramVals)
{
    QVariant qvar;
    int bpp = 8;
	int binning = 0;
	int gain = 0;

	int inttemp = 0;
	double dtemp = 0.0;

    setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    // added by itobiege, Mar. 2013, but not tested!

	QMap<QString, ito::Param>::const_iterator paramIt = (*paramVals).constFind("x0");	// To check if this parameter exist
	if (paramIt != ((*paramVals).constEnd()))
	{
        inttemp = ((*paramVals)["x0"]).getVal<int>();    
	    ui.spinBox_x0->setValue(inttemp);
	    inttemp = (int)((*paramVals)["x0"]).getMax(); 
	    ui.spinBox_x0->setMaximum(inttemp);
	    inttemp = (int)((*paramVals)["x0"]).getMin(); 
	    ui.spinBox_x0->setMinimum(inttemp);
    }
    else
    {
        ui.spinBox_x0->setEnabled(false);
    }

    paramIt = (*paramVals).constFind("sizex");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
	    inttemp = ((*paramVals)["sizex"]).getVal<int>();    
	    ui.spinBox_xsize->setValue(inttemp);
	    inttemp = (int)((*paramVals)["sizex"]).getMax(); 
	    ui.spinBox_xsize->setMaximum(inttemp);
	    inttemp = (int)((*paramVals)["sizex"]).getMin(); 
	    ui.spinBox_xsize->setMinimum(inttemp);
    }
    else
    {
        ui.spinBox_xsize->setEnabled(false);
    }


    paramIt = (*paramVals).constFind("y0");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
        inttemp = ((*paramVals)["y0"]).getVal<int>();    
	    ui.spinBox_y0->setValue(inttemp);
	    inttemp = (int)((*paramVals)["y0"]).getMax(); 
	    ui.spinBox_y0->setMaximum(inttemp);
	    inttemp = (int)((*paramVals)["y0"]).getMin(); 
	    ui.spinBox_y0->setMinimum(inttemp);
    }
    else
    {
        ui.spinBox_y0->setEnabled(false);
    }


    paramIt = (*paramVals).constFind("sizey");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
        inttemp = ((*paramVals)["sizey"]).getVal<int>();    
	    ui.spinBox_ysize->setValue(inttemp);
	    inttemp = (int)((*paramVals)["sizey"]).getMax(); 
	    ui.spinBox_ysize->setMaximum(inttemp);
	    inttemp = (int)((*paramVals)["sizey"]).getMin(); 
	    ui.spinBox_ysize->setMinimum(inttemp);
    }
    else
    {
        ui.spinBox_ysize->setEnabled(false);
    }

    paramIt = (*paramVals).constFind("offset");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
        dtemp = ((*paramVals)["offset"]).getVal<double>();
        ui.doubleSpinBox_offset->setValue(dtemp);   
    }
    else
    {
        ui.doubleSpinBox_offset->setEnabled(false);
    }

    paramIt = (*paramVals).constFind("shift_bits");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
        inttemp = ((*paramVals)["shift_bits"]).getVal<int>();
        ui.spinBox_edit_shift_bits->setValue(inttemp);
    }
    else
    {
        ui.spinBox_edit_shift_bits->setEnabled(false);
    }

    paramIt = (*paramVals).constFind("integration_time");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
        dtemp = ((*paramVals)["integration_time"]).getVal<double>()*1000;	// Is saved as [s] but displayed as [ms]
        ui.doubleSpinBox_integration_time->setValue(dtemp);   
    }
    else
    {
        ui.doubleSpinBox_integration_time->setEnabled(false);
    }

    paramIt = (*paramVals).constFind("bpp");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
	    bpp = ((*paramVals)["bpp"]).getVal<int>();				
        ui.spinBox_bpp->setValue(bpp);

	    switch (bpp)
        {
            case 8:
                ui.combo_bpp->setCurrentIndex(0);
			    ui.spinBox_edit_shift_bits->setEnabled(1);  

            break;
            case 12:
                ui.combo_bpp->setCurrentIndex(1);
			    ui.spinBox_edit_shift_bits->setEnabled(0);  
            break;
        }
    }
    else
    {
        ui.spinBox_bpp->setEnabled(false);
    }

    paramIt = (*paramVals).constFind("gain");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
	    gain = ((*paramVals)["gain"]).getVal<int>();
	    ui.checkBox_gain->setChecked(gain);
    }
    else
    {
        ui.checkBox_gain->setEnabled(false);
    }


    paramIt = (*paramVals).constFind("binning");	// To check if this parameter exists
	if (paramIt != ((*paramVals).constEnd()))
	{
	    binning = ((*paramVals)["binning"]).getVal<int>();
	    switch (binning)
        {
            case 0:
                ui.combo_binning->setCurrentIndex(0);
            break;
            case 1:
                ui.combo_binning->setCurrentIndex(1);
            break;
        }
    }
    else
    {
        ui.combo_binning->setEnabled(false);
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function writes back the value of the different GUI-Elements to the Grabber before the dialog is deleted. Only the grab depth and the binning will be seperated, because they need a realloc of memory.
 *
 * \sa RetVal OpenCVGrabber::showConfDialog(void)
 * \date	Oct.2011
 * \author	Wolfram Lyda
 * \warning	NA
*/
int dialogOpenCVGrabber::getVals(QMap<QString, ito::Param> *paramVals)
{
    QVariant qvar;
    int bpp = 8;
    int binning = 0;
	int inttemp = 0;
	double dtemp = 0.0;

    if(ui.spinBox_x0->isEnabled())
    {
	    inttemp = ui.spinBox_x0->value();
        ((*paramVals)["x0"]).setVal<int>(inttemp);
    }

    if(ui.spinBox_xsize->isEnabled())
    {
	    inttemp = ui.spinBox_xsize->value();
        ((*paramVals)["sizex"]).setVal<int>(inttemp);
    }

    if(ui.spinBox_y0->isEnabled())
    {
	    inttemp = ui.spinBox_y0->value();
        ((*paramVals)["y0"]).setVal<int>(inttemp);
    }

    if(ui.spinBox_ysize->isEnabled())
    {
        inttemp = ui.spinBox_ysize->value();
        ((*paramVals)["sizey"]).setVal<int>(inttemp);
    }

    if(ui.doubleSpinBox_offset->isEnabled())
    {
	    dtemp = ui.doubleSpinBox_offset->value();
        ((*paramVals)["offset"]).setVal<double>(dtemp);
    }

    if(ui.spinBox_edit_shift_bits->isEnabled())
    {
        inttemp = ui.spinBox_edit_shift_bits->value();
        ((*paramVals)["shift_bits"]).setVal<int>(inttemp);
    }

    if(ui.doubleSpinBox_integration_time->isEnabled())
    {
        dtemp = ui.doubleSpinBox_integration_time->value()/1000;
        ((*paramVals)["integration_time"]).setVal<double>(dtemp);
    }

    if(ui.combo_bpp->isEnabled())
    {
        qvar = ui.combo_bpp->currentIndex();
        switch (qvar.toInt())
        {
            case 0:
                ((*paramVals)["bpp"]).setVal<double>(8);
            break;
            case 1:
                ((*paramVals)["bpp"]).setVal<double>(12);
            break;
        }
    }

    if(ui.combo_binning->isEnabled())
    {
	    qvar = ui.combo_binning->currentIndex();
	    switch (qvar.toInt())
        {
            case 0:
                ((*paramVals)["binning"]).setVal<double>(0);
            break;
            case 1:
                ((*paramVals)["binning"]).setVal<double>(1);
            break;
        }
    }

    if(ui.checkBox_gain->isEnabled())
    {
	    inttemp = (int)ui.checkBox_gain->isChecked();
	    ((*paramVals)["gain"]).setVal<double>(inttemp);
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------
void dialogOpenCVGrabber::valuesChanged(QMap<QString, ito::Param> params)
{
	setVals(&params);
	return;
}
//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function resets the x-size of the ROI to the maximum value!
 *
 * \date	Oct.2011
 * \author	Wolfram Lyda
 * \warning	NA
*/
void dialogOpenCVGrabber::on_pushButton_setSizeXMax_clicked()
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
 * \date	Oct.2011
 * \author	Wolfram Lyda
 * \warning	NA
*/
void dialogOpenCVGrabber::on_pushButton_setSizeYMax_clicked()
{
	int inttemp = 0;

	inttemp = ui.spinBox_ysize->maximum();
	ui.spinBox_ysize->setValue(inttemp);
	
	inttemp = ui.spinBox_y0->minimum();
	ui.spinBox_y0->setValue(inttemp);
}

//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail If the applyButton is clicked, the bpp and the binning of the attached camera is changed!
 *  Changes of parameters lead to a reload of all camera parameters. Other unapplied values are lost!
 *
 * \date	Oct.2011
 * \author	Wolfram Lyda
 * \warning	NA
*/
void dialogOpenCVGrabber::on_applyButton_clicked()
{
	QVariant qvar;
	int bpp = 12;
	int binning = 0;
	ito::ParamBase param;
	QMap<QString, ito::ParamBase> paramsVals;

	qvar = ui.combo_bpp->currentIndex();
    switch (qvar.toInt())
    {
        case 0:
            bpp = 8;
        break;
        case 1:
            bpp = 12;
        break;
    }

	param = ito::ParamBase("bpp", ito::ParamBase::Int, bpp);
	paramsVals.insert(param.getName(), param);

	qvar = ui.combo_binning->currentIndex();
	switch (qvar.toInt())
    {
        case 0:
            binning = 0;
        break;
        case 1:
            binning = 1;
        break;
    }

	param = ito::ParamBase("binning", ito::ParamBase::Int, binning);
	paramsVals.insert(param.getName(), param);

	emit changeParameters(paramsVals);
}

//----------------------------------------------------------------------------------------------------------------------------------