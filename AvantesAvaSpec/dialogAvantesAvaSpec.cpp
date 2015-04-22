//#include "dialogAS5216.h"
//
////----------------------------------------------------------------------------------------------------------------------------------
//dialogAS5216::dialogAS5216() 
//{ 
//    ui.setupUi(this); 
//}
////----------------------------------------------------------------------------------------------------------------------------------
///**
// * \detail This function sets the values of the different GUI-Elements during startup of the window
// *
// * \sa AS5216::showConfDialog(void)
// * \date    Oct.2011
// * \author    Wolfram Lyda
// * \warning    NA
//*/
//int dialogAS5216::setVals(QMap<QString, ito::Param> *paramVals)
//{
//    QVariant qvar;
//
//    double dgain = 0.0;
//    int inttemp =0;
//    double dtemp = 0.0;
//    
//    setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
//    // added by itobiege, Mar. 2013, but not tested!
//
//    inttemp = ((*paramVals)["x0"]).getVal<int>();    
//    ui.spinBox_x0->setValue(inttemp);
//    inttemp = (int)((*paramVals)["x0"]).getMax(); 
//    ui.spinBox_x0->setMaximum(inttemp);
//    inttemp = (int)((*paramVals)["x0"]).getMin(); 
//    ui.spinBox_x0->setMinimum(inttemp);
//    
//    inttemp = ((*paramVals)["sizex"]).getVal<int>();    
//    ui.spinBox_xsize->setValue(inttemp);
//    inttemp = (int)((*paramVals)["sizex"]).getMax(); 
//    ui.spinBox_xsize->setMaximum(inttemp);
//    inttemp = (int)((*paramVals)["sizex"]).getMin(); 
//    ui.spinBox_xsize->setMinimum(inttemp);
//
///*    inttemp = ((*paramVals)["y0"]).getVal<int>();    
//    ui.spinBox_y0->setValue(inttemp);
//    inttemp = (int)((*paramVals)["y0"]).getMax(); 
//    ui.spinBox_y0->setMaximum(inttemp);
//    inttemp = (int)((*paramVals)["y0"]).getMin(); 
//    ui.spinBox_y0->setMinimum(inttemp);
//
//    inttemp = ((*paramVals)["sizey"]).getVal<int>();    
//    ui.spinBox_ysize->setValue(inttemp);
//    inttemp = (int)((*paramVals)["sizey"]).getMax(); 
//    ui.spinBox_ysize->setMaximum(inttemp);
//    inttemp = (int)((*paramVals)["sizey"]).getMin(); 
//    ui.spinBox_ysize->setMinimum(inttemp);
//*/
//    dtemp = ((*paramVals)["offset"]).getVal<double>();
//    ui.doubleSpinBox_offset->setValue(dtemp);   
//
//    dgain = ((*paramVals)["gain"]).getVal<double>();
//    ui.doubleSpinBox_gain->setValue(dgain); 
//
//    return 0;
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
///**
// * \detail This function writes back the value of the different GUI-Elements to the 1394 before the dialog is deleted. Only the grab depth and the binning will be seperated, because they need a realloc of memory.
// *
// * \sa RetVal AS5216::showConfDialog(void)
// * \date    Oct.2011
// * \author    Wolfram Lyda
// * \warning    NA
//*/
//int dialogAS5216::getVals(QMap<QString, ito::Param> *paramVals)
//{
//    QVariant qvar;
//
//    int inttemp = 0;
//    double dtemp = 0.0;
//
//    inttemp = ui.spinBox_x0->value();
//    ((*paramVals)["x0"]).setVal<int>(inttemp);
//    
//    inttemp = ui.spinBox_xsize->value();
//    ((*paramVals)["sizex"]).setVal<int>(inttemp);
//
///*    inttemp = ui.spinBox_y0->value();
//    ((*paramVals)["y0"]).setVal<int>(inttemp);
//
//    inttemp = ui.spinBox_ysize->value();
//    ((*paramVals)["sizey"]).setVal<int>(inttemp);
//*/   
//    dtemp = ui.doubleSpinBox_offset->value();
//    ((*paramVals)["offset"]).setVal<double>(dtemp);
//
//    dtemp = ui.doubleSpinBox_gain->value();
//    ((*paramVals)["gain"]).setVal<double>(dtemp);
//
//    return 0;
//}
////----------------------------------------------------------------------------------------------------------------------------------
//void dialogAS5216::valuesChanged(QMap<QString, ito::Param> params)
//{
//    setVals(&params);
//    return;
//}
////----------------------------------------------------------------------------------------------------------------------------------
///**
// * \detail This function resets the x-size of the ROI to the maximum value!
// *
// * \date    Oct.2011
// * \author    Wolfram Lyda
// * \warning    NA
//*/
//void dialogAS5216::on_pushButton_setSizeXMax_clicked()
//{
//    int inttemp = 0;
//
//    inttemp = ui.spinBox_x0->minimum();
//    ui.spinBox_x0->setValue(inttemp);
//    
//    inttemp = ui.spinBox_xsize->maximum();
//    ui.spinBox_xsize->setValue(inttemp);
//}
//
//
////----------------------------------------------------------------------------------------------------------------------------------