#include "dockWidgetQCam.h"

 DockWidgetQCam::DockWidgetQCam()
 {
	ui.setupUi(this); 
 }

 void DockWidgetQCam::valuesChanged(QMap<QString, ito::Param> params)
 {
    if (params.contains("offset"))
	{
		ui.doubleSpinBoxOffset->setValue( params["offset"].getVal<double>() );

		//set the value of the slider, too
	}

	if (params.contains("gain"))
	{
		ui.doubleSpinBoxGain->setValue( params["gain"].getVal<double>() );

		//set the value of the slider, too
	}
 }

void DockWidgetQCam::on_doubleSpinBoxGain_editingFinished()
{
    double gain = ui.doubleSpinBoxGain->value();

	ui.horizontalSliderGain->setValue((int)(ui.doubleSpinBoxGain->value()*100+0.5));
    
    QMap<QString, ito::ParamBase> paramsVals;
    ito::ParamBase param("gain", ito::ParamBase::Double, gain);
    paramsVals.insert(param.getName(), param);

    emit changeParameters(paramsVals);
}

void DockWidgetQCam::on_doubleSpinBoxOffset_editingFinished()
{
//    double offset = ui.doubleSpinBoxOffset->value();
//
////    ui.horizontalSliderOffset->setValue((int)(ui.doubleSpinBoxOffset->value()*100+0.5));
//
//    QMap<QString, ito::ParamBase> paramsVals;
//    ito::ParamBase param("offset", ito::ParamBase::Double, offset);
//    paramsVals.insert(param.getName(), param);
//
//    /*if (!updating)
//    {
//        updating = 1;
//        ui.horizontalSliderOffset->setValue(offset * 100.0);
//        emit changeParameters(paramsVals);
//        updating = 0;
//    }*/
}

void DockWidgetQCam::on_horizontalSliderGain_valueChanged(int Value)
{
    //double gain = ui.horizontalSliderGain->value()/100.0;
    //QMap<QString, ito::ParamBase> paramsVals;
    //ito::ParamBase param("gain", ito::ParamBase::Double, gain);
    //paramsVals.insert(param.getName(), param);

    ///*if (!updating)
    //{
    //    updating = 1;
    //    ui.doubleSpinBoxGain->setValue(gain);
	   // emit changeParameters(paramsVals);
    //    updating = 0;
    //}*/
}

void DockWidgetQCam::on_horizontalSliderOffset_valueChanged(int Value)
{
    //double offset = ui.horizontalSliderOffset->value()/100.0;
    //QMap<QString, ito::ParamBase> paramsVals;
    //ito::ParamBase param("offset", ito::ParamBase::Double, offset);
    //ui.doubleSpinBoxOffset->setValue(offset);

    ///*if (!updating)
    //{
    //    updating = 1;
    //    paramsVals.insert(param.getName(), param);
    //    emit changeParameters(paramsVals);
    //    updating = 0;
    //}*/
}