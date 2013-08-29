#include "dialogQCam.h"

DialogQCam::DialogQCam(ito::AddInDataIO *grabber) 
    :
    m_grabber(grabber)
{ 
	ui.setupUi(this); 
}

//----------------------------------------------------------------------------------------------------------------------------------
int DialogQCam::getVals()
{
    QVector<QSharedPointer<ito::ParamBase> > outVector;
    bool binning_changed = false;

    /*if(m_paramsVals.size() < 1)
    {
        return 0;
    }

    QVector<QSharedPointer<ito::ParamBase> > outVector;


    if((ui.spinBox_binX->isEnabled() || ui.spinBox_binY->isEnabled()))
    {
        int ival = ui.spinBox_binX->value() *100 + ui.spinBox_binY->value();
        if((m_paramsVals["binning"].getVal<int>() !=  ival))
        {
            outVector.append(QSharedPointer<ito::ParamBase>( new ito::ParamBase("binning", ito::ParamBase::Int, ival) ));
            binning_changed = true;
        }
    }

    if(!binning_changed)
    {
        int ivalFirst, ivalLast;
        bool changeX0 = false;
        bool changeX1 = false;
        bool changeY0 = false;
        bool changeY1 = false;

        if(ui.spinBox_x0->isEnabled())
        {
            ivalFirst = ui.spinBox_x0->value();
            if(m_paramsVals["x0"].getVal<int>() !=  ivalFirst)
            {
                changeX0 = true;
            }
        }

        if(ui.spinBox_x1->isEnabled())
        {
            ivalLast = ui.spinBox_x1->value();
            if(m_paramsVals["x1"].getVal<int>() !=  ivalLast)
            {
                changeX1 = true;
            }
        }

        if(changeX0 && changeX1)
        {
            if(ivalFirst > m_paramsVals["x1"].getVal<int>())
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
        else if(changeX0)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x0", ito::ParamBase::Int, ivalFirst)));
        }
        else if(changeX1)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("x1", ito::ParamBase::Int, ivalLast)));
        }

        if(ui.spinBox_y0->isEnabled())
        {
            ivalFirst = ui.spinBox_y0->value();
            if(m_paramsVals["y0"].getVal<int>() !=  ivalFirst)
            {
                changeY0 = true;
            }
        }

        if(ui.spinBox_y1->isEnabled())
        {
            ivalLast = ui.spinBox_y1->value();
            if(m_paramsVals["y1"].getVal<int>() !=  ivalLast)
            {
                changeY1 = true;
            }
        }

        if(changeY0 && changeY1)
        {
            if(ivalFirst > m_paramsVals["y1"].getVal<int>())
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
        else if(changeY0)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y0", ito::ParamBase::Int, ivalFirst)));
        }
        else if(changeY1)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("y1", ito::ParamBase::Int, ivalLast)));
        }

    }

    if(ui.spinBox_gain->isEnabled())
    {
        double dval = ui.spinBox_gain->value()/100.0;
        if(m_paramsVals["gain"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.spinBox_offset->isEnabled())
    {
        double dval = ui.spinBox_offset->value()/100.0;
        if(m_paramsVals["offset"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.doubleSpinBox_integration_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_integration_time->value()/1000.0;
        if(m_paramsVals["integration_time"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.doubleSpinBox_frame_time->isEnabled())
    {
        double dval = ui.doubleSpinBox_frame_time->value()/1000.0;
        if(m_paramsVals["frame_time"].getVal<double>() !=  dval)
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_time", ito::ParamBase::Double, dval)));
        }
    }

    if(ui.combo_bpp->isEnabled())
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
        if((bppNew > 0) && (m_paramsVals["bpp"].getVal<int>() !=  bppNew))
        {
            outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("bpp", ito::ParamBase::Int, bppNew)));
        }
    }*/

    if(m_grabber)   // Grabber exists
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
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------------------
void DialogQCam::valuesChanged(QMap<QString, ito::Param> params)
{
	//QVariant qvar;

 //   double dgain = 0.0;
 //   int inttemp =0;
	//double dtemp = 0.0;
	//
 //   setWindowTitle(QString((*paramVals)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
 //   // added by itobiege, Mar. 2013, but not tested!

 //   inttemp = ((*paramVals)["x0"]).getVal<int>();    
	//ui.spinBox_x0->setValue(inttemp);
	//inttemp = (int)((*paramVals)["x0"]).getMax(); 
	//ui.spinBox_x0->setMaximum(inttemp);
	//inttemp = (int)((*paramVals)["x0"]).getMin(); 
	//ui.spinBox_x0->setMinimum(inttemp);
 //   
 //   inttemp = ((*paramVals)["sizex"]).getVal<int>();    
	//ui.spinBox_xsize->setValue(inttemp);
	//inttemp = (int)((*paramVals)["sizex"]).getMax(); 
	//ui.spinBox_xsize->setMaximum(inttemp);
	//inttemp = (int)((*paramVals)["sizex"]).getMin(); 
	//ui.spinBox_xsize->setMinimum(inttemp);

 //   inttemp = ((*paramVals)["y0"]).getVal<int>();    
	//ui.spinBox_y0->setValue(inttemp);
	//inttemp = (int)((*paramVals)["y0"]).getMax(); 
	//ui.spinBox_y0->setMaximum(inttemp);
	//inttemp = (int)((*paramVals)["y0"]).getMin(); 
	//ui.spinBox_y0->setMinimum(inttemp);

 //   inttemp = ((*paramVals)["sizey"]).getVal<int>();    
	//ui.spinBox_ysize->setValue(inttemp);
	//inttemp = (int)((*paramVals)["sizey"]).getMax(); 
	//ui.spinBox_ysize->setMaximum(inttemp);
	//inttemp = (int)((*paramVals)["sizey"]).getMin(); 
	//ui.spinBox_ysize->setMinimum(inttemp);

 //   dtemp = ((*paramVals)["offset"]).getVal<double>();
 //   ui.doubleSpinBox_offset->setValue(dtemp);   

	//dgain = ((*paramVals)["gain"]).getVal<double>();
	//ui.doubleSpinBox_gain->setValue(dgain);
}


////----------------------------------------------------------------------------------------------------------------------------------
///**
// * \detail This function resets the x-size of the ROI to the maximum value!
//*/
//void DialogQCam::on_pushButton_setSizeXMax_clicked()
//{
//	int inttemp = 0;
//
//	inttemp = ui.spinBox_x0->minimum();
//	ui.spinBox_x0->setValue(inttemp);
//	
//	inttemp = ui.spinBox_xsize->maximum();
//	ui.spinBox_xsize->setValue(inttemp);
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
///**
// * \detail This function resets the y-size of the ROI to the maximum value!
//*/
//void DialogQCam::on_pushButton_setSizeYMax_clicked()
//{
//	int inttemp = 0;
//
//	inttemp = ui.spinBox_ysize->maximum();
//	ui.spinBox_ysize->setValue(inttemp);
//	
//	inttemp = ui.spinBox_y0->minimum();
//	ui.spinBox_y0->setValue(inttemp);
//}


//----------------------------------------------------------------------------------------------------------------------------------