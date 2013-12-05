#include "dialogQCam.h"

DialogQCam::DialogQCam(ito::AddInDataIO *grabber) 
    :
    m_grabber(grabber),
	m_gainChanged(false),
	m_offsetChanged(false)
{ 
	ui.setupUi(this);

	connect(ui.doubleSpinBox_offset, SIGNAL(valueChanged(double)), this, SLOT(spinboxchanged(double)));
}

//----------------------------------------------------------------------------------------------------------------------------------
int DialogQCam::getVals()
{
    QVector<QSharedPointer<ito::ParamBase> > outVector;
    QSharedPointer<ito::ParamBase> param;

	if (m_offsetChanged)
	{
		param = QSharedPointer<ito::ParamBase>( new ito::ParamBase("offset", ito::ParamBase::Double, ui.doubleSpinBox_offset->value() ) );
		outVector.append( param );
	}

	if (m_gainChanged)
	{
		param = QSharedPointer<ito::ParamBase>( new ito::ParamBase("gain", ito::ParamBase::Double, ui.doubleSpinBox_gain->value() ) );
		outVector.append( param );
	}
    

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
	ui.doubleSpinBox_offset->setValue( params["offset"].getVal<double>() );
	ui.doubleSpinBox_gain->setValue( params["gain"].getVal<double>() );
	
	m_gainChanged = false;
	m_offsetChanged = false;
}


//----------------------------------------------------------------------------------------------------------------------------------
/**
 * \detail This function resets the x-size of the ROI to the maximum value!
*/
void DialogQCam::on_pushButton_setSizeXMax_clicked()
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
*/
void DialogQCam::on_pushButton_setSizeYMax_clicked()
{
	int inttemp = 0;

	inttemp = ui.spinBox_ysize->maximum();
	ui.spinBox_ysize->setValue(inttemp);
	
	inttemp = ui.spinBox_y0->minimum();
	ui.spinBox_y0->setValue(inttemp);
}


//----------------------------------------------------------------------------------------------------------------------------------