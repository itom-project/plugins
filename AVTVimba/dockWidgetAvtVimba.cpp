/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dockWidgetAvtVimba.h"

#include <qmetaobject.h>


//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetAvtVimba::DockWidgetAvtVimba(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
	ui.sW_Gain->setTracking(false);
    ui.sW_intTime->setTracking(false);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        //first time call
        //get all given parameters and adjust all widgets according to them (min, max, stepSize, values...)
		int propCount = 0;

		if (params.contains("gain"))
        {
            ui.lB_Gain->setVisible(true);
            ui.sW_Gain->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["gain"].getMeta());
            ui.sW_Gain->setSingleStep(dm->getStepSize());
            ++propCount;
        }
        else
        {
            ui.lB_Gain->setVisible(false);
            ui.sW_Gain->setVisible(false);
        }

		if (params.contains("intTime"))
        {
            ui.lB_intTime->setVisible(true);
            ui.sW_intTime->setVisible(true);
            ito::DoubleMeta* dm = (ito::DoubleMeta*)(params["intTime"].getMeta());
            ui.sW_intTime->setSingleStep(dm->getStepSize());
			ui.sW_intTime->setMinimum(dm->getMin()+0.000001);
			ui.sW_intTime->setMaximum(dm->getMax()-0.000001);
            ++propCount;
        }
        else
        {
            ui.lB_intTime->setVisible(false);
            ui.sW_intTime->setVisible(false);
        }

        m_firstRun = false;
    }

    if (!m_inEditing)
    {
        m_inEditing = true;
        //check the value of all given parameters and adjust your widgets according to them (value only should be enough)
        if (params.contains("gain"))
        {
            ui.sW_Gain->setValue(params["gain"].getVal<double>());
            ui.sW_Gain->setEnabled(params["gainAuto"].getVal<int>() == 0);
        }
        if (params.contains("intTime"))
        {
            ui.sW_intTime->setValue(params["intTime"].getVal<double>());
            ui.sW_intTime->setEnabled(params["intTimeAuto"].getVal<int>() == 0);
        }



        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
double getStepValue(double value, double stepSize)
{
    int stepCount = (int)((value / stepSize) + .5);
    //qDebug() << "----------------- getStepValue stepCount: " << stepCount << "; alt: " << value / stepSize;  // getStepValue stepCount:  77.4818 ; alt:  77.4 
    return stepSize * stepCount;
}

//----------------------------------------------------------------------------------------------------------------------------------
 //void DockWidgetAvtVimba::on_contrast_valueChanged(int i)
 //{
 //    if (!m_inEditing)
 //    {
 //        m_inEditing = true;
 //        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrast",ito::ParamBase::Int,i));
 //        setPluginParameter(p, msgLevelWarningAndError);
 //        m_inEditing = false;
 //    }
 //}

//----------------------------------------------------------------------------------------------------------------------------------

void DockWidgetAvtVimba::on_sW_Gain_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        double d2 = getStepValue(d, ui.sW_Gain->singleStep());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,d2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------

void DockWidgetAvtVimba::on_sW_intTime_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        double d2 = getStepValue(d, ui.sW_intTime->singleStep());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("intTime",ito::ParamBase::Double,d2));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}


//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetAvtVimba::identifierChanged(const QString &identifier)
{
    ui.lblIdentifier->setText(identifier);
}