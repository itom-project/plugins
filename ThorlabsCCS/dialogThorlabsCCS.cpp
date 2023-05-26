/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dialogThorlabsCCS.h"

#include "common/addInInterface.h"

#include <qdialogbuttonbox.h>
#include <qvector.h>
#include <qsharedpointer.h>
#include "DataObject/dataobj.h"

//----------------------------------------------------------------------------------------------------------------------------------
DialogThorlabsCCS::DialogThorlabsCCS(ito::AddInBase *grabber) :
    AbstractAddInConfigDialog(grabber),
    m_firstRun(true)
{
    ui.setupUi(this);

    //disable dialog, since no parameters are known yet. Parameters will immediately be sent by the slot parametersChanged.
    enableDialog(false);
};


//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsCCS::parametersChanged(QMap<QString, ito::Param> params)
{
    //save the currently set parameters to m_currentParameters
    m_currentParameters = params;

    if (m_firstRun)
    {
        setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

        ito::RectMeta *rm = static_cast<ito::RectMeta*>(params["roi"].getMeta());
        ui.rangeX01->setLimitsFromIntervalMeta(rm->getWidthRangeMeta());

        //this is the first time that parameters are sent to this dialog,
        //therefore you can add some initialization work here
        m_firstRun = false;

        //now activate group boxes, since information is available now (at startup, information is not available, since parameters are sent by a signal)
        enableDialog(true);
    }

    int *roi = params["roi"].getVal<int*>();
    qDebug() << roi[0] << roi[1] << roi[2] << roi[3];
    ui.rangeX01->setValues(roi[0], roi[0] + roi[2] - 1);
    ui.rangeX01->setEnabled(! (params["roi"].getFlags() & ito::ParamBase::Readonly));

    ui.spinSizeX->setValue(params["sizex"].getVal<int>());

    ui.sliderIntegrationTime->setMinimum(params["integration_time"].getMin()*1000);
    ui.sliderIntegrationTime->setMaximum(params["integration_time"].getMax()*1000);
    ui.sliderIntegrationTime->setValue(params["integration_time"].getVal<double>()*1000);
    ui.sliderIntegrationTime->setEnabled(!(params["integration_time"].getFlags() & ito::ParamBase::Readonly));

}

//----------------------------------------------------------------------------------------------------------------------------------
ito::RetVal DialogThorlabsCCS::applyParameters()
{
    ito::RetVal retValue(ito::retOk);
    QVector<QSharedPointer<ito::ParamBase> > values;
    bool success = false;

    if(ui.rangeX01->isEnabled())
    {
        int x0, x1;
        ui.rangeX01->values(x0,x1);
        int roi[] = {0,0,0,0};
        memcpy(roi, m_currentParameters["roi"].getVal<int*>(), 4*sizeof(int));

        if (roi[0] != x0 || roi[2] != (x1-x0+1))
        {
            roi[0] = x0;
            roi[2] = x1-x0+1;
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("roi", ito::ParamBase::IntArray, 4, roi)));
        }
    }

    if(ui.sliderIntegrationTime->isEnabled())
    {
        double dval = ui.sliderIntegrationTime->value()/1000.0;
        if(qAbs(m_currentParameters["integration_time"].getVal<double>() - dval) >= std::numeric_limits<double>::epsilon())
        {
            values.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
        }
    }

    retValue += setPluginParameters(values, msgLevelWarningAndError);

    return retValue;
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsCCS::on_buttonBox_clicked(QAbstractButton* btn)
{
    ito::RetVal retValue(ito::retOk);

    QDialogButtonBox::ButtonRole role = ui.buttonBox->buttonRole(btn);

    if (role == QDialogButtonBox::RejectRole)
    {
        reject(); //close dialog with reject
    }
    else if (role == QDialogButtonBox::AcceptRole)
    {
        accept(); //AcceptRole
    }
    else
    {
        applyParameters(); //ApplyRole
    }
}

//---------------------------------------------------------------------------------------------------------------------
void DialogThorlabsCCS::enableDialog(bool enabled)
{
    //e.g.
    ui.groupBoxIntegration->setEnabled(enabled);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DialogThorlabsCCS::on_rangeX01_valuesChanged(int minValue, int maxValue)
{
    ui.spinSizeX->setValue(maxValue - minValue + 1);
}

//------------------------------------------------------------------------------
void DialogThorlabsCCS::on_btnFullROI_clicked()
{
    if (m_currentParameters.contains("sizex"))
    {
        ui.rangeX01->setValues(0, m_currentParameters["sizex"].getMax());
    }
}
