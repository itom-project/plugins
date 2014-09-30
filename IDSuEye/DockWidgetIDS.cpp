#include "dockWidgetIDS.h"

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetIDS::DockWidgetIDS(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetIDS::parametersChanged(QMap<QString, ito::Param> params)
{    
    int inEditing = m_inEditing;
    m_inEditing = true;

    ui.lblWidth->setText(QString::number(params["sizex"].getVal<int>()));
    ui.lblHeight->setText(QString::number(params["sizey"].getVal<int>()));

    ui.sliderGain->setDisabled( params["gain"].getFlags() & ito::ParamBase::Readonly );
    ui.sliderOffset->setDisabled( params["offset"].getFlags() & ito::ParamBase::Readonly );
    ui.sliderExposure->setDisabled( params["exposure"].getFlags() & ito::ParamBase::Readonly );

    if (m_firstRun)
    {
        if (params.contains("cam_model"))
        {
            ui.lblModel->setText(params["cam_model"].getVal<char*>());
        }
        
        if (params.contains("sensor_type"))
        {
            ui.lblSensor->setText(params["sensor_type"].getVal<char*>());
        }

        if (params.contains("cam_id"))
        {
            ui.lblID->setText(QString::number(params["cam_id"].getVal<int>()));
        }

        m_firstRun = false;
    }

    if (strcmp(params["color_mode"].getVal<char*>(),"gray") == 0)
    {
        ui.lblBitDepth->setText(QString("gray (%1 bit)").arg(params["bpp"].getVal<int>()));
    }
    else
    {
        ui.lblBitDepth->setText(QString("RGB (8 bit)"));
    }

    if (params.contains("integration_time"))
    {
        ito::DoubleMeta *dm = (ito::DoubleMeta*)(params["integration_time"].getMeta());
        ui.sliderExposure->setMinimum(dm->getMin());
        ui.sliderExposure->setMaximum(dm->getMax());
        if (dm->getStepSize() != 0)
        {
            ui.sliderExposure->setSingleStep(std::max(dm->getStepSize(), 0.00001)); //0.00001 is the minimal step of the spin box
        }
        else
        {
            ui.sliderExposure->setSingleStep((dm->getMax() - dm->getMin()) / 100);
        }
        ui.sliderExposure->setValue(params["integration_time"].getVal<double>());
    }

    if (params.contains("gain"))
    {
        ui.sliderGain->setValue(params["gain"].getVal<double>());
    }

    if (params.contains("offset"))
    {
        ui.sliderOffset->setValue(params["offset"].getVal<double>());
    }

    m_inEditing = inEditing;

    m_currentParams = params;
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetIDS::on_sliderExposure_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("integration_time",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetIDS::on_sliderGain_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetIDS::on_sliderOffset_valueChanged(double value)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("offset",ito::ParamBase::Double,value));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}