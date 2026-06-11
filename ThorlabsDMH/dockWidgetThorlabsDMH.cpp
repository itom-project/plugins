/* ********************************************************************
    Template for an actuator plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dockWidgetThorlabsDMH.h"

#include "TLDFMX.h"


//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetThorlabsDMH::DockWidgetThorlabsDMH(ThorlabsDMH* actuator) :
    AbstractAddInDockWidget(actuator), m_pActuator(actuator), m_inEditing(false), m_firstRun(true)
{
    ui.setupUi(this);

    m_sliderWidget.append(ui.sliderZ4);
    m_sliderWidget.append(ui.sliderZ5);
    m_sliderWidget.append(ui.sliderZ6);
    m_sliderWidget.append(ui.sliderZ7);
    m_sliderWidget.append(ui.sliderZ8);
    m_sliderWidget.append(ui.sliderZ9);
    m_sliderWidget.append(ui.sliderZ10);
    m_sliderWidget.append(ui.sliderZ11);
    m_sliderWidget.append(ui.sliderZ12);
    m_sliderWidget.append(ui.sliderZ13);
    m_sliderWidget.append(ui.sliderZ14);
    m_sliderWidget.append(ui.sliderZ15);
    for (size_t i = 0; i < m_sliderWidget.size(); ++i)
    {
        SliderWidget* slider = m_sliderWidget[i];
        connect(slider, &SliderWidget::valueChanged, this, [this, i](double value) {
            on_slider_valueChanged(value, static_cast<int>(i));
        });
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::parametersChanged(QMap<QString, ito::Param> params)
{
    this->updateSlider();

    ui.lblSerialNo->setText(params["serialNumber"].getVal<char*>());
}

void DockWidgetThorlabsDMH::updateSlider()
{
    m_inEditing = true;

    QVector<double> zernikeAmplitude;
    m_pActuator->getZernikeAmplitude(zernikeAmplitude);

    for (size_t i = 0; i < m_sliderWidget.size(); ++i)
    {
        m_sliderWidget[i]->setValue(zernikeAmplitude[i]);
    }

    m_inEditing = false;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::setZernike(QVector<double> zernikeAmplitude)
{
    QSharedPointer<QVector<ito::ParamBase>> pMand(
        new QVector<ito::ParamBase>());
    QVector<int> zernikeID = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    ito::ParamBase param1(
        "ZernikeIDs", ito::ParamBase::IntArray, zernikeID.size(), zernikeID.data());

    ito::ParamBase param2(
        "ZernikeValues",
        ito::ParamBase::DoubleArray,
        zernikeAmplitude.size(),
        zernikeAmplitude.data());

    pMand->append(param1);
    pMand->append(param2);
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pActuator->execFunc("setZernikes", pMand, _dummy, _dummy, nullptr);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::on_slider_valueChanged(double value, int sliderID)
{
    if (!m_inEditing)
    {
        QVector<double> zernikeAmplitude;
        m_pActuator->getZernikeAmplitude(zernikeAmplitude);

        zernikeAmplitude[sliderID] = value;

        this->setZernike(zernikeAmplitude);
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::on_relaxMirror_clicked()
{
    QSharedPointer<QVector<ito::ParamBase>> _dummy;
    m_pActuator->execFunc("relaxMirror", _dummy, _dummy, _dummy, nullptr);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::on_resetZernike_clicked()
{
    QVector<double> zernikeAmplitude(12, 0.0);
    this->setZernike(zernikeAmplitude);
    this->updateSlider();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::enableWidgets(bool enabled)
{
    ui.axisController->setEnabled(enabled);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::identifierChanged(const QString& identifier)
{
}

//-------------------------------------------------------------------------------------------------------------------------------------------------
void DockWidgetThorlabsDMH::dockWidgetVisibilityChanged(bool visible)
{
    if (visible)
    {
        // to connect the signals
        QPointer<ito::AddInActuator> actuator(m_pActuator);
        ui.axisController->setActuator(actuator);
        ui.axisController->setNumAxis(40);
        ui.axisController->setAxisEnabled(0, true);
        ui.axisController->setAxisType(0, MotorAxisController::TypeRotational);
        ui.axisController->setEnabled(true);
    }
    else
    {
        ui.axisController->setActuator(QPointer<ito::AddInActuator>());
    }
}
