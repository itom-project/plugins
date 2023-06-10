/* ********************************************************************
    Template for a camera / grabber plugin for the software itom

    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#include "dockWidgetNewport2936.h"


//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetNewport2936::DockWidgetNewport2936(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true),
    m_plugin(grabber),
    m_timerIsRunning(false)
{
    qRegisterMetaType<QSharedPointer<QList<double> > >("QSharedPointer<QList<double> >");
    ui.setupUi(this);
    QPointer<ito::AddInBase> plugin(grabber);
    ui.editorWidget->setPlugin(plugin);
}
void DockWidgetNewport2936::parametersChanged(QMap<QString, ito::Param> params)
{

    if (m_firstRun)
    {
        ui.lcdValueA->setPalette(Qt::red);
        ui.lcdValueB->setPalette(Qt::red);
        m_firstRun = false;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewport2936::identifierChanged(const QString &identifier)
{
    ui.label_Identifier->setText(identifier);
    ui.editorWidget->refresh();
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewport2936::on_checkAutograbbing_stateChanged(int val)
{
    manageTimer(true);
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewport2936::timerEvent(QTimerEvent *event)
{

    ito::RetVal retval(ito::retOk);
    ItomSharedSemaphore* waitCond = new ItomSharedSemaphore();
    QSharedPointer<QList<double> > value = QSharedPointer<QList <double> >(new QList<double>);
    QPair<double, QString> result(0, "");
    QMetaObject::invokeMethod(m_plugin, "acquireAutograbbing", Q_ARG(QSharedPointer<QList<double> >, value), Q_ARG(ItomSharedSemaphore*, waitCond));
    if (waitCond->waitAndProcessEvents(10000))
    {
        retval += waitCond->returnValue;
        if (!retval.containsError())
        {
            double i = value->at(0);
            calculateUnit(value->at(0), result);
            ui.lcdValueA->display(result.first);
            ui.unitA->setText(result.second);
            if (value->length() == 2)
            {
              calculateUnit(value->at(1), result);
              ui.lcdValueB->display(result.first);
              ui.unitB->setText(result.second);
            }
        }
    }
    waitCond->deleteSemaphore();


}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewport2936::manageTimer(const bool &visible)
{
    if (visible && ui.checkAutograbbing->checkState() == Qt::Checked)
    {
        m_timerId = startTimer(100);
        m_timerIsRunning = true;
    }
    else if (m_timerIsRunning)
    {
        killTimer(m_timerId);
        m_timerIsRunning = false;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetNewport2936::calculateUnit(const double &val, QPair<double, QString> &result)
{

    double exp(log10(abs(val)));
    if (exp >= 0.0)
    {
        result.first = val;
        result.second = "W";
    }
    else if (exp >= -3.0)
    {
        result.first = val*10e2;
        result.second = "mW";
    }
    else if (exp >= -6.0)
    {
        result.first = val*10e5;
        result.second = QString::fromLatin1("\u00B5W");
    }
    else if (exp >= -9.0)
    {
        result.first = val*10e8;
        result.second = "nW";
    }
    else if (exp >= -12.0)
    {
        result.first = val*10e11;
        result.second = "pW";
    }
    else if (exp >= -15.0)
    {
        result.first = val*10e14;
        result.second = "fW";
    }
    else
    {
        result.first = val;
        result.second = "unknown";
    }
}
