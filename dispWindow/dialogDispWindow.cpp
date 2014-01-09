#include "dialogDispWindow.h"
#include "dispWindow.h"
//#include "../Common/sharedStructuresQt.h"

//----------------------------------------------------------------------------------------------------------------------------------
int dialogDispWindow::setVals(QMap<QString, ito::Param> *params, const int numImages)
{
    QVariant qvar;

    setWindowTitle(QString((*params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));

    qvar = (*params)["x0"].getVal<int>();
    ui.spinBox_x0->setValue(qvar.toInt());
    qvar = (*params)["y0"].getVal<int>();
    ui.spinBox_y0->setValue(qvar.toInt());
    qvar = (*params)["xsize"].getVal<int>();
    ui.spinBox_xsize->setValue(qvar.toInt());
    qvar = (*params)["ysize"].getVal<int>();
    ui.spinBox_ysize->setValue(qvar.toInt());
    qvar = (*params)["period"].getVal<int>();
    ui.spinBox_period->setValue(qvar.toInt());

    qvar = (*params)["phaseshift"].getVal<int>();
    switch(qvar.toInt())
    {
        case 3:
            ui.comboBox_phaseshifts->setCurrentIndex(0);
        break;

        default:
        case 4:
            ui.comboBox_phaseshifts->setCurrentIndex(1);
        break;

        case 5:
            ui.comboBox_phaseshifts->setCurrentIndex(2);
        break;

        case 8:
            ui.comboBox_phaseshifts->setCurrentIndex(3);
        break;
    }

    qvar = (*params)["color"].getVal<int>();
    ui.comboBox_color->setCurrentIndex(qvar.toInt());

    qvar = (*params)["orientation"].getVal<int>();
    ui.comboBox_orientation->setCurrentIndex(qvar.toInt());

    qvar = (*params)["gamma"].getVal<int>();
    ui.checkBox_gamma->setChecked(qvar.toInt());

    ui.label_imgend->setText(QVariant(numImages - 1).toString());
//    ui.slider_image->setMaximum(numImages - 1);
//    ui.slider_image->setValue((*params)["numimg"].getVal<int>());

    ui.horizontalSlider->setMinimum(0);
    ui.horizontalSlider->setMaximum(numImages - 1);
    ui.horizontalSlider->setValue((*params)["numimg"].getVal<int>());

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
int dialogDispWindow::getVals(QMap<QString, ito::Param> *params)
{
    QVariant qvar;

    qvar = ui.spinBox_x0->value();
    (*params)["x0"].setVal<int>(qvar.toInt());
    
    qvar = ui.spinBox_y0->value();
    (*params)["y0"].setVal<int>(qvar.toInt());
    
    qvar = ui.spinBox_xsize->value();
    (*params)["xsize"].setVal<int>(qvar.toInt());

    qvar = ui.spinBox_ysize->value();
    (*params)["ysize"].setVal<int>(qvar.toInt());

    qvar = ui.spinBox_period->value();
    (*params)["period"].setVal<int>(qvar.toInt());

    switch(ui.comboBox_phaseshifts->currentIndex())
    {
        case 0:
            (*params)["phaseshift"].setVal<int>(3); 
        break;

        default:
        case 1:
            (*params)["phaseshift"].setVal<int>(4); 
        break;

        case 2:
            (*params)["phaseshift"].setVal<int>(5); 
        break;

        case 3:
            (*params)["phaseshift"].setVal<int>(8); 
        break;
    }

    (*params)["color"].setVal<int>(ui.comboBox_color->currentIndex());
    (*params)["orientation"].setVal<int>(ui.comboBox_orientation->currentIndex());
    (*params)["gamma"].setVal<int>(ui.checkBox_gamma->isChecked());
//    (*params)["numimg"].setVal<int>(ui.slider_image->value());
    (*params)["numimg"].setVal<int>(ui.horizontalSlider->value());

    return 0;
}

//----------------------------------------------------------------------------------------------------------------------------------
dialogDispWindow::dialogDispWindow(const void *prjWindow)
{ 
    m_pWindow = (void*)prjWindow;
    ui.setupUi(this);
}

//----------------------------------------------------------------------------------------------------------------------------------
dialogDispWindow::~dialogDispWindow()
{ }

//----------------------------------------------------------------------------------------------------------------------------------
void dialogDispWindow::on_pushButtonSet_clicked(void)
{
    QVariant qvar;
    ito::RetVal retval = ito::retOk;

    qvar = ui.spinBox_x0->value();
    int x0 = qvar.toInt();
    qvar = ui.spinBox_y0->value();
    int y0 = qvar.toInt();
    qvar = ui.spinBox_xsize->value();
    int xsize = qvar.toInt();
    qvar = ui.spinBox_ysize->value();
    int ysize = qvar.toInt();
    qvar = ui.spinBox_period->value();
    int period = qvar.toInt();

    qvar = ui.comboBox_phaseshifts->currentIndex();
    int phaShift = 4;
    switch (qvar.toInt())
    {
        case 0:
            phaShift = 3;
        break;
        default:
        case 1:
            phaShift = 4;
        break;
        case 2:
            phaShift = 5;
        break;
        case 3:
            phaShift = 8;
        break;
    }

    DispWindow *pWindow = (DispWindow *)m_pWindow;

    PrjWindow *prjWin = (PrjWindow*)pWindow->getPrjWindow();

    //prjWin->setPeriod(period);
    //prjWin->setPhaseShift(phaShift);

    //ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
    prjWin->configProjectionFull(x0, xsize, y0, ysize, period, phaShift, ui.comboBox_orientation->currentIndex());

    //QMetaObject::invokeMethod(prjWin, "configProjectionFull", 
    //                          Q_ARG(int, x0), Q_ARG(int, xsize),
    //                          Q_ARG(int, y0), Q_ARG(int, ysize),
    //                          Q_ARG(int, period), 
    //                          Q_ARG(int, phaShift), 
    //                          Q_ARG(int, ui.comboBox_orientation->currentIndex()));
    //locker.getSemaphore()->wait(-1);
    //retval += locker.getSemaphore()->returnValue;

    //QMetaObject::invokeMethod(prjWin, "setPos", Q_ARG(int, x0), Q_ARG(int, y0));
    //QMetaObject::invokeMethod(prjWin, "setSize", Q_ARG(int, xsize), Q_ARG(int, ysize));

    qvar = ui.horizontalSlider->value();
    prjWin->showImageNum(qvar.toInt());
}

//----------------------------------------------------------------------------------------------------------------------------------
/*void dialogDispWindow::on_slider_image_valueChanged()
{
    DispWindow *pWindow = (DispWindow *)m_pWindow;
    QVariant qvar = ui.slider_image->value();
    PrjWindow *prjWin = (PrjWindow*)pWindow->getPrjWindow();
    prjWin->showImageNum(qvar.toInt());
}*/

//----------------------------------------------------------------------------------------------------------------------------------
void dialogDispWindow::on_horizontalSlider_valueChanged()
{
    DispWindow *pWindow = (DispWindow *)m_pWindow;
    QVariant qvar = ui.horizontalSlider->value();
    PrjWindow *prjWin = (PrjWindow*)pWindow->getPrjWindow();
    
    prjWin->showImageNum(qvar.toInt());
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogDispWindow::on_comboBox_orientation_currentIndexChanged(int index)
{
    DispWindow *pWindow = (DispWindow *)m_pWindow;
    PrjWindow *prjWin = (PrjWindow*)pWindow->getPrjWindow();

    QMetaObject::invokeMethod(prjWin, "setOrientation", Q_ARG(int, index));

//    ui.slider_image->setMaximum(prjWin->getNumImages() - 1);
    ui.horizontalSlider->setMaximum(prjWin->getNumImages() - 1);
    ui.label_imgend->setText(QVariant(prjWin->getNumImages() - 1).toString());
//    QVariant qvar = ui.slider_image->value();
    QVariant qvar = ui.horizontalSlider->value();
    prjWin->showImageNum(qvar.toInt());
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogDispWindow::on_comboBox_color_currentIndexChanged(int /*index*/)
{
    DispWindow *pWindow = (DispWindow *)m_pWindow;
    PrjWindow *prjWin = (PrjWindow*)pWindow->getPrjWindow();
    int idx = ui.comboBox_color->currentIndex();
    if ((idx >= 0) && (idx <= 3))
        prjWin->setColor(idx);
}

//----------------------------------------------------------------------------------------------------------------------------------
void dialogDispWindow::on_checkBox_gamma_stateChanged(int state)
{
    DispWindow *pWindow = (DispWindow *)m_pWindow;
    PrjWindow *prjWin = (PrjWindow*)pWindow->getPrjWindow();
    prjWin->setGamma(state);
}

//----------------------------------------------------------------------------------------------------------------------------------
