#include "dialogVistek.h"

#include <qmetaobject.h>



void DialogVistek::valuesChanged(QMap<QString, ito::Param> params)
{
	m_params = params;

    setWindowTitle(QString((params)["name"].getVal<char*>()) + " - " + tr("Configuration Dialog"));
    // added by itobiege, Mar. 2013, but not tested!

	//adapt values of ui-elements to values given in params.
}

void DialogVistek::sendVals(ito::AddInGrabber *receiverGrabber)
{
    QVector<QSharedPointer<ito::ParamBase> > outVector;

    //if(ui.spinBox_gain->isEnabled())
    //{
    //    double dval = ui.spinBox_gain->value()/100.0;
    //    if(m_params["gain"].getVal<double>() !=  dval)
    //    {
    //        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("gain", ito::ParamBase::Double, dval)));
    //    }
    //}

    //if(ui.spinBox_offset->isEnabled())
    //{
    //    double dval = ui.spinBox_offset->value()/100.0;
    //    if(m_params["offset"].getVal<double>() !=  dval)
    //    {
    //        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("offset", ito::ParamBase::Double, dval)));
    //    }
    //}

    //if(ui.doubleSpinBox_integration_time->isEnabled())
    //{
    //    double dval = ui.doubleSpinBox_integration_time->value()/1000.0;
    //    if(m_params["integration_time"].getVal<double>() !=  dval)
    //    {
    //        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("integration_time", ito::ParamBase::Double, dval)));
    //    }
    //}

    //if(ui.doubleSpinBox_frame_time->isEnabled())
    //{
    //    double dval = ui.doubleSpinBox_frame_time->value()/1000.0;
    //    if(m_params["frame_time"].getVal<double>() !=  dval)
    //    {
    //        outVector.append(QSharedPointer<ito::ParamBase>(new ito::ParamBase("frame_time", ito::ParamBase::Double, dval)));
    //    }
    //}

    if(receiverGrabber)   // Grabber exists
    {
        ItomSharedSemaphoreLocker locker(new ItomSharedSemaphore());
        QMetaObject::invokeMethod(receiverGrabber, "setParamVector", Q_ARG(const QVector<QSharedPointer<ito::ParamBase> >, outVector), Q_ARG(ItomSharedSemaphore*, locker.getSemaphore()));

        while (!locker.getSemaphore()->wait(5000))
        {
            if (!receiverGrabber->isAlive())
            {
                break;
            }
        }
    }
}

////----------------------------------------------------------------------------------------------------------------------------------
//int DialogVistek::setVals(QMap<QString, ito::Param> *paramVals)
//{
//    QVariant qvar;
//    int bpp = 8;
//    int mode = 0;
//
//    qvar = ((*paramVals)["sizex"]).getVal<double>();
//    ui.edit_sizex->setText(qvar.toString());
//    qvar = ((*paramVals)["sizey"]).getVal<double>();
//    ui.edit_sizey->setText(qvar.toString());
//    qvar = ((*paramVals)["gain"]).getVal<double>();
//    ui.edit_gain->setText(qvar.toString());
//    qvar = ((*paramVals)["offset"]).getVal<double>();
//    ui.edit_offset->setText(qvar.toString());
//
//    bpp = ((*paramVals)["bpp"]).getVal<int>();
//    switch (bpp)
//    {
//        case 8:
//            ui.combo_bpp->setCurrentIndex(0);
//        break;
//        case 10:
//            ui.combo_bpp->setCurrentIndex(1);
//        break;
//        case 12:
//            ui.combo_bpp->setCurrentIndex(2);
//        break;
//        case 14:
//            ui.combo_bpp->setCurrentIndex(3);
//        break;
//        case 16:
//            ui.combo_bpp->setCurrentIndex(4);
//        break;
//        case 24:
//            ui.combo_bpp->setCurrentIndex(5);
//        break;
//        case 32:
//            ui.combo_bpp->setCurrentIndex(6);
//        break;
//    }
//
//    qvar = ((*paramVals)["wli_frames"]).getVal<double>();
//    ui.edit_wli_frames->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_noise"]).getVal<double>();
//    ui.edit_wli_noise->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_amplitude"]).getVal<double>();
//    ui.edit_wli_amplitude->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_offset"]).getVal<double>();
//    ui.edit_wli_offset->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_phi"]).getVal<double>();
//    ui.edit_wli_phi0->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_stepsperlambda"]).getVal<double>();
//    ui.edit_wli_steps->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_width"]).getVal<double>();
//    ui.edit_wli_width->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_r"]).getVal<double>();
//    ui.edit_wli_featwidth->setText(qvar.toString());
//    qvar = ((*paramVals)["wli_h"]).getVal<double>();
//    ui.edit_wli_featheight->setText(qvar.toString());
//    mode = ((*paramVals)["wli_mode"]).getVal<int>();
//    ui.combo_wli_mode->setCurrentIndex(mode);
//
//    qvar = ((*paramVals)["conf_frames"]).getVal<double>();
//    ui.edit_conf_frames->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_noise"]).getVal<double>();
//    ui.edit_conf_noise->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_amplitude"]).getVal<double>();
//    ui.edit_conf_amplitude->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_offset"]).getVal<double>();
//    ui.edit_conf_offset->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_zoffset"]).getVal<double>();
//    ui.edit_conf_zoffset->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_muhperstep"]).getVal<double>();
//    ui.edit_conf_musteps->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_fwhm"]).getVal<double>();
//    ui.edit_conf_fwhm->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_r"]).getVal<double>();
//    ui.edit_conf_featwidth->setText(qvar.toString());
//    qvar = ((*paramVals)["conf_h"]).getVal<double>();
//    ui.edit_conf_featheight->setText(qvar.toString());
//    mode = ((*paramVals)["conf_mode"]).getVal<int>();
//    ui.combo_conf_mode->setCurrentIndex(mode);
//
//    return 0;
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//int DialogVistek::getVals(QMap<QString, ito::Param> *paramVals)
//{
//    QVariant qvar;
//    //int bpp = 8;
//    int mode = 0;
//
//    qvar = ui.edit_sizex->text();
//    ((*paramVals)["sizex"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_sizey->text();
//    ((*paramVals)["sizey"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_gain->text();
//    ((*paramVals)["gain"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_offset->text();
//    ((*paramVals)["offset"]).setVal<double>(qvar.toDouble());
//
//    qvar = ui.combo_bpp->currentIndex();
//    switch (qvar.toInt())
//    {
//        case 0:
//            ((*paramVals)["bpp"]).setVal<double>(8);
//        break;
//        case 1:
//            ((*paramVals)["bpp"]).setVal<double>(10);
//        break;
//        case 2:
//            ((*paramVals)["bpp"]).setVal<double>(12);
//        break;
//        case 3:
//            ((*paramVals)["bpp"]).setVal<double>(14);
//        break;
//        case 4:
//            ((*paramVals)["bpp"]).setVal<double>(16);
//        break;
//        case 5:
//            ((*paramVals)["bpp"]).setVal<double>(24);
//        break;
//        case 6:
//            ((*paramVals)["bpp"]).setVal<double>(32);
//        break;
//    }
//
//    qvar = ui.edit_wli_frames->text();
//    ((*paramVals)["wli_frames"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_noise->text();
//    ((*paramVals)["wli_noise"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_amplitude->text();
//    ((*paramVals)["wli_amplitude"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_offset->text();
//    ((*paramVals)["wli_offset"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_phi0->text();
//    ((*paramVals)["wli_phi"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_steps->text();
//    ((*paramVals)["wli_stepsperlambda"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_width->text();
//    ((*paramVals)["wli_width"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_featwidth->text();
//    ((*paramVals)["wli_r"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_wli_featheight->text();
//    ((*paramVals)["wli_h"]).setVal<double>(qvar.toDouble());
//    mode = ui.combo_wli_mode->currentIndex();
//    ((*paramVals)["wli_mode"]).setVal<int>(mode);
//
//    qvar = ui.edit_conf_frames->text();
//    ((*paramVals)["conf_frames"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_noise->text();
//    ((*paramVals)["conf_noise"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_amplitude->text();
//    ((*paramVals)["conf_amplitude"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_offset->text();
//    ((*paramVals)["conf_offset"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_zoffset->text();
//    ((*paramVals)["conf_zoffset"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_musteps->text();
//    ((*paramVals)["conf_muhperstep"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_fwhm->text();
//    ((*paramVals)["conf_fwhm"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_featwidth->text();
//    ((*paramVals)["conf_r"]).setVal<double>(qvar.toDouble());
//    qvar = ui.edit_conf_featheight->text();
//    ((*paramVals)["conf_h"]).setVal<double>(qvar.toDouble());
//    mode = ui.combo_conf_mode->currentIndex();
//    ((*paramVals)["conf_mode"]).setVal<int>(mode);
//
//
//    return 0;
//}

//----------------------------------------------------------------------------------------------------------------------------------
