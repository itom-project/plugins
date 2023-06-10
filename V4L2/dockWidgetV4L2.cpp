/* ********************************************************************
    Plugin "V4L2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2018, Institut fuer Technische Optik (ITO),
    Universitaet Stuttgart, Germany

    This file is part of a plugin for the measurement software itom.

    This itom-plugin is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public Licence as published by
    the Free Software Foundation; either version 2 of the Licence, or (at
    your option) any later version.

    itom and its plugins are distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Library
    General Public Licence for more details.

    You should have received a copy of the GNU Library General Public License
    along with itom. If not, see <http://www.gnu.org/licenses/>.
*********************************************************************** */

#include "dockWidgetV4L2.h"


#include <qmetaobject.h>
#include <QString>
#include <QMap>
#include <QObject>
#include <v4l2_itom_api.h>

//----------------------------------------------------------------------------------------------------------------------------------
DockWidgetV4L2::DockWidgetV4L2(ito::AddInDataIO *grabber) :
    AbstractAddInDockWidget(grabber),
    m_inEditing(false),
    m_firstRun(true)
{
    ui.setupUi(this);
}


void DockWidgetV4L2::initialize(Device *device)
{
    m_device=device;
}

void DockWidgetV4L2::setup_widget(const QMap<QString, ito::Param> &params)
{
        QString info_txt=QString("Device: '%1' [%2: %3 x %4]").arg(m_device->m_dev_name).arg(m_device->get_pixelformat_str()).arg(m_device->m_vfmt.fmt.pix.width).arg(m_device->m_vfmt.fmt.pix.height).toLatin1().data();
        ui.gb_controls->setTitle(info_txt);

        QMap<QString,QSharedPointer<V4L2Ctrl> >* ctrls =m_device->get_parameters();
        QMap<QString, QSharedPointer<V4L2Ctrl> >::iterator i;
        //iterate through ctrls
        int ctrl_counter=1;
        for(i=ctrls->begin();i!=ctrls->end();++i)
        {
            QString name = i.key();
            QSharedPointer<V4L2Ctrl> ctrl = m_device->get_ctrl_by_name(name);
            if (ctrl->get_type() == "Int")
            {
                QLabel *ctrl_label;
                ctrl_label = new QLabel(ui.gb_controls);
                ctrl_label->setObjectName(QString::fromUtf8("ctrl_label"));
                ctrl_label->setText(QString("%1").arg(name).toLatin1().data());
                ui.gridLayout_controls->addWidget(ctrl_label, ctrl_counter, 0, 1, 1);

                QLabel *def_label;
                def_label= new QLabel();
                def_label = new QLabel(ui.gb_controls);
                def_label->setObjectName(QString::fromUtf8("def_label"));
                def_label->setText(QString("[D: %1]").arg(ctrl->default_value()).toLatin1().data());
                ui.gridLayout_controls->addWidget(def_label, ctrl_counter, 1, 1, 1);

                m_sliders[name] = new SliderWidget(ui.gb_controls);
                m_sliders[name]->setObjectName(name);
                ui.gridLayout_controls->addWidget(m_sliders[name], ctrl_counter, 2, 1, 1);

                ito::IntMeta* im = (ito::IntMeta*)(params[name].getMeta());
                m_sliders[name]->setRange(im->getMin(),im->getMax());
                m_sliders[name]->setValue(params[name].getVal<int>());
                m_sliders[name]->setSingleStep(im->getStepSize());

                QObject::connect( m_sliders[name], SIGNAL(valueChanged(double)),this, SLOT(on_slider_valueChanged(double)) );

                ++ctrl_counter;
            }
            else if (ctrl->get_type() == "Bool")
            {
                QLabel *ctrl_label;
                ctrl_label = new QLabel(ui.gb_controls);
                ctrl_label->setObjectName(QString::fromUtf8("ctrl_label"));
                ctrl_label->setText(QString("%1").arg(name).toLatin1().data());
                ui.gridLayout_controls->addWidget(ctrl_label, ctrl_counter, 0, 1, 1);

                QLabel *def_label;
                def_label= new QLabel();
                def_label = new QLabel(ui.gb_controls);
                def_label->setObjectName(QString::fromUtf8("def_label"));
                def_label->setText(QString("[D: %1]").arg(ctrl->default_value()).toLatin1().data());
                ui.gridLayout_controls->addWidget(def_label, ctrl_counter, 1, 1, 1);

                m_checkbox[name] = new QCheckBox(ui.gb_controls);
                m_checkbox[name]->setObjectName(name);
                ui.gridLayout_controls->addWidget(m_checkbox[name], ctrl_counter, 2, 1, 1);

                QObject::connect( m_checkbox[name], SIGNAL(toggled(bool)),this, SLOT(on_cB_toggled(bool)) );

                ++ctrl_counter;
            }
            else
            {
                QLabel *ctrl_label;
                ctrl_label = new QLabel(ui.gb_controls);
                ctrl_label->setObjectName(QString::fromUtf8("ctrl_label"));
                ctrl_label->setText(QString("%1").arg(name).toLatin1().data());
                ui.gridLayout_controls->addWidget(ctrl_label, ctrl_counter, 0, 1, 1);

                QLabel *def_label;
                def_label= new QLabel();
                def_label = new QLabel(ui.gb_controls);
                def_label->setObjectName(QString::fromUtf8("def_label"));
                def_label->setText(QString("[not supported]").toLatin1().data());
                ui.gridLayout_controls->addWidget(def_label, ctrl_counter, 1, 1, 1);

                ++ctrl_counter;
            }
        }

        ui.label_general->setText(m_device->get_cap_string().toLatin1().data());
        ui.label_media->setText(m_device->get_fmt_str(m_device->m_vfmt).toLatin1().data());
        QString image_str=QString("Width:\t\t'%1'\nHeight:\t\t'%2'\nBit depth:\t'%3'")
                        .arg(params["sizex"].getVal<int>())
                        .arg(params["sizey"].getVal<int>())
                        .arg(params["bpp"].getVal<int>()).toLatin1().data();
        ui.label_image->setText(image_str);

//
//        QLabel *media_label;
//        media_label= new QLabel();
//        media_label->setText(m_device->get_fmt_str(m_device->m_vfmt).toLatin1().data());
//        ui.vbox_media->addWidget(media_label);
//
//        QLabel *image_label;
//        image_label= new QLabel(ui.groupBox_image);
//        QString image_str=QString("Width:\t'%1'\nHeight:\t'%2'\nBit depth:\t'%3'")
//                .arg(params["sizex"].getVal<int>())
//                .arg(params["sizey"].getVal<int>())
//                .arg(params["bpp"].getVal<int>()).toLatin1().data();
//        image_label->setText(image_str);
//        ui.vbox_image->addWidget(image_label);




//                if (params.contains("sizex"))
//                {
//                    ui.lblWidth->setText(QString("%1").arg(params["sizex"].getVal<int>()));
//                }
//
//                if (params.contains("sizey"))
//                {
//                    ui.lblHeight->setText(QString("%1").arg(params["sizey"].getVal<int>()));
//                }
//
//                if (params.contains("bpp"))
//                {
//                    ui.lblBitDepth->setText(QString("%1").arg(params["bpp"].getVal<int>()));
//                }
//
//                if (propCount == 0)
//                {
//                    ui.groupBox_3->setVisible(false);
//                    setMinimumHeight(109);
//                }
//                else
//                {
//                    setMinimumHeight((26 * propCount) + 129);
//                }

}
//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetV4L2::parametersChanged(QMap<QString, ito::Param> params)
{
    if (m_firstRun)
    {
        setup_widget(params);

        m_firstRun = false;
    }
    if (!m_inEditing)
    {
        m_inEditing = true;

        QMap<QString,QSharedPointer<V4L2Ctrl> >* ctrls =m_device->get_parameters();
        QMap<QString, QSharedPointer<V4L2Ctrl> >::iterator i;
        for(i=ctrls->begin();i!=ctrls->end();++i)
        {
            QString name = i.key();
            QSharedPointer<V4L2Ctrl> ctrl = m_device->get_ctrl_by_name(name);
            if (ctrl->get_type() == "Int")
            {
                m_sliders[name]->setValue(params[name].getVal<int>());
            }
            else if(ctrl->get_type() == "Bool")
            {
                m_checkbox[name]->setChecked(params[name].getVal<int>());
            }
        }

        m_inEditing = false;
    }



//
//        m_firstRun = false;
//    }
//
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        if (params.contains("brightness"))
//        {
//            ui.sW_Brightness->setValue(params["brightness"].getVal<int>());
//        }
//        if (params.contains("brightnessAuto"))
//        {
//            ui.cB_Brightness->setChecked(params["brightnessAuto"].getVal<int>());
//            ui.sW_Brightness->setEnabled(params["brightnessAuto"].getVal<int>() == 0);
//        }
//
//        if (params.contains("contrast"))
//        {
//            ui.sW_Contrast->setValue(params["contrast"].getVal<int>());
//        }
//        if (params.contains("contrastAuto"))
//        {
//            ui.cB_Contrast->setChecked(params["contrastAuto"].getVal<int>());
//            ui.sW_Contrast->setEnabled(params["contrastAuto"].getVal<int>() == 0);
//        }
//
//        if (params.contains("gain"))
//        {
//            ui.sW_Gain->setValue(params["gain"].getVal<int>());
//        }
//        if (params.contains("gainAuto"))
//        {
//            ui.cB_Gain->setChecked(params["gainAuto"].getVal<int>());
//            ui.sW_Gain->setEnabled(params["gainAuto"].getVal<int>() == 0);
//        }
//
//
//
//        if (params.contains("saturation"))
//        {
//            ui.sW_Saturation->setValue(params["saturation"].getVal<int>());
//        }
//        if (params.contains("saturationAuto"))
//        {
//            ui.cB_Saturation->setChecked(params["saturationAuto"].getVal<int>());
//            ui.sW_Saturation->setEnabled(params["saturationAuto"].getVal<int>() == 0);
//        }
//
//        if (params.contains("sharpness"))
//        {
//            ui.sW_Sharpness->setValue(params["sharpness"].getVal<int>());
//        }
//        if (params.contains("sharpnessAuto"))
//        {
//            ui.cB_Sharpness->setChecked(params["sharpnessAuto"].getVal<int>());
//            ui.sW_Sharpness->setEnabled(params["sharpnessAuto"].getVal<int>() == 0);
//        }
//        m_inEditing = false;
//    }
}

//----------------------------------------------------------------------------------------------------------------------------------
double getStepValue(double value, double stepSize)
{
    int stepCount = (int)((value / stepSize) + .5);
    //qDebug() << "----------------- getStepValue stepCount: " << stepCount << "; alt: " << value / stepSize;  // getStepValue stepCount:  77.4818 ; alt:  77.4
    return stepSize * stepCount;
}

void DockWidgetV4L2::on_slider_valueChanged(double d)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        SliderWidget *slider = (SliderWidget *)sender();
        d = getStepValue(d, slider->singleStep());
        slider->setValue(d);
        QString name =QString("%1").arg(slider->objectName());
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase(name.toLatin1().data(),ito::ParamBase::Int,d));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

void DockWidgetV4L2::on_cB_toggled(bool checked)
{
    if (!m_inEditing)
    {
        m_inEditing = true;
        QCheckBox *cb = (QCheckBox *)sender();
        QString name =QString("%1").arg(cb->objectName());
        m_checkbox[name]->setChecked(!checked);
        QSharedPointer<ito::ParamBase> p(new ito::ParamBase(name.toLatin1().data(),ito::ParamBase::Int,checked ? 1 : 0));
        setPluginParameter(p, msgLevelWarningAndError);
        m_inEditing = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_sW_Brightness_valueChanged(double d)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        d = getStepValue(d, ui.sW_Brightness->singleStep());
//        ui.sW_Brightness->setValue(d);
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("brightness",ito::ParamBase::Int,d));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_sW_Contrast_valueChanged(double d)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double d2 = getStepValue(d, ui.sW_Contrast->singleStep());
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrast",ito::ParamBase::Int,d2));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_sW_Gain_valueChanged(double d)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double d2 = getStepValue(d, ui.sW_Gain->singleStep());
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gain",ito::ParamBase::Int,d2));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_sW_Saturation_valueChanged(double d)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double d2 = getStepValue(d, ui.sW_Saturation->singleStep());
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("saturation",ito::ParamBase::Int,d2));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_sW_Sharpness_valueChanged(double d)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        double d2 = getStepValue(d, ui.sW_Sharpness->singleStep());
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("sharpness",ito::ParamBase::Int,d2));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_cB_Brightness_toggled(bool checked)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        ui.sW_Brightness->setEnabled(!checked);
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("brightnessAuto",ito::ParamBase::Int,checked ? 1 : 0));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_cB_Contrast_toggled(bool checked)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        ui.sW_Contrast->setEnabled(!checked);
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("contrastAuto",ito::ParamBase::Int,checked ? 1 : 0));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_cB_Gain_toggled(bool checked)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        ui.sW_Gain->setEnabled(!checked);
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("gainAuto",ito::ParamBase::Int,checked ? 1 : 0));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_cB_Saturation_toggled(bool checked)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        ui.sW_Saturation->setEnabled(!checked);
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("saturationAuto",ito::ParamBase::Int,checked ? 1 : 0));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------
//void DockWidgetV4L2::on_cB_Sharpness_toggled(bool checked)
//{
//    if (!m_inEditing)
//    {
//        m_inEditing = true;
//        ui.sW_Sharpness->setEnabled(!checked);
//        QSharedPointer<ito::ParamBase> p(new ito::ParamBase("sharpnessAuto",ito::ParamBase::Int,checked ? 1 : 0));
//        setPluginParameter(p, msgLevelWarningAndError);
//        m_inEditing = false;
//    }
//}

//----------------------------------------------------------------------------------------------------------------------------------
void DockWidgetV4L2::identifierChanged(const QString &identifier)
{
    //ui.label_Identifier->setText(identifier);
}
