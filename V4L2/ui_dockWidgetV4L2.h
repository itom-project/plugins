/* ********************************************************************
    Plugin "V4L2" for itom software
    URL: http://www.uni-stuttgart.de/ito
    Copyright (C) 2014, Institut fuer Technische Optik (ITO),
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

#ifndef UI_DOCKWIDGETV4L2_H
#define UI_DOCKWIDGETV4L2_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <QtGui/QScrollArea>
#include "sliderWidget.h"


QT_BEGIN_NAMESPACE

class Ui_DockWidgetV4L2
{
public:
    QVBoxLayout *layout_main;
    QTabWidget *tabWidget;
    QWidget *tab_ctrl;
    QVBoxLayout *layout_tab1;
    QGroupBox *gb_controls;
    QGridLayout *gridLayout_controls;
    QLabel *lbl_slider1;
    QSlider *horizontalSlider;
    QLabel *lbl_slider2;
    QSlider *horizontalSlider_2;
    QWidget *tab_info;
    QVBoxLayout *layout_tab2;
    QGroupBox *gb_general;
    QVBoxLayout *layout_general;
    QLabel *label_general;
    QGroupBox *gb_image;
    QVBoxLayout *layout_image;
    QLabel *label_image;
    QGroupBox *gb_media;
    QVBoxLayout *layout_media;
    QLabel *label_media;
    QScrollArea *scroll_area_ctrltab;
    QWidget *scroll_area_ctrltab_contents;
    QVBoxLayout *ctrltab_layout;
    QScrollArea *scroll_area_infotab;
    QWidget *scroll_area_infotab_contents;
    QVBoxLayout *infotab_layout;

    void setupUi(QWidget *DockWidgetV4L2)
    {

    	if (DockWidgetV4L2->objectName().isEmpty())
    		DockWidgetV4L2->setObjectName(QString::fromUtf8("DockWidgetV4L2"));

    	//DockWidgetV4L2->resize(400, 300);

    	//Main Layout
    	layout_main = new QVBoxLayout(DockWidgetV4L2);
    	layout_main->setObjectName(QString::fromUtf8("layout_main"));

    	//Tab Widget
    	tabWidget = new QTabWidget(DockWidgetV4L2);
    	tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    	tabWidget->setTabPosition(QTabWidget::South);

    	QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    	sizePolicy.setHorizontalStretch(0);
    	sizePolicy.setVerticalStretch(0);
    	//sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());

    	tabWidget->setSizePolicy(sizePolicy);

    	//Control Tab
    	tab_ctrl = new QWidget();
    	tab_ctrl->setObjectName(QString::fromUtf8("tab_ctrl"));

    	layout_tab1 = new QVBoxLayout(tab_ctrl);
    	layout_tab1->setObjectName(QString::fromUtf8("layout_tab1"));

    	//Scroll Area for ctrltab
    	scroll_area_ctrltab = new QScrollArea(tab_ctrl);
    	scroll_area_ctrltab->setObjectName(QString::fromUtf8("scroll_area1"));
    	scroll_area_ctrltab->setWidgetResizable(true);

    	//Content Widget for scroll area
    	scroll_area_ctrltab_contents = new QWidget();
    	scroll_area_ctrltab_contents->setObjectName(QString::fromUtf8("scroll_area_ctrltab_contents"));

    	ctrltab_layout = new QVBoxLayout(scroll_area_ctrltab_contents);
    	ctrltab_layout->setObjectName(QString::fromUtf8("ctrltab_layout"));

    	gb_controls = new QGroupBox(scroll_area_ctrltab_contents);
    	gb_controls->setObjectName(QString::fromUtf8("gb_controls"));

    	gridLayout_controls = new QGridLayout(gb_controls);
    	gridLayout_controls->setObjectName(QString::fromUtf8("gridLayout_controls"));

    	//lbl_slider1 = new QLabel(gb_controls);
    	//lbl_slider1->setObjectName(QString::fromUtf8("lbl_slider1"));

    	//gridLayout_controls->addWidget(lbl_slider1, 0, 0, 1, 1);

    	//horizontalSlider = new QSlider(gb_controls);
    	//horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
    	//horizontalSlider->setOrientation(Qt::Horizontal);

    	//gridLayout_controls->addWidget(horizontalSlider, 0, 1, 1, 1);

    	//lbl_slider2 = new QLabel(gb_controls);
    	//lbl_slider2->setObjectName(QString::fromUtf8("lbl_slider2"));

    	//gridLayout_controls->addWidget(lbl_slider2, 1, 0, 1, 1);

    	//horizontalSlider_2 = new QSlider(gb_controls);
    	//horizontalSlider_2->setObjectName(QString::fromUtf8("horizontalSlider_2"));
    	//horizontalSlider_2->setOrientation(Qt::Horizontal);

    	//gridLayout_controls->addWidget(horizontalSlider_2, 1, 1, 1, 1);

    	// add all to tab1
    	ctrltab_layout->addWidget(gb_controls);
    	scroll_area_ctrltab->setWidget(scroll_area_ctrltab_contents);
    	layout_tab1->addWidget(scroll_area_ctrltab);
    	tabWidget->addTab(tab_ctrl, QString());

    	//Info Tab
    	tab_info = new QWidget();
    	tab_info->setObjectName(QString::fromUtf8("tab_info"));

    	layout_tab2 = new QVBoxLayout(tab_info);
    	layout_tab2->setObjectName(QString::fromUtf8("layout_tab2"));

    	//Scroll Area for ctrltab
    	scroll_area_infotab = new QScrollArea(tab_info);
    	scroll_area_infotab->setObjectName(QString::fromUtf8("scroll_area_infotab"));
    	scroll_area_infotab->setWidgetResizable(true);

    	//Content Widget for scroll area
    	scroll_area_infotab_contents = new QWidget();
    	scroll_area_infotab_contents->setObjectName(QString::fromUtf8("scroll_area_infotab_contents"));
    	infotab_layout = new QVBoxLayout(scroll_area_infotab_contents);
    	infotab_layout->setObjectName(QString::fromUtf8("infotab_layout"));

    	//Groupbox General
    	gb_general = new QGroupBox(scroll_area_infotab_contents);
    	gb_general->setObjectName(QString::fromUtf8("gb_general"));
    	layout_general = new QVBoxLayout(gb_general);
    	layout_general->setObjectName(QString::fromUtf8("layout_general"));
    	label_general = new QLabel(gb_general);
    	label_general->setObjectName(QString::fromUtf8("label_general"));

    	layout_general->addWidget(label_general);
    	infotab_layout->addWidget(gb_general);

    	//Groupbox Image
    	gb_image = new QGroupBox(scroll_area_infotab_contents);
    	gb_image->setObjectName(QString::fromUtf8("gb_image"));
    	layout_image = new QVBoxLayout(gb_image);
    	layout_image->setObjectName(QString::fromUtf8("layout_image"));
    	label_image = new QLabel(gb_image);
    	label_image->setObjectName(QString::fromUtf8("label_image"));

    	layout_image->addWidget(label_image);
    	infotab_layout->addWidget(gb_image);

    	//Groupbox Media
    	gb_media = new QGroupBox(scroll_area_infotab_contents);
    	gb_media->setObjectName(QString::fromUtf8("gb_media"));
    	layout_media = new QVBoxLayout(gb_media);
    	layout_media->setObjectName(QString::fromUtf8("layout_media"));
    	label_media = new QLabel(gb_media);
    	label_media->setObjectName(QString::fromUtf8("label_media"));

    	layout_media->addWidget(label_media);
    	infotab_layout->addWidget(gb_media);

    	//Add all to tab2
    	scroll_area_infotab->setWidget(scroll_area_infotab_contents);
    	layout_tab2->addWidget(scroll_area_infotab);
    	tabWidget->addTab(tab_info, QString());

    	//add tabwidget to main
    	layout_main->addWidget(tabWidget);

    	retranslateUi(DockWidgetV4L2);

    	tabWidget->setCurrentIndex(0);

    } // setupUi

    void retranslateUi(QWidget *DockWidgetV4L2)
    {
    	DockWidgetV4L2->setWindowTitle(QApplication::translate("DockWidgetV4L2", "Form", 0, QApplication::UnicodeUTF8));
    	gb_controls->setTitle(QApplication::translate("DockWidgetV4L2", "Device Info", 0, QApplication::UnicodeUTF8));
    	//lbl_slider1->setText(QApplication::translate("DockWidgetV4L2", "lbl_slider1", 0, QApplication::UnicodeUTF8));
    	//lbl_slider2->setText(QApplication::translate("DockWidgetV4L2", "lbl_slider2", 0, QApplication::UnicodeUTF8));
    	tabWidget->setTabText(tabWidget->indexOf(tab_ctrl), QApplication::translate("DockWidgetV4L2", "Control", 0, QApplication::UnicodeUTF8));
    	gb_general->setTitle(QApplication::translate("DockWidgetV4L2", "General Information", 0, QApplication::UnicodeUTF8));
    	label_general->setText(QApplication::translate("DockWidgetV4L2", "label_general", 0, QApplication::UnicodeUTF8));
    	gb_image->setTitle(QApplication::translate("DockWidgetV4L2", "Image Information", 0, QApplication::UnicodeUTF8));
    	label_image->setText(QApplication::translate("DockWidgetV4L2", "label_image", 0, QApplication::UnicodeUTF8));
    	gb_media->setTitle(QApplication::translate("DockWidgetV4L2", "Media Information", 0, QApplication::UnicodeUTF8));
    	label_media->setText(QApplication::translate("DockWidgetV4L2", "label_media", 0, QApplication::UnicodeUTF8));
    	tabWidget->setTabText(tabWidget->indexOf(tab_info), QApplication::translate("DockWidgetV4L2", "Info", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class DockWidgetV4L2: public Ui_DockWidgetV4L2 {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DOCKWIDGETV4L2_H
