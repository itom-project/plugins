#ifndef DIALOGSERIALIO_H
#define DIALOGSERIALIO_H

#include "common/sharedStructures.h"

#include "ui_dialogDispWindow.h"
#include <QtGui>
#include <qdialog.h>

class dialogDispWindow : public QDialog 
{
    Q_OBJECT

    private:
        Ui::dialogDispWindow ui;
        void *m_pWindow;

    public:
        dialogDispWindow(const void *prjWindow);
        ~dialogDispWindow();
        int setVals(QMap<QString, ito::Param> *params, const int numImages);
        int getVals(QMap<QString, ito::Param> *params);

    public slots:
        void on_pushButtonSet_clicked(void);
//        void on_slider_image_valueChanged();
        void on_horizontalSlider_valueChanged();
        void on_comboBox_orientation_currentIndexChanged(int index);
        void on_comboBox_color_currentIndexChanged(int index);
        void on_checkBox_gamma_stateChanged(int state);
};

#endif
