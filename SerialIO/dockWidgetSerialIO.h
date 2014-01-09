#ifndef DOCKWIDGETSERIALIO_H
#define DOCKWIDGETSERIALIO_H

#include "common/sharedStructures.h"
#include "common/sharedStructuresQt.h"
#include "ui_dockWidgetSerialIO.h"

#include <QtGui>
#include <qwidget.h>
#include <qmap.h>
#include <qstring.h>
#include <qbytearray.h>

class DockWidgetSerialIO : public QWidget
{
    Q_OBJECT

    public:
        DockWidgetSerialIO(QMap<QString, ito::Param> params, int uniqueID);
        ~DockWidgetSerialIO() {};

    private:
        Ui::DockWidgetSerialIO ui;

    signals:

    public slots:
        void valuesChanged(QMap<QString, ito::Param> params);
//        void uniqueIDChanged(const int uniqueID);
        void serialLog(QByteArray data, QByteArray endline, const char InOutChar);

    private slots:
        void on_ClrButton_clicked();
};

#endif
