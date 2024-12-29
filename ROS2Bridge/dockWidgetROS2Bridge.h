// DockWidgetROS2Bridge.h
#ifndef DOCKWIDGETROS2BRIDGE_H
#define DOCKWIDGETROS2BRIDGE_H

#include "ui_dockWidgetROS2Bridge.h"
#include <QWidget>
#include <QMap>
#include "common/sharedStructures.h"

class DockWidgetROS2Bridge : public QMainWindow
{
    Q_OBJECT

public:
    DockWidgetROS2Bridge(QMap<QString, ito::Param> params, int uniqueID, QWidget *parent = nullptr);
    ~DockWidgetROS2Bridge();

public slots:
    void valuesChanged(QMap<QString, ito::Param> params);
    void updateTopicsList(const QStringList &topics, const QStringList &types);

signals:
    void requestRefreshTopics();

private slots:
    void onRefreshButtonClicked();

private:
    Ui::DockWidgetROS2Bridge ui;
};

#endif // DOCKWIDGETROS2BRIDGE_H
