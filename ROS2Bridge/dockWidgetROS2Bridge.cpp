// DockWidgetROS2Bridge.cpp
#include "dockWidgetROS2Bridge.h"

DockWidgetROS2Bridge::DockWidgetROS2Bridge(QMap<QString, ito::Param> params, int uniqueID, QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    // 设置插件名称和ID


    // 连接按钮点击信号到槽函数
    connect(ui.btnRefreshTopics, &QPushButton::clicked, this, &DockWidgetROS2Bridge::onRefreshButtonClicked);
}

DockWidgetROS2Bridge::~DockWidgetROS2Bridge()
{
}

void DockWidgetROS2Bridge::onRefreshButtonClicked()
{
    emit requestRefreshTopics();
}

void DockWidgetROS2Bridge::updateTopicsList(const QStringList &topics, const QStringList &types)
{
    ui.tableTopics->clearContents();
    ui.tableTopics->setRowCount(topics.size());

    for (int i = 0; i < topics.size(); ++i)
    {
        ui.tableTopics->setItem(i, 0, new QTableWidgetItem(topics.at(i)));
        ui.tableTopics->setItem(i, 1, new QTableWidgetItem(types.at(i)));
    }
}

void DockWidgetROS2Bridge::valuesChanged(QMap<QString, ito::Param> params)
{
    // 根据需要更新UI元素
}
