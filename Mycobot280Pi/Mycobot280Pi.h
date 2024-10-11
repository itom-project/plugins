#ifndef MyCobot280Pi_H
#define MyCobot280Pi_H

#include "common/addInInterface.h"
#include "dockWidgetMyCobot280Pi.h"    // 控制对话框

#include <qsharedpointer.h>
#include <qmetatype.h>
#include <QTcpSocket>
#include <QString>
#include <QtWidgets/QDockWidget>


//----------------------------------------------------------------------------------------------------------------------------------
/** @class MyCobot280PiInterface
*   @brief MyCobot280Pi functionality
*
*   AddIn Interface for the MyCobot280Pi class.
*/
class MyCobot280PiInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        MyCobot280PiInterface(QObject *parent = 0);
        ~MyCobot280PiInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);    //!< 创建一个新的 MyCobot280Pi 实例，并赋予其唯一标识符
        bool hasDockWidget() { return true; }


    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class MyCobot280Pi
*   @brief MyCobot280Pi functionality
*
*   The MyCobot280Pi-Class is used to control the MyCobot robot via TCP socket commands.
*/
class MyCobot280Pi : public ito::AddInActuator
{
    Q_OBJECT

    public:
        friend class MyCobot280PiInterface;
        int hasConfDialog(void) { return 1; } //!< 指示此插件是否具有配置对话框
        ito::RetVal connectToSocket();
        ito::RetVal sendSocketData(const QString &data);
        ito::RetVal updateStatus(); 

    protected:
        ~MyCobot280Pi(); //!< 析构函数
        MyCobot280Pi();    //!< 构造函数

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

    private:
        QString m_host;  //!< 存储主机名或 IP 地址
        int m_port;      //!< 存储端口号
        QTcpSocket *m_socket;  //!< Qt 的 socket 对象
        bool m_async;
        int m_nrOfAxes;
        QVector<double> m_targetPos;  //!< 存储每个轴的目标位置

    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal getStatus(QSharedPointer<QVector<int>> status, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double>> pos, ItomSharedSemaphore *waitCond);
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);

    private slots:
        void dockWidgetVisibilityChanged(bool visible); // 重写自 AddInBase
};

#endif // MyCobot280Pi_H
