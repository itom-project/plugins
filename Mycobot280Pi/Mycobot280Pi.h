#ifndef MYCOBOTCONTROL_H
#define MYCOBOTCONTROL_H

#include "common/addInInterface.h"
 //! 配置对话框
#include "dockWidgetMycobotControl.h"    //! 控制对话框

#include <qsharedpointer.h>
#include <qmetatype.h>
#include <QTcpSocket>
#include <QString>

//----------------------------------------------------------------------------------------------------------------------------------
/** @class MycobotControlInterface
*   @brief MycobotControl functionality
*
*   AddIn Interface for the MycobotControl class.
*/
class MycobotControlInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase")
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        MycobotControlInterface(QObject *parent = 0);
        ~MycobotControlInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);    //!< 创建一个新的 MycobotControl 实例，并赋予其唯一标识符
        bool hasDockWidget() { return true; }

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};

//----------------------------------------------------------------------------------------------------------------------------------
/** @class MycobotControl
*   @brief MycobotControl functionality
*
*   The MycobotControl-Class is used to control the MyCobot robot via TCP socket commands.
*/
class MycobotControl : public ito::AddInActuator
{
    Q_OBJECT

    public:
        friend class MycobotControlInterface;
        const ito::RetVal showConfDialog(void);    //!< 这是调用模态配置对话框的方法
        int hasConfDialog(void) { return 1; } //!< 指示此插件是否具有配置对话框

    protected:
        ~MycobotControl(); //!< 析构函数
        MycobotControl();    //!< 构造函数

        ito::RetVal waitForDone(const int timeoutMS = -1, const QVector<int> axis = QVector<int>() /*if empty -> all axis*/, const int flags = 0 /*for your use*/);

    private:

        
        QString m_host;  //!< 存储主机名或 IP 地址
        int m_port;      //!< 存储端口号
        QTcpSocket *m_socket;  //!< Qt 的 socket 对象
        
        QVector<double> m_targetPos;  //!< 存储每个轴的目标位置


        ito::RetVal connectToSocket();
        ito::RetVal sendSocketData(const QString &data);


    public slots:
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        ito::RetVal calib(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal calib(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const int axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setOrigin(const QVector<int> axis, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal getStatus(QSharedPointer<QVector<int> > status, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const int axis, QSharedPointer<double> pos, ItomSharedSemaphore *waitCond);
        ito::RetVal getPos(const QVector<int> axis, QSharedPointer<QVector<double> > pos, ItomSharedSemaphore *waitCond);
        ito::RetVal setPosAbs(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosAbs(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const int axis, const double pos, ItomSharedSemaphore *waitCond = nullptr);
        ito::RetVal setPosRel(const QVector<int> axis, QVector<double> pos, ItomSharedSemaphore *waitCond = nullptr);

        // ito::RetVal requestStatusAndPosition(bool sendCurrentPos, bool sendTargetPos);    //!< Slot to trigger a Status and position request

    private slots:
        void dockWidgetVisibilityChanged(bool visible); //overwritten from AddInBase
};

//----------------------------------------------------------------------------------------------------------------------------------

#endif // MYCOBOTCONTROL_H
