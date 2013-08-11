#ifndef VISTEKINTERFACE_H
#define VISTEKINTERFACE_H

#include "common/addInGrabber.h"

#include <qsharedpointer.h>
#include <QTimerEvent>
#include <qmutex.h>

class VistekInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
        Q_INTERFACES(ito::AddInInterfaceBase)

    public:
        VistekInterface(QObject *parent = 0);
        ~VistekInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    protected:

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

		//! auto-increment, static instance counter for all dummy-grabber instances
		static int m_instCounter;
};

#endif // VISTEKINTERFACE_H