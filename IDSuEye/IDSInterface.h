#if !defined(IDSINTERFACE_H)
#define IDSINTERFACE_H

// itom standard-stuff
#include "common/addInInterface.h"
#include "common/addInGrabber.h"

class IDSInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_INTERFACES(ito::AddInInterfaceBase)
	PLUGIN_ITOM_API

    public:

        /**
         *  Defines the plugin type (dataIO and grabber) and sets the plugins object name. If the real plugin (here: Vistek) should or must
         *  be initialized (e.g. by a Python call) with mandatory or optional parameters, please initialize both vectors m_initParamsMand
         *  and m_initParamsOpt within this constructor.
         *
         *  @param [in] parent is the plugin interface's parent object
         **/
        IDSInterface(QObject *parent = 0);
        ~IDSInterface();

        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal checkVersionConsistency();
};

#endif // #if !defined(IDSINTERFACE_H)


