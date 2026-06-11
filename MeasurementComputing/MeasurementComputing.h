#ifndef MeasurementComputing_H
#define MeasurementComputing_H

#include "common/addInInterface.h"
#include "DataObject/dataobj.h"
#include <qobject.h>
#include <qsharedpointer.h>
#include <qbytearray.h>
#include <qlibrary.h>
#include "cbw.h"        // include cbw.h library functions for AD-DA

#include "dialogMeasurementComputing.h"

//----------------------------------------------------------------------------------------------------------------------------------
class MeasurementComputing : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~MeasurementComputing();
        MeasurementComputing();
        //TaskHandle taskHandle;
        //bool taskCreated;                            // 0: task has not been created, 1: task has successfully been created
        //bool taskStarted;                            // 0: task has not been started, 1: task has successfully been started
        //ito::RetVal errorMSG(int32 error);            // returns the error according to NI-DAQmx error handling


    public:
        friend class MeasurementComputingInterface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }        //!< indicates that this plugin has got a configuration dialog


    private:
        ito::DataObject m_data;
        int m_boardNum;
        int m_devNum;
        int m_input_range;
        int m_output_range;
        int m_temperature_scale;
        int m_ad_resolution;
        int m_da_resolution;
        float m_revision;
        float m_vxdrevnum;

        HGLOBAL m_pBuffer;
        HGLOBAL m_pOutputBuffer;

        int m_numInputChannels;
        int m_numInputSamples;

        ito::RetVal m_acquisitionResult;
        bool m_dataAcquired;

        enum SyncParams {
            sMData = 0x0001,
            sOutputData = 0x0002,

            sAll = sMData | sOutputData
        };

        ito::RetVal getErrStr(const int error, const QString &value);
        ito::RetVal synchronizeSettings(int what = sAll);
        ito::RetVal getVIn(ito::ParamBase &channel, ito::ParamBase &voltage);
        ito::RetVal setVOut(ito::ParamBase &channel, ito::ParamBase &voltage);
        ito::RetVal getTIn(ito::ParamBase &channel, ito::ParamBase &temperature);
        ito::RetVal getCIn(ito::ParamBase &channel, ito::ParamBase &counterSet, ito::ParamBase &counter);
        ito::RetVal getDIn(ito::ParamBase &port, ito::ParamBase &value);
        ito::RetVal setDOut(ito::ParamBase &port, ito::ParamBase &value);
        ito::RetVal getBitIn(ito::ParamBase &port, ito::ParamBase &bit, ito::ParamBase &value);
        ito::RetVal setBitOut(ito::ParamBase &port, ito::ParamBase &bit, ito::ParamBase &value);

        int rangeCodeStringToInt(char* rangeCodeString);
        QString rangeCodeIntToString(int rangeCodeInt);

        int tempScaleStringToInt(char* tempScaleString);
        QString tempScaleIntToString(int tempScaleInt);

        int digDevTypeStringToInt(char* digDevTypeString);
        QString digDevTypeIntToString(int digDevTypeInt);

        inline int bitToInt(int bit)
        {
            int maxDigValue = 2;
            int cnt = 1;
            while (cnt < bit)
            {
                maxDigValue = maxDigValue * 2;
                cnt++;
            }
            return maxDigValue;
        }

    public slots:

        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond = NULL);

        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal close(ItomSharedSemaphore *waitConde = NULL);

        // create task
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);

        // clear task
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);

        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        ito::RetVal getVal(void *data, ItomSharedSemaphore *waitCond);
        ito::RetVal copyVal(void *dObj, ItomSharedSemaphore *waitCond);
        ito::RetVal setVal(const char *dObj, const int length, ItomSharedSemaphore *waitCond);

        ito::RetVal execFunc(const QString funcName, QSharedPointer<QVector<ito::ParamBase> > paramsMand, QSharedPointer<QVector<ito::ParamBase> > paramsOpt, QSharedPointer<QVector<ito::ParamBase> > paramsOut, ItomSharedSemaphore *waitCond = NULL);

    private slots:

};


//----------------------------------------------------------------------------------------------------------------------------------
class MeasurementComputingInterface : public ito::AddInInterfaceBase
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    protected:

    public:
        MeasurementComputingInterface();
        ~MeasurementComputingInterface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    protected:

        int m_numberOfInstances;

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);

    signals:

    public slots:
};


#endif // MeasurementComputing_H
