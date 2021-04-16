/* ********************************************************************
    Template for a camera / grabber plugin for the software itom
    
    You can use this template, use it in your plugins, modify it,
    copy it and distribute it without any license restrictions.
*********************************************************************** */

#ifndef SPIDER8_H
#define SPIDER8_H

#include "common/addInInterface.h"
#include <qsharedpointer.h>
#include <qmap.h>
#include <DataObject/dataobj.h>

#include "dialogSpider8.h"

#if (USEOPENMP)
    #define USEOMP 1
#else
    #define USEOMP 0
#endif

class Spider8Funcs; //forward declaration

extern int NTHREADS;

//----------------------------------------------------------------------------------------------------------------------------------
/**
*\class    Spider8Interface 
*
*\brief    Interface-Class for Spider8-Class
*
*\sa    AddInDataIO, Spider8
*
*/
class Spider8Interface : public ito::AddInInterfaceBase
{
    Q_OBJECT
#if QT_VERSION >=  QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID "ito.AddInInterfaceBase" )
#endif
    Q_INTERFACES(ito::AddInInterfaceBase)
    PLUGIN_ITOM_API

    public:
        Spider8Interface();
        ~Spider8Interface();
        ito::RetVal getAddInInst(ito::AddInBase **addInInst);

    private:
        ito::RetVal closeThisInst(ito::AddInBase **addInInst);
};

//----------------------------------------------------------------------------------------------------------------------------------
/**
*\class     Spider8Channel
\brief     Private class holding Spider8 channel parameters

*/
class Spider8Channel
{
    public:
        // default constructor for class compilation only, must not be used irl
        Spider8Channel()
            : m_num(-1), m_trigger(0), m_filter(0), m_type(0), m_mode(0), m_range(0), m_measMode(0), m_scale(1.0) { setScaleFactors(); }
        Spider8Channel(const int chaNum, const int type, const QMap<int, QVector<int> > &modRanges)
            : m_num(chaNum), m_trigger(0), m_filter(0), m_type(type),
            m_allowedModRanges(modRanges), m_mode(0), m_range(0), m_measMode(0), m_scale(1.0) { setScaleFactors(); }
        ~Spider8Channel() {}
        void setScaleFactors(void)
        {
            mapSFactors.insert(700, 0.003 / 25000.0);         // 0.003 V/V
            mapSFactors.insert(701, 0.012 / 25000.0);         // 0.012 V/V
            mapSFactors.insert(702, 0.125 / 25000.0);         // 0.125 V/V
            mapSFactors.insert(703, 0.500 / 25000.0);         // 0.500 V/V
            mapSFactors.insert(710, 0.1 / 25000.0);           // 0.1 V
            mapSFactors.insert(711, 1.0 / 25000.0);           // 1.0 V
            mapSFactors.insert(712, 10.0 / 25000.0);          // 10.0 V
            mapSFactors.insert(720, 0.020 / 25000.0);         // 0.020 A
            mapSFactors.insert(721, 0.200 / 25000.0);         // 0.200 A
            mapSFactors.insert(730, 400.0 / 25000.0);         // 400.0 ohm
            mapSFactors.insert(731, 4000.0 / 25000.0);        // 4000.0 ohm
            mapSFactors.insert(744, 100.0 / 25000.0);         // 100.0 Hz
            mapSFactors.insert(743, 1000.0 / 25000.0);        // 1000.0 Hz  (1 kHz)
            mapSFactors.insert(742, 10000.0 / 25000.0);       // 10000.0 Hz (10 kHz)
            mapSFactors.insert(741, 100000.0 / 25000.0);      // 100000.0 Hz (100 kHz)
            mapSFactors.insert(740, 1000000.0 / 25000.0);     // 1000000.0 Hz (1 MHz)
            mapSFactors.insert(750, 0.010 / 25000.0);         // 0.010 s
            mapSFactors.insert(751, 0.100 / 25000.0);         // 0.100 s
            mapSFactors.insert(752, 1.0 / 25000.0);           // 1.0 s
            mapSFactors.insert(753, 10.0 / 25000.0);          // 10.0 s
            mapSFactors.insert(754, 100.0 / 25000.0);         // 100.0 s
            mapSFactors.insert(760, 1.0 / 25000.0);           // 1.0 (counter)
            mapSFactors.insert(761, 0.010 / 25000.0);         // 100.0 (counter / 100)
            mapSFactors.insert(765, 1.0 / 25000.0);           // 1.0 (digit)
            mapSFactors.insert(770, 1.0 / 25000.0);           // 1.0 (temperature)
        }

        int m_num;
        int m_trigger;
        int m_filter;
        int m_type;
        QMap<int, QVector<int> > m_allowedModRanges;
        int m_mode;
        int m_range;
        int m_measMode;
        double m_scale;

        QMap<int, double> mapSFactors;

    private:
};

//----------------------------------------------------------------------------------------------------------------------------------
/**
*\class    Spider8

*/
class Spider8 : public ito::AddInDataIO
{
    Q_OBJECT

    protected:
        ~Spider8();
        Spider8();
        
    public:
        friend class Spider8Interface;
        const ito::RetVal showConfDialog(void);
        int hasConfDialog(void) { return 1; }; //!< indicates that this plugin has got a configuration dialog
        enum status {
            noData          = 0, //!< no valid measurement data is available
            waiting         = 1, //!< performing mode switch, not concluded yet
            fillPreBuffer   = 2, //!< pre-trigger buffer is being filled
            waitTrigger     = 3, //!< pre-trigger buffer is full, waiting for trigger event
            fillPostBuffer  = 4, //!< filling post-trigger buffer
            acqError        = 5, //!< acquisition terminated with an error
            acqOk           = 6, //!< acquisition terminated without an error
            chaPassive      = 7, //!< channel is passive
            chaEnabled      = 8, //!< channel measure enabled
            chaTare         = 9, //!< channel tare enabled
            chaEnabledTare  = 10 //!< channel measure and tare enabled
        };
        enum trigger {
            high     = 0, //!< trigger when above threshold
            low      = 1, //!< trigger when below threshold
            positive = 2, //!< trigger on positive edge
            negative = 3  //!< trigger on negative edge
        };
        Spider8Channel * getChannel(const int chaNum)
        {
            if (m_channels.contains(chaNum))
                return &m_channels[chaNum];
            else
                return NULL;
        }
        QMap<int, Spider8Channel> * getChannels(void)
        {
            return &m_channels;
        }

    private:
        ito::RetVal checkData(ito::DataObject *externalDataObject, int channels, int samples);

        bool m_isgrabbing; //!< Check if acquire was executed 
        ito::AddInDataIO *m_pSer;

        // These three bools are set true by the acquire method to indicate what kind of data is
        // received in the getVal method. The getVal method also resets the three bools to false
        bool m_aInIsAcquired;
        bool m_dInIsAcquired;

        bool m_dOutIsAcquired;
        
        ito::DataObject m_data;
        Spider8Funcs *m_pSpider;
        int m_baud;
        QMap<int, Spider8Channel> m_channels;

        // Read-functions
        ito::RetVal readAnalog(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        ito::RetVal readDigital(ito::DataObject *externalDataObject = NULL); /*!< Wait for acquired picture */
        
        // Write-functions
        //ito::RetVal writeAnalog(const ito::DataObject *externalDataObject = NULL);
        ito::RetVal writeDigital(const int channel, ito::DataObject *externalDataObject = NULL);
        
    public slots:
        //!< Get ADC-Parameter
        ito::RetVal getParam(QSharedPointer<ito::Param> val, ItomSharedSemaphore *waitCond);
        //!< Set ADC-Parameter
        ito::RetVal setParam(QSharedPointer<ito::ParamBase> val, ItomSharedSemaphore *waitCond);
        //!< Initialise board, load dll, allocate buffer
        ito::RetVal init(QVector<ito::ParamBase> *paramsMand, QVector<ito::ParamBase> *paramsOpt, ItomSharedSemaphore *waitCond = NULL);
        //!< Free buffer, delete board, unload dll
        ito::RetVal close(ItomSharedSemaphore *waitCond);

        //!< Start the ADC to enable acquire-commands
        ito::RetVal startDevice(ItomSharedSemaphore *waitCond);
        //!< Stop the ADC to disable acquire-commands
        ito::RetVal stopDevice(ItomSharedSemaphore *waitCond);
        //!< Softwaretrigger for the ADC
        ito::RetVal acquire(const int trigger, ItomSharedSemaphore *waitCond = NULL);
        //!< Wait for acquired picture, copy the Values to dObj of right type and size
        ito::RetVal getVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        //!< 
        ito::RetVal setVal(const char *data, const int length, ItomSharedSemaphore *waitCond = NULL);
        //!< 
        ito::RetVal copyVal(void *vpdObj, ItomSharedSemaphore *waitCond);
        
        //checkData usually need not to be overwritten (see comments in source code)
        //ito::RetVal checkData(ito::DataObject *externalDataObject = NULL);

    private slots:
        void dockWidgetVisibilityChanged(bool visible);
};

//----------------------------------------------------------------------------------------------------------------------------------
/**
*\class     Spider8Funcs
 \brief     Private class for communication with HBM Spider8 device

*/
class Spider8Funcs
{
    public:
        Spider8Funcs(ito::AddInDataIO *serial) : m_pSer(serial), m_lRdSleep(20), m_sRdSleep(20), m_rdSleep(20), m_lastIdxRead(0)
        {
            m_readBuf = QSharedPointer<char>(new char[255]);
            m_readBufLen = QSharedPointer<int>(new int);
            *m_readBufLen = 255;
        }
        ~Spider8Funcs() {}

        ito::RetVal hbmListDevices(QVector<int> &devices, QVector<QString> *devNames);
        ito::RetVal hbmListChannels(QMap<int, Spider8Channel> &channels);
        ito::RetVal hbmGetShunt(const int channel, bool &value);
        ito::RetVal hbmSetShunt(const int channel, const bool value);
        ito::RetVal hbmSetMemMngmnt(const bool firstReadClear);
        ito::RetVal hbmGetMemMngmnt(bool &firstReadClear);
        ito::RetVal hbmSetSamplingRate(double &rate);
        ito::RetVal hbmGetSamplingRate(double &rate);
        ito::RetVal hbmGetTrigger(int &channel, int &mode, int &threshold);
        ito::RetVal hbmSetTrigger(const int channel, const int mode, const int threshold);
        ito::RetVal hbmStop(void);
        ito::RetVal hbmGetStatus(int &status, const int channel);
        ito::RetVal hbmSetBaud(const int baud);
        ito::RetVal hbmGetBaud(int &baud);
        ito::RetVal hbmGetError(QString &err, int &errNum);
        ito::RetVal hbmStopAcq(QStringList &channels);
        static ito::RetVal hbmGetErrStr(const int errNum, QString &errMsg);
        ito::RetVal hbmSetChConfig(Spider8Channel &channel, const int num, const int mode, const int range);
        ito::RetVal hbmCheckChModeRange(Spider8Channel &channel, const int mode, const int range, bool &check);
        ito::RetVal hbmActivateCh(const QMap<int, Spider8Channel> &channels, const int &chnum, const int &chactmode);
        ito::RetVal hbmStartMeas(const int trigger, const int numSamp, const int cycles, const int preTrgSamples);
        ito::RetVal addActiveChannel(const int channel) { if (!m_activeChs.contains(channel)) m_activeChs.append(channel); return ito::retOk; }
        ito::RetVal delActiveChannel(const int channel) { int idx = m_activeChs.indexOf(channel);  if (idx >= 0) m_activeChs.remove(idx); return ito::retOk; }
        ito::RetVal getActiveChs(QVector<int> *&channels) { channels = &m_activeChs; return ito::retOk; }
        ito::RetVal hbmGetNumActChs(int &numChs);
        ito::RetVal hbmGetNumSamples(int &status, int &numSamples);
        ito::RetVal hbmReadData(QSharedPointer<char> data, QSharedPointer<int> dataSize, const int numValues);
        ito::RetVal hbmReadNScale(const int numChs, const int samples, double *iscales, double *voffsets, double *vscales, ito::DataObject *dataObj);
        inline ito::RetVal scaleValues(const ito::uint16 *dInPtr, ito::DataObject *dOut, const int numSamples, const int numChs, double *scales)
        {
            ito::RetVal retValue(ito::retOk);
            double *doPtr = (double*)dOut->rowPtr(0, 0);

            #if (USEOMP)
            #pragma omp parallel num_threads(NTHREADS)
            #pragma omp for schedule(guided)
            #endif
            for (int ns = 0; ns < numSamples; ns++)
            {
                for (int nc = 0; nc < numChs; nc++)
                {
                    doPtr[ns + nc * numSamples] = dInPtr[ns + nc] > 32767 ? (dInPtr[ns + nc] - 65536) * scales[nc] : dInPtr[ns + nc] * scales[nc];
                }
            }
            
            return retValue;
        }
        inline int getLastIdx() const { return m_lastIdxRead; }
        inline void resetLastIdx() { m_lastIdxRead = 0; }
        inline int getRdSleep() const { return m_rdSleep; }

    private:
        ito::AddInDataIO *m_pSer;
        int m_lRdSleep; //!< low baud rate read sleep
        int m_sRdSleep; //!< high baud rate read sleep
        int m_rdSleep;  //!< actually used read sleep
        int m_lastIdxRead;
        QSharedPointer<char> m_readBuf;
        QSharedPointer<int> m_readBufLen;
        QVector<int> m_activeChs;
};

#endif // SPIDER8_H
