#include "OphirLMMeasurement.h"
#include <string>
#include <map>

#include <comutil.h>
#include <comdef.h>

namespace{
    class AutoBstr
    {
    public:
        AutoBstr() : m_str(nullptr) {}
        AutoBstr(const std::wstring& wstring) : m_str(SysAllocStringLen(wstring.data(), static_cast<UINT>(wstring.size()))) {}
        ~AutoBstr() { SysFreeString(m_str); }

        BSTR* OutParam() { return &m_str; }
        BSTR InParam() { return m_str; }

        std::wstring str() const { return std::wstring(m_str, SysStringLen(m_str)); }
        const wchar_t* c_str() const { return m_str; }

    private:
        // noncopyable
        AutoBstr(AutoBstr&);
        void operator=(AutoBstr&);

    private:
        BSTR m_str;
    };

    template<class T>
    class SafeArrayAccessor
    {
    public:
        explicit SafeArrayAccessor(SAFEARRAY* safearray)
            : m_safearray(safearray)
            , m_data(nullptr)
            , m_size(0)
        {
            long lbound = 0, ubound = -1;
            SafeArrayGetLBound(safearray, 1, &lbound);
            SafeArrayGetUBound(safearray, 1, &ubound);
            m_size = ubound - lbound + 1;
            SafeArrayAccessData(m_safearray, reinterpret_cast<void**>(&m_data));
        }

        ~SafeArrayAccessor() { SafeArrayUnaccessData(m_safearray); }

        T* data() const { return m_data; }
        long size() const { return m_size; }

        T& operator[](long i) { return data()[i]; }
        const T& operator[](long i) const { return data()[i]; }

    private:
        // noncopyable
        SafeArrayAccessor(SafeArrayAccessor&);
        void operator=(SafeArrayAccessor&);

    private:
        SAFEARRAY* m_safearray;
        T* m_data;
        long m_size;
    };

    std::vector<std::wstring> getStrings(const VARIANT& source)
    {
        if (source.vt != (VT_ARRAY | VT_BSTR))
            throw "source is not an array of strings";

        SafeArrayAccessor<BSTR> accessor(source.parray);
        std::vector<std::wstring> result;
        for (int i = 0; i < accessor.size(); ++i)
            result.push_back(std::wstring(accessor[i], SysStringLen(accessor[i])));
        return result;
    }

    std::vector<double> getDoubles(const VARIANT& source)
    {
        if (source.vt != (VT_ARRAY | VT_R8))
            throw "source is not an array of doubles";

        SafeArrayAccessor<double> accessor(source.parray);
        std::vector<double> result(accessor.data(), accessor.data() + accessor.size());
        return result;
    }

    std::vector<long> getLongs(const VARIANT& source)
    {
        if (source.vt != (VT_ARRAY | VT_I4))
            throw "source is not an array of longs";

        SafeArrayAccessor<long> accessor(source.parray);
        std::vector<long> result(accessor.data(), accessor.data() + accessor.size());
        return result;
    }

    std::vector<OphirLMMeasurement::Status> getStatuses(std::vector<long> source)
    {
        std::vector<OphirLMMeasurement::Status> result;
        for (size_t i = 0; i < source.size(); ++i)
            result.push_back(static_cast<OphirLMMeasurement::Status>(source[i]));
        return result;
    }

    bool GetBool(VARIANT_BOOL varBool)
    {
        VARIANT_BOOL v_bool = VARIANT_FALSE;
        v_bool = varBool;
        return v_bool == VARIANT_TRUE;
    }

    std::tm getTm(const DATE& date)
    {
        std::tm result;
        UDATE udate;
        HRESULT hr = VarUdateFromDate(date, 0, &udate);
        if (FAILED(hr)) _com_issue_error(hr);
        result.tm_sec = 0;   // seconds after the minute - [0, 60] including leap second
        result.tm_min = 0;   // minutes after the hour - [0, 59]
        result.tm_hour = 0;  // hours since midnight - [0, 23]
        result.tm_mday = udate.st.wDay;  // day of the month - [1, 31]
        result.tm_mon = udate.st.wMonth - 1;   // months since January - [0, 11]
        result.tm_year = udate.st.wYear - 1900;  // years since 1900
        result.tm_wday = udate.st.wDayOfWeek;  // days since Sunday - [0, 6]
        result.tm_yday = udate.wDayOfYear - 1;  // days since January 1 - [0, 365]
        result.tm_isdst = 0; // daylight savings time flag
        return result;
    }

    typedef std::map< OphirLMMeasurement::Status ,std::wstring > Status_txt;
    const Status_txt::value_type rawData[] = {
        Status_txt::value_type(OphirLMMeasurement::ok, L"ok"),
        Status_txt::value_type(OphirLMMeasurement::overrange, L"overrange"),
        Status_txt::value_type(OphirLMMeasurement::wayOverrange, L"wayOverrange"),
        Status_txt::value_type(OphirLMMeasurement::missing,L"missing"),
        Status_txt::value_type(OphirLMMeasurement::energyReset, L"energyReset"),
        Status_txt::value_type(OphirLMMeasurement::energyWaiting, L"energyWaiting"),
        Status_txt::value_type(OphirLMMeasurement::energySumming, L"energySumming"),
        Status_txt::value_type(OphirLMMeasurement::energyTimeout, L"energyTimeout"),
        Status_txt::value_type(OphirLMMeasurement::energyPeakOver, L"energyPeakOver"),
        Status_txt::value_type(OphirLMMeasurement::energyOver, L"energyOver"),
        Status_txt::value_type(OphirLMMeasurement::xOk, L"xOk"),
        Status_txt::value_type(OphirLMMeasurement::xError, L"xError"),
        Status_txt::value_type(OphirLMMeasurement::yOk, L"yOk"),
        Status_txt::value_type(OphirLMMeasurement::yError, L"yError"),
        Status_txt::value_type(OphirLMMeasurement::sizeOk, L"sizeOk"),
        Status_txt::value_type(OphirLMMeasurement::sizeError, L"sizeError"),
        Status_txt::value_type(OphirLMMeasurement::accuracyWarning, L"accuracyWarning"),
        Status_txt::value_type(OphirLMMeasurement::settingChanged, L"settingChanged"),
    };
    const int numElems = sizeof rawData / sizeof rawData[0];
    const Status_txt status_txt(rawData, rawData + numElems);
}

OphirLMMeasurement::OphirLMMeasurement(void):
    m_coLM(__uuidof(OphirLMMeasurementLib::CoLMMeasurement)),
    m_pIConnectionPoint(0),
    m_dwEventCookie(0)
{
    SetupConnectionPoint();
}

OphirLMMeasurement::~OphirLMMeasurement(void)
{
    ShutdownConnectionPoint();
}

void OphirLMMeasurement::ScanWireless(std::vector<std::wstring>& serialNumbers)
{
    _variant_t v_serialNumbers;
    m_coLM->ScanWireless(&v_serialNumbers);
    serialNumbers = getStrings(v_serialNumbers);
}

void OphirLMMeasurement::GetKnownWirelessDevices(std::vector<std::wstring>& serialNumbers)
{
    _variant_t v_serialNumbers;
    m_coLM->GetKnownWirelessDevices(&v_serialNumbers);
    serialNumbers = getStrings(v_serialNumbers);
}

void OphirLMMeasurement::OpenWirelessDevice(std::wstring serialNumber,long& hDevice)
{
    AutoBstr ab_serialNumber(serialNumber);
    m_coLM->OpenWirelessDevice(ab_serialNumber.InParam(), &hDevice);
}

void OphirLMMeasurement::ScanUSB(std::vector<std::wstring>& serialNumbers)
{
    _variant_t v_serialNumbers;
    m_coLM->ScanUSB(&v_serialNumbers);
    serialNumbers = getStrings(v_serialNumbers);
}

void OphirLMMeasurement::OpenUSBDevice(std::wstring serialNumber,long& hDevice)
{
    AutoBstr ab_serialNumber(serialNumber);
    m_coLM->OpenUSBDevice(ab_serialNumber.InParam(), &hDevice);
}

void OphirLMMeasurement::ResetDevice(long hDevice)
{
    m_coLM->ResetDevice(hDevice);
}

void OphirLMMeasurement::ResetAllDevices()
{
    m_coLM->ResetAllDevices();
}

void OphirLMMeasurement::Close(long hDevice)
{
    m_coLM->Close(hDevice);
}

void OphirLMMeasurement::CloseAll()
{
    m_coLM->CloseAll();
}

//------------------------
// General Information and Diagnostics
//------------------------
void OphirLMMeasurement::GetVersion(long& version)
{
    m_coLM->GetVersion(&version);
}

void OphirLMMeasurement::GetDeviceInfo(long hDevice, std::wstring& deviceName, std::wstring& romVersion, std::wstring& serialNumber)
{
    AutoBstr ab_deviceName, ab_romVersion, ab_deviceSN;
    m_coLM->GetDeviceInfo(hDevice, ab_deviceName.OutParam(), ab_romVersion.OutParam(), ab_deviceSN.OutParam());
    deviceName = ab_deviceName.str();
    romVersion = ab_romVersion.str();
    serialNumber = ab_deviceSN.str();
}

void OphirLMMeasurement::GetDeviceCalibrationDueDate(long hDevice, std::tm& dueDate)
{
    DATE date;
    m_coLM->GetDeviceCalibrationDueDate(hDevice, &date);
    dueDate = getTm(date);
}

void OphirLMMeasurement::GetDriverVersion(std::wstring& info)
{
    AutoBstr ab_info;
    m_coLM->GetDriverVersion(ab_info.OutParam());
    info = ab_info.str();
}

void OphirLMMeasurement::GetErrorFromCode(HRESULT errorCode, std::wstring& errorString)
{
    AutoBstr ab_errorString;
    m_coLM->GetErrorFromCode(errorCode, ab_errorString.OutParam());
    errorString = ab_errorString.str();
}

void OphirLMMeasurement::GetSensorInfo(long hDevice, long channel, std::wstring& serialNumber, std::wstring& headType, std::wstring& headName)
{
    AutoBstr  ab_headSN, ab_headType, ab_headName;
    m_coLM->GetSensorInfo(hDevice ,channel, ab_headSN.OutParam(), ab_headType.OutParam(), ab_headName.OutParam());
    serialNumber = ab_headSN.str();
    headType = ab_headType.str();
    headName = ab_headName.str();
}

void OphirLMMeasurement::GetSensorCalibrationDueDate(long hDevice, long channel, std::tm& dueDate)
{
    DATE date;
    m_coLM->GetSensorCalibrationDueDate(hDevice, channel, &date);
    dueDate = getTm(date);
}

void OphirLMMeasurement::IsSensorExists(long hDevice, long channel, bool& exists)
{
    VARIANT_BOOL v_exists;
    m_coLM->IsSensorExists(hDevice, channel, &v_exists);
    exists = GetBool(v_exists);
}

//------------------------
// Head Configuration
//------------------------
void OphirLMMeasurement::AddWavelength(long hDevice, long channel, long wavelength)
{
    m_coLM->AddWavelength(hDevice, channel, wavelength);
}

void OphirLMMeasurement::DeleteWavelength(long hDevice, long channel, long wlIndex)
{
    m_coLM->DeleteWavelength(hDevice, channel, wlIndex);
}

void OphirLMMeasurement::ModifyWavelength(long hDevice, long channel, long wlIndex, long wavelength)
{
    m_coLM->ModifyWavelength(hDevice, channel, wlIndex, wavelength);
}

void OphirLMMeasurement::GetWavelengths(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    _variant_t v_options;
    m_coLM->GetWavelengths(hDevice, channel, &index, &v_options);
    options= getStrings(v_options);
}

void OphirLMMeasurement::GetWavelengthsExtra(long hDevice, long channel, bool& modifiable, long& minWavelength, long& maxWavelength)
{
    VARIANT_BOOL v_modifiable;
    m_coLM->GetWavelengthsExtra(hDevice, channel, &v_modifiable, &minWavelength, &maxWavelength);
    modifiable = GetBool(v_modifiable);
}

void OphirLMMeasurement::SetWavelength(long hDevice, long channel, long index)
{
    m_coLM->SetWavelength(hDevice, channel, index);
}

void OphirLMMeasurement::GetDiffuser(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    _variant_t v_options;
    m_coLM->GetDiffuser(hDevice, channel, &index, &v_options);
    options = getStrings(v_options);
}

void OphirLMMeasurement::SetDiffuser(long hDevice, long channel, long index)
{
    m_coLM->SetDiffuser(hDevice, channel, index);
}

void OphirLMMeasurement::GetFilter(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetFilter(hDevice, channel, &index, &v_options);
    options = getStrings(v_options);
}

void OphirLMMeasurement::SetFilter(long hDevice, long channel, long index)
{
    m_coLM->SetFilter(hDevice, channel, index);
}

void OphirLMMeasurement::GetMeasurementMode(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetMeasurementMode(hDevice, channel, &index, &v_options);
    options = getStrings(v_options);
}

void OphirLMMeasurement::SetMeasurementMode(long hDevice, long channel, long index)
{
    m_coLM->SetMeasurementMode(hDevice, channel, index);
}

void OphirLMMeasurement::GetPulseLengths(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetPulseLengths(hDevice, channel, &index, &v_options);
    options = getStrings(v_options);
}

void OphirLMMeasurement::SetPulseLength(long hDevice, long channel, long index)
{
    m_coLM->SetPulseLength(hDevice, channel, index);
}

void OphirLMMeasurement::GetRanges(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetRanges(hDevice, channel, &index, &v_options);
    options= getStrings(v_options);
}

void OphirLMMeasurement::SetRange(long hDevice, long channel, long index)
{
    m_coLM->SetRange(hDevice, channel, index);
}

void OphirLMMeasurement::GetThreshold(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetThreshold(hDevice, channel, &index, &v_options);
    options= getStrings(v_options);
}

void OphirLMMeasurement::SetThreshold(long hDevice, long channel, long index)
{
    m_coLM->SetThreshold(hDevice, channel, index);
}

void OphirLMMeasurement::GetPulsedPowerPulseWidth(long hDevice, long channel, long& value_ms, long& min_ms, long& max_ms)
{
    m_coLM->GetPulsedPowerPulseWidth(hDevice, channel, &value_ms, &min_ms, &max_ms);
}

void OphirLMMeasurement::SetPulsedPowerPulseWidth(long hDevice, long channel, long value_ms)
{
    m_coLM->SetPulsedPowerPulseWidth(hDevice, channel, value_ms);
}

void OphirLMMeasurement::GetLowFreqPowerPulseFreq(long hDevice, long channel, double& value, double& min, double& max)
{
    m_coLM->GetLowFreqPowerPulseFreq(hDevice, channel, &value, &min, &max);
}

void OphirLMMeasurement::SetLowFreqPowerPulseFreq(long hDevice, long channel, double value)
{
    m_coLM->SetLowFreqPowerPulseFreq(hDevice, channel, value);
}

void OphirLMMeasurement::SaveSettings(long hDevice, long channel)
{
    m_coLM->SaveSettings(hDevice, channel);
}

//------------------------
// Trigger Settings (for Pulsar)
//------------------------
void OphirLMMeasurement::GetExtTrigModes(long hDevice, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetExtTrigModes(hDevice, &index, &v_options);
    options = getStrings(v_options);
}

void OphirLMMeasurement::SetExtTrigMode(long hDevice, long index)
{
    m_coLM->SetExtTrigMode(hDevice, index);
}

void OphirLMMeasurement::GetExtTrigOnOff(long hDevice, long channel, long& index, std::vector<std::wstring>& options)
{
    variant_t v_options;
    m_coLM->GetExtTrigOnOff(hDevice, channel, &index, &v_options);
    options = getStrings(v_options);
}

void OphirLMMeasurement::SetExtTrigOnOff(long hDevice, long channel, long index)
{
    m_coLM->SetExtTrigOnOff(hDevice, channel, index);
}

void OphirLMMeasurement::GetExtTrigWindowTime(long hDevice, long& extTrigWindowTime)
{
    m_coLM->GetExtTrigWindowTime(hDevice, &extTrigWindowTime);
}

void OphirLMMeasurement::SetExtTrigWindowTime(long hDevice, long extTrigWindowTime)
{
    m_coLM->SetExtTrigWindowTime(hDevice, extTrigWindowTime);
}

//------------------------
// Measurement Delivery
//------------------------
void OphirLMMeasurement::ConfigureStreamMode(long hDevice, long channel, long mode, long nOff)
{
    m_coLM->ConfigureStreamMode(hDevice, channel, mode, nOff);
}

void OphirLMMeasurement::StartStream(long hDevice, long channel)
{
    m_coLM->StartStream(hDevice, channel);
}

void OphirLMMeasurement::GetData(long hDevice, long channel, std::vector<double>& arrayValue, std::vector<double>& arrayTimestamp, std::vector<OphirLMMeasurement::Status>& arrayStatus)
{
    _variant_t v_values;
    _variant_t v_timestamps;
    _variant_t v_statuses;

    m_coLM->GetData(hDevice, channel, &v_values, &v_timestamps, &v_statuses);

    arrayValue = getDoubles(v_values);
    arrayTimestamp = getDoubles(v_timestamps);
    arrayStatus = getStatuses(getLongs(v_statuses));
}

void OphirLMMeasurement::StopStream(long hDevice, long channel)
{
    m_coLM->StopStream(hDevice, channel);
}

void OphirLMMeasurement::StopAllStreams()
{
    m_coLM->StopAllStreams();
}

//------------------------
// Legacy Methods
//------------------------
void OphirLMMeasurement::Read(long hDevice, std::wstring& reply)
{
    AutoBstr ab_reply;
    m_coLM->Read(hDevice, ab_reply.OutParam());
    reply = ab_reply.str();
}

void OphirLMMeasurement::Write(long hDevice, std::wstring command)
{
    AutoBstr ab_command(command);
    m_coLM->Write(hDevice, ab_command.InParam());
}

//------------------------
// COM events
//------------------------
void OphirLMMeasurement::RegisterPlugAndPlay (std::function<void()> plugAndPlayCallback)
{
    m_plugAndPlayCallback = plugAndPlayCallback;
}

void OphirLMMeasurement::RegisterDataReady (std::function<void(long hDevice, long channel)> dataReadyCallback)
{
    m_dataReadyCallback = dataReadyCallback;
}

HRESULT OphirLMMeasurement::OnCOMEventFiring(
    DISPID dispidMember,
    REFIID riid,
    LCID lcid,
    WORD wFlags,
    DISPPARAMS* pdispparams,
    VARIANT* pvarResult,
    EXCEPINFO* pexcepinfo,
    UINT* puArgErr
    )
{
    if (dispidMember==1)// Data ready
    {
        if(m_dataReadyCallback && pdispparams->cArgs >=2)
            m_dataReadyCallback(pdispparams->rgvarg[1].lVal, pdispparams->rgvarg[0].lVal);
    }
    else if(dispidMember==2)// Plug and play
        if (m_plugAndPlayCallback)
            m_plugAndPlayCallback();

    return S_OK;
}

void OphirLMMeasurement::SetupConnectionPoint()
{
    IConnectionPointContainer*    pIConnectionPointContainerTemp = NULL;
    IUnknown*                    pIUnknown = NULL;

    this -> QueryInterface(IID_IUnknown, (void**)&pIUnknown);

    if (pIUnknown)
    {
        m_coLM -> QueryInterface (IID_IConnectionPointContainer, (void**)&pIConnectionPointContainerTemp);

        if (pIConnectionPointContainerTemp)
        {
            pIConnectionPointContainerTemp -> FindConnectionPoint(__uuidof(OphirLMMeasurementLib::_ICoLMMeasurementEvents), &m_pIConnectionPoint);
            pIConnectionPointContainerTemp -> Release();
            pIConnectionPointContainerTemp = NULL;
        }

        if (m_pIConnectionPoint)
        {
            m_pIConnectionPoint -> Advise(pIUnknown, &m_dwEventCookie);
        }

        pIUnknown -> Release();
        pIUnknown = NULL;
    }
}

void OphirLMMeasurement::ShutdownConnectionPoint()
{
    if (m_pIConnectionPoint)
    {
        m_pIConnectionPoint -> Unadvise(m_dwEventCookie);
        m_dwEventCookie = 0;
        m_pIConnectionPoint -> Release();
        m_pIConnectionPoint = NULL;
    }
}

std::wstring OphirLMMeasurement::StatusString(Status status)
{
    auto search = status_txt.find(status);
    if(search != status_txt.end())
        return search->second ;

    return L"unknown";
}
