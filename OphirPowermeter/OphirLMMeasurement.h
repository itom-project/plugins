#pragma once

// Class OphirLMMeasurement.
// To use, add OphirLMMeasurement.h and OphirLMMeasurement.cpp to your project, and #include "OphirLMMeasurement.h"
//
// This class wraps the OphirLMMeasurement.CoLMMeasurement COM object and
// provides access to its features. To use this class, ensure COM is
// initialized and create an object of this class. The class provides member
// functions as documented in the COM object documentation, with all
// parameters translated into standard C++ types (std::wstring, std::vector).
// An enum Status is provided for the reading status and a member function
// StatusString to convert the status into a string. If you wish to to receive
// events, call the Register... functions with an appropriate callback or with
// nullptr to remove the callback. Note that the event system requires a
// functioning Windows message loop.

#include <string>
#include <vector>
#include <functional>

#import "progid:OphirLMMeasurement.CoLMMeasurement"


class OphirLMMeasurement : private IDispatch
{
public:
	OphirLMMeasurement();
	~OphirLMMeasurement();

	enum Status {
		// normal
		ok = 0, overrange = 1, wayOverrange = 2, missing = 3, energyReset = 4, energyWaiting = 5,
		energySumming = 6, energyTimeout = 7, energyPeakOver = 8, energyOver = 9,

		// pulse frequency
		frequency = 0x50000,

		// x position
		xOk = 0x10000,
		xError = 0x10001,

		// y position
		yOk = 0x20000,
		yError = 0x20001,

		// size
		sizeOk = 0x30000,
		sizeError = 0x30001,
		accuracyWarning = 0x30002,

		// events
		settingChanged = 0x40001
	};

	void ScanWireless(std::vector<std::wstring>& serialNumbers);
	void GetKnownWirelessDevices(std::vector<std::wstring>& serialNumbers);
	void OpenWirelessDevice(std::wstring serialNumber, long& hDevice);
	void ScanUSB(std::vector<std::wstring>& serialNumbers);
	void OpenUSBDevice(std::wstring serialNumber, long& hDevice);
	void ResetDevice(long hDevice);
	void ResetAllDevices();
	void Close(long hDevice);
	void CloseAll();

	// General Information and Diagnostics
	void GetVersion(long& version);
	void GetDeviceInfo(long hDevice, std::wstring& deviceName, std::wstring& romVersion, std::wstring& serialNumber);
	void GetDeviceCalibrationDueDate(long hDevice, std::tm& dueDate);
	void GetDriverVersion(std::wstring& info);
	void GetErrorFromCode(HRESULT errorCode, std::wstring& errorString);
	void GetSensorInfo(long hDevice, long channel, std::wstring& serialNumber, std::wstring& headType, std::wstring& headName);
	void GetSensorCalibrationDueDate(long hDevice, long channel, std::tm& dueDate);
	void IsSensorExists(long hDevice, long channel, bool& exists);

	// Head Configuration
	void AddWavelength(long hDevice, long channel, long wavelength);
	void DeleteWavelength(long hDevice, long channel, long wlIndex);
	void ModifyWavelength(long hDevice, long channel, long wlIndex, long wavelength);
	void GetWavelengths(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void GetWavelengthsExtra(long hDevice, long channel, bool& modifiable, long& minWavelength, long& maxWavelength);
	void SetWavelength(long hDevice, long channel, long index);
	void GetDiffuser(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetDiffuser(long hDevice, long channel, long index);
	void GetFilter(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetFilter(long hDevice, long channel, long index);
	void GetMeasurementMode(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetMeasurementMode(long hDevice, long channel, long index);
	void GetPulseLengths(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetPulseLength(long hDevice, long channel, long index);
	void GetRanges(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetRange(long hDevice, long channel, long index);
	void GetThreshold(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetThreshold(long hDevice, long channel, long index);
	void GetPulsedPowerPulseWidth(long hDevice, long channel, long& value_ms, long& min_ms, long& max_ms);
	void SetPulsedPowerPulseWidth(long hDevice, long channel, long value_ms);
	void GetLowFreqPowerPulseFreq(long hDevice, long channel, double& value, double& min, double& max);
	void SetLowFreqPowerPulseFreq(long hDevice, long channel, double value);
	void SaveSettings(long hDevice, long channel);

	// Trigger Settings(for Pulsar)
	void GetExtTrigModes(long hDevice, long& index, std::vector<std::wstring>& options);
	void SetExtTrigMode(long hDevice, long index);
	void GetExtTrigOnOff(long hDevice, long channel, long& index, std::vector<std::wstring>& options);
	void SetExtTrigOnOff(long hDevice, long channel, long index);
	void GetExtTrigWindowTime(long hDevice, long& extTrigWindowTime);
	void SetExtTrigWindowTime(long hDevice, long extTrigWindowTime);

	// Measurement Delivery
	void ConfigureStreamMode(long hDevice, long channel, long mode, long nOff);
	void StartStream(long hDevice, long channel);
	void GetData(long hDevice, long channel, std::vector<double>& arrayValue, std::vector<double>& arrayTimestamp, std::vector<Status>& arrayStatus);
	void StopStream(long hDevice, long channel);
	void StopAllStreams();

	// Legacy Methods
	void Read(long hDevice, std::wstring& reply);
	void Write(long hDevice, std::wstring command);

	// COM Events
	void RegisterPlugAndPlay(std::function<void()> callback);
	void RegisterDataReady(std::function<void(long hDevice,long channel)> callback);

	std::wstring StatusString(Status s);

private:
	STDMETHOD_(ULONG, AddRef)()
	{
		return 1;
	}

	STDMETHOD_(ULONG, Release)()
	{
		return 1;
	}

	STDMETHOD(QueryInterface)(REFIID riid, void ** ppvObject)
	{
		if (riid == IID_IUnknown)
		{
			*ppvObject = static_cast<IUnknown*>(this);
			return S_OK;
		}

		if ((riid == IID_IDispatch) || (riid == __uuidof(OphirLMMeasurementLib::_ICoLMMeasurementEvents)))
		{
			*ppvObject = static_cast<IDispatch* >(this);
			return S_OK;
		}

		return E_NOINTERFACE;
	}

	STDMETHOD(GetTypeInfoCount)(UINT* pctinfo)
	{
		return E_NOTIMPL;
	}

	STDMETHOD(GetTypeInfo)(UINT itinfo, LCID lcid, ITypeInfo** pptinfo)
	{
		return E_NOTIMPL;
	}

	STDMETHOD(GetIDsOfNames)(REFIID riid, LPOLESTR* rgszNames, UINT cNames,
		LCID lcid, DISPID* rgdispid)
	{
		return E_NOTIMPL;
	}

	STDMETHOD(Invoke)(DISPID dispidMember, REFIID riid,
		LCID lcid, WORD wFlags, DISPPARAMS* pdispparams, VARIANT* pvarResult,
		EXCEPINFO* pexcepinfo, UINT* puArgErr)
	{
		return OnCOMEventFiring(dispidMember, riid, lcid, wFlags, pdispparams, pvarResult, pexcepinfo, puArgErr);
	}

	HRESULT OnCOMEventFiring(DISPID dispidMember, REFIID riid,LCID lcid, WORD wFlags,
		DISPPARAMS* pdispparams, VARIANT* pvarResult, EXCEPINFO* pexcepinfo, UINT* puArgErr);

	void SetupConnectionPoint();

	void ShutdownConnectionPoint();

	std::function<void()> m_plugAndPlayCallback;
	std::function<void(long hDevice, long channel)> m_dataReadyCallback;
	const OphirLMMeasurementLib::ICoLMMeasurement2Ptr   m_coLM;
	IConnectionPoint*			m_pIConnectionPoint;
	DWORD						m_dwEventCookie;
};
