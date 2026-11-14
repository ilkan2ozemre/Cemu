#include "input/api/SDL/SystemIMU.h"
#include "util/helpers/helpers.h"

#include <windows.h>
#include <initguid.h> // instantiate the SENSOR_* GUIDs and property keys from sensors.h
#include <sensorsapi.h>
#include <sensors.h>
#include <PortableDeviceTypes.h>

#include <cctype>
#include <future>
#include <optional>

// Some handhelds (MSI Claw 8 AI+ / A2VM, Claw 8 EX AI+ / CG3EM) expose their IMU as
// "Physical Gyrometer" / "Physical Accelerometer" sensors. They report data in the custom
// sensor value fields instead of the standard ones, so SDL's Windows sensor backend can't
// read them. Field IDs and axis mapping are taken from Handheld Companion's device
// configs (Resources/Devices/ClawA2VM.json).
static const PROPERTYKEY kCustomValueX = {{0xb14c764f, 0x07cf, 0x41e8, {0x9d, 0x82, 0xeb, 0xe3, 0xd0, 0x77, 0x6a, 0x6f}}, 7};
static const PROPERTYKEY kCustomValueY = {{0xb14c764f, 0x07cf, 0x41e8, {0x9d, 0x82, 0xeb, 0xe3, 0xd0, 0x77, 0x6a, 0x6f}}, 8};
static const PROPERTYKEY kCustomValueZ = {{0xb14c764f, 0x07cf, 0x41e8, {0x9d, 0x82, 0xeb, 0xe3, 0xd0, 0x77, 0x6a, 0x6f}}, 9};

// raw -> SDL frame: SDL.x = raw.x, SDL.y = raw.z, SDL.z = -raw.y
static constexpr const char* kDefaultAxes = "x,z,-y";

// Cemu's SDL path divides by 9.81, use the same value so the conversion round-trips exactly
static constexpr float kGravity = 9.81f;
static constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

namespace
{
	enum class SensorKind
	{
		Gyro,
		Accel,
	};

	struct AxisMap
	{
		int index[3]{0, 1, 2};
		float sign[3]{1.0f, 1.0f, 1.0f};
	};

	struct OpenSensor
	{
		ISensor* sensor = nullptr;
		PROPERTYKEY fields[3]{};
		bool customFields = false;
		std::string name;

		void Release()
		{
			if (sensor)
				sensor->Release();
			sensor = nullptr;
		}
	};

	std::mutex s_lifecycleMutex;
	std::thread s_thread;
	std::atomic_bool s_running{false};
	std::atomic_bool s_available{false};

	std::string ToUTF8(const wchar_t* str)
	{
		if (!str)
			return {};
		const int length = WideCharToMultiByte(CP_UTF8, 0, str, -1, nullptr, 0, nullptr, nullptr);
		if (length <= 1)
			return {};
		std::string result(length - 1, '\0');
		WideCharToMultiByte(CP_UTF8, 0, str, -1, result.data(), length, nullptr, nullptr);
		return result;
	}

	std::string ToLower(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return (char)std::tolower(c); });
		return str;
	}

	std::optional<std::string> GetEnv(const char* name)
	{
		char* value = nullptr;
		size_t length = 0;
		if (_dupenv_s(&value, &length, name) != 0 || !value)
			return std::nullopt;
		std::string result(value);
		free(value);
		return result;
	}

	// parses "x,z,-y" style mappings: output axis i = sign * raw[axis]
	std::optional<AxisMap> ParseAxisMap(std::string_view text)
	{
		AxisMap map{};
		bool used[3]{};
		int axis = 0;
		size_t pos = 0;
		while (axis < 3)
		{
			while (pos < text.size() && text[pos] == ' ')
				++pos;
			float sign = 1.0f;
			if (pos < text.size() && (text[pos] == '-' || text[pos] == '+'))
			{
				sign = text[pos] == '-' ? -1.0f : 1.0f;
				++pos;
			}
			if (pos >= text.size())
				return std::nullopt;
			const char c = (char)std::tolower((unsigned char)text[pos++]);
			if (c < 'x' || c > 'z' || used[c - 'x'])
				return std::nullopt;
			used[c - 'x'] = true;
			map.index[axis] = c - 'x';
			map.sign[axis] = sign;
			++axis;
			while (pos < text.size() && text[pos] == ' ')
				++pos;
			if (axis < 3)
			{
				if (pos >= text.size() || text[pos] != ',')
					return std::nullopt;
				++pos;
			}
		}
		if (pos != text.size())
			return std::nullopt;
		return map;
	}

	AxisMap GetAxisMap(const char* specificVar)
	{
		for (const char* var : {specificVar, "CEMU_SYSTEM_IMU_AXES"})
		{
			if (auto value = GetEnv(var))
			{
				if (auto map = ParseAxisMap(*value))
				{
					cemuLog_log(LogType::Force, "SystemIMU: using {}={}", var, *value);
					return *map;
				}
				cemuLog_log(LogType::Force, "SystemIMU: ignoring invalid {}={} (expected e.g. \"x,z,-y\")", var, *value);
			}
		}
		return *ParseAxisMap(kDefaultAxes);
	}

	bool SupportsFields(ISensor* sensor, const PROPERTYKEY (&fields)[3])
	{
		for (const auto& field : fields)
		{
			VARIANT_BOOL supported = VARIANT_FALSE;
			if (FAILED(sensor->SupportsDataField(field, &supported)) || supported == VARIANT_FALSE)
				return false;
		}
		return true;
	}

	std::optional<SensorKind> GetKind(ISensor* sensor, const std::string& name)
	{
		SENSOR_TYPE_ID type{};
		if (SUCCEEDED(sensor->GetType(&type)))
		{
			if (IsEqualGUID(type, SENSOR_TYPE_GYROMETER_3D))
				return SensorKind::Gyro;
			if (IsEqualGUID(type, SENSOR_TYPE_ACCELEROMETER_3D))
				return SensorKind::Accel;
		}
		// custom sensors are only recognizable by their name
		const std::string lowerName = ToLower(name);
		if (lowerName.find("accelerometer") != std::string::npos)
			return SensorKind::Accel;
		if (lowerName.find("gyrometer") != std::string::npos || lowerName.find("gyroscope") != std::string::npos)
			return SensorKind::Gyro;
		return std::nullopt;
	}

	void SetReportInterval(ISensor* sensor, ULONG requestedMs)
	{
		ULONG intervalMs = requestedMs;
		PROPVARIANT minInterval;
		PropVariantInit(&minInterval);
		if (SUCCEEDED(sensor->GetProperty(SENSOR_PROPERTY_MIN_REPORT_INTERVAL, &minInterval)) && minInterval.vt == VT_UI4)
			intervalMs = std::max<ULONG>(intervalMs, minInterval.ulVal);
		PropVariantClear(&minInterval);

		IPortableDeviceValues* values = nullptr;
		if (FAILED(CoCreateInstance(__uuidof(PortableDeviceValues), nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&values))))
			return;
		values->SetUnsignedIntegerValue(SENSOR_PROPERTY_CURRENT_REPORT_INTERVAL, intervalMs);
		IPortableDeviceValues* results = nullptr;
		sensor->SetProperties(values, &results);
		if (results)
			results->Release();
		values->Release();
	}

	// returns the first sensor of the requested kind, preferring ones with the custom fields
	OpenSensor FindSensor(ISensorManager* manager, SensorKind kind)
	{
		OpenSensor best;
		ISensorCollection* collection = nullptr;
		if (FAILED(manager->GetSensorsByCategory(SENSOR_CATEGORY_ALL, &collection)) || !collection)
			return best;

		const PROPERTYKEY customFields[3] = {kCustomValueX, kCustomValueY, kCustomValueZ};
		const PROPERTYKEY standardFields[3] = {
			kind == SensorKind::Gyro ? SENSOR_DATA_TYPE_ANGULAR_VELOCITY_X_DEGREES_PER_SECOND : SENSOR_DATA_TYPE_ACCELERATION_X_G,
			kind == SensorKind::Gyro ? SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Y_DEGREES_PER_SECOND : SENSOR_DATA_TYPE_ACCELERATION_Y_G,
			kind == SensorKind::Gyro ? SENSOR_DATA_TYPE_ANGULAR_VELOCITY_Z_DEGREES_PER_SECOND : SENSOR_DATA_TYPE_ACCELERATION_Z_G,
		};

		ULONG count = 0;
		collection->GetCount(&count);
		for (ULONG i = 0; i < count; ++i)
		{
			ISensor* sensor = nullptr;
			if (FAILED(collection->GetAt(i, &sensor)) || !sensor)
				continue;

			BSTR friendlyName = nullptr;
			std::string name = "Unknown Sensor";
			if (SUCCEEDED(sensor->GetFriendlyName(&friendlyName)) && friendlyName)
				name = ToUTF8(friendlyName);
			if (friendlyName)
				SysFreeString(friendlyName);

			const auto sensorKind = GetKind(sensor, name);
			if (sensorKind != kind)
			{
				sensor->Release();
				continue;
			}

			OpenSensor candidate;
			candidate.name = name;
			if (SupportsFields(sensor, customFields))
			{
				std::copy(std::begin(customFields), std::end(customFields), candidate.fields);
				candidate.customFields = true;
			}
			else if (SupportsFields(sensor, standardFields))
			{
				std::copy(std::begin(standardFields), std::end(standardFields), candidate.fields);
			}
			else
			{
				cemuLog_log(LogType::Force, "SystemIMU: skipping \"{}\", it has no known data fields", name);
				sensor->Release();
				continue;
			}

			candidate.sensor = sensor;
			if (!best.sensor || (candidate.customFields && !best.customFields))
			{
				best.Release();
				best = candidate;
			}
			else
			{
				candidate.Release();
			}
		}
		collection->Release();
		return best;
	}

	bool ReadValue(ISensorDataReport* report, const PROPERTYKEY& key, float& out)
	{
		PROPVARIANT value;
		PropVariantInit(&value);
		if (FAILED(report->GetSensorValue(key, &value)))
			return false;
		bool ok = true;
		switch (value.vt)
		{
		case VT_R8: out = (float)value.dblVal; break;
		case VT_R4: out = value.fltVal; break;
		case VT_I4: out = (float)value.lVal; break;
		case VT_UI4: out = (float)value.ulVal; break;
		case VT_I2: out = (float)value.iVal; break;
		case VT_UI2: out = (float)value.uiVal; break;
		default: ok = false; break;
		}
		PropVariantClear(&value);
		return ok;
	}

	bool ReadSensor(const OpenSensor& sensor, float (&out)[3], bool& errorLogged)
	{
		ISensorDataReport* report = nullptr;
		const HRESULT hr = sensor.sensor->GetData(&report);
		if (FAILED(hr) || !report)
		{
			if (!errorLogged)
				cemuLog_log(LogType::Force, "SystemIMU: reading \"{}\" failed (HRESULT 0x{:08x})", sensor.name, (uint32_t)hr);
			errorLogged = true;
			return false;
		}
		const bool ok = ReadValue(report, sensor.fields[0], out[0]) &&
						ReadValue(report, sensor.fields[1], out[1]) &&
						ReadValue(report, sensor.fields[2], out[2]);
		report->Release();
		if (!ok && !errorLogged)
			cemuLog_log(LogType::Force, "SystemIMU: \"{}\" returned a report without numeric x/y/z values", sensor.name);
		errorLogged |= !ok;
		return ok;
	}

	void Remap(const AxisMap& map, const float (&raw)[3], float (&out)[3], float scale)
	{
		for (int i = 0; i < 3; ++i)
			out[i] = raw[map.index[i]] * map.sign[i] * scale;
	}

	void PollThread(SystemIMU::SampleCallback callback, std::promise<bool> probeResult)
	{
		SetThreadName("SystemIMU");
		const HRESULT coInit = CoInitializeEx(nullptr, COINIT_MULTITHREADED);

		ISensorManager* manager = nullptr;
		OpenSensor gyro, accel;
		if (SUCCEEDED(CoCreateInstance(__uuidof(SensorManager), nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&manager))) && manager)
		{
			gyro = FindSensor(manager, SensorKind::Gyro);
			accel = FindSensor(manager, SensorKind::Accel);
		}
		else
		{
			cemuLog_log(LogType::Force, "SystemIMU: Windows sensor manager unavailable");
		}

		const bool available = gyro.sensor && accel.sensor;
		if (gyro.sensor)
			cemuLog_log(LogType::Force, "SystemIMU: gyrometer \"{}\" ({} fields)", gyro.name, gyro.customFields ? "custom" : "standard");
		if (accel.sensor)
			cemuLog_log(LogType::Force, "SystemIMU: accelerometer \"{}\" ({} fields)", accel.name, accel.customFields ? "custom" : "standard");
		if (!available)
			cemuLog_log(LogType::Force, "SystemIMU: no built-in gyrometer + accelerometer pair found");

		int rateHz = 250;
		if (auto rate = GetEnv("CEMU_SYSTEM_IMU_RATE"))
			rateHz = std::clamp(std::atoi(rate->c_str()), 30, 1000);
		const auto period = std::chrono::nanoseconds(1'000'000'000 / rateHz);

		AxisMap gyroMap{}, accelMap{};
		if (available)
		{
			gyroMap = GetAxisMap("CEMU_SYSTEM_IMU_GYRO_AXES");
			accelMap = GetAxisMap("CEMU_SYSTEM_IMU_ACCEL_AXES");
			SetReportInterval(gyro.sensor, (ULONG)std::max(1, 1000 / rateHz));
			SetReportInterval(accel.sensor, (ULONG)std::max(1, 1000 / rateHz));
		}

		s_available = available;
		probeResult.set_value(available);

		uint64_t sampleCount = 0;
		bool gyroErrorLogged = false, accelErrorLogged = false;
		auto nextPoll = std::chrono::steady_clock::now();
		while (available && s_running.load(std::memory_order_relaxed))
		{
			float rawGyro[3], rawAccel[3];
			if (ReadSensor(gyro, rawGyro, gyroErrorLogged) && ReadSensor(accel, rawAccel, accelErrorLogged))
			{
				// raw data is in deg/s and g, convert to SDL's rad/s and m/s^2
				float sdlGyro[3], sdlAccel[3];
				Remap(gyroMap, rawGyro, sdlGyro, kDegToRad);
				Remap(accelMap, rawAccel, sdlAccel, kGravity);

				const auto now = std::chrono::steady_clock::now();
				const uint64_t ts = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
				callback(ts, sdlGyro, sdlAccel);

				// log a sample at startup and after a few seconds, to help with checking the axis mapping
				if (sampleCount == 0 || sampleCount == (uint64_t)rateHz * 5)
				{
					cemuLog_log(LogType::Force, "SystemIMU: raw gyro (deg/s) {:.2f} {:.2f} {:.2f} | raw accel (g) {:.3f} {:.3f} {:.3f}",
								rawGyro[0], rawGyro[1], rawGyro[2], rawAccel[0], rawAccel[1], rawAccel[2]);
					cemuLog_log(LogType::Force, "SystemIMU: mapped accel (g) {:.3f} {:.3f} {:.3f} (lying flat, screen up, should be about 0 1 0)",
								sdlAccel[0] / kGravity, sdlAccel[1] / kGravity, sdlAccel[2] / kGravity);
				}
				++sampleCount;
			}

			nextPoll += period;
			const auto now = std::chrono::steady_clock::now();
			if (nextPoll < now)
				nextPoll = now;
			std::this_thread::sleep_until(nextPoll);
		}

		gyro.Release();
		accel.Release();
		if (manager)
			manager->Release();
		if (SUCCEEDED(coInit))
			CoUninitialize();
	}
}

bool SystemIMU::Start(SampleCallback callback)
{
	std::scoped_lock lock(s_lifecycleMutex);
	if (s_thread.joinable())
		return s_available;

	if (auto enabled = GetEnv("CEMU_SYSTEM_IMU"); enabled && *enabled == "0")
	{
		cemuLog_log(LogType::Force, "SystemIMU: disabled by CEMU_SYSTEM_IMU=0");
		return false;
	}

	std::promise<bool> probeResult;
	auto probeFuture = probeResult.get_future();
	s_running = true;
	s_thread = std::thread(PollThread, callback, std::move(probeResult));
	const bool available = probeFuture.get();
	if (!available)
	{
		s_running = false;
		s_thread.join();
	}
	return available;
}

void SystemIMU::Stop()
{
	std::scoped_lock lock(s_lifecycleMutex);
	s_running = false;
	if (s_thread.joinable())
		s_thread.join();
	s_available = false;
}

bool SystemIMU::IsAvailable()
{
	return s_available;
}
