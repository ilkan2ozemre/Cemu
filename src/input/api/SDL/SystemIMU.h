#pragma once

// Built-in IMU of Windows handhelds (e.g. MSI Claw 8 AI+), read through the legacy
// Windows Sensor API. Their gamepads are XInput devices without motion sensors, so SDL
// reports no gyro for them. SDLController uses this as a fallback motion source.
//
// Samples are delivered in SDL's gamepad sensor frame and units (gyro in rad/s,
// accelerometer in m/s^2) so they can share SDL's motion integration code.
//
// Environment overrides:
//   CEMU_SYSTEM_IMU=0              disable the fallback
//   CEMU_SYSTEM_IMU_AXES=x,z,-y    raw -> SDL axis mapping for both sensors
//   CEMU_SYSTEM_IMU_GYRO_AXES=...  gyro-only mapping (overrides CEMU_SYSTEM_IMU_AXES)
//   CEMU_SYSTEM_IMU_ACCEL_AXES=... accel-only mapping (overrides CEMU_SYSTEM_IMU_AXES)
//   CEMU_SYSTEM_IMU_RATE=250       polling rate in Hz
namespace SystemIMU
{
	using SampleCallback = void (*)(uint64_t timestampNs, const float gyro[3], const float accel[3]);

	// Finds the sensors and starts polling them. Returns true if both a gyrometer and an
	// accelerometer were found. Blocks until the sensor probe has finished.
	bool Start(SampleCallback callback);
	void Stop();
	bool IsAvailable();
}
