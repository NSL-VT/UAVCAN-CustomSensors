# UAVCAN-CustomSensors
Instructions and examples for creating custom UAVCAN sensors for integration with PX4

Outline of files modified our added:

PX4:
	uavcan bridge:
		uavcan/sensors/adu.hpp
		uavcan/sensors/custom_rpm.hpp
		uavcan/sensor_bridge.cpp

	Cmake:
		uavcan/CMakeLists.txt

	Params:
		uavcan_params.c

Teensy:
	CustomRPM/src/main.cpp
	CustomRPM/lib/
