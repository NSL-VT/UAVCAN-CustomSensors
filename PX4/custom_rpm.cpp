/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @author RJ Gritter <rjgritter657@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include "custom_rpm.hpp"
#include <math.h>
#include <lib/geo/geo.h> // For CONSTANTS_*


const char *const UavcanCustomRPMBridge::NAME = "custom_rpm";

UavcanCustomRPMBridge::UavcanCustomRPMBridge(uavcan::INode &node) :
	UavcanSensorBridgeBase("uavcan_custom_rpm", ORB_ID(esc_status)),
	_sub_rpm_data(node)
{ }

int UavcanCustomRPMBridge::init()
{
	int res = _sub_rpm_data.start(RPMCbBinder(this, &UavcanCustomRPMBridge::rpm_sub_cb));
	// int res = _sub_tas_data.start(TASCbBinder(this, &UavcanCustomRPMBridge::tas_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}


void
UavcanCustomRPMBridge::rpm_sub_cb(const
				 uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	//esc_status_s _esc_status{};
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_rpm         = msg.rpm;
		// ref.esc_errorcount  = msg.error_count;

		// _esc_status.esc_count = _rotor_count;
		_esc_status.counter += 1;
		// _esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		// _esc_status.esc_online_flags = check_escs_status();
		// _esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_esc_status.timestamp = hrt_absolute_time();
		_esc_status_pub.publish(_esc_status);
	}
}
