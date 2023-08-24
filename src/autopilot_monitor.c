/*******************************************************************************
 * Copyright 2023 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <stdio.h>
#include <pthread.h>

#include <c_library_v2/development/mavlink.h> // include before modal_pipe !!
#include <modal_pipe_server.h>
#include <modal_pipe_interfaces.h>

#include <math.h>
#include <rc_math.h>

#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "pipe_channels.h"
#include "misc.h"
#include "macros.h"
// #include "voxl_vision_hub.h"
// #include "config_file.h"
// #include "mavlink_for_ros.h"
// #include "horizon_cal.h"
// #include "geometry.h"


// connected flag set to 1 by the scrape_data function and set to 0 by the uart
// mavlink module when mavparser times out
static int is_connected=0;

// keep track of the autopilot sysid
static uint8_t current_sysid = 0;

// TODO request this from AP if we don't have it
static int is_autopilot_version_known = 0;
static mavlink_autopilot_version_t autopilot_version;

// Telemetry data that needs to be pulled from the relevant mavlink messages
static uint8_t  heartbeat_base_mode         = 0;
static uint32_t heartbeat_custom_mode       = 0;
static uint8_t  heartbeat_system_status     = 0;

// voltages already converted to volts/percent when read from message
static double   sys_status_battery_volts    = 0.0;
static double   sys_status_battery_percent  = 0.0;

// protect multi-byte states such as the attitude struct with this mutex
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

// some packets we keep intact for full reference
static mavlink_attitude_t attitude;
static mavlink_odometry_t odometry;

// on qrb5165 this remains 0
static int64_t px4_nanos_ahead_of_voxl = 0;


// grab modes from the heartbeat and flag as connected
static void _handle_heartbeat(mavlink_message_t* msg)
{

	pthread_mutex_lock(&data_mutex);
	heartbeat_system_status = mavlink_msg_heartbeat_get_system_status(msg);
	heartbeat_base_mode     = mavlink_msg_heartbeat_get_base_mode(msg);
	heartbeat_custom_mode   = mavlink_msg_heartbeat_get_custom_mode(msg);
	// we just got a heartbeat from PX4 over uart so flag as connected
	if(!is_connected){
		is_connected = 1;
		printf("Successfully connected to PX4 through voxl-mavlink-server\n");
	}
	pthread_mutex_unlock(&data_mutex);
	return;
}


// copy out useful battery values
static void _handle_sys_status(mavlink_message_t* msg)
{
	// keep local copy of the battery state for future use
	pthread_mutex_lock(&data_mutex);
	sys_status_battery_volts = ((double) mavlink_msg_sys_status_get_voltage_battery(msg)) / 1000.0;
	sys_status_battery_percent = ((double) mavlink_msg_sys_status_get_battery_remaining(msg)) / 100.0;
	pthread_mutex_unlock(&data_mutex);

	return;
}


// just copy attitude out and send to auto horizon if that's enabled
static void _handle_attitude(mavlink_message_t* msg)
{
	mavlink_attitude_t new_attitude;
	mavlink_msg_attitude_decode(msg, &new_attitude);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	attitude = new_attitude;
	pthread_mutex_unlock(&data_mutex);

	// use message timestamp if timestamp sync is working, otherwise just assume
	// current time
	int64_t ts_ns;
	if(px4_nanos_ahead_of_voxl!=0){
		ts_ns = (int64_t)new_attitude.time_boot_ms*1000000 - px4_nanos_ahead_of_voxl;
	}
	else{
		// subtract a millisecond for transport latency (just a guess)
		ts_ns = my_time_monotonic_ns() - 1000000;
	}

	// send out the pipe for normal clients
	static pose_vel_6dof_t mpa_attitude;
	mpa_attitude.magic_number = POSE_VEL_6DOF_MAGIC_NUMBER;
	mpa_attitude.timestamp_ns = ts_ns;

	rc_matrix_t R_att = RC_MATRIX_INITIALIZER;
	rc_rotation_matrix_from_tait_bryan(new_attitude.roll, new_attitude.pitch, new_attitude.yaw, &R_att);
	matrix_to_float(R_att, mpa_attitude.R_child_to_parent);

	mpa_attitude.w_child_wrt_child[0] = new_attitude.rollspeed;
	mpa_attitude.w_child_wrt_child[1] = new_attitude.pitchspeed;
	mpa_attitude.w_child_wrt_child[2] = new_attitude.yawspeed;

	return;
}


// just copy odometry out
static void _handle_odometry(mavlink_message_t* msg)
{
	mavlink_odometry_t new_odometry;
	mavlink_msg_odometry_decode(msg, &new_odometry);

	// save local copy
	pthread_mutex_lock(&data_mutex);
	odometry = new_odometry;
	pthread_mutex_unlock(&data_mutex);

	return;
}


// TODO probably need to request this but not needed yet
static void _handle_autopilot_version(mavlink_message_t* msg)
{
	is_autopilot_version_known = 1;
	mavlink_autopilot_version_t new;
	mavlink_msg_autopilot_version_decode(msg, &new);

	// save local copy
	pthread_mutex_lock(&data_mutex);

	if(strncmp(	(char*)new.flight_custom_version,\
				(char*)autopilot_version.flight_custom_version, 8))
	{
		printf("Detected autopilot version: %s\n", new.flight_custom_version);
	}
	autopilot_version = new;

	pthread_mutex_unlock(&data_mutex);
}


// ADDED BY MICHAEL ZENG (not part of original autopilot_monitor.c from VVHub)
static void _handle_rc(mavlink_message_t* msg) 
{
	// Using a Commando8 RC Transmitter, Channel 7 is the 3-position SWD and Channel 8 is the toggle button SWD.
	mavlink_rc_channels_t rc_channels;
	mavlink_msg_rc_channels_decode(msg, &rc_channels);

	// For now, only tracking position of SWC
	if (rc_channels.chan7_raw < 1333) {
		SWC_pos = 3;
	}
	else if (rc_channels.chan7_raw < 1667) {
		SWC_pos = 2;
	}
	else if (rc_channels.chan7_raw > 1667) {
		SWC_pos = 1;
	}
	else {
		printf("ERROR: invalid input to RC channel 7.\n");
	}
}


// return 0 for normal messages
// return nonzero to indicate to mavlink-io.c not to forward this msg to ROS
int autopilot_monitor_scrape_data(mavlink_message_t* msg)
{
	
	// always monitor the autopilot sysid in case it changes which may happen
	// during setup and config
	if(msg->compid == MAV_COMP_ID_AUTOPILOT1 && msg->sysid != current_sysid){
		current_sysid = msg->sysid;
		printf("Detected Autopilot Mavlink SYSID %d\n", current_sysid);
	}

	switch(msg->msgid){

		// messages handled by both qrb5165 and apq8096
		case MAVLINK_MSG_ID_HEARTBEAT:
			_handle_heartbeat(msg);
			break;

		case MAVLINK_MSG_ID_SYS_STATUS:
			_handle_sys_status(msg);
			break;

		case MAVLINK_MSG_ID_ATTITUDE:
			_handle_attitude(msg);
			break;

		case MAVLINK_MSG_ID_ODOMETRY:
			_handle_odometry(msg);
			break;

		case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
			_handle_autopilot_version(msg);
			break;

		// ADDED BY MICHAEL ZENG (not part of original autopilot_monitor.c from VVHub)
		case MAVLINK_MSG_ID_RC_CHANNELS:
			_handle_rc(msg);

		default:
			break;
	}

	return 0; // most messages don't block
}


int autopilot_monitor_is_connected(void)
{
	return is_connected;
}

uint8_t autopilot_monitor_get_sysid(void)
{
	return current_sysid;
}

int autopilot_monitor_is_armed(void)
{
	if(heartbeat_base_mode & MAV_MODE_FLAG_SAFETY_ARMED){
		return 1;
	}
	return 0;
}


px4_main_mode autopilot_monitor_get_main_mode(void)
{
	return (heartbeat_custom_mode&0x00FF0000)>>16;
}


px4_sub_mode autopilot_monitor_get_sub_mode(void)
{
	return (heartbeat_custom_mode&0xFF000000)>>24;
}


double autopilot_monitor_get_bat_voltage(void)
{
	return sys_status_battery_volts;
}


double autopilot_monitor_get_bat_percentage(void)
{
	return sys_status_battery_percent;
}


// TODO Handle Ardupilot modes too
void autopilot_monitor_print_main_mode(void)
{
	px4_main_mode mode = autopilot_monitor_get_main_mode();
	switch(mode){
		case PX4_MAIN_MODE_UNKNOWN :
			printf("UNKNOWN");
			break;
		case PX4_MAIN_MODE_MANUAL :
			printf("MANUAL");
			break;
		case PX4_MAIN_MODE_ALTCTL :
			printf("ALTITUDE");
			break;
		case PX4_MAIN_MODE_POSCTL :
			printf("POSITION");
			break;
		case PX4_MAIN_MODE_AUTO :
			printf("AUTO");
			break;
		case PX4_MAIN_MODE_ACRO :
			printf("ACRO");
			break;
		case PX4_MAIN_MODE_OFFBOARD :
			printf("OFFBOARD");
			break;
		case PX4_MAIN_MODE_STABILIZED :
			printf("STABILIZED");
			break;
		case PX4_MAIN_MODE_RATTITUDE:
			printf("RATTITUDE");
			break;
		default:
			printf("unknown main flight mode");
	}
	return;
}


// used by the offboard mode controllers to see if PX4 is currenly obeying
// offboard commands. returns 1 if armed and in offboard mode, otherwise 0
int autopilot_monitor_is_armed_and_in_offboard_mode(void)
{
	// yes I could make this one big if statement but we may add extra
	// conditions in the future and this is easier to read if it gets longer.
	if(!is_connected) return 0;
	if(!autopilot_monitor_is_armed()) return 0;
	if(autopilot_monitor_get_main_mode()!=PX4_MAIN_MODE_OFFBOARD) return 0;
	return 1;
}


mavlink_attitude_t autopilot_monitor_get_attitude(void)
{
	mavlink_attitude_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = attitude;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}

mavlink_odometry_t autopilot_monitor_get_odometry(void)
{
	mavlink_odometry_t ret;
	pthread_mutex_lock(&data_mutex);
	ret = odometry;
	pthread_mutex_unlock(&data_mutex);
	return ret;
}


int autopilot_monitor_get_rpy(float* roll, float* pitch, float* yaw)
{
	if(!is_connected){
		return -1;
	}

	pthread_mutex_lock(&data_mutex);
	*roll = attitude.roll;
	*pitch = attitude.pitch;
	*yaw = attitude.yaw;
	pthread_mutex_unlock(&data_mutex);
	return 0;
}


void autopilot_monitor_set_connected_flag(int flag)
{
	is_connected = flag;
	return;
}


int autopilot_monitor_init(void)
{
	// nothing to do anymore
	return 0;
}


int autopilot_monitor_stop(void)
{
	// nothing to do anymore
	return 0;
}

float get_yaw(mavlink_odometry_t* cur_pos) {
	// find drone's approximate current yaw based on the quaternion in cur_pos
	double yaw_unbounded = atan2(2 * (cur_pos->q[0] * cur_pos->q[3] + cur_pos->q[1] * cur_pos->q[2]), 1 - 2 * (cur_pos->q[2] * cur_pos->q[2] + cur_pos->q[3] * cur_pos->q[3]));
	// ensure yaw is between -180 and 180
	return (fmod(yaw_unbounded + PI, TWO_PI) - PI);
}
