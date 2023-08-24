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

#define _GNU_SOURCE		// for pthread_timedjoin_np and possibly other things
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "pipe_channels.h"
#include "misc.h"
#include "voxl_app_manager.h"
// #include "config_file.h"
// #include "mavlink_for_ros.h"
// #include "geometry.h"

#include <c_library_v2/common/mavlink.h>
#include <modal_pipe_server.h>
#include <modal_pipe_client.h>
#include <sys/syscall.h>

// local variables
static int running = 0;
static pthread_t thread_id;
static int print_debug_send = 0;
static int print_debug_recv = 0;
static int64_t ts_last_received_msg;


void mavlink_io_en_print_debug_send(int en_print_debug)
{
	print_debug_send = en_print_debug;
	return;
}

void mavlink_io_en_print_debug_recv(int en_print_debug)
{
	print_debug_recv = en_print_debug;
	return;
}


static void _connect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	printf("Connected to voxl-mavlink-server\n");
	// but not necessarily connected through to Autopilot yet, wait for first message
	return;
}

static void _disconnect_cb(__attribute__((unused)) int ch, __attribute__((unused)) void* context)
{
	autopilot_monitor_set_connected_flag(0);
	fprintf(stderr, "WARNING: Autopilot Mavlink Disconnected\n");
	return;
}




// note, reading must be done in a pthread, not in main()!!!
// the signal handler will interfere with the RPC read call otherwise
static void* _timer_thread_func(__attribute__((unused))void* context)
{
	while(running){

		// check at 10hz
		usleep(100000);

		// no need to check if we are already disconnected
		if(!autopilot_monitor_is_connected()) continue;

		// check for timeout
		int64_t age = my_time_monotonic_ns() - ts_last_received_msg;
		if(age>1000000000){
			autopilot_monitor_set_connected_flag(0);
			fprintf(stderr, "WARNING: Autopilot Mavlink Disconnected\n");
			mavlink_io_send_text_to_gcs("vision hub disconnected from autopilot");
		}
	}


	return NULL;
}



static void _data_from_autopilot_helper_cb(__attribute__((unused))int ch, char* data, \
							int bytes, __attribute__((unused)) void* context)
{
	// validate that the data makes sense
	int n_packets;
	mavlink_message_t* msg_array = pipe_validate_mavlink_message_t(data, bytes, &n_packets);
	if(msg_array == NULL){
		return;
	}

	// log that we got data at this time and are connected
	autopilot_monitor_set_connected_flag(1);
	ts_last_received_msg = my_time_monotonic_ns();


	// scrape every message for our own use, then decide if we should forward it
	for(int i=0; i<n_packets; i++){

		if(print_debug_recv){
			printf("from AP msgid: %3d sysid:%3d compid:%3d\n", msg_array[i].msgid, msg_array[i].sysid, msg_array[i].compid);
		}

		mavlink_message_t msg = msg_array[i];
		int should_block = autopilot_monitor_scrape_data(&msg);
	}

	return;
}



int mavlink_io_init(void)
{
	printf("Waiting to connect to voxl-mavlink-server\n");

	// open the pipe from voxl-mavlink-server
	pipe_client_set_connect_cb(MAVLINK_ONBOARD_CH, _connect_cb, NULL);
	pipe_client_set_disconnect_cb(MAVLINK_ONBOARD_CH, _disconnect_cb, NULL);
	pipe_client_set_simple_helper_cb(MAVLINK_ONBOARD_CH, _data_from_autopilot_helper_cb, NULL);
	pipe_client_open(MAVLINK_ONBOARD_CH, MAVLINK_ONBOARD_NAME, PIPE_CLIENT_NAME,
								CLIENT_FLAG_EN_SIMPLE_HELPER,\
								MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE);

	// Also open the voxl-mavlink-server pipe to the GCS just so we can send messages
	// straight to the CGS
	pipe_client_open(MAVLINK_TO_GCS_CH, MAVLINK_TO_GCS_NAME, PIPE_CLIENT_NAME,
								CLIENT_FLAG_EN_SIMPLE_HELPER,\
								MAVLINK_MESSAGE_T_RECOMMENDED_READ_BUF_SIZE);

	// start the timer thread
	running = 1;
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	if(pthread_create(&thread_id, &attr, _timer_thread_func, NULL) != 0){
		fprintf(stderr, "couldn't start mavlink timer thread\n");
		return -1;
	}

	// ADDED BY MICHAEL ZENG
	// Declare last_mavlink_command with default values
	pthread_mutex_lock(&last_mavlink_cmd_mutex);
	last_mavlink_cmd.time_boot_ms = 0;
    last_mavlink_cmd.coordinate_frame = MAV_FRAME_LOCAL_NED;
    last_mavlink_cmd.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                			     POSITION_TARGET_TYPEMASK_VY_IGNORE |
                			     POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    			 POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    			 POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    			 POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    			 POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                    			 POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    last_mavlink_cmd.target_system = 0;  // will reset later when sending
	last_mavlink_cmd.target_component = AUTOPILOT_COMPID;
	pthread_mutex_unlock(&last_mavlink_cmd_mutex);

	return 0;
}


int mavlink_io_stop(void)
{
	if(running==0){
		return 0;
	}

	// do a timed join, 1 second timeout
	printf("waiting for mavlink timer thread to join\n");
	running = 0;
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 1;
	errno = pthread_timedjoin_np(thread_id, NULL, &thread_timeout);
	if(errno==ETIMEDOUT){
		fprintf(stderr, "WARNING, timeout joining mavlink timer thread\n");
	}

	return 0;
}


int mavlink_io_send_msg_to_ap(mavlink_message_t* msg)
{
	if(!running){
		return -1;
	}
	if(print_debug_send){
		printf("to AP: msgid: %3d sysid:%3d compid:%3d\n", msg->msgid, msg->sysid, msg->compid);
	}

	int ret = pipe_client_send_control_cmd_bytes(MAVLINK_ONBOARD_CH, msg, sizeof(mavlink_message_t));
	return ret;
}


int mavlink_io_send_local_setpoint(uint8_t sysid, uint8_t compid, mavlink_set_position_target_local_ned_t local_sp)
{
	printf("SENDING SETPOINT: pos: (%f, %f, %f, %f), vel: (%f, %f, %f), acc: (%f, %f, %f). type_mask = %d\n", local_sp.x, local_sp.y, local_sp.z, local_sp.yaw, local_sp.vx, local_sp.vy, local_sp.vz, local_sp.afx, local_sp.afy, local_sp.afz, local_sp.type_mask);

	mavlink_message_t msg;
	local_sp.target_system = autopilot_monitor_get_sysid();
	mavlink_msg_set_position_target_local_ned_encode(sysid,compid, &msg, &local_sp);
	// Added for safety
	if (MOVE_PROPS) mavlink_io_send_msg_to_ap(&msg);
	return 0;
}


int mavlink_io_send_msg_to_gcs(mavlink_message_t* msg)
{
	if(!running){
		return -1;
	}
	if(print_debug_send){
		printf("to GCS: msgid: %3d sysid:%3d compid:%3d\n", msg->msgid, msg->sysid, msg->compid);
	}

	int ret = pipe_client_send_control_cmd_bytes(MAVLINK_TO_GCS_CH, msg, sizeof(mavlink_message_t));
	return ret;
}


int mavlink_io_send_text_to_gcs(const char* string)
{
	mavlink_message_t msg;

	int len = strlen(string);
	if(len>50){
		fprintf(stderr, "WARNING in %s, only 50 characters can be sent\n", __FUNCTION__);
	}

	// mavlink library does a memcpy, not a strcpy, this could result
	// in a segfault for strings with length less than 50. So copy into
	// our own buffer first!!
	char newstring[51]; // extra byte for null terminator
	strncpy(newstring, string, 51); // this will pad the end with zeros

	// send message
	mavlink_msg_statustext_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg,\
						   MAV_SEVERITY_INFO, newstring, 0, 0);
	return 0;
}

// NOT PART OF ORIGINAL mavlink_io.c; ADDED BY MICHAEL ZENG
void set_last_mavlink_cmd_xyz(float x, float y, float z) {
	pthread_mutex_lock(&last_mavlink_cmd_mutex);
	last_mavlink_cmd.x = x;
	last_mavlink_cmd.y = y;
	last_mavlink_cmd.z = z;
	// ensure none of the positions are masked
	last_mavlink_cmd.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
								 POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
								 POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
	// set vz to 0 to maintain altitude hold, in case it was set elsewhere
	last_mavlink_cmd.vz = 0;
	pthread_mutex_unlock(&last_mavlink_cmd_mutex);
}


// NOT PART OF ORIGINAL mavlink_io.c; ADDED BY MICHAEL ZENG
void set_last_mavlink_cmd_on_ground() {
	pthread_mutex_lock(&last_mavlink_cmd_mutex);
	last_mavlink_cmd.vz = 1.25;  // give it velocity into the floor so it stays put
	// ignore Z-pos (and PX4 will automatically ignore all positions, so just mask them all)
	last_mavlink_cmd.type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_X_IGNORE;
	// do NOT ignore z-velocity (and PX4 will automatically not ignore all velocities, so just ensure none are masked)
	last_mavlink_cmd.type_mask &= POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
								  POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
								  POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
	pthread_mutex_unlock(&last_mavlink_cmd_mutex);
}