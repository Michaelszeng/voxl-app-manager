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

/**
 * Copied from voxl-vision-hub.
 */


#ifndef PX4_MAVLINK_H
#define PX4_MAVLINK_H

#include <stdint.h>
#include <c_library_v2/common/mavlink.h>

// all messages sent from VOXL are tagged with this component ID to
// differentiate their origin from packets from QGC or PX4
#define VOXL_COMPID			MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

#define AUTOPILOT_COMPID	MAV_COMP_ID_AUTOPILOT1


int mavlink_io_init(void);
int mavlink_io_stop(void);

void mavlink_io_en_print_debug_send(int en_print_debug);
void mavlink_io_en_print_debug_recv(int en_print_debug);

int mavlink_io_send_msg_to_ap(mavlink_message_t* msg);


/**
 * @brief      send a position setpoint in local frame
 *
 *             PX4 is only aware of local frame. it flies in local frame, and
 *             take setpoints in local frame. This frame is centered whever VIO started.
 *
 * @param[in]  local_sp  local setpoin
 *
 * @return     0 on success, -1 on failure
 */
int mavlink_io_send_local_setpoint(uint8_t sysid, uint8_t compid, mavlink_set_position_target_local_ned_t local_sp);


int mavlink_io_send_msg_to_gcs(mavlink_message_t* msg);


// used to send messages (usually warnings) to QGC
int mavlink_io_send_text_to_gcs(const char* string);

// Very simple helper to just set the xyz coordinates of last_mavlink_cmd
void set_last_mavlink_cmd_xyz(float x, float y, float z);

// Helper to set the last mavlink cmd to keep the drone on the ground
void set_last_mavlink_cmd_on_ground();

#endif // end #define PX4_MAVLINK_H
