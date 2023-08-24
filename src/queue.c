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
 * QUEUE ARCHITECTURE OVERVIEW
 * 
 * Background: There is a single queue that is used for the operation of all
 * apps. When a new app is loaded, the queue is reset and reused. This queue is
 * managed by voxl-app-manager.
 * 
 * Every app is run in a child process, which means they do not share memory
 * space with voxl-app-manager at all; hence, apps do not actually have access
 * to the queue even though they are the ones that possess the user-defined
 * primitives and are responsible for pushing those primities to the queue. 
 * Therefore, a system of communication between voxl-app-manager and each app
 * must be established. This is described below:
 * 
 * 1. There are 2 pipe between each app and voxl-app-manager; one for each
 *    direction of communication.
 * 2. Both voxl-app-manager and app-runner have a "listener" thread that is
 *    constantly reading 8 bytes from their respective read pipes.
 * 3. All communication is prefaced with an 8-byte "header". The first 4 bytes
 *    of the header contians the magic number (used to validate that pipe
 *    communication is in sync. The next 2 bytes indicate the purpose/type of
 *    the message. The last 2 bytes indicate the number of bytes. The listener 
 *    threads will perform additional pipe reads and perform specific actions
 *    based on the header that is read.
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>

#include <c_library_v2/development/mavlink.h>

// #include <modal_journal.h>  // libmodal_journal is not compatible with multi-processing
#include "modal_journal_lite.h"

#include "queue.h"
#include "primitives.h"
#include "voxl_app_manager.h"
#include "app_runner.h"
#include "mavlink_io.h"


/**
 * Note: the queue is 1-indexed (queue_end starts at 1 instead of 0).
 * 
 * This no longer serves any real purpose, but is how it was designed.
 */
int queue_pos = 0;
int queue_end = 1;  // also serves as the current length of the queue

uint8_t QUEUE_PAUSED_FLAG = 0;


void push(char* ptr, primitive_t type) {
    /**
     * Notes about the structure of the data when it is copied from *ptr to the
     * 'data' char array in primitive_generic_t:
     *  - little endian (LSB on the left)
     *  - fields of the struct are stored in the order they appear in the struct
     *    declaration in primitives.h
     * 
     * Example: 04 00 00 00 01 00 00 00 00 00 80 3F 00 00 00 00 63 00 00 00 63 00 00 00
     *  - type (int): 4 (TAKE_OFF)
     *  - id (uint32_t): 1
     *  - height (float): 1 (equal to 0x3F800000 in hex)
     *  - yaw (float): 0
     *  - reserved1 (uint32_t): 99 (equal to 0x00000063 in hex)
     *  - reserved2 (uint32_t): 99 (equal to 0x00000063 in hex)
     */

    // set queue item type
    queue[queue_end].type = type;  // set type of generic primitive struct

    switch(type) {
        case PAUSE:
            // set queue item data
            queue[queue_end].data = (char*)malloc(sizeof(primitive_pause_t));  // allocate memory for data array
            memcpy(queue[queue_end].data, ptr, sizeof(primitive_pause_t));  // copy data from struct into char array
            M_DEBUG("adding primitive of type %d with assigned ID %lu to queue position %d.\n", type, ((primitive_pause_t*)ptr)->id, queue_end);
            break;
        case LAND:
            // set queue item data
            queue[queue_end].data = (char*)malloc(sizeof(primitive_land_t));  // allocate memory for data array
            memcpy(queue[queue_end].data, ptr, sizeof(primitive_land_t));  // copy data from struct into char array
            M_DEBUG("adding primitive of type %d with assigned ID %lu to queue position %d.\n", type, ((primitive_land_t*)ptr)->id, queue_end);
            break;
        case TAKE_OFF:
            // set queue item data
            queue[queue_end].data = (char*)malloc(sizeof(primitive_take_off_t));  // allocate memory for data array
            memcpy(queue[queue_end].data, ptr, sizeof(primitive_take_off_t));  // copy data from struct into char array
            M_DEBUG("adding primitive of type %d with assigned ID %lu to queue position %d.\n", type, ((primitive_take_off_t*)ptr)->id, queue_end);
            break;
        case SETPOINT:
            // set queue item data
            queue[queue_end].data = (char*)malloc(sizeof(primitive_setpoint_t));  // allocate memory for data array
            memcpy(queue[queue_end].data, ptr, sizeof(primitive_setpoint_t));  // copy data from struct into char array
            // M_DEBUG("adding primitive of type %d with assigned ID %lu to queue position %d.\n", type, ((primitive_setpoint_t*)ptr)->id, queue_end);
            break;
        case GO_TO_WAYPOINT:
            // set queue item data
            queue[queue_end].data = (char*)malloc(sizeof(primitive_go_to_waypoint_t));  // allocate memory for data array
            memcpy(queue[queue_end].data, ptr, sizeof(primitive_go_to_waypoint_t));  // copy data from struct into char array
            // M_DEBUG("adding primitive of type %d with assigned ID %lu to queue position %d.\n", type, ((primitive_go_to_waypoint_t*)ptr)->id, queue_end);
            break;
        case LOCAL_POLYNOMIAL:
            // set queue item data
            queue[queue_end].data = (char*)malloc(sizeof(primitive_local_polynomial_t));  // allocate memory for data array
            memcpy(queue[queue_end].data, ptr, sizeof(primitive_local_polynomial_t));  // copy data from struct into char array
            M_DEBUG("adding primitive of type %d with assigned ID %lu to queue position %d.\n", type, ((primitive_local_polynomial_t*)ptr)->id, queue_end);
            break;
    }

    queue_end++;
}


void manager_clear_queue() {
    pthread_mutex_lock(&queue_pos_mutex);
    queue_pos = queue_end-1;
    pthread_mutex_unlock(&queue_pos_mutex);
}


void clear_queue() {
    send_msg_to_app_manager(CLEAR_QUEUE_MSG);
}


void manager_stop_queue() {
    M_WARN("stop_queue called.\n");
    QUEUE_STOPPED_FLAG = 1;
    manager_clear_queue();
}


void stop_queue() {
    send_msg_to_app_manager(STOP_QUEUE_MSG);
}


void manager_pause_queue() {
    M_WARN("pause_queue called.\n");
    QUEUE_PAUSED_FLAG = 1;
}


void pause_queue() {
    send_msg_to_app_manager(PAUSE_QUEUE_MSG);
}


void manager_continue_queue() {
    M_WARN("continue_queue called.\n");
    QUEUE_PAUSED_FLAG = 0;
    PAUSE_PRIMITIVE_RUNNING = 0;
}


void continue_queue() {
    send_msg_to_app_manager(CONTINUE_QUEUE_MSG);
}


void manager_kill_power() {
    M_WARN("kill_power called.\n");

    // Send mavlink command to force disarm (MAV_CMD_COMPONENT_ARM_DISARM)
    mavlink_message_t msg;

    // Set up the command parameters
    uint8_t arm_disarm = 0; // 1 for arm, 0 for disarm
    uint8_t force = 21196;  // 0 to obey safety checks, 21196 to force

    mavlink_msg_command_long_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, \
                                  autopilot_monitor_get_sysid(), AUTOPILOT_COMPID,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_disarm, force, 0, 0, 0, 0, 0);

    mavlink_io_send_msg_to_ap(&msg);
    
}


void kill_power() {
    send_msg_to_app_manager(KILL_POWER_MSG);
}


void manager_force_arm() {
    M_WARN("force_arm called.\n");

    set_drone_vz_on_ground = 1;

    // Send mavlink command to force disarm (MAV_CMD_COMPONENT_ARM_DISARM)
    mavlink_message_t msg;

    // Set up the command parameters
    uint8_t arm_disarm = 1; // 1 for arm, 0 for disarm
    int force = 21196;  // 0 to obey safety checks, 21196 to force

    mavlink_msg_command_long_pack(autopilot_monitor_get_sysid(), VOXL_COMPID, &msg, \
                                  autopilot_monitor_get_sysid(), AUTOPILOT_COMPID,
                                  MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_disarm, force, 0, 0, 0, 0, 0);

    mavlink_io_send_msg_to_ap(&msg);
}


void force_arm() {
    send_msg_to_app_manager(FORCE_ARM_MSG);

    usleep(2500000);
}
