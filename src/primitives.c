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
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <limits.h>  // include this header for INT_MAX
#include <sys/time.h>
#include <stdint.h> // Include for uint64_t


#include <c_library_v2/development/mavlink.h>

// #include <modal_journal.h>  // libmodal_journal is not compatible with multi-processing
#include "modal_journal_lite.h"

#include "primitives.h"
#include "queue.h"
#include "voxl_app_manager.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "macros.h"
#include "misc.h"

#define MILLIS_PER_SEC 1000
#define NANOS_PER_MILLIS 1000000

#define PRIMITIVE_LOOP_HZ 40

#define DEFAULT_XY_ACCURACY 0.1
#define DEFAULT_Z_ACCURACY 0.1
#define DEFAULT_SPEED_ACCURACY 0.1
#define DEFAULT_SPEED_ACCURACY_LANDED 0.05  // allowed speed in m/s for the drone to be considered "landed"
#define DEFAULT_YAW_ACCUARCY 0.1309  // ~7.5 deg
#define DEFAULT_YAW_RATE_ACCURACY 0.523599  // ~30 deg per sec


void primitive_default_pause(primitive_pause_t* prim, int msecs) {
    prim->type = PAUSE;
    prim->duration = msecs;
}

void primitive_default_land(primitive_land_t* prim) {
    prim->type = LAND;
    prim->yaw = 0;
    prim->height = 0;
    prim->abort_height = prim->height - 0.25;
    prim->descent_rate = 0.2;
    prim->accuracy = DEFAULT_XY_ACCURACY;
}

void primitive_default_take_off(primitive_take_off_t* prim) {
    prim->type = TAKE_OFF;
    prim->height = 1;
    // prim->yaw = 0;
    prim->ascent_rate = 0.2;

    prim->x_accuracy = DEFAULT_XY_ACCURACY;
    prim->y_accuracy = DEFAULT_XY_ACCURACY;
    prim->z_accuracy = DEFAULT_Z_ACCURACY;
    prim->max_final_speed = DEFAULT_SPEED_ACCURACY;
    prim->yaw_accuracy = DEFAULT_YAW_ACCUARCY;
    prim->yaw_rate_accuracy = DEFAULT_YAW_RATE_ACCURACY;
}

void primitive_default_setpoint(primitive_setpoint_t* prim) {
    prim->type = SETPOINT;

    prim->duration = 33000;  // if adding 30 setpoints per second (30 Hz)

    prim->time_boot_ms = 0;  // this var is currently not used
    prim->target_component = AUTOPILOT_COMPID;

    prim->coordinate_frame = MAV_FRAME_LOCAL_NED;

    // apm_execute_primitive will later catch these NAN values and convert them to whatever the drone's current pos is
    prim->x = NAN, prim->y = NAN, prim->z = NAN;
    prim->vx = 0, prim->vy = 0, prim->vz = 0;
    prim->afx = 0, prim->afy = 0, prim->afz = 0;

    prim->yaw = NAN;
    prim->yaw_rate = 0;

    prim->type_mask = 0;  // do not ignore any of the fields
}

void primitive_default_go_to_waypoint(primitive_go_to_waypoint_t* prim) {
    prim->type = GO_TO_WAYPOINT;

    prim->x_accuracy = DEFAULT_XY_ACCURACY;
    prim->y_accuracy = DEFAULT_XY_ACCURACY;
    prim->z_accuracy = DEFAULT_Z_ACCURACY;
    prim->max_final_speed = DEFAULT_SPEED_ACCURACY;
    prim->yaw_accuracy = DEFAULT_YAW_ACCUARCY;
    prim->yaw_rate_accuracy = DEFAULT_YAW_RATE_ACCURACY;

    prim->time_boot_ms = 0;  // this var is currently not used
    prim->target_component = AUTOPILOT_COMPID;

    prim->coordinate_frame = MAV_FRAME_LOCAL_NED;

    // apm_execute_primitive will later catch these NAN values and convert them to whatever the drone's current pos is
    prim->x = NAN, prim->y = NAN, prim->z = NAN;
    prim->vx = 0, prim->vy = 0, prim->vz = 0;
    prim->afx = 0, prim->afy = 0, prim->afz = 0;

    prim->yaw = NAN;
    prim->yaw_rate = 0;

    // Only consider position
    prim->type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                      POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                      POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
}

void primitive_default_local_polynomial(primitive_local_polynomial_t* prim) {
    prim->n_coef = 2;  // parabolic trajectory
    prim->duration_s = 2;  // end at 
    prim->cx[0] = 0, prim->cx[1] = 1, prim->cx[2] = 0;
    prim->cy[0] = 0, prim->cy[1] = 0, prim->cy[2] = 1;
    prim->cz[0] = 1.5, prim->cz[1] = 0, prim->cz[2] = 0;
    prim->cyaw[0] = 0, prim->cyaw[1] = 0, prim->cyaw[2] = 0;
}


int check_app_status(primitive_t type) {
    if (available_apps.apps[available_apps.active_app[0]].status == INACTIVE) {
        switch(type) {
            case PAUSE:
                M_WARN("STOPPING PRIMITIVE TYPE PAUSE. Detected that corresponding app was interrupted.\n");
                break;
            case LAND:
                M_WARN("STOPPING PRIMITIVE TYPE LAND. Detected that corresponding app was interrupted.\n");
                break;
            case TAKE_OFF:
                M_WARN("STOPPING PRIMITIVE TYPE TAKE_OFF. Detected that corresponding app was interrupted.\n");
                break;
            case SETPOINT:
                M_WARN("STOPPING PRIMITIVE TYPE SETPOINT. Detected that corresponding app was interrupted.\n");
                break;
            case GO_TO_WAYPOINT:
                M_WARN("STOPPING PRIMITIVE TYPE GO_TO_WAYPOINT. Detected that corresponding app was interrupted.\n");
                break;
            case LOCAL_POLYNOMIAL:
                M_WARN("STOPPING PRIMITIVE TYPE LOCAL_POLYNOMIAL. Detected that corresponding app was interrupted.\n");
                break;
        }
        return APP_INTERRUPTED;
    }
    return 0;
}


void update_primitive_progress(float f, uint8_t add_instead_of_set) {
    if (add_instead_of_set) {
        pthread_mutex_lock(&primitive_progress_mutex);
        primitive_progress += f;
        pthread_mutex_unlock(&primitive_progress_mutex);
    }
    else {
        pthread_mutex_lock(&primitive_progress_mutex);
        primitive_progress = f;
        pthread_mutex_unlock(&primitive_progress_mutex);
    }
}


void catch_NAN_fields(mavlink_set_position_target_local_ned_t* pos) {
    // Set default values for target position if they are NaN
    if (isnan(pos->x) || isnan(pos->y) || isnan(pos->z) || isnan(pos->yaw)) {
        // Get current pos of drone
        mavlink_odometry_t cur_pos = autopilot_monitor_get_odometry();
        if (isnan(pos->x) || isnan(pos->y) || isnan(pos->z)) {
            printf("Coordinates of waypoint contained NAN. Defaulting to current position of the drone.\n");
            pos->x = cur_pos.x;
            pos->y = cur_pos.y;
            pos->z = cur_pos.z;
        }
        if (isnan(pos->yaw)) {
            printf("Waypoint yaw is equal to NAN. Defaulting to current yaw of the drone.\n");
            pos->yaw = get_yaw(&cur_pos);
        }
        pos->coordinate_frame = cur_pos.frame_id;
    }
}


int apm_execute_primitive(void* params, primitive_t type) {
    /**
     * IMPORTANT: MUST CHECK AT A FREQUENCY OF AT LEAST 10 Hz (this number was
     * derived from the usleep(100000) in main.c stop_active_app) WHETHER THE
     * CURRENT ACTIVE APP IS NO LONGER ACTIVE, IN WHICH CASE, HALT THE PRIMITIVE
     * AND RETURN `APP_INTERRUPTED`. IF THE NEW APP IS ALLOWED TO BEGIN BEFORE
     * THIS PRIMITIVE IS HALTED, TWO INSTANCES OF THIS FUNCTION MAY RUN AT THE
     * SAME TIME AND FIGHT FOR CONTROL OVER THE DRONE.
     * 
     * Note: for this reason, the duration of SETPOINT is limited to 100 msecs.
     * 
     * IMPORTANT: MUST CALL mavlink_io_send_local_setpoint AT
     * LEAST 2X PER SECOND IN ORDER TO PREVENT PX4 FROM AUTOMATICALLY EXITING
     * OFFBOARD MODE. 
     * 
     * IMPORTANT: MUST set_last_mavlink_cmd_xyz BEFORE RETURNING FROM THIS
     * FUNCTION! IF THIS PRIMITIVE HAPPENS TO BE THE LAST PRIMITIVE IN QUEUE,
     * THE DRONE WILL CONTINUE EXECUTING last_mavlink_cmd AFTER THIS FUNCTION
     * RETURNS. THEREFORE, SET last_mavlink_cmd TO A REASONABLE VALUE.
     */

    // Rate that the loop that runs the primitives will run at
    #define RATE 100  // Hz

    // Struct to contain measured time when this function ends
    struct timeval currentTime;

    // Save the drone's start position
    mavlink_odometry_t start_pos = autopilot_monitor_get_odometry();

    // Create mavlink_set_position_target_local_ned_t that will be used throughout this function and sent to PX4 (for the relevant primitive types)
    mavlink_set_position_target_local_ned_t pos;

    // Perform any 1-time operations for each primitive type
    uint8_t override_loop_sleep = 0;  // used by SETPOINT primitive to conduct a more precise sleep
    float max_ms;  // timeout/duration for primitive
    uint8_t timed_out;  // denote whether max_ms is a timeout or just a duration 
    float start_dist;  // only used by GO_TO_WAYPOINT and LAND
    uint8_t stage;       // only used by LAND and TAKEOFF
    switch(type) {
        case PAUSE: {
            primitive_pause_t* primitive = (primitive_pause_t*)params;
            M_DEBUG("pausing queue execution (primitive id %lu).\n", primitive->id);
            printf("SENDING SETPOINT: pos: (%f, %f, %f, %f), vel: (%f, %f, %f), acc: (%f, %f, %f). type_mask = %d\n", last_mavlink_cmd.x, last_mavlink_cmd.y, last_mavlink_cmd.z, last_mavlink_cmd.yaw, last_mavlink_cmd.vx, last_mavlink_cmd.vy, last_mavlink_cmd.vz, last_mavlink_cmd.afx, last_mavlink_cmd.afy, last_mavlink_cmd.afz, last_mavlink_cmd.type_mask);

            // Set flag to 1. If this flag is set to 0 during the pause (if continue_queue() is called), the pause will end.
            PAUSE_PRIMITIVE_RUNNING = 1;

            // Set primitive duration
            if (primitive->duration == -1) {
                max_ms = INT_MAX;
            }
            else {
                max_ms = primitive->duration;
            }
            timed_out = 0;  // default value for timed out is 0 (it's not possible for pause to time out)
        } break;
        case LAND: {
            primitive_land_t* primitive = (primitive_land_t*)params;
            M_DEBUG("landing primitive id %lu with yaw %f, height %f, and abort height %f.\n", primitive->id, primitive->yaw, primitive->height, primitive->abort_height);

            max_ms = 20000;
            timed_out = 1;

            stage = 1;  // the landing is a 2 stage process, starting with stage 1

            // These position values are for stage 1 of the landing
            pos.time_boot_ms = 0;
            pos.target_system = 0;
            pos.target_component = AUTOPILOT_COMPID;
            pos.coordinate_frame = MAV_FRAME_LOCAL_NED;

            // Target the desired yaw and the z-midpoint (between start height and target landing height) while maintaining current X and Y pos
            pos.x = start_pos.x;
            pos.y = start_pos.y;
            pos.z = (primitive->height + start_pos.z) / 2;
            // zero out vel and acc which are being masked out anyway
            pos.vx = 0;
            pos.vy = 0;
            pos.vz = 0;
            pos.afx = 0;
            pos.afy = 0;
            pos.afz = 0;
            pos.yaw = primitive->yaw;

            // Use PX4's default velocities/accelerations
            pos.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                            POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                            POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

            catch_NAN_fields(&pos);

            start_dist = fabs(primitive->height - start_pos.z);

        } break;
        case TAKE_OFF: {
            primitive_take_off_t* primitive = (primitive_take_off_t*)params;

            max_ms = 20000;
            timed_out= 1;  // default value for timed out is 1, if the loop exits naturally after max_ms
            stage = 1;

            pos.time_boot_ms = 0;
            pos.target_system = 0;
            pos.target_component = AUTOPILOT_COMPID;
            pos.coordinate_frame = MAV_FRAME_LOCAL_NED;

            // Set initial values (will change as trajectory happens)
            pos.x = start_pos.x;
            pos.y = start_pos.y;
            pos.z = start_pos.z;
            pos.vx = 0;
            pos.vy = 0;
            pos.vz = 0;
            pos.afx = 0;
            pos.afy = 0;
            pos.afz = 0;
            // Simply target the desired yaw
            // pos.yaw = primitive->yaw;
            pos.yaw = get_yaw(&start_pos);
            pos.type_mask = 0;

            catch_NAN_fields(&pos);

            M_DEBUG("taking off (primitive id %lu).\n", primitive->id);
            M_DEBUG("target pos: {%f, %f, %f, %f}\n", pos.x, pos.y, pos.z, pos.yaw);
        } break;
        case SETPOINT: {
            primitive_setpoint_t* primitive = (primitive_setpoint_t*)params;

            // Set a max of 100 msecs for duration of SETPOINT (see function block comment to understand why this is necessary)
            if (primitive -> duration > 100) primitive->duration = 100;

            max_ms = primitive->duration;

            timed_out = 0;  // default value for timed out is 0; expected behavior is that the loop exits naturally after max_ms

            /*
            For the most part, we can just cast the instance of primitive_setpoint_t into a mavlink_set_position_target_local_ned_t.
            We just need to offset by the size of the first few fields in primitive_setpoint_t that are not in mavlink_set_position_target_local_ned_t.
            */
            mavlink_set_position_target_local_ned_t* pos_temp = (mavlink_set_position_target_local_ned_t*)(params + sizeof(primitive->type) + sizeof(primitive->id) + sizeof(primitive->duration));
            // Manually set target_system
            pos_temp->target_system = 0;

            // copy data from block-scope pos_temp to function-scope pos
            memcpy(&pos, pos_temp, sizeof(*pos_temp));
            
            catch_NAN_fields(&pos);

            // If we know when the last primitive finished, override the loop sleep and dynamically calculate how long to sleep to maintain timing of setpoints
            if (last_primitive_finish_time) {
                override_loop_sleep = 1;
            }

            M_DEBUG("following setpoint (primitive id %lu).\n", primitive->id);
            M_DEBUG("target pos: {%f, %f, %f, %f}\n", pos.x, pos.y, pos.z, pos.yaw);
        } break;
        case GO_TO_WAYPOINT: {
            primitive_go_to_waypoint_t* primitive = (primitive_go_to_waypoint_t*)params;

            max_ms = 600000;
            timed_out = 1;  // default value for timed out is 1, if the loop exits naturally after max_ms

            /*
            For the most part, we can just cast the instance of primitive_go_to_waypoint_t into a mavlink_set_position_target_local_ned_t.
            We just need to offset by the size of the first few fields in primitive_go_to_waypoint_t that are not in mavlink_set_position_target_local_ned_t.
            */
            mavlink_set_position_target_local_ned_t* pos_temp = (mavlink_set_position_target_local_ned_t*)(params + sizeof(primitive->type) + sizeof(primitive->id) + sizeof(primitive->x_accuracy) + sizeof(primitive->y_accuracy) + sizeof(primitive->z_accuracy) + sizeof(primitive->max_final_speed) + sizeof(primitive->yaw_accuracy) + sizeof(primitive->yaw_rate_accuracy));
            // Manually set target_system
            pos_temp->target_system = 0;

            printf("pos_temp->x: %f\n", pos_temp->x);
            printf("pos_temp->x: %f\n", pos_temp->y);
            printf("pos_temp->x: %f\n", pos_temp->z);

            // copy data from block-scope pos_temp to function-scope pos
            memcpy(&pos, pos_temp, sizeof(*pos_temp));

            catch_NAN_fields(&pos);

            M_DEBUG("going to waypoint (primitive id %lu).\n", primitive->id);
            M_DEBUG("target pos: {%f, %f, %f, %f}\n", pos.x, pos.y, pos.z, pos.yaw);

            // Compute starting distance (squared) from waypoint to keep track of percent progress
            pthread_mutex_lock(&cur_pos_mutex);
            start_dist = pow(pos.x - cur_pos.x, 2) + pow(pos.y - cur_pos.y, 2) + pow(pos.z - cur_pos.z, 2);
            pthread_mutex_unlock(&cur_pos_mutex);

        } break;
        case LOCAL_POLYNOMIAL: {
            primitive_local_polynomial_t* primitive = (primitive_local_polynomial_t*)params;
            M_DEBUG("following local polynomial (primitive id %lu).\n", primitive->id);

            max_ms = 600000;
            timed_out = 1;  // default value for timed out is 1, if the loop exits naturally after max_ms

            // TO BE IMPLEMENTED
        } break;
    }


    // Reset progress variable
    update_primitive_progress(0.0, 0);


    // Set variables to run loop at constant rate
    int64_t next_time = 0;
    int ctr = max_ms * (RATE / 1000.0);  // divide by 10^3 to convert rate to loops per msec

    // Start a loop to keep track of drone's progress toward target altitude and monitor app's status
    while(ctr >= 0) {

        switch(type) {

        ////////////////////////////////////////////////////////////////////////
        // PAUSE
        ////////////////////////////////////////////////////////////////////////
        case PAUSE: {
            primitive_pause_t* primitive = (primitive_pause_t*)params;
            
            // Even tho the drone isn't supposed to move, need to continue sending stream of setpoints so it doesn't fall out of ofboard mode
            mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, last_mavlink_cmd);

            if (max_ms != INT_MAX) {
                update_primitive_progress(1/ctr, 1);  // increase progress as time passes
            }
        } break;


        ////////////////////////////////////////////////////////////////////////
        // LAND
        ////////////////////////////////////////////////////////////////////////
        case LAND: {
            primitive_land_t* primitive = (primitive_land_t*)params;
            
            /*
            Landing is 2-stage: first, the drone goes to a position-controlled
            setpoint a small distance above the ground. Then, the drone will
            switch to a velocity-controlled descent until touching down.

            NOTE: STAGE 1 IS CURRENTLY DISABLED (commented out).
            */

            // LANDING STAGE 1 (high-accuracy position setpoint)
            // if (stage == 1) {
            //     // tell PX4 to go to position setpoint
            //     mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

            //     // lock mutex so that cur_pos does not change during this comparison
            //     pthread_mutex_lock(&cur_pos_mutex);
            //     // check if we're within accuracy of the position setpoint
            //     // printf("distance: %f\n", pow(cur_pos.x - pos.x, 2) + pow(cur_pos.y - pos.y, 2) + pow(cur_pos.z - pos.z, 2));
            //     // printf("pow(primitive->accuracy, 2): %f\n", pow(primitive->accuracy, 2));
            //     if (pow(cur_pos.x - start_pos.x, 2) + pow(cur_pos.y - start_pos.y, 2) + pow(cur_pos.z - pos.z, 2) <= pow(primitive->accuracy, 2)) {
                    
            //         pthread_mutex_unlock(&cur_pos_mutex);
                    
                    // modify the setpoint target to ignore position and consider velocity in preparation for stage 2 of landing
                    pos.vx = 0;
                    pos.vy = 0;
                    pos.vz = 0.35;
                    pos.type_mask = POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE | POSITION_TARGET_TYPEMASK_Z_IGNORE |
                                    POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                    POSITION_TARGET_TYPEMASK_FORCE_SET | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

            //         stage = 2;  // switch to stage 2 of landing
            //     }
            //     else {
            //         pthread_mutex_unlock(&cur_pos_mutex);
            //     }
            // }


            // LANDING STAGE 2 (velocity-controlled touchdown)
            // else if (stage == 2) {
            if (1) {
                // tell PX4 to go to velocity setpoint
                mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

                // lock mutex so that cur_pos does not change during this comparison
                pthread_mutex_lock(&cur_pos_mutex);
                // Just check if we're at 0 velocity, which means we're likely touching the ground
                printf("velocity^2: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
                if (pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2) <= pow(DEFAULT_SPEED_ACCURACY_LANDED, 2)) {
                    
                    pthread_mutex_unlock(&cur_pos_mutex);
                    
                    // set progress to 100% complete
                    update_primitive_progress(1.0, 0);
                    
                    timed_out = 0;
                    goto PRIMITIVE_FINISHED;
                }
                else {
                    pthread_mutex_unlock(&cur_pos_mutex);
                }
            }

            // primitive progress based on z-distance from target landing height
            update_primitive_progress(fabs(cur_pos.z - start_pos.z) / start_dist, 0);

            if (!MOVE_PROPS) {
                // printf("cur_pos: (%f, %f, %f, %f)\n", cur_pos.x, cur_pos.y, cur_pos.z, get_yaw(&cur_pos));
            }
        } break;


        ////////////////////////////////////////////////////////////////////////
        // TAKE_OFF
        ////////////////////////////////////////////////////////////////////////
        case TAKE_OFF: {
            primitive_take_off_t* primitive = (primitive_take_off_t*)params;
            

            /*
            Calculate a partial-S-curve trajectory for the takeoff:

            Velocity:
            |            .'.           
            |          .'   '.
            |       ..'       '.
            |....'''            '.
            ------------------------- 0

            Accel:
            |        ......
            |    ..''     .
            |..''         .
            |-------------.---------- 0
            |             .
            |             ........
            */

            if (stage == 1) {  // velocity is increasing
                // Check if we need to begin constant-accel deceleration now
                float distance_from_target = pos.z - (start_pos.z - primitive->height);
                // This means that even if we decelerated at max acceleration now, we will overshoot the target
                if (pow(pos.vz, 2) / (2*MAX_Z_ACC) >= distance_from_target) {
                    stage = 2;  // Switch to the "deceleration stage"
                    pos.afz = MAX_Z_ACC;  // Set new constant decceleration
                }
                else {
                    // Increase accel from constant jerk
                    pos.afz = fmax(-MAX_Z_ACC, pos.afz - MAX_JERK*(1.0/RATE));
                    // Increase velocity based on constant jerk and current accel
                    pos.vz = pos.vz + pos.afz*(1.0/RATE) + (1/2)*MAX_JERK*pow((1.0/RATE), 2);
                    // Calculate new target position based on current velocity, accel, and constant jerk
                    pos.z = pos.z + pos.vz*(1.0/RATE) + (1/2)*pos.afz*pow((1.0/RATE), 2) + (1/6)*MAX_JERK*pow((1.0/RATE), 3);
                }
            }
            if (stage == 2) {  // do not use `else if` so that when the first `if` statement sets stage = 2, this second `if` statement will run immediately after
                // Constant accel deceleration (note that pos.afz was already set to a constant value above)
                pos.vz = fmin(0, pos.vz + pos.afz*(1.0/RATE));
                pos.z = fmax(start_pos.z - primitive->height, pos.z + pos.vz*(1.0/RATE) + (1/2)*pos.afz*pow((1.0/RATE), 2));
            }
            

            // Send Mavlink target setpoint to PX4
            // printf("pos.z: %f\n", pos.z);
            // printf("pos.vz: %f\n", pos.vz);
            // printf("pos.afz: %f\n", pos.afz);
            mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

            // lock mutex so that the cur_pos (a global variable) doesn't change during this comparison
            pthread_mutex_lock(&cur_pos_mutex);

            // calculate z-distance to target takeoff height to determine progress
            update_primitive_progress(zero_bound((cur_pos.z - start_pos.z) / primitive->height), 0);  // approximate z-distance from target

            // Check if drone has reached required accuracy requirement
            // printf("x_dist: %f\n", fabs(cur_pos.x - pos.x));
            // printf("y_dist: %f\n", fabs(cur_pos.y - pos.y));
            // printf("z_dist: %f\n", fabs(cur_pos.z - (start_pos.z - primitive->height)));
            // printf("speed^2: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
            // printf("yaw_delta: %f\n", fabs(get_yaw(&cur_pos) - pos.yaw));
            // printf("yaw_rate: %f\n", fabs(cur_pos.yawspeed));
            if (fabs(cur_pos.x - pos.x) <= primitive->x_accuracy &&
                fabs(cur_pos.y - pos.y) <= primitive->y_accuracy &&
                fabs(cur_pos.z - (start_pos.z - primitive->height)) <= primitive->z_accuracy &&
                pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2) <= pow(primitive->max_final_speed, 2) &&
                fabs(get_yaw(&cur_pos) - pos.yaw) <= primitive->yaw_accuracy &&
                fabs(cur_pos.yawspeed) <= primitive->yaw_rate_accuracy) {
                    
                pthread_mutex_unlock(&cur_pos_mutex);
                
                // set progress to 100% complete
                update_primitive_progress(1.0, 0);
                
                timed_out = 0;

                goto PRIMITIVE_FINISHED;
            }
            else {
                pthread_mutex_unlock(&cur_pos_mutex);
            }

            if (!MOVE_PROPS) {
                printf("cur_pos: (%f, %f, %f, %f)\n", cur_pos.x, cur_pos.y, cur_pos.z, get_yaw(&cur_pos));
                // printf("cur_pos.v: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
                // printf("primitive_progress: %f\n\n", primitive_progress);
            }
        } break;


        ////////////////////////////////////////////////////////////////////////
        // SETPOINT
        ////////////////////////////////////////////////////////////////////////
        case SETPOINT: {
            primitive_setpoint_t* primitive = (primitive_setpoint_t*)params;

            // Send setpoint Mavlink command to make the drone move
            mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

            if (override_loop_sleep) {
                // Calculate how much time has elapsed since last primitive finished
                struct timeval currentTime;
                gettimeofday(&currentTime, NULL);
                uint64_t time_passed_usecs = (uint64_t)currentTime.tv_sec * 1000000 + (uint64_t)currentTime.tv_usec - last_primitive_finish_time;
                
                // sleep for the appropriate amount of time so that precisely `primitive->duration` msecs has elapsed between the end of the last primitive and now.
                usleep(primitive->duration*1000 - time_passed_usecs);

                // No need to keep looping, so break out
                goto PRIMITIVE_FINISHED;
            }
        } break;


        ////////////////////////////////////////////////////////////////////////
        // GO_TO_WAYPOINT
        ////////////////////////////////////////////////////////////////////////
        case GO_TO_WAYPOINT: {
            primitive_go_to_waypoint_t* primitive = (primitive_go_to_waypoint_t*)params;

            // Send setpoint Mavlink command to make the drone move
            mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

            // Compute current distance from waypoint (squared) to determine percentage progress
            pthread_mutex_lock(&cur_pos_mutex);
            float x_dist = fabs(cur_pos.x - pos.x);
            float y_dist = fabs(cur_pos.y - pos.y);
            float z_dist = fabs(cur_pos.z - pos.z);
            pthread_mutex_unlock(&cur_pos_mutex);
            float cur_dist = pow(x_dist, 2) + pow(y_dist, 2) + pow(z_dist, 2);

            // Compute current progress (inversely proportional to distance from setpoint)
            update_primitive_progress(sqrt(zero_bound((start_dist - cur_dist) / start_dist)), 0);

            // Check if we're within final accuracy to waypoint
            // printf("x_dist: %f\n", x_dist);
            // printf("y_dist: %f\n", y_dist);
            // printf("z_dist: %f\n", z_dist);
            // printf("speed^2: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
            // printf("yaw_delta: %f\n", fabs(get_yaw(&cur_pos) - primitive->yaw));
            // printf("yaw_rate: %f\n", fabs(cur_pos.yawspeed));
            pthread_mutex_lock(&cur_pos_mutex);
            if (x_dist <= primitive->x_accuracy &&
                y_dist <= primitive->y_accuracy &&
                z_dist <= primitive->z_accuracy &&
                pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2) <= pow(primitive->max_final_speed, 2) &&
                fabs(get_yaw(&cur_pos) - primitive->yaw) <= primitive->yaw_accuracy &&
                fabs(cur_pos.yawspeed) <= primitive->yaw_rate_accuracy) {

                pthread_mutex_unlock(&cur_pos_mutex);

                timed_out = 0;
                goto PRIMITIVE_FINISHED;
            }
            else {
                pthread_mutex_unlock(&cur_pos_mutex);
            }

            if (!MOVE_PROPS) {
                printf("cur_pos: (%f, %f, %f, %f)\n", cur_pos.x, cur_pos.y, cur_pos.z, get_yaw(&cur_pos));
                // printf("cur_pos.v: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
                // printf("primitive_progress: %f\n\n", primitive_progress);
            }
        } break;


        ////////////////////////////////////////////////////////////////////////
        // LOCAL_POLYNOMIAL
        ////////////////////////////////////////////////////////////////////////
        case LOCAL_POLYNOMIAL: {
            primitive_local_polynomial_t* primitive = (primitive_local_polynomial_t*)params;
            
            // TO BE IMPLEMENTED
        } break;
        }



        ////////////////////////////////////////////////////////////////////////
        // LOOP COUNTING/SLEEPING AND BREAK CHECKS
        ////////////////////////////////////////////////////////////////////////

        // Check that app hasn't been interrupted
        if (check_app_status(type) == APP_INTERRUPTED) {
            mavlink_odometry_t odom = autopilot_monitor_get_odometry();
            set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
            return APP_INTERRUPTED;
        }

        // Check that primitive hasn't been stopped
        if (QUEUE_STOPPED_FLAG) {
            mavlink_odometry_t odom = autopilot_monitor_get_odometry();
            set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
            return QUEUE_STOPPED;
        } 

        // Check that primitive hasn't been paused
        if (QUEUE_PAUSED_FLAG) {
            int64_t start_pause_time = my_time_monotonic_ns();
            while (QUEUE_PAUSED_FLAG) {
                usleep(10000);  // wait until unpaused
            }
            int64_t end_pause_time = my_time_monotonic_ns();

            // offset next_time so that the paused time doesn't affect the primitive's own timer
            next_time += ((end_pause_time - start_pause_time));
        }

        /* 
        Note: voxl-app-manager already monitors whether drone is armed and in
        offboard mode; if not, voxl-app-manager will stop the running app. 
        Therefore, there is no need to do an additional check of whether the
        drone is armed and in offboard mode in this loop.
        */

        // If primitive is of type PAUSE, check whether pause was ended (if continue_queue() has been run)
        if (type == PAUSE && !PAUSE_PRIMITIVE_RUNNING) {
            break;
        }

        ctr--;

        // Sleep the appropriate amount of time to maintain loop rate
        if(!override_loop_sleep && my_loop_sleep(RATE, &next_time)){
            // fprintf(stderr, "WARNING: sleep loop fell behind.\n");
        }
    }

PRIMITIVE_FINISHED:

    // Keep track of time when this primitive ends (so the next primitive can be timed appropriately)
    // NOTE: this code should be placed as close as possible to when the elapsed time is measured (which is in the while loop above) for most accurate timing.
    gettimeofday(&currentTime, NULL);
    last_primitive_finish_time = (uint64_t)currentTime.tv_sec * 1000000 + (uint64_t)currentTime.tv_usec;

    update_primitive_progress(1.0, 0);

    // Set last_mavlink_cmd depending on which primitive the drone just executed
    switch(type) {
        case PAUSE:
            break;
        case LAND:
            set_last_mavlink_cmd_on_ground();
            break;
        case TAKE_OFF:
        case SETPOINT:
        case GO_TO_WAYPOINT:
        case LOCAL_POLYNOMIAL:
            set_last_mavlink_cmd_xyz(pos.x, pos.y, pos.z);
            break;
    }

    // Return appropriate event depending on whether the primitive finished with a success or a timeout
    if (timed_out) {
        return PRIMITIVE_TIMEOUT;
    }
    else {
        return 0;  // success
    }
    












    // switch(type) {
    //     ////////////////////////////////////////////////////////////////////////
    //     // PAUSE
    //     ////////////////////////////////////////////////////////////////////////
    //     case PAUSE: {
    //         primitive_pause_t* primitive = (primitive_pause_t*)params;  // cast char array back to struct
    //         M_DEBUG("pausing queue execution (primitive id %lu).\n", primitive->id);

    //         PAUSE_PRIMITIVE_RUNNING = 1;

    //         mavlink_odometry_t start_pos = autopilot_monitor_get_odometry();

    //         // Construct mavlink_set_position_target_local_ned_t that we can send to PX4 over mavlink
    //         mavlink_set_position_target_local_ned_t pos;
    //         pos.time_boot_ms = 0;
    //         // Target the desired Z height and altitude while maintaining current X and Y pos
    //         pos.x = start_pos.x;
    //         pos.y = start_pos.y;
    //         pos.z = start_pos.z;
    //         // For now, takeoff will be forced to use PX4's default velocity/acceleration
    //         pos.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    //                         POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    //                         POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    //         pos.target_system = 0;
    //         pos.target_component = AUTOPILOT_COMPID;
    //         pos.coordinate_frame = MAV_FRAME_LOCAL_NED;

    //         update_primitive_progress(0.0, 0);

    //         if (primitive->duration == -1) {
    //             // pause indefinitely
    //             while (1) {
    //                 // Even tho the drone isn't supposed to move, need to continue sending stream of setpoints so it doesn't fall out of ofboard mode
    //                 mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

    //                 // check if app was halted
    //                 if (check_app_status(type) == APP_INTERRUPTED) {
    //                     set_last_mavlink_cmd_xyz(start_pos.x, start_pos.y, start_pos.z);
    //                     return APP_INTERRUPTED;
    //                 }
    //                 usleep(1000);
    //             }
    //         }
    //         else if (primitive->duration >= 0) {

    //             int64_t next_time = 0;
    //             int rate = 100;  // loop 100 times per sec
    //             int ctr = primitive->duration * (rate / 1000.0);  // divide by 10^3 to convert rate to loops per msec

    //             while(ctr > 0) {
    //                 // Even tho the drone isn't supposed to move, need to continue sending stream of setpoints so it doesn't fall out of ofboard mode
    //                 mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

    //                 // check if app was halted
    //                 if (check_app_status(type) == APP_INTERRUPTED) {
    //                     mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    //                     set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
    //                     return APP_INTERRUPTED;
    //                 }

    //                 ctr--;
    //                 update_primitive_progress(1/ctr, 1);  // increase progress as time passes
                    
    //                 // Sleep the appropriate amount of time to maintain loop rate
    //                 if(my_loop_sleep(rate, &next_time)){
    //                     fprintf(stderr, "WARNING: pause sleep loop fell behind.\n");
    //                 }
    //             }

    //             update_primitive_progress(1.0, 0);
    //             set_last_mavlink_cmd_xyz(start_pos.x, start_pos.y, start_pos.z);
    //             return 0;  // SUCCESS
    //         }
    //         else {
    //             M_ERROR("PAUSE primitive has invalid duration of %d ms. Must be between -1 and inf. Unable to execute pause, aborting.\n", primitive->duration);
    //         }
    //     } break;


    //     ////////////////////////////////////////////////////////////////////////
    //     // TAKE_OFF
    //     ////////////////////////////////////////////////////////////////////////
    //     case TAKE_OFF: {
    //         primitive_take_off_t* primitive = (primitive_take_off_t*)params;  // cast char array back to struct
    //         M_DEBUG("taking off from primitive id %lu with yaw %f and height %f.\n", primitive->id, primitive->yaw, primitive->height);
            
    //         update_primitive_progress(0.0, 0);

    //         // Find the drone's starting position (so that we can try to maintain the XY pos during takeoff)
    //         mavlink_odometry_t start_pos = autopilot_monitor_get_odometry();

    //         // Construct mavlink_set_position_target_local_ned_t that we can send to PX4 over mavlink
    //         mavlink_set_position_target_local_ned_t pos;
    //         pos.time_boot_ms = 0;
    //         // Target the desired Z height and altitude while maintaining current X and Y pos
    //         pos.x = start_pos.x;
    //         pos.y = start_pos.y;
    //         pos.z = primitive->height;
    //         pos.yaw = primitive->yaw;
    //         // For now, takeoff will be forced to use PX4's default velocity/acceleration
    //         pos.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    //                         POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    //                         POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    //         pos.target_system = 0;
    //         pos.target_component = AUTOPILOT_COMPID;
    //         pos.coordinate_frame = MAV_FRAME_LOCAL_NED;

    //         // Start a loop to keep track of drone's progress toward target altitude and monitor app's status
    //         int64_t next_time = 0;
    //         int rate = 100;  // loop 100 times per sec
    //         int max_takeoff_ms = 20000;  // if takeoff takes more than 20 seconds, return an error
    //         int ctr = max_takeoff_ms * (rate / 1000.0);  // divide by 10^3 to convert rate to loops per msec
    //         while(1) {
    //             // Send Mavlink target to make the drone move
    //             mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);
                
    //             // check if app was halted
    //             if (check_app_status(type) == APP_INTERRUPTED) {
    //                 mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    //                 set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
    //                 return APP_INTERRUPTED;
    //             }
    //             if (ctr <= 0) {
    //                 M_ERROR("Takeoff timeout reached but drone is not within desired accuracy. Returning error.\n");
    //                 mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    //                 set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
    //                 return PRIMITIVE_TIMEOUT;
    //             }

    //             ctr--;

    //             pthread_mutex_lock(&cur_pos_mutex);
    //             update_primitive_progress(zero_bound((cur_pos.z - start_pos.z) / (primitive->height - start_pos.z)), 0);  // approximate z-distance from target

    //             if (fabs(cur_pos.x - pos.x) <= primitive->x_accuracy &&
    //                 fabs(cur_pos.y - pos.y) <= primitive->y_accuracy &&
    //                 fabs(cur_pos.z - pos.z) <= primitive->z_accuracy &&
    //                 pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2) <= pow(primitive->max_final_speed, 2) &&
    //                 fabs(get_yaw(&cur_pos) - pos.yaw) <= primitive->yaw_accuracy &&
    //                 fabs(cur_pos.yawspeed) <= primitive->yaw_rate_accuracy) {
                        
    //                 pthread_mutex_unlock(&cur_pos_mutex);
                    
    //                 // achieved the required accuracy requirements
    //                 update_primitive_progress(1.0, 0);
                    
    //                 set_last_mavlink_cmd_xyz(pos.x, pos.y, pos.z);
    //                 return 0;  // SUCCESS
    //             }
    //             else {
    //                 pthread_mutex_unlock(&cur_pos_mutex);
    //             }
                

    //             // printf("cur_pos: (%f, %f, %f, %f)\n", cur_pos.x, cur_pos.y, cur_pos.z, get_yaw(&cur_pos));
    //             // printf("cur_pos.yaw: %f\n", get_yaw(&cur_pos));
    //             // printf("cur_pos.v: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
    //             // printf("primitive_progress: %f\n\n", primitive_progress);

    //             // Sleep the appropriate amount of time to maintain loop rate
    //             if(my_loop_sleep(rate, &next_time)){
    //                 fprintf(stderr, "WARNING: takeoff sleep loop fell behind.\n");
    //             }
    //         }
    //     } break;


    //     ////////////////////////////////////////////////////////////////////////
    //     // SETPOINT
    //     ////////////////////////////////////////////////////////////////////////
    //     case SETPOINT: {
    //         primitive_setpoint_t* primitive = (primitive_setpoint_t*)params;  // cast char array back to struct
    //         M_DEBUG("following setpoint (primitive id %lu).\n", primitive->id);

    //         update_primitive_progress(0.0, 0);

    //         M_DEBUG("target pos: {%f, %f, %f, %f}\n", primitive->x, primitive->y, primitive->z, primitive->yaw);

    //         /*
    //         For the most part, we can just cast the instance of primitive_setpoint_t into a mavlink_set_position_target_local_ned_t.
    //         We just need to offset by the size of the first few fields in primitive_setpoint_t that are not in mavlink_set_position_target_local_ned_t.
    //         */
    //         mavlink_set_position_target_local_ned_t* pos = (mavlink_set_position_target_local_ned_t*)(params + sizeof(primitive->type) + sizeof(primitive->id) + sizeof(primitive->duration));

    //         // Manually set target_system
    //         pos->target_system = 0;

    //         // Set default values for target position if they are NaN
    //         if (isnan(pos->x) || isnan(pos->y) || isnan(pos->z) || isnan(pos->yaw)) {
    //             // Get current pos of drone
    //             mavlink_odometry_t new_pos = autopilot_monitor_get_odometry();
    //             if (isnan(pos->x) || isnan(pos->y) || isnan(pos->z)) {
    //                 printf("Coordinates of setpoint contained NAN. Defaulting to current position of the drone.\n");
                    
    //                 pos->x = new_pos.x;
    //                 pos->y = new_pos.y;
    //                 pos->z = new_pos.z;
    //                 pos->coordinate_frame = new_pos.frame_id;
    //             }
    //             if (isnan(pos->yaw)) {
    //                 printf("Setpoint yaw is equal to NAN. Defaulting to current yaw of the drone.\n");
    //                 pos->yaw = get_yaw(&new_pos);
    //             }
    //         }

    //         // Enter loop to constantly check app's status while waiting for the setpoint's duration
    //         int64_t next_time = 0;
    //         int rate = 100;  // loop 100 times per sec
    //         int ctr = primitive->duration * (rate / 1000000.0);  // divide by 10^6 to convert rate to loops per microsecond
    //         while(ctr > 0) {
    //             // Send setpoint Mavlink command to make the drone move
    //             mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, *pos);

    //             // check if app was halted
    //             if (check_app_status(type) == APP_INTERRUPTED) {
    //                 mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    //                 set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
    //                 return APP_INTERRUPTED;
    //             }

    //             ctr--;
    //             update_primitive_progress(1/ctr, 1);  // increase progress as time passes
                
    //             // Sleep the appropriate amount of time to maintain loop rate
    //             if(my_loop_sleep(rate, &next_time)){
    //                 fprintf(stderr, "WARNING: setpoint sleep loop fell behind.\n");
    //             }
    //         }

    //         // Set primitive progress to 100% after it's finished
    //         update_primitive_progress(1.0, 0);
    //         set_last_mavlink_cmd_xyz(pos->x, pos->y, pos->z);
    //         return 0;  // SUCCESS
    //     } break;


    //     ////////////////////////////////////////////////////////////////////////
    //     // GO_TO_WAYPOINT
    //     ////////////////////////////////////////////////////////////////////////
    //     case GO_TO_WAYPOINT: {
    //         primitive_go_to_waypoint_t* primitive = (primitive_go_to_waypoint_t*)params;  // cast char array back to struct
    //         M_DEBUG("going to waypoint (primitive id %lu).\n", primitive->id);

    //         update_primitive_progress(0.0, 0);

    //         M_DEBUG("target pos: {%f, %f, %f, %f}\n", primitive->x, primitive->y, primitive->z, primitive->yaw);

    //         /*
    //         For the most part, we can just cast the instance of primitive_go_to_waypoint_t into a mavlink_set_position_target_local_ned_t.
    //         We just need to offset by the size of the first few fields in primitive_go_to_waypoint_t that are not in mavlink_set_position_target_local_ned_t.
    //         */
    //         mavlink_set_position_target_local_ned_t* pos = (mavlink_set_position_target_local_ned_t*)(params + sizeof(primitive->type) + sizeof(primitive->id) + sizeof(primitive->x_accuracy) + sizeof(primitive->y_accuracy) + sizeof(primitive->z_accuracy) + sizeof(primitive->max_final_speed) + sizeof(primitive->yaw_accuracy) + sizeof(primitive->yaw_rate_accuracy));
            
    //         // Manually set target_system
    //         pos->target_system = 0;

    //         // Set default values for target position if they are NaN
    //         if (isnan(pos->x) || isnan(pos->y) || isnan(pos->z) || isnan(pos->yaw)) {
    //             // Get current pos of drone
    //             mavlink_odometry_t new_pos = autopilot_monitor_get_odometry();
    //             if (isnan(pos->x) || isnan(pos->y) || isnan(pos->z)) {
    //                 printf("Coordinates of waypoint contained NAN. Defaulting to current position of the drone.\n");
                    
    //                 pos->x = new_pos.x;
    //                 pos->y = new_pos.y;
    //                 pos->z = new_pos.z;
    //                 pos->coordinate_frame = new_pos.frame_id;
    //             }
    //             if (isnan(pos->yaw)) {
    //                 printf("Waypoint yaw is equal to NAN. Defaulting to current yaw of the drone.\n");
    //                 pos->yaw = get_yaw(&new_pos);
    //             }
    //         }

    //         // compute starting distance (squared) from waypoint to keep track of percent progress
    //         pthread_mutex_lock(&cur_pos_mutex);
    //         float start_dist = pow(pos->x - cur_pos.x, 2) + pow(pos->y - cur_pos.y, 2) + pow(pos->z - cur_pos.z, 2);
    //         pthread_mutex_unlock(&cur_pos_mutex);

    //         // begin a while to track drone's progress toward waypoint and monitor app's status
    //         int64_t next_time = 0;
    //         int rate = 100;  // loop 100 times per sec
    //         int max_waypoint_ms = 60000;  // if waypoint takes more than 60 seconds, return an error
    //         int ctr = max_waypoint_ms * (rate / 1000.0);  // divide by 10^3 to convert rate to loops per msec
    //         while(1) {
    //             // Send setpoint Mavlink command to make the drone move
    //             mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, *pos);

    //             // check if app was halted
    //             if (check_app_status(type) == APP_INTERRUPTED) {
    //                 mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    //                 set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
    //                 return APP_INTERRUPTED;
    //             }
    //             if (ctr <= 0) {
    //                 M_ERROR("Waypoint timeout reached but drone is not within desired accuracy. Returning error.\n");
    //                 mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    //                 set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);
    //                 return PRIMITIVE_TIMEOUT;
    //             }

    //             ctr--;

    //             pthread_mutex_lock(&cur_pos_mutex);
    //             float x_dist = fabs(fabs(cur_pos.x) - pos->x);
    //             float y_dist = fabs(fabs(cur_pos.y) - pos->y);
    //             float z_dist = fabs(fabs(cur_pos.z) - pos->z);
    //             pthread_mutex_unlock(&cur_pos_mutex);
    //             // Compute current distance from waypoint (squared) to determine percentage progress
    //             float cur_dist = pow(x_dist, 2) + pow(y_dist, 2) + pow(z_dist, 2);

    //             // Compute current progress (inversely proportional to distance from setpoint)
    //             update_primitive_progress(sqrt(zero_bound((start_dist - cur_dist) / start_dist)), 0);

    //             // Check if we're within final accuracy to waypoint
    //             pthread_mutex_lock(&cur_pos_mutex);
    //             if (x_dist <= primitive->x_accuracy &&
    //                 y_dist <= primitive->y_accuracy &&
    //                 z_dist <= primitive->z_accuracy &&
    //                 pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2) <= pow(primitive->max_final_speed, 2) &&
    //                 fabs(get_yaw(&cur_pos) - primitive->yaw) <= primitive->yaw_accuracy &&
    //                 fabs(cur_pos.yawspeed) <= primitive->yaw_rate_accuracy) {

    //                 pthread_mutex_unlock(&cur_pos_mutex);
                    
    //                 // achieved the required accuracy requirements
    //                 update_primitive_progress(1.0, 0);
    //                 set_last_mavlink_cmd_xyz(pos->x, pos->y, pos->z);
    //                 return 0;  // SUCCESS
    //             }
    //             else {
    //                 pthread_mutex_unlock(&cur_pos_mutex);
    //             }

    //             // printf("cur_pos: (%f, %f, %f, %f)\n", cur_pos.x, cur_pos.y, cur_pos.z, get_yaw(&cur_pos));
    //             // printf("cur_pos.v: %f\n", pow(cur_pos.vx, 2) + pow(cur_pos.vy, 2) + pow(cur_pos.vz, 2));
    //             // printf("primitive_progress: %f\n\n", primitive_progress);

    //             // Sleep the appropriate amount of time to maintain loop rate
    //             if(my_loop_sleep(rate, &next_time)){
    //                 fprintf(stderr, "WARNING: waypoint sleep loop fell behind.\n");
    //             }
    //         }

    //     } break;
    // }
    
    // /* 
    // by default, if we somehow end up here, just set last_mavlink_cmd as the
    // drone's current position to be safe.    
    // */
    // mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    // set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);

    // return 0;
}