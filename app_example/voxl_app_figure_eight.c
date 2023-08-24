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
#include <pthread.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <time.h>

#include "voxl_app_figure_eight.h"
#include "voxl_app_manager.h"
#include "primitives.h"
#include "queue.h"
#include "app_runner.h"
#include "macros.h"      // for mathematical constants
#include "mavlink_io.h"  // for AUTOPILOT_COMPID
#include "misc.h"

/**
 * ATTENTION
 * 
 * This figure eight demo app will result in the drone flying in a figure 8
 * configuration. The XY-center of the figure 8 is whereever the drone was when
 * this app was *started*. The figure 8 will be oriented toward 0-degrees in
 * local frame and fly at 1.5 meters above Z=0 in local frame.
 * 
 * Please take appropriate precautions before flight.
 */



static int en_debug = 0;

#define FLIGHT_ALTITUDE	-0.75f
// #define FLIGHT_ALTITUDE 0
#define RATE			30	// loop rate hz
#define RADIUS			1.0	// radius of figure 8 in meters
#define CYCLE_S			8	// time to complete one figure 8 cycle in seconds (8 seconds default)
#define STEPS			(CYCLE_S*RATE)

static primitive_setpoint_t path[STEPS];
static primitive_go_to_waypoint_t home_position;

uint64_t end_setpoint_id;
uint8_t added_figure_eight = 0;

int num_figure_eights = 0;
#define MAX_NUMBER_FIGURE_EIGHTS 1000

/**
 * Flag that indicates whether this app is actively running or not.
 * 
 * Best practice to wrap all actions in this app in an if-ACTIVE_FLAG-statement
 * so that this app doesn't continue trying to push to queue or expend resources
 * when it doesn't have control of the drone anyway.
 * 
 * Note: this flag is automatically updated in the onUpdate() function.
 */
uint8_t ACTIVE_FLAG = 0;

static float x_offset;
static float y_offset;
// Note: Z-offset is 0. The figure 8 will always run at FLIGHT_ALTITUDE meters relative to local frame.


int onInit() {
    printf("INITIALIZING...\n");
    
    ////////////////////////////////////////////////////////////////////////////
    // Generate a path following Bernoulli's lemiscate as a parametric equation
    // in NED coordinates.
    ////////////////////////////////////////////////////////////////////////////

    int i;
	const double dt = 1.0/RATE;
	const double dadt = (TWO_PI)/CYCLE_S; // first derivative of angle with respect to time
	const double r = RADIUS;

	for(i=0;i<STEPS;i++){
        primitive_default_setpoint(&path[i]);

		// calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
		// through the curve
		double a = (-PI_2) + i*(TWO_PI/STEPS);
		double c = cos(a);
		double c2a = cos(2.0*a);
		double c4a = cos(4.0*a);
		double c2am3 = c2a-3.0;
		double c2am3_cubed = c2am3*c2am3*c2am3;
		double s = sin(a);
		double cc = c*c;
		double ss = s*s;
		double sspo = (s*s)+1.0; // sin squared plus one
		double ssmo = (s*s)-1.0; // sin squared minus one
		double sspos = sspo*sspo;

		// Position
		// https://www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
		path[i].x = -(r*c*s) / sspo;
		// https://www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
		path[i].y =  (r*c)   / sspo;
		path[i].z =  FLIGHT_ALTITUDE;

		// Velocity
		// https://www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].vx =   dadt*r* ( ss*ss + ss + (ssmo*cc) )   /  sspos;
		// https://www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].vy =  -dadt*r* s*( ss + 2.0*cc + 1.0 )  / sspos;
		path[i].vz =  0.0f;

		// Acceleration
		// https://www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].afx =  -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/ c2am3_cubed;
		// https://www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
		path[i].afy =  dadt*dadt*r*c*((44.0*c2a) + c4a - 21.0) / c2am3_cubed;
		path[i].afz =  0.0f;

		// calculate yaw as direction of velocity
		path[i].yaw = atan2(path[i].vy, path[i].vx);

        // so that the drone spends 33.333 ms following each setpoint before moving onto the next one
        path[i].duration = 1000/RATE;
	}

	// calculate yaw_rate by dirty differentiating yaw
	for(i=0;i<STEPS;i++){
		double next = path[(i+1)%STEPS].yaw;
		double curr = path[i].yaw;
		// account for wrap around +- PI
		if((next-curr) < -PI) next+=(TWO_PI);
		if((next-curr) >  PI) next-=(TWO_PI);
		path[i].yaw_rate = (next-curr)/dt;
	}

	if(en_debug){
		// dump out the trajectory for debugging.
		printf("===========================================================\n");
		printf("   X     Y\n");
		for(i=0;i<STEPS;i++){
			printf("x:%7.3f  y:%7.3f\n", (double)path[i].x, (double)path[i].y);
		}
		printf("===========================================================\n");
		printf("   vx    dx/dt     vy    dy/dt\n");
		for(i=0;i<STEPS;i++){
			double dx = (double)(path[(i+1)%STEPS].x - path[i].x)/dt;
			double dy = (double)(path[(i+1)%STEPS].y - path[i].y)/dt;
			printf("vx:%7.3f dx/dt:%7.3f  vy:%7.3f dy/dt:%7.3f\n", (double)path[i].vx, dx, (double)path[i].vy, dy);
		}
		printf("===========================================================\n");
		printf("   ax    d^2x/dt     ay    d^2y/dt\n");
		for(i=0;i<STEPS;i++){
			double d2x = (double)(path[(i+1)%STEPS].vx - path[i].vx)/dt;
			double d2y = (double)(path[(i+1)%STEPS].vy - path[i].vy)/dt;
			printf("Ax:%7.3f d2x/dt:%7.3f  Ay:%7.3f d2y/dt:%7.3f\n", (double)path[i].afx, d2x, (double)path[i].afy, d2y);
		}
		printf("===========================================================\n");
		printf("   yaw     yaw_rate\n");
		for(i=0;i<STEPS;i++){
			printf("yaw:%7.1f deg  yaw_rate: %7.1f deg/s\n", (double)(path[i].yaw)*180.0/PI, (double)(path[i].yaw_rate)*180.0/PI);
		}
		printf("===========================================================\n");
	}

	// now set home position
	// this will move later if figure_eight_move_home is enabled
    primitive_default_go_to_waypoint(&home_position);
	home_position.coordinate_frame = path[0].coordinate_frame;
    home_position.x = 0, home_position.y = 0, home_position.z = path[0].z;
	home_position.yaw = path[0].yaw;
	// ignore velocity and acceleration (use PX4's default)
	home_position.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
							  POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
							  POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    return 0;
}

void add_figure_eight_to_queue() {
	if (ACTIVE_FLAG) printf("======================ADDING NEW FIGURE 8 TO QUEUE!!======================\n");
    
    // Print timestamp that figure 8 is being added
    struct timeval tv;
    struct tm *timeinfo;
    char time_str[30];

    gettimeofday(&tv, NULL);
    timeinfo = localtime(&tv.tv_sec);

    strftime(time_str, sizeof(time_str), "%H:%M:%S", timeinfo);
    printf("Current Time: %s.%03ld\n", time_str, tv.tv_usec / 1000);


    for (int i=0; i<STEPS; i++) {
		if (!ACTIVE_FLAG) break;

        uint64_t id = queue_push(&path[i]);

        if (i == STEPS-1) {
            end_setpoint_id = id;
		}
    }

    num_figure_eights++;
}

int onStart(update_t update) {
	ACTIVE_FLAG = 1;
    printf("STARTING app voxl_app_figure_eight.c.\n");

	if (en_debug) {
		printf("Timestamp: %ld.%09ld\n", (long)update.timestamp.tv_sec, update.timestamp.tv_nsec);
		printf("Current App: %s\n", update.active_app);
		printf("Current Primitive Type: %d\n", update.current_prim_type);
		printf("Queue Length: %d\n", update.queue_len);
		printf("Progress: %.2f%%\n", update.progress);
		printf("Battery Voltage: %.2f\n", update.batt_volts);
		printf("Frame ID: %d\n", update.frame_id);
		printf("Position (x, y, z): %.2f, %.2f, %.2f\n", update.x, update.y, update.z);
		printf("Velocity (vx, vy, vz): %.2f, %.2f, %.2f\n", update.vx, update.vy, update.vz);
		printf("Quaternion (q[0], q[1], q[2], q[3]): %.2f, %.2f, %.2f, %.2f\n", update.q[0], update.q[1], update.q[2], update.q[3]);
		printf("Roll: %.2f\n", update.roll);
		printf("Pitch: %.2f\n", update.pitch);
		printf("Yaw: %.2f\n", update.yaw);
		printf("Roll Speed: %.2f\n", update.rollspeed);
		printf("Pitch Speed: %.2f\n", update.pitchspeed);
		printf("Yaw Speed: %.2f\n", update.yawspeed);
		printf("Autopilot Main Mode: %d\n", update.autopilot_main_mode);
		printf("Autopilot Sub Mode: %d\n", update.autopilot_sub_mode);
		printf("Autopilot is Armed: %u\n", update.autopilot_is_armed);
	}

	x_offset = update.x;
	y_offset = update.y;

	// Add X and Y offsets to home position
	home_position.x += x_offset;
	home_position.y += y_offset;

	queue_push(&home_position);

    // Give 5 seconds for the drone's position to stabilize
    primitive_pause_t pause;
    primitive_default_pause(&pause, 2000);
    queue_push(&pause);

    // Add X and Y offsets to every position in the figure 8 path
    for (int i=0; i<STEPS; i++) {
		path[i].x += x_offset;
		path[i].y += y_offset;
    }

    add_figure_eight_to_queue();

    return 0;
}

void onUpdate(update_t update) {

	if (en_debug) {
		printf("Timestamp: %ld.%09ld\n", (long)update.timestamp.tv_sec, update.timestamp.tv_nsec);
		printf("Current App: %s\n", update.active_app);
		printf("Current Primitive Type: %d\n", update.current_prim_type);
		printf("Queue Length: %d\n", update.queue_len);
		printf("Progress: %.2f%%\n", update.progress);
		printf("Battery Voltage: %.2f\n", update.batt_volts);
		printf("Frame ID: %d\n", update.frame_id);
		printf("Position (x, y, z): %.2f, %.2f, %.2f\n", update.x, update.y, update.z);
		printf("Velocity (vx, vy, vz): %.2f, %.2f, %.2f\n", update.vx, update.vy, update.vz);
		printf("Quaternion (q[0], q[1], q[2], q[3]): %.2f, %.2f, %.2f, %.2f\n", update.q[0], update.q[1], update.q[2], update.q[3]);
		printf("Roll: %.2f\n", update.roll);
		printf("Pitch: %.2f\n", update.pitch);
		printf("Yaw: %.2f\n", update.yaw);
		printf("Roll Speed: %.2f\n", update.rollspeed);
		printf("Pitch Speed: %.2f\n", update.pitchspeed);
		printf("Yaw Speed: %.2f\n", update.yawspeed);
		printf("Autopilot Main Mode: %d\n", update.autopilot_main_mode);
		printf("Autopilot Sub Mode: %d\n", update.autopilot_sub_mode);
		printf("Autopilot is Armed: %u\n", update.autopilot_is_armed);
	}

    // if reach the end of the figure 8 add another figure 8
    if (!added_figure_eight && update.current_prim_type == SETPOINT && ((primitive_setpoint_t*)update.current_prim)->id == end_setpoint_id) {
		added_figure_eight = 1;

        if (num_figure_eights <= MAX_NUMBER_FIGURE_EIGHTS) {
            add_figure_eight_to_queue();
        }
    }
	else {
		added_figure_eight = 0;
	}
}

void onException(uint64_t primitive_id, primitive_t primitive_type, event_t event_type, void* event_data) {
    printf("EXCEPTION type %d occured with primitve ID %lu of type %d.\n", event_type, primitive_id, primitive_type);
}

void onStop() {
	ACTIVE_FLAG = 0;
    printf("STOPPING app voxl_app_figure_eight.c\n");
}

void onDeinit() {
    printf("DEINITIALIZING...\n");
}