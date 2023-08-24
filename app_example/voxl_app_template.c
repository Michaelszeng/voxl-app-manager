// /*******************************************************************************
//  * Copyright 2023 ModalAI Inc.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  *
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  *
//  * 3. Neither the name of the copyright holder nor the names of its contributors
//  *    may be used to endorse or promote products derived from this software
//  *    without specific prior written permission.
//  *
//  * 4. The Software is used solely in conjunction with devices provided by
//  *    ModalAI Inc.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  * POSSIBILITY OF SUCH DAMAGE.
//  ******************************************************************************/

// #include <stdio.h>
// #include <unistd.h>
// #include <pthread.h>
// #include <errno.h>
// #include <unistd.h>

// #include "primitives.h"
// #include "queue.h"
// #include "app_runner.h"

// #include "voxl_app_template.h"


// /**
//  * Flag that indicates whether this app is actively running or not.
//  * 
//  * Best practice to wrap all actions in this app in an if-ACTIVE_FLAG-statement
//  * so that this app doesn't continue trying to push to queue or expend resources
//  * when it doesn't have control of the drone anyway.
//  * 
//  * Note: this flag is automatically updated in the onUpdate() function.
//  */
// uint8_t ACTIVE_FLAG = 0;



// int onInit() {
//     printf("INITIALIZING...\n");
//     return 0;
// }

// int onStart(update_t update) {
//     printf("STARTING app voxl_app_template.c.\n");

//     ACTIVE_FLAG = 1;

//     force_arm();

//     // /*
//     // WAYPOINTS TEST

//     // Drone should fly a square.
//     // */
//     // primitive_go_to_waypoint_t wp;
//     // primitive_default_go_to_waypoint(&wp);
//     // wp.x = 0.5;
//     // wp.y = 0;
//     // wp.z = -0.8;
//     // // wp.z = -0.2;
//     // wp.yaw = 0.0;
//     // wp.vx = 0.25;
//     // wp.vx = 0.25;
//     // wp.vx = 0.25;
//     // wp.afx = 0.25;
//     // wp.afy = 0.25;
//     // wp.afz = 0.25;
//     // queue_push(&wp);

//     // primitive_go_to_waypoint_t wp1;
//     // primitive_default_go_to_waypoint(&wp1);
//     // wp1.x = 0.5;
//     // wp1.y = 0.5;
//     // wp1.z = -0.8;
//     // // wp1.z = -0.2;
//     // wp1.yaw = 1.5708;
//     // wp1.vx = 0.25;
//     // wp1.vx = 0.25;
//     // wp1.vx = 0.25;
//     // wp1.afx = 0.25;
//     // wp1.afy = 0.25;
//     // wp1.afz = 0.25;
//     // queue_push(&wp1);

//     // primitive_go_to_waypoint_t wp2;
//     // primitive_default_go_to_waypoint(&wp2);
//     // wp2.x = 0;
//     // wp2.y = 0.5;
//     // wp2.z = -0.8;
//     // // wp2.z = -0.2;
//     // wp2.yaw = 3.141592;
//     // wp2.vx = 0.25;
//     // wp2.vx = 0.25;
//     // wp2.vx = 0.25;
//     // wp2.afx = 0.25;
//     // wp2.afy = 0.25;
//     // wp2.afz = 0.25;
//     // queue_push(&wp2);

//     // primitive_go_to_waypoint_t wp3;
//     // primitive_default_go_to_waypoint(&wp3);
//     // wp3.x = 0;
//     // wp3.y = 0;
//     // wp3.z = -0.8;
//     // // wp3.z = -0.2;
//     // wp3.yaw = -1.5708;
//     // wp3.vx = 0.25;
//     // wp3.vx = 0.25;
//     // wp3.vx = 0.25;
//     // wp3.afx = 0.25;
//     // wp3.afy = 0.25;
//     // wp3.afz = 0.25;
//     // queue_push(&wp3);

//     // primitive_go_to_waypoint_t wp4;
//     // primitive_default_go_to_waypoint(&wp4);
//     // wp4.x = 0;
//     // wp4.y = 0;
//     // wp4.z = -0.8;
//     // // wp4.z = -0.2;
//     // wp4.yaw = 0;
//     // wp4.vx = 0.25;
//     // wp4.vx = 0.25;
//     // wp4.vx = 0.25;
//     // wp4.afx = 0.25;
//     // wp4.afy = 0.25;
//     // wp4.afz = 0.25;
//     // queue_push(&wp4);



//     // /*
//     // TESTING STOP_QUEUE

//     // Drone should fly slowly toward (1.0, 0.0, -0.8), then stop in the middle. Then,
//     // fly back to (0.0, 0.0, -0.8).
//     // */
//     // primitive_go_to_waypoint_t wp;
//     // primitive_default_go_to_waypoint(&wp);
//     // wp.x = 1;
//     // wp.y = 0;
//     // wp.z = -0.8;
//     // // wp.z = -0.2;
//     // wp.yaw = 0.0;
//     // wp.vx = 0.1;
//     // wp.vx = 0.1;
//     // wp.vx = 0.1;
//     // wp.afx = 0.1;
//     // wp.afy = 0.1;
//     // wp.afz = 0.1;
//     // queue_push(&wp);

//     // primitive_go_to_waypoint_t wp1;
//     // primitive_default_go_to_waypoint(&wp1);
//     // wp1.x = 1;
//     // wp1.y = 1;
//     // wp1.z = -0.8;
//     // // wp1.z = -0.2;
//     // wp1.yaw = 0.0;
//     // wp1.vx = 0.1;
//     // wp1.vx = 0.1;
//     // wp1.vx = 0.1;
//     // wp1.afx = 0.1;
//     // wp1.afy = 0.1;
//     // wp1.afz = 0.1;
//     // queue_push(&wp1);

//     // usleep(2000000);
//     // stop_queue();
//     // usleep(5000000);

//     // primitive_go_to_waypoint_t wp2;
//     // primitive_default_go_to_waypoint(&wp2);
//     // wp2.x = 0.0;
//     // wp2.y = 0.0;
//     // wp2.z = -0.8;
//     // // wp2.z = -0.2;
//     // wp2.yaw = 0.0;
//     // wp2.vx = 0.25;
//     // wp2.vx = 0.25;
//     // wp2.vx = 0.25;
//     // wp2.afx = 0.25;
//     // wp2.afy = 0.25;
//     // wp2.afz = 0.25;
//     // queue_push(&wp2);




//     // /*
//     // TESTING PAUSE_QUEUE

//     // Drone should fly slowly toward (1.0, 0.0, -0.8), then stop in the middle. Then,
//     // after 5 seconds of pausing, finish the trajectory toward (1.0, 0.0, -0.8).
//     // */
//     // primitive_go_to_waypoint_t wp;
//     // primitive_default_go_to_waypoint(&wp);
//     // wp.x = 1;
//     // wp.y = 0;
//     // wp.z = -0.8;
//     // // wp.z = -0.2;
//     // wp.yaw = 0.0;
//     // wp.vx = 0.1;
//     // wp.vx = 0.1;
//     // wp.vx = 0.1;
//     // wp.afx = 0.1;
//     // wp.afy = 0.1;
//     // wp.afz = 0.1;
//     // queue_push(&wp);

//     // usleep(1000000);
//     // pause_queue();

//     // usleep(5000000);
//     // continue_queue();



//     // /*
//     // TESTING CONTINUE_QUEUE

//     // Drone should pause for 1 sec, then fly to (0.5, 0.0, -0.8)
//     // */
//     // primitive_pause_t pause;
//     // primitive_default_pause(&pause, 5000);
//     // queue_push(&pause);

//     // primitive_go_to_waypoint_t wp;
//     // primitive_default_go_to_waypoint(&wp);
//     // wp.x = 0.5;
//     // wp.y = 0;
//     // wp.z = -0.8;
//     // // wp.z = -0.2;
//     // wp.yaw = 0.0;
//     // wp.vx = 0.25;
//     // wp.vx = 0.25;
//     // wp.vx = 0.25;
//     // wp.afx = 0.25;
//     // wp.afy = 0.25;
//     // wp.afz = 0.25;
//     // queue_push(&wp);

//     // usleep(1000000);
//     // continue_queue();



//     // /*
//     // TESTING KILL_POWER

//     // (Start drone at a higher altitude.) Drone should fly to (0, 0, 0), then,
//     // after 5 seconds, kill power.
//     // */
//     // primitive_go_to_waypoint_t wp;
//     // primitive_default_go_to_waypoint(&wp);
//     // wp.x = 0;
//     // wp.y = 0;
//     // wp.z = 0;
//     // // wp.z = -0.2;
//     // wp.yaw = 0.0;
//     // wp.vx = 0.25;
//     // wp.vx = 0.25;
//     // wp.vx = 0.25;
//     // wp.afx = 0.25;
//     // wp.afy = 0.25;
//     // wp.afz = 0.25;
//     // queue_push(&wp);

//     // usleep(5000000);
//     // kill_power();



//     // /*
//     // SWITCH APP TEST

//     // Drone should fly toward (1.0, 0.0, -0.8), then, after 5 seconds, return to
//     // (0.0, 0.0, 0.8).
//     // */
//     // primitive_go_to_waypoint_t wp;
//     // primitive_default_go_to_waypoint(&wp);
//     // wp.x = 1;
//     // wp.y = 0;
//     // wp.z = -0.8;
//     // // wp.z = -0.2;
//     // wp.yaw = 0.0;
//     // wp.vx = 0.1;
//     // wp.vx = 0.1;
//     // wp.vx = 0.1;
//     // wp.afx = 0.1;
//     // wp.afy = 0.1;
//     // wp.afz = 0.1;
//     // queue_push(&wp);

//     // usleep(5000000);
//     // switch_to_app("voxl_app_return_home_0_8m.so");



//     /*
//     TESTING TAKEOFF and LANDING

//     Drone should take off, pause for 2 sec, then land.
//     */
//     primitive_take_off_t take_off;
//     primitive_default_take_off(&take_off);
//     take_off.height = 0.6;
//     queue_push(&take_off);

//     primitive_pause_t pause;
//     primitive_default_pause(&pause, 2000);
//     queue_push(&pause);

//     primitive_land_t land;
//     primitive_default_land(&land);
//     queue_push(&land);

//     return 0;
// }

// void onUpdate(update_t update) {
    
// }

// void onException(uint64_t primitive_id, primitive_t primitive_type, event_t event_type, void* event_data) {
//     printf("EXCEPTION type %d occured with primitve ID %lu of type %d.\n", event_type, primitive_id, primitive_type);
// }

// void onStop() {
//     printf("STOPPING app voxl_app_template.c\n");

//     ACTIVE_FLAG = 0;
// }

// void onDeinit() {
//     printf("DEINITIALIZING...\n");
// }
