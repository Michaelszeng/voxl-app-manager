/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
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


#define _GNU_SOURCE  // for pthread_timedjoin_np and possibly other things
#include <stdio.h>   // for printf
#include <stdlib.h>
#include <unistd.h> 
#include <getopt.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include <dirent.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/poll.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/inotify.h>
#include <sys/syscall.h>

#include <modal_pipe.h>
#include <voxl_cutils.h>
#include <modal_start_stop.h>
#include <modal_pipe_server.h>
// #include <modal_journal.h>  // libmodal_journal is not compatible with multi-processing
#include "modal_journal_lite.h"

#include "voxl_app_manager.h"
#include "app_runner.h"
#include "primitives.h"
#include "queue.h"
#include "pipe_channels.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "config_file.h"


#define PREFIX   COLOR_LIT_BLU "app-manager:\t"
#define POSTFIX  RESET_COLOR
int debugLevel;
static uint8_t print_threads = 0;

static char app_name_argument[MAX_APP_NAME_LENGTH];
static bool app_name_argument_given = false;

static pthread_mutex_t available_apps_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t start_app_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t queue_pos_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cur_pos_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t last_mavlink_cmd_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t primitive_progress_mutex = PTHREAD_MUTEX_INITIALIZER;

static char CONTROL_COMMANDS[10000];

// flag to know whether we're executing the first primitive or not
// necessary for execute_queue
static uint8_t first_primitive;

// magic numbers for pipe communication between voxl-app-manager and app-runner
const uint32_t APPS_MAGIC_NUMBER      = 0x41505053;  // 1095782483
const uint16_t START_APP_MSG          = 0x81;  // 129
const uint16_t INIT_DONE_MSG          = 0x82;  // 130
const uint16_t QUEUE_PUSH_CONFIRM_MSG = 0x83;  // 131
const uint16_t QUEUE_POS_MSG          = 0x84;  // 132
const uint16_t UPDATE_MSG             = 0x85;  // 133
const uint16_t EXCEPTION_MSG          = 0x86;  // 134
const uint16_t CLEAR_QUEUE_MSG        = 0x87;  // 135
const uint16_t STOP_QUEUE_MSG         = 0x88;  // 136
const uint16_t PAUSE_QUEUE_MSG        = 0x89;  // 137
const uint16_t CONTINUE_QUEUE_MSG     = 0x8a;  // 138
const uint16_t LIB_REMOVED_MSG        = 0x8b;  // 139
const uint16_t STOP_APP_MSG           = 0x8c;  // 140
const uint16_t SWITCH_TO_APP_MSG      = 0x8d;  // 141
const uint16_t FORCE_ARM_MSG          = 0x8e;  // 142
const uint16_t TERMINATE_APP_MSG      = 0xFFFD;
const uint16_t APP_TERMINATED_MSG     = 0xFFFE;
const uint16_t KILL_POWER_MSG         = 0xFFFF;



static void _print_usage() {
    /**
     * Print the help message
     */
    M_PRINT("voxl-app-manager usually runs as a systemd background service and will automatically\n\
    scan for apps to run and await commands from its control pipe. However, for debug\n\
    purposes, it can be started from the command line manually with any of the following\n\
    options.\n\
    voxl-app-manager also creates a control pipe to test sending commands back to\n\
    the server from either a client or from the command line. To test, try\n\
    voxl-send-command.\n\
    ");
    M_PRINT("\nCommand line arguments are as follows:\n\n");
    M_PRINT("-a, --app               : path to a app (.so) file to run immediately. This argument will only work if STATIC_TESTING is set to false.\n");
    M_PRINT("-b, --debug_mav_send    : show debug info on mavlink packets going to autopilot\n");
    M_PRINT("-d, --debug_uart_recv   : show debug info on mavlink packets coming from autopilot\n");
    M_PRINT("-d, --debug-level       : Log debug level (Default 2)\n");
    M_PRINT("                      0 : Print verbose logs\n");
    M_PRINT("                      1 : Print >= info logs\n");
    M_PRINT("                      2 : Print >= warning logs\n");
    M_PRINT("                      3 : Print only fatal logs\n");
    M_PRINT("-h, --help              : Print this help message\n");
    M_PRINT("\nFor example: voxl-app-manager -d 1 -a /etc/voxl-apps/libvoxl_app_template.so\n");
}


static int _parse_opts(int         argc,            ///< Number of arguments
		               char* const argv[])          ///< arguments
{
    /**
     * Parses the command line arguments to the main function
     */
    static struct option LongOptions[] =
    {
        {"app",              required_argument, 0, 'a'},
        {"debug_mav_send",   no_argument,       0, 'b'},
        {"debug_mav_recv",   no_argument,       0, 'c'},
        {"debug-level",      required_argument, 0, 'd'},
        {"help",             no_argument,       0, 'h'},
    };

    int optionIndex      = 0;
    int status           = 0;
    int option;

    while ((status == 0) && (option = getopt_long (argc, argv, "bchd:a:", &LongOptions[0], &optionIndex)) != -1) {
        switch(option) {
            case 'a':
                sscanf(optarg, "%s", app_name_argument);
                app_name_argument_given = true;
                break;
            case 'b':
                mavlink_io_en_print_debug_send(1);
                break;
            case 'c':
                mavlink_io_en_print_debug_recv(1);
                break;
            case 'd':
                if (sscanf(optarg, "%d", &debugLevel) != 1) {
                    M_ERROR("Failed to parse debug level specified after -d flag\n");
                    return -1;
                }

                if (debugLevel >= PRINT || debugLevel < VERBOSE) {
                    M_ERROR("Invalid debug level specified: %d\n", debugLevel);
                    return -1;
                }

                M_JournalSetLevel((M_JournalLevel) debugLevel);
                break;
            case 'h':
                status = -1;
                break;
            // Unknown argument
            case '?':
            default:
                M_VERBOSE(PREFIX "Invalid argument passed!\n");
                status = -1;
                break;
        }
    }

    if (status != 0) return status;

    return status;
}


void build_msg_header(char* msg, uint16_t type_int, uint16_t write_size) {
    memcpy(msg, &APPS_MAGIC_NUMBER, sizeof(uint32_t));
    memcpy(msg + sizeof(uint32_t), &type_int, sizeof(uint16_t));
    memcpy(msg + sizeof(uint32_t) + sizeof(uint16_t), &write_size, sizeof(uint16_t));
}


void unpack_msg_header(char* msg, uint32_t* magic_num, uint16_t* type_int, uint16_t* write_size) {
    memcpy(magic_num, msg, sizeof(uint32_t));
    memcpy(type_int, msg + sizeof(uint32_t), sizeof(uint16_t));
    memcpy(write_size, msg + sizeof(uint32_t) + sizeof(uint16_t), sizeof(uint16_t));
}


uint8_t check_app_exists(int ino) {
    /**
     * app exists if there's an app in available_apps corresponding to the inode
     * that isn't REMOVED
     */
    for (int i=0; i<available_apps.num_apps; i++) {
        if (ino == available_apps.apps[i].id && available_apps.apps[i].status != REMOVED) {
            return 1;
        }
    }
    return 0;
}


void update_control_commands() {
    // build CONTROL_COMMANDS from the list of initialized available apps
    CONTROL_COMMANDS[0] = '\0';  // reset available apps so they are ready to be re-scanned
    for (int i=0; i<available_apps.num_apps; i++) {
        if (available_apps.apps[i].status == READY) {
            strcat(CONTROL_COMMANDS, available_apps.apps[i].name);
            strcat(CONTROL_COMMANDS, ",");
            // M_VERBOSE("UPDATED CONTROL COMMANDS WITH %s\n", available_apps.apps[i].name);
        }
    }
    // remove the extra comma from CONTROL_COMMANDS
    CONTROL_COMMANDS[strlen(CONTROL_COMMANDS)-1] = '\0';

    // update available control pipe commands in pipe info file
    pipe_server_set_available_control_commands(AVAILABLE_APPS_CH, CONTROL_COMMANDS);
}


static void stop_active_app(uint8_t switching_app) {
    /**
     * Helper function that gracefully stops an app that is in the middle
     * of running.
     */

    if (available_apps.active_app[0] == -1 || available_apps.active_app[1] == -1) {
        return;
    }

    // send termination command to app runner
    char stop_app_msg[HEADER_SIZE];
    build_msg_header(stop_app_msg, STOP_APP_MSG, 0);
    int err = write(available_apps.apps[available_apps.active_app[0]].manager_w_pipe_fds[1], stop_app_msg, HEADER_SIZE);  // pipe write
    if (err == -1) {
        M_ERROR(PREFIX "Pipe write errored in STOP_APP_MSG.\n" POSTFIX);
    }
    
    // change app status to INACTIVE
    pthread_mutex_lock(&available_apps_mutex);
    available_apps.apps[available_apps.active_app[0]].status = INACTIVE;
    pthread_mutex_unlock(&available_apps_mutex);

    // clear the queue
    manager_clear_queue();

    // sleep for 100 ms to give apm_execute_primitive enough time to realize that the app has been stopped before starting a new one
    usleep(100000);

    if (!switching_app) {
        // remove active app since no app is running now
        pthread_mutex_lock(&available_apps_mutex);
        available_apps.active_app[0] = -1;
        available_apps.active_app[1] = -1;
        pthread_mutex_unlock(&available_apps_mutex);
    }
}


static void terminate_app(int pid, int fd) {
    /**
     * Helper function to gracefully terminate an app process
     */
    // Send message to app_runner to stop its listener thread.
    // This write is necessary in addition to the SIGTERM bc the only way to cleanly
    // halt the app's own listener thread is by sending a message to it.
    char term_msg[HEADER_SIZE];
    build_msg_header(term_msg, TERMINATE_APP_MSG, 0);
    int err = write(fd, term_msg, HEADER_SIZE);  // pipe write
    if (err == -1) {
        M_ERROR(PREFIX "Pipe write TERMINATE_APP_MSG errored. errno: %d\n" POSTFIX, errno);
    }

    usleep(50000);  // give time for message to be received

    // kill the app's process
    kill((pid_t)pid, SIGTERM);
}


static int build_update(update_t* update) {
    /**
     * Helper function to build an instance update_t. Takes the update struct to
     * populate and returns prim_data_len, the length of the primitive type
     * contained in the update.
     */
    if (timespec_get(&update->timestamp, TIME_UTC) == 0) M_ERROR(PREFIX "timespec_get failed.\n" POSTFIX);

    // set the active app name
    if (available_apps.active_app[0] == -1) {  // there is no active app
        update->active_app[0] = '\0';
    }
    else {
        memcpy(update->active_app, available_apps.apps[available_apps.active_app[0]].name, sizeof(available_apps.apps[available_apps.active_app[0]].name));
    }

    // set the current primitive type
    if (queue_pos == 0) {  // if there are no primitives in the queue
        update->current_prim_type = -1;
    }
    else {
        update->current_prim_type = queue[queue_pos].type;
    }
    
    update->current_prim = NULL;  // this will be set in app_runner.c when the UPDATE_MSG is received
    update->queue_len = queue_end - queue_pos;
    pthread_mutex_lock(&primitive_progress_mutex);
    update->progress = primitive_progress;
    pthread_mutex_unlock(&primitive_progress_mutex);
    update->batt_volts = autopilot_monitor_get_bat_voltage();

    pthread_mutex_lock(&cur_pos_mutex);
    cur_pos = autopilot_monitor_get_odometry();
    update->frame_id = cur_pos.frame_id;
    update->x = cur_pos.x;
    update->y = cur_pos.y;
    update->z = cur_pos.z;
    update->vx = cur_pos.vx;
    update->vy = cur_pos.vy;
    update->vz = cur_pos.vz;
    
    update->q[0] = cur_pos.q[0];
    update->q[1] = cur_pos.q[1];
    update->q[2] = cur_pos.q[2];
    update->q[3] = cur_pos.q[3];
    update->rollspeed = cur_pos.rollspeed;
    update->pitchspeed = cur_pos.pitchspeed;
    update->yawspeed = cur_pos.yawspeed;
    pthread_mutex_unlock(&cur_pos_mutex);

    cur_attitude = autopilot_monitor_get_attitude();
    update->roll = cur_attitude.roll;
    update->pitch = cur_attitude.pitch;
    update->yaw = cur_attitude.yaw;

    update->autopilot_main_mode = autopilot_monitor_get_main_mode();
    update->autopilot_sub_mode = autopilot_monitor_get_sub_mode();
    update->autopilot_is_armed = autopilot_monitor_is_armed();

    // Depending on which primitive type is contained in update, select the correct length for the update pipe message
    
    int prim_data_len;
    switch(update->current_prim_type) {
        case -1:
            prim_data_len = 0;
            break;
        case PAUSE:
            prim_data_len = sizeof(primitive_pause_t);
            break;
        case LAND:
            prim_data_len = sizeof(primitive_land_t);
            break;
        case TAKE_OFF:
            prim_data_len = sizeof(primitive_take_off_t);
            break;
        case SETPOINT:
            prim_data_len = sizeof(primitive_setpoint_t);
            break;
        case GO_TO_WAYPOINT:
            prim_data_len = sizeof(primitive_go_to_waypoint_t);
            break;
        case LOCAL_POLYNOMIAL:
            prim_data_len = sizeof(primitive_local_polynomial_t);
            break;
    }

    return prim_data_len;
}


static void* _app_listener_thread_func(void* void_star_app) {
    /**
     * This thread runs during app execution, constantly listening for messages
     * from the app process over pipe.
     * 
     * One instance of this thread exists for every app. The lifetime of this
     * thread is during the ACTIVE period of the app.
     */
    if (print_threads) printf("_app_listener_thread_func thread id: %d\n", syscall(SYS_gettid));

    app_t* app = (app_t*)void_star_app;

    // listen so long as the app is not terminated
    while (1) {
        /////////////////////////////////////////////////////
        // NOTE: THIS IS NOT A SPIN LOOP. read() IS BLOCKING.
        /////////////////////////////////////////////////////
        char read_buff[HEADER_SIZE];
        memset(read_buff, 0, sizeof(read_buff));  // initialize buff to 0's
        int err = read(app->app_w_pipe_fds[0], read_buff, HEADER_SIZE);
        if (err == -1) {
            M_ERROR(PREFIX "Pipe read errored with errno: %d\n" POSTFIX, errno);
            break;
        }

        uint32_t magic_num;
        uint16_t type, num_bytes;
        unpack_msg_header(read_buff, &magic_num, &type, &num_bytes);
        if (magic_num != APPS_MAGIC_NUMBER) {
            M_ERROR(PREFIX "ERROR: Received message over pipe without valid header--magic num: %d; msg type: %d; num bytes: %d. App process likely segfaulted.\n" POSTFIX, magic_num, type, num_bytes);
            usleep(1000000);
            continue;
        }
        
        M_VERBOSE(PREFIX "Received command from pipe of type %d, %d bytes.\n" POSTFIX, type, num_bytes);

        if (type <= 128) {  // QUEUE_PUSH_MSG
            /**
             * Read primitive data over pipe, then call queue.c's push() function.
             * Then update app_runner.c's values for queue_pos and queue_end.
             */
            // Read all bytes associated with this message
            char buf[num_bytes];
            err = read(app->app_w_pipe_fds[0], buf, num_bytes);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n"POSTFIX);
            }
            // Determine number of bytes to read depending on primitive type
            int primitive_data_read_size;
            primitive_t primitive_type = (primitive_t)type;  // the message type is the primitive type
            switch(primitive_type) {
                case PAUSE:
                    primitive_data_read_size = sizeof(primitive_pause_t);
                    break;
                case LAND:
                    primitive_data_read_size = sizeof(primitive_land_t);
                    break;
                case TAKE_OFF:
                    primitive_data_read_size = sizeof(primitive_take_off_t);
                    break;
                case SETPOINT:
                    primitive_data_read_size = sizeof(primitive_setpoint_t);
                    break;
                case GO_TO_WAYPOINT:
                    primitive_data_read_size = sizeof(primitive_go_to_waypoint_t);
                    break;
                case LOCAL_POLYNOMIAL:
                    primitive_data_read_size = sizeof(primitive_local_polynomial_t);
                    break;
            }

            // copy out the bytes containing the primitive data
            char primitive_data_buf[primitive_data_read_size];
            memcpy(primitive_data_buf, buf, primitive_data_read_size);

            // Hex Dump primitive_data_buf
            // for (uint32_t j=0; j<primitive_data_read_size; j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
            // printf("\n");

            // CALL queue.c's PUSH FUNCTION
            push(primitive_data_buf, primitive_type);

            // construct message with QUEUE_POS_MSG, queue_end, and queue_pos
            int queue_pos_msg_length = HEADER_SIZE + sizeof(queue_end) + sizeof(queue_pos);
            char queue_pos_msg[queue_pos_msg_length];
            build_msg_header(queue_pos_msg, QUEUE_POS_MSG, queue_pos_msg_length - HEADER_SIZE);
            memcpy(queue_pos_msg + HEADER_SIZE, &queue_end, sizeof(queue_end));
            pthread_mutex_lock(&queue_pos_mutex);
            memcpy(queue_pos_msg + HEADER_SIZE + sizeof(queue_end), &queue_pos, sizeof(queue_pos));
            pthread_mutex_unlock(&queue_pos_mutex);
            err = write(app->manager_w_pipe_fds[1], queue_pos_msg, queue_pos_msg_length);  // pipe write
            if (err == -1) {
                M_ERROR(PREFIX "Pipe write errored. errno: %d\n" POSTFIX, errno);
            }
        }
        else if (type == CLEAR_QUEUE_MSG) {
            /**
             * Clear the queue (sets queue_pos equal to queue_end).
             */
            manager_clear_queue();
        }
        else if (type == STOP_QUEUE_MSG) {
            /**
             * Stop the queue (stop the current primitive and clear queue).
             */
            manager_stop_queue();
        }
        else if (type == PAUSE_QUEUE_MSG) {
            /**
             * Pause the queue and currently running primitive (until
             * continue_queue() is called).
             */
            manager_pause_queue();
        }
        else if (type == CONTINUE_QUEUE_MSG) {
            /**
             * Continue the queue if it was paused using pause_queue() or if a
             * primitive_pause_t is currently running.
             */
            manager_continue_queue();
        }
        else if (type == SWITCH_TO_APP_MSG) {
            /**
             * Stop the active app and start the new app sent in the pipe.
             */
            M_VERBOSE(PREFIX "Attempting to switch to new app...\n" POSTFIX);
            // read the name of the app to switch to
            char new_app_name[num_bytes];
            err = read(app->app_w_pipe_fds[0], new_app_name, num_bytes);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n"POSTFIX);
            }
            
            // loop through available apps to try to find one with matching name
            uint8_t found = 0;
            for (int i=0; i<available_apps.num_apps; i++) {
                if (strncmp(new_app_name, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0 && (available_apps.apps[i].status >= READY && available_apps.apps[i].status <= ACTIVE)) {
                    // M_VERBOSE(PREFIX "Found matching app %s!\n" POSTFIX, available_apps.apps[i].name);

                    // mutex lock this section so that other threads don't try to stop and start another app simultaneously
                    pthread_mutex_lock(&start_app_mutex);
                    if (available_apps.active_app[0] != -1) {  // there is another app already running, run stop procedure
                        M_VERBOSE(PREFIX "Stopping currently running app in order to switch to new app.\n" POSTFIX);
                        stop_active_app(1);  // param 1 means that another app is being started immediately after active app is stopped
                    }

                    start_app(i);
                    pthread_mutex_unlock(&start_app_mutex);

                    found = 1;
                }
            }
            if (!found) {
                M_WARN(PREFIX "Attempted to switch to new app but failed. No app matching inputted name with status READY or ACTIVE found.\n" POSTFIX);
            
                // Send a message over pipe to app-runner so app-runner can call onException
                event_t event = SWITCH_TO_APP_FAILED;
                // type and id set to "null" values since the app-switch failing is not related to any primitive.
                int type = -1;
                uint64_t id = 0;

                int exception_msg_length = HEADER_SIZE + sizeof(event) + sizeof(type) + sizeof(id);
                char exception_msg[exception_msg_length];
                build_msg_header(exception_msg, EXCEPTION_MSG, exception_msg_length - HEADER_SIZE);
                memcpy(exception_msg + HEADER_SIZE, &event, sizeof(event));
                memcpy(exception_msg + HEADER_SIZE + sizeof(event), &type, sizeof(primitive_t));
                memcpy(exception_msg + HEADER_SIZE + sizeof(event) + sizeof(primitive_t), &id, sizeof(id));
                int err = write(available_apps.apps[available_apps.active_app[0]].manager_w_pipe_fds[1], exception_msg, exception_msg_length);  // pipe write
                if (err == -1) {
                    M_ERROR(PREFIX "Pipe write errored. errno: %d\n" POSTFIX, errno);
                }
            }
        }
        else if (type == FORCE_ARM_MSG) {
            /**
             * Arm the drone.
             */
            manager_force_arm();
        }
        else if (type == APP_TERMINATED_MSG) {
            /**
             * Set app status to terminated and close pipes and stop this thread.
             */
            close(app->manager_w_pipe_fds[1]);
            close(app->app_w_pipe_fds[0]);

            pthread_mutex_lock(&available_apps_mutex);
            app->status = TERMINATED;
            pthread_mutex_unlock(&available_apps_mutex);

            break;
        }
        else if (type == KILL_POWER_MSG) {
            /**
             * Kill power to the motors/force disarm (even if the drone is in flight).
             */
            manager_kill_power();
        }
    }

    M_VERBOSE(PREFIX "Terminating _app_listener_thread_func.\n" POSTFIX);

    return NULL;
}


void start_app(int i) {
    pthread_mutex_lock(&available_apps_mutex);
    // change available_apps active app to new one
    available_apps.active_app[0] = i;
    available_apps.active_app[1] = available_apps.apps[i].id;
    available_apps.apps[i].status = ACTIVE;
    pthread_mutex_unlock(&available_apps_mutex);

    // construct an update_t containing the latest info about the drone at the moment the app is started
    update_t update;
    // lock queue_pos so it doesn't change while we create the update
    pthread_mutex_lock(&queue_pos_mutex);
    int prim_data_len = build_update(&update);

    // Send start message with update data over pipe
    int start_app_msg_len = HEADER_SIZE + sizeof(update_t) + prim_data_len;
    char start_app_msg[start_app_msg_len];
    build_msg_header(start_app_msg, START_APP_MSG, start_app_msg_len - HEADER_SIZE);
    memcpy(start_app_msg + HEADER_SIZE, (char*)(&update), sizeof(update_t));
    // Since voxl-app-manager is in a separate memory space from app-runner, it is necessary to send over all the data for the primitive via pipe instead of just setting update.current_prim (which is a void* pointer)
    memcpy(start_app_msg + HEADER_SIZE + sizeof(update_t), queue[queue_pos].data, prim_data_len);
    pthread_mutex_unlock(&queue_pos_mutex);
    int err = write(available_apps.apps[i].manager_w_pipe_fds[1], start_app_msg, start_app_msg_len);  // pipe write
    if (err == -1) {
        M_ERROR(PREFIX "Pipe write START_APP_MSG errored. errno: %d\n" POSTFIX, errno);
    }
}


static void* _app_initializer_thread_func(void* void_star_app) {
    /**
     * This thread waits for the app to indicate that it's finished initializing
     * and setd the app's status to READY.
     * 
     * One instance of this thread is created every time a new .so file is found
     * in /etc/voxl-apps. This thread terminates as soon as the respective app
     * sends a message indicating that it's finished initializing.
     */
    if (print_threads) printf("_app_initializer_thread_func thread id: %d\n", syscall(SYS_gettid));
    
    app_t* app = (app_t*)void_star_app;

    M_VERBOSE(PREFIX "Waiting for initialization procedure to finish for app id %lu\n" POSTFIX, app->id);

    // wait for app to send message indicating initialization is finished
    char init_done_buff[8];
    memset(init_done_buff, 0, sizeof(init_done_buff));  // initialize buff to 0's

    // Use poll to wait for data or a timeout
    struct pollfd poll_fds[1];
    poll_fds[0].fd = app->app_w_pipe_fds[0];
    poll_fds[0].events = POLLIN;

    int pipe_has_data = poll(poll_fds, 1, INIT_TIMEOUT_DEFAULT);
    if (pipe_has_data == -1) {
        M_ERROR(PREFIX "ERROR: Poll errored.\n" POSTFIX);
    } else if (pipe_has_data == 0) {
        M_ERROR(PREFIX "Initialization procedure timed out for app id %lu. App will be ignored.\n" POSTFIX, app->id);
        terminate_app(app->pid, app->manager_w_pipe_fds[1]);
        return NULL;
    } else {
        if (poll_fds[0].revents & POLLIN) {
            // Data is available to read from the pipe
            int err = read(app->app_w_pipe_fds[0], init_done_buff, HEADER_SIZE);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
            }
        }
    }

    // unpack the pipe message
    uint32_t magic_num;
    uint16_t type, num_bytes;
    unpack_msg_header(init_done_buff, &magic_num, &type, &num_bytes);

    // check that it is INIT_DONE_MSG
    if (magic_num == APPS_MAGIC_NUMBER && type == INIT_DONE_MSG) {
        // after initialization procedure complete, mark app as READY
        pthread_mutex_lock(&available_apps_mutex);
        app->status = READY;
        pthread_mutex_unlock(&available_apps_mutex);
    }

    // safety feature: in case INIT_DONE_MSG has expanded, eat the extra bytes into a temporary buffer so the next pipe read doesn't fail
    char buf[num_bytes];
    if (num_bytes > 0) {
        int err = read(app->app_w_pipe_fds[0], buf, num_bytes);
        if (err == -1) {
            M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
        }
    }

    M_VERBOSE(PREFIX "Received message that initialization finished for %d.\n" POSTFIX, app->id);

    // create thread to listen to pipe with app
    pthread_t app_listener_thread_id;
    if (pthread_create(&app_listener_thread_id, NULL, _app_listener_thread_func, void_star_app) != 0) {
        M_ERROR(PREFIX "ERROR, couldn't create app_listener_thread_id thread.\n" POSTFIX);
    }
    if (pthread_detach(app_listener_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);


    // add this app to list of control commands
    update_control_commands();

    return NULL;
}


static void* _available_apps_thread_func(__attribute__((unused)) void* arg) {
    /**
     * This thread searches the etc/voxl-apps/ directory searching for new .so
     * files. Upon finding one, it will fork a new child process for the app and
     * perform initialization steps for it.
     */
    if (print_threads) printf("_available_apps_thread_func thread id: %d\n", syscall(SYS_gettid));

    // scan /etc/voxl-apps folder for .so files
    struct dirent *dp;
    const char *path =  "/etc/voxl-apps";  // Directory target
    DIR *dir = opendir(path);  // Open the directory - dir contains a pointer to manage the dir
    while ((dp=readdir(dir))) {  // if dp is null, there's no more content to read
        // check if item in directory is a REGular file and file name ends with .so
        char *dot = strrchr(dp->d_name, '.');  // get ptr to '.' in the filename
        if (dp->d_type != DT_REG || !dot || strcmp(dot, ".so")) {
            continue;
        }

        if (strlen(dp->d_name) > MAX_APP_NAME_LENGTH) {
            M_VERBOSE(PREFIX "App name - %s - too long (more than 63 chars). Ignoring app.\n" POSTFIX, dp->d_name);
            continue;
        }

        if (available_apps.num_apps >= MAX_CONCURRENT_APPS) {
            M_VERBOSE(PREFIX "ERROR: available_apps STRUCT IS FULL; (%d) APPS REACHED. ADDITIONAL APPS WILL BE IGNORED.\n" POSTFIX, MAX_CONCURRENT_APPS);
            break;
        }

        // a process was previously created for this app already, so skip it
        // this includes if the process was previously created but was killed for some reason; then this app will still be skipped
        if (check_app_exists(dp->d_ino)) {
            continue;
        }
        
        M_VERBOSE(PREFIX "Detected new app .so file: %s, ID=%d\n" POSTFIX, dp->d_name, dp->d_ino);

        // create new app struct
        app_t new_app;
        new_app.id = dp->d_ino;
        new_app.pid = -1;  // indicate that it hasn't been assigned a process yet
        strncpy(new_app.name, dp->d_name, MAX_APP_NAME_LENGTH+1);  // MAX_APP_NAME_LENGTH + 1 ensures that new_app.name gets a \0 (since strlen(dp->d_name) <= MAX_APP_NAME_LENGTH)
        new_app.status = UNINITIALIZED;

        // create new process and pipes for the app
        pipe(new_app.manager_w_pipe_fds);
        pipe(new_app.app_w_pipe_fds);
        int fork_pid = fork();
        if (fork_pid == -1) {
            M_ERROR("Failed to fork child process for app %s. This app will be ignored.\n", dp->d_name);
            continue;
        }
        
        /**
         * NOTE: There is a persisting issue of segfaults when a child process
         * terminates. This occurs specifically if the child process's
         * corresponding .so file is *overwritten* (i.e. using the `cp` command)
         * while the child process is running. This is beacuse the dlfcn
         * library's behavior is undefined when the loaded library is
         * removed. There doesn't seem to be a way to fix this, however, in the
         * future, we can write a SIGSEGV handler to print a nicer error message
         * when this occurs.
         */
        new_app.pid = fork_pid;
        if (new_app.pid == 0) {  // CHILD PROCESS
            run_app(&new_app);
            M_VERBOSE(PREFIX "CHILD PROCESS EXITED GRACEFULLY.\n\n" POSTFIX);
            usleep(1000000);
            // return NULL;
            exit(EXIT_SUCCESS);
        }

        // PARENT PROCESS
        // close unused pipes
        close(new_app.manager_w_pipe_fds[0]);
        close(new_app.app_w_pipe_fds[1]);

        // Copy app into available_apps
        // NOTE: BECAUSE OF THIS COPY, MODIFICATIONS TO new_app ITSELF DO NOT MATTER.
        pthread_mutex_lock(&available_apps_mutex);
        available_apps.apps[available_apps.num_apps] = new_app;
        pthread_mutex_unlock(&available_apps_mutex);

        // create thread for the initializing process
        // utilize the app struct's pthread instead of creating a new variable so that the thread id survives beyond this loop
        pthread_mutex_lock(&available_apps_mutex);  // mutex lock since the app's init_thread_id field is being written
        if (pthread_create(&available_apps.apps[available_apps.num_apps].init_thread_id, NULL, _app_initializer_thread_func, &available_apps.apps[available_apps.num_apps]) != 0) {
            M_ERROR(PREFIX "ERROR, couldn't create new_app_init_thread_id thread.\n" POSTFIX);
        }
        if (pthread_detach(available_apps.apps[available_apps.num_apps].init_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);
        pthread_mutex_unlock(&available_apps_mutex);

        pthread_mutex_lock(&available_apps_mutex);
        available_apps.num_apps++;
        pthread_mutex_unlock(&available_apps_mutex);

        usleep(50000);  // calling fork() in rapid succession causes occasional failures; this is needed to slow it down.
    }
    closedir(dir); // close the handle (pointer)

    update_control_commands();

    // send available apps to pipe; cast available_apps to char ptr for pipe writing
    pipe_server_write(AVAILABLE_APPS_CH, (char*)&available_apps, sizeof(available_apps));

    return NULL;
}


static void* _apps_dir_monitor_thread_func(void* fd) {
    /**
     * Thread that watches for changes to app .so files. If an app is modified,
     * it must be deinitialized (and re-loaded if it wasn't deleted).
     * 
     * Created once the .so file is detected, and this thread continues so long
     * as the .so file is not modified or deleted.
     * 
     * One instance of this thread exists for each .so file.
     */
    if (print_threads) printf("_apps_dir_monitor_thread_func thread id: %d\n", syscall(SYS_gettid));

    M_VERBOSE(PREFIX "Starting directory monitor.\n" POSTFIX);

    while (main_running) {
        char buffer[INOTIFY_BUF_LEN];
        ssize_t num_bytes_read = read(*((int*)fd), buffer, INOTIFY_BUF_LEN);

        if (num_bytes_read == -1) {
            M_ERROR(PREFIX "inotify read errored with errno: %d\n" POSTFIX, errno);
            break;
        }

        int i = 0;
        while (i < num_bytes_read) {  // loop through all bytes in event message
            struct inotify_event *event = (struct inotify_event *)&buffer[i];

            // check that the event that occurred happened to a .so file
            char *dot = strrchr(event->name, '.');  // get ptr to '.' in the filename
            if (!dot || (strcmp(dot, ".so") && strcmp(dot, ".dpkg-new"))) {
                i += EVENT_SIZE + event->len;  // go onto next event
                continue;
            }
            
            // Print different messages based on the event type
            if (event->mask & IN_MODIFY) {
                M_VERBOSE(PREFIX "File modified: %s\n" POSTFIX, event->name);
            }
            if (event->mask & IN_MOVED_FROM) {
                M_VERBOSE(PREFIX "File moved from: %s\n" POSTFIX, event->name);
            }
            if (event->mask & IN_MOVED_TO) {
                M_VERBOSE(PREFIX "File moved to: %s\n" POSTFIX, event->name);
            }
            if (event->mask & IN_CREATE) {
                M_VERBOSE(PREFIX "File created: %s\n" POSTFIX, event->name);
            }
            if (event->mask & IN_DELETE) {
                M_VERBOSE(PREFIX "File/directory deleted: %s\n" POSTFIX, event->name);
            }

            // Find the app that the event occurred to
            uint8_t found = 0;
            for (int i=0; i<available_apps.num_apps; i++) {
                // Find app in available_apps with same filename, with status not REMOVED.
                // If there is an app with same filename but has status REMOVED, treat it as a completely new app.
                if (strncmp(event->name, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0 && available_apps.apps[i].status != REMOVED) {

                    M_VERBOSE(PREFIX "Found app without corresponding .so file, killing app's process: %s\n" POSTFIX, available_apps.apps[i].name);

                    // send app-runner message that .so file is gone
                    char lib_removed_msg[HEADER_SIZE];
                    build_msg_header(lib_removed_msg, LIB_REMOVED_MSG, 0);
                    int err = write(available_apps.apps[i].manager_w_pipe_fds[1], lib_removed_msg, HEADER_SIZE);  // pipe write
                    if (err == -1) {
                        M_ERROR(PREFIX "Pipe write LIB_REMOVED_MSG errored. errno: %d\n" POSTFIX, errno);
                    }

                    // if this app process is currently flying the drone, gracefully abort
                    if (available_apps.apps[i].status == ACTIVE) {
                        pthread_mutex_lock(&start_app_mutex);
                        stop_active_app(0);  // 0 to indicate that there is no app being run after active app is stopped
                        pthread_mutex_unlock(&start_app_mutex);
                    }

                    // kill the app's process
                    // note: it's critical that SIGTERM is signaled before the app's status
                    // is updated so that the listener thread for this app doesn't join
                    // prematurely (it needs to listen for the APP_TERMINATED_MSG message).
                    terminate_app(available_apps.apps[i].pid, available_apps.apps[i].manager_w_pipe_fds[1]);

                    usleep(100000);  // give some time for the APP_TERMINATED_MSG message to come through

                    // change the app's ID to -1
                    // this is necessary in case this app's .so file is somehow added back;
                    // we don't want the new app_t to have the same ID as the old app_t.
                    pthread_mutex_lock(&available_apps_mutex);
                    available_apps.apps[i].id = -1;

                    // set app status to REMOVED
                    available_apps.apps[i].status = REMOVED;
                    pthread_mutex_unlock(&available_apps_mutex);  
                    found = 1;

                    // remove this app from control commands
                    update_control_commands();

                    break;
                }
            }
            if (found == 0) {
                // add slight delay so that, in the case that deploy_to_voxl.sh
                // was the cause of this event, the script has time to finish
                // extracting the .deb before _available_apps_thread_func
                // is called.
                usleep(250000);

                // this means a new app added to etc/voxl-apps, so call available_apps to activate it
                pthread_t available_apps_thread_id;
                int ret = pthread_create(&available_apps_thread_id, NULL, _available_apps_thread_func, NULL);
                if (ret) return -1;
                if (pthread_detach(available_apps_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);
            }

            break;

            // i += EVENT_SIZE + event->len;
        }
    }

    M_VERBOSE(PREFIX "Terminating _apps_dir_monitor_thread.\n" POSTFIX);

    return NULL;
}


static void* _autopilot_monitor_thread_func(__attribute__((unused)) void* arg) {
    /**
     * Thread to watch whether AP is armed and in offboard mode. Performs
     * necessary behaviors when disarmed or not in offboard mode.
     * 
     * One instance of this thread is maintained during the lifetime of
     * voxl-app-manager.
     */
    if (print_threads) printf("_autopilot_monitor_thread_func thread id: %d\n", syscall(SYS_gettid));

    uint8_t prev_state = autopilot_monitor_is_armed_and_in_offboard_mode();

    while (main_running) {
        // printf("Current mode: %d\n", autopilot_monitor_get_main_mode());

        auto_pilot_state = autopilot_monitor_is_armed_and_in_offboard_mode();

        // if drone's state was just changed
        if (auto_pilot_state != prev_state) {
            M_VERBOSE(PREFIX "Detected change in autopilot state.\n" POSTFIX);
            if (auto_pilot_state == 0) {
                /*
                If the drone just exited offboard mode, stop the active app.

                _execute_queue_thread_func will ensure the drone continues to
                receive a stream of setpoints (in preparation for when/if the
                drone returns to offboard mode).
                */
                
                // protect in mutex to prevent multiple threads from trying to stop/start apps at the same time
                pthread_mutex_lock(&start_app_mutex);
                stop_active_app(0);  // 0 to indicate that there is no app being run after active app is stopped
                pthread_mutex_unlock(&start_app_mutex);

                // Set flag to zero since, after exiting offboard mode, the drone will need to reset last_mavlink_cmd before it re-enters offboard mode
                last_mavlink_cmd_set = 0;
            }
            else if (auto_pilot_state == 1) {
                /*
                If the drone has just entered offboard mode, save its position
                at this moment as last_mavlink_cmd. So the drone will continue
                to follow this position until an app tells it otherwise.
                */
                mavlink_odometry_t odom = autopilot_monitor_get_odometry();
                set_last_mavlink_cmd_xyz(odom.x, odom.y, odom.z);

                if (set_drone_vz_on_ground) {
                    set_last_mavlink_cmd_on_ground();
                    set_drone_vz_on_ground = 0;
                }

                last_mavlink_cmd_set = 1;

                M_VERBOSE(PREFIX "last_mavlink_cmd set to current position.\n" POSTFIX);
            }
        }

        prev_state = auto_pilot_state;

        usleep(10000);
    }

    return NULL;
}


static void* _update_app_thread_func(__attribute__((unused)) void* arg) {
    /**
     * The function that periodically sends updated data to app-runner via pipe.
     * 
     * One instance of this thread is maintained during the lifetime of
     * voxl-app-manager (it automatically selects the correct apps to update).
     */
    if (print_threads) printf("_update_app_thread_func thread id: %d\n", syscall(SYS_gettid));
    
    // Wait for a primitive to have been executed before starting the update loop
    while (queue_pos == 0) {
        usleep(10000);
    }

    while (main_running) {  // run until voxl-app-manager is terminated (there will always be apps to update once an app is started)
        // send updates to all apps that are in foreground or background (that are running now or were running but were stopped)
        for (int i=0; i<available_apps.num_apps; i++) {
            // if the app is either running now or was previously run
            if (available_apps.apps[i].status == ACTIVE || available_apps.apps[i].status == INACTIVE) {

                // Create struct with latest update data
                update_t update;

                // lock queue_pos so it doesn't change while we create the update
                pthread_mutex_lock(&queue_pos_mutex);
                int prim_data_len = build_update(&update);

                // Send update data over pipe
                int update_msg_len = HEADER_SIZE + sizeof(update_t) + prim_data_len;
                char update_msg[update_msg_len];
                build_msg_header(update_msg, UPDATE_MSG, update_msg_len - HEADER_SIZE);
                memcpy(update_msg + HEADER_SIZE, (char*)(&update), sizeof(update_t));
                // Since voxl-app-manager is in a separate memory space, it is necessary to send over all the data for the primitive via pipe instead of just setting update.current_prim (which is a void* pointer)
                // printf("prim_data_len: %d. queue_pos: %d. queue_end: %d\n", prim_data_len, queue_pos, queue_end);
                memcpy(update_msg + HEADER_SIZE + sizeof(update_t), queue[queue_pos].data, prim_data_len);
                // printf(".\n");
                pthread_mutex_unlock(&queue_pos_mutex);
                
                int err = write(available_apps.apps[i].manager_w_pipe_fds[1], update_msg, update_msg_len);  // pipe write
                if (err == -1) {
                    M_ERROR(PREFIX "Pipe write UPDATE_MSG errored. errno: %d\n" POSTFIX, errno);
                }
            }
        }

        usleep(1000000/UPDATE_FREQUENCY_HZ);
    }

    return NULL;
}


static void* _execute_queue_thread_func(__attribute__((unused)) void* arg) {
    /**
     * This thread is responsible for monitoring and running primitives that are
     * pushed to the queue. Or, if the queue is empty, this thread will ensure
     * PX4 continues to get a steady stream of position targets, keeping the
     * drone in offboard mode.
     * 
     * One instance of this thread is maintained during the lifetime of
     * voxl-app-manager. It constantly checks the central queue and executes any
     * new primitives that are found.
     */
    if (print_threads) printf("_execute_queue_thread_func thread id: %d\n", syscall(SYS_gettid));
    
    first_primitive = 0;

    last_primitive_finish_time = 0;

    // Initialize a position target to be used if the drone exits offboard mode
    static mavlink_set_position_target_local_ned_t pos;
    pos.time_boot_ms = 0;
    pos.coordinate_frame = MAV_FRAME_LOCAL_NED;
    pos.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                    POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    pos.target_system = 0;  // will reset later when sending
	pos.target_component = AUTOPILOT_COMPID;

    last_mavlink_cmd_set = 0;
    set_drone_vz_on_ground = 0;

    printf("\n");
    M_VERBOSE(PREFIX "Starting Queue Execution Thread.\n" POSTFIX);
    while (main_running) {  // run app forever, waiting for more primitives to be pushed to queue (no matter from what app)
        // if we find a new primitive in queue.
        if (queue_pos < queue_end-1 && (!MOVE_PROPS || autopilot_monitor_is_armed_and_in_offboard_mode())) {

            // lock this section so that no other thread tries to access queue[queue_pos].data after its been freed
            pthread_mutex_lock(&queue_pos_mutex);
            if (!first_primitive) {
                free(queue[queue_pos].data);  // free the previous primitive's malloc'ed memory
                queue[queue_pos].data = NULL;  // pointer nullification for safety (so we won't get a silent error if we try to access this memory again)
                first_primitive = 1;
            }
            
            queue_pos++;

            M_VERBOSE(PREFIX "Queue data: queue_pos=%d, queue_end=%d\n" POSTFIX, queue_pos, queue_end);
            // helper variables to avoid repeatedly indexing the queue
            primitive_t primitive_type = queue[queue_pos].type;
            char* primitive_data_buf = queue[queue_pos].data;

            // make a copy so that, after primitive execution, even if queue_pos changed during primitive execution, we know what the index of this primitive is
            int queue_pos_copy = queue_pos;
            pthread_mutex_unlock(&queue_pos_mutex);

            uint64_t primitive_id;
            int ret;

            switch(primitive_type) {
                case PAUSE: {  // PAUSE
                    primitive_pause_t* primitive = (primitive_pause_t*)primitive_data_buf;  // cast char array back to struct
                    primitive_id = primitive->id;

                    // printing
                    // for (uint32_t j=0; j<sizeof(primitive_pause_t); j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
                    // printf("\n");
                    M_VERBOSE(PREFIX "Executing id %lu and type %d.\n" POSTFIX, primitive_id, primitive->type);
                    
                    ret = apm_execute_primitive(primitive, PAUSE);
                    if (ret != 0) {
                        M_WARN(PREFIX "Execution of primitive id %lu, type PAUSE returned fail status.\n" POSTFIX, primitive->id);
                    }
                } break;
                case LAND: {  // LAND
                    primitive_land_t* primitive = (primitive_land_t*)primitive_data_buf;  // cast char array back to struct
                    primitive_id = primitive->id;

                    // printing
                    // for (uint32_t j=0; j<sizeof(primitive_land_t); j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
                    // printf("\n");
                    M_VERBOSE(PREFIX "Executing id %lu and type %d.\n" POSTFIX, primitive_id, primitive->type);
                    
                    ret = apm_execute_primitive(primitive, LAND);
                    if (ret != 0) {
                        M_WARN(PREFIX "Execution of primitive id %lu, type LAND returned fail status.\n" POSTFIX, primitive->id);
                    }
                } break;
                case TAKE_OFF: {  // TAKE_OFF
                    primitive_take_off_t* primitive = (primitive_take_off_t*)primitive_data_buf;  // cast char array back to struct
                    primitive_id = primitive->id;

                    // printing
                    // for (uint32_t j=0; j<sizeof(primitive_take_off_t); j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
                    // printf("\n");
                    M_VERBOSE(PREFIX "Executing id %lu and type %d.\n" POSTFIX, primitive_id, primitive->type);
                    
                    ret = apm_execute_primitive(primitive, TAKE_OFF);
                    if (ret != 0) {
                        M_WARN(PREFIX "Execution of primitive id %lu, type TAKE_OFF returned fail status.\n" POSTFIX, primitive->id);
                    }
                } break;
                case SETPOINT: {  // SETPOINT
                    primitive_setpoint_t* primitive = (primitive_setpoint_t*)primitive_data_buf;  // cast char array back to struct
                    primitive_id = primitive->id;

                    // printing
                    // for (uint32_t j=0; j<sizeof(primitive_setpoint_t); j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
                    // printf("\n");
                    M_VERBOSE(PREFIX "Executing id %lu and type %d.\n" POSTFIX, primitive_id, primitive->type);
                    
                    ret = apm_execute_primitive(primitive, SETPOINT);
                    if (ret != 0) {
                        M_WARN(PREFIX "Execution of primitive id %lu, type SETPOINT returned fail status.\n" POSTFIX, primitive->id);
                    }
                } break;
                case GO_TO_WAYPOINT: {
                    primitive_go_to_waypoint_t* primitive = (primitive_go_to_waypoint_t*)primitive_data_buf;  // cast char array back to struct
                    primitive_id = primitive->id;

                    // printing
                    // for (uint32_t j=0; j<sizeof(primitive_go_to_waypoint_t); j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
                    // printf("\n");
                    M_VERBOSE(PREFIX "Executing id %lu and type %d.\n" POSTFIX, primitive_id, primitive->type);
                    
                    ret = apm_execute_primitive(primitive, GO_TO_WAYPOINT);
                    if (ret != 0) {
                        M_WARN(PREFIX "Execution of primitive id %lu, type go_to_waypoint returned fail status.\n" POSTFIX, primitive->id);
                    }
                } break;
                case LOCAL_POLYNOMIAL: {  // LOCAL_POLYNOMIAL
                    primitive_local_polynomial_t* primitive = (primitive_local_polynomial_t*)primitive_data_buf;  // cast char array back to struct
                    primitive_id = primitive->id;

                    // printing
                    // for (uint32_t j=0; j<sizeof(primitive_local_polynomial_t); j++)  printf("%02X ", *(primitive_data_buf + j));  // debug print
                    // printf("\n");
                    M_VERBOSE(PREFIX "Executing id %lu and type %d.\n" POSTFIX, primitive_id, primitive->type);
                    
                    ret = apm_execute_primitive(primitive, LOCAL_POLYNOMIAL);
                    if (ret != 0) {
                        M_WARN(PREFIX "Execution of primitive id %lu, type LOCAL_POLYNOMIAL returned fail status.\n" POSTFIX, primitive->id);
                    }
                } break;
            }

            // if executing the primitive results in a known exception
            if (ret != 0) {
                // Send a message over pipe to app-runner so app-runner can call onException
                int exception_msg_length = HEADER_SIZE + sizeof(ret) + sizeof(queue[queue_pos_copy].type) + sizeof(primitive_id);
                char exception_msg[exception_msg_length];
                build_msg_header(exception_msg, EXCEPTION_MSG, exception_msg_length - HEADER_SIZE);
                memcpy(exception_msg + HEADER_SIZE, &ret, sizeof(ret));
                memcpy(exception_msg + HEADER_SIZE + sizeof(ret), &queue[queue_pos_copy].type, sizeof(queue[queue_pos_copy].type));
                memcpy(exception_msg + HEADER_SIZE + sizeof(ret) + sizeof(queue[queue_pos_copy].type), &primitive_id, sizeof(primitive_id));
                int err = write(available_apps.apps[available_apps.active_app[0]].manager_w_pipe_fds[1], exception_msg, exception_msg_length);  // pipe write
                if (err == -1) {
                    M_ERROR(PREFIX "Pipe write errored. errno: %d\n" POSTFIX, errno);
                }
            }

            printf("\n");
        }
        else {
            // Queue is empty (and may be for an unknown amount of time), so the finish time of the last primitive is no longer relevant. Set value to NULL.
            last_primitive_finish_time = 0;

            // state is a global variable
            if (auto_pilot_state) {  // Drone is armed and in offboard mode

                while (!last_mavlink_cmd_set) {
                    usleep(1000);
                }

                // Maintain the last mavlink command
                pthread_mutex_lock(&last_mavlink_cmd_mutex);
                mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,last_mavlink_cmd);
                pthread_mutex_unlock(&last_mavlink_cmd_mutex);
            }
            else {  // Drone is NOT armed and in offboard mode
                /*
                Constantly send the drone setpoints identical to its current position.

                This is needed bc PX4 will not allow the drone to enter offboard
                mode unless there is already a steady stream of setpoints being
                sent.
                */
                
                // If there were primitives in queue, but drone isn't armed and in offboard mode, notify user
                if (MOVE_PROPS && queue_pos < queue_end-1) {
                    M_WARN(PREFIX "Attempted to execute primitive, but drone is not armed and in offboard mode. Waiting for drone to be armed and in offboard mode to continue.\n" POSTFIX);
                    usleep(50000);
                }

                mavlink_odometry_t odom = autopilot_monitor_get_odometry();
                pos.x = odom.x;
                pos.y = odom.y;
                pos.z = odom.z;
                mavlink_io_send_local_setpoint(autopilot_monitor_get_sysid(),VOXL_COMPID,pos);
            }

            usleep(10000);  // if there are no primitives to run, sleep to avoid spin loop
        }

        // Reset QUEUE_STOPPED_FLAG now that the primitive (if one was running) has detected the flag, stopped, and returned
        if (QUEUE_STOPPED_FLAG) {
            QUEUE_STOPPED_FLAG = 0;
        }
    }
    return NULL;
}


static void* _rc_monitor_thread_func(__attribute__((unused)) void* arg) {
    /**
     * Watch the position of SWC to switch app on a switch position change.
     * Note: This does exactly the same thing as _control_pipe_handler.
     */
    if (print_threads) printf("_rc_monitor_thread_func thread id: %d\n", syscall(SYS_gettid));
    
    usleep(500000);

    M_VERBOSE("Ready to receive RC inputs.\n");

    // Set initial value for prev_SWC_pos
    int prev_SWC_pos = SWC_pos;

    while (main_running) {
        if (SWC_pos == prev_SWC_pos) {
            usleep(50000);
        }
        else {  // switch position has changed in the last 50 msecs
            prev_SWC_pos = SWC_pos;

            // Find the app corresponding to the switch position (correspondence defined in config file)
            int app_idx = -1;
            for (int i=0; i<available_apps.num_apps; i++) {
                switch(SWC_pos) {
                    case 1:
                        if (strncmp(app1_name, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0) {
                            printf("SWC changed to position %d. Starting app %s.\n", SWC_pos, app1_name);
                            app_idx = i;
                        }
                        break;
                    case 2:
                        if (strncmp(app2_name, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0) {
                            printf("SWC changed to position %d. Starting app %s.\n", SWC_pos, app2_name);
                            app_idx = i;
                        }
                        break;
                    case 3:
                        if (strncmp(app3_name, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0) {
                            printf("SWC changed to position %d. Starting app %s.\n", SWC_pos, app3_name);
                            app_idx = i;
                        }
                        break;
                }
            }

            if (app_idx == -1 || !(available_apps.apps[app_idx].status >= READY && available_apps.apps[app_idx].status <= ACTIVE)) {
                M_WARN(PREFIX "Detected that SWC was moved to position %d, however no corresponding app that is ready to run was found for the app name in the config file. Aborting.\n" POSTFIX, SWC_pos);
                continue;
            }

            // mutex lock this section so that other threads try to stop and start another app simultaneously
            pthread_mutex_lock(&start_app_mutex);
            if (available_apps.active_app[0] != -1) {  // there is another app already running, run stop procedure
                M_VERBOSE(PREFIX "Another app is currently running, stopping it...\n" POSTFIX);
                stop_active_app(1);  // param 1 means that another app is being started immediately after active app is stopped
            }

            start_app(app_idx);
            pthread_mutex_unlock(&start_app_mutex);

        }
    }

    return NULL;
}


static void _control_pipe_handler(int ch, char* string, int bytes, __attribute__((unused)) void* context) {
    /**
     * libmodal_pipe callback function to receive and handle data from control
     * pipe (either from voxl-send-command or from another service).
     */
    printf("\n\n");
    M_VERBOSE(PREFIX "CONTROL PIPE HANDLER: Received command on channel %d. bytes: %d. string: %s\n" POSTFIX, ch, bytes, string);
    for (int i=0; i<available_apps.num_apps; i++) {
        if (strncmp(string, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0 && (available_apps.apps[i].status >= READY && available_apps.apps[i].status <= ACTIVE)) {
            // M_VERBOSE(PREFIX "Found matching app %s!\n" POSTFIX, available_apps.apps[i].name);
            
            // mutex lock this section so that other threads don't try to stop and start another app simultaneously
            pthread_mutex_lock(&start_app_mutex);
            if (available_apps.active_app[0] != -1) {  // there is another app already running, run stop procedure
                M_VERBOSE(PREFIX "Another app is currently running, stopping it...\n" POSTFIX);
                stop_active_app(1);  // param 1 means that another app is being started immediately after active app is stopped
            }

            start_app(i);
            pthread_mutex_unlock(&start_app_mutex);

            return;
        }
    }
    M_VERBOSE(PREFIX "No app matching inputted name with status READY or ACTIVE found.\n" POSTFIX);
    return;
}


static void _connect_handler(int ch, int client_id, char* name, __attribute__((unused)) void* context) {
	/**
     * libmodal_pipe function to handle when another service connects to a pipe
     * published by voxl-app-manager.
     */
    M_VERBOSE(PREFIX "client \"%s\" connected to channel %d  with client id %lu\n" POSTFIX, name, ch, client_id);
	return;
}


static void _disconnect_handler(int ch, int client_id, char* name, __attribute__((unused)) void* context) {
	/**
     * libmodal_pipe function to handle when another service disconnects from a 
     * pipe published by voxl-app-manager.
     */
    M_VERBOSE(PREFIX "client \"%s\" with id %lu has disconnected from channel %d\n" POSTFIX, name, client_id, ch);
	return;
}


int main(int argc, char* const argv[]) {
    if (print_threads) printf("main thread id: %d\n", syscall(SYS_gettid));

    if (_parse_opts(argc, argv)) {
        _print_usage();
        return -1;
    }

    // load config file
    if(config_file_load()) return -1;
	config_file_print();

    // make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if (kill_existing_process(PIPE_SERVER_NAME, 2.0)<-2) return -1;

    // start signal handler so we can exit cleanly
	if (enable_signal_handler()==-1) {
		M_ERROR(PREFIX "ERROR: failed to start signal handler\n" POSTFIX);
		return -1;
	}

    ///////////////////////////////////////////////////////////////////////////////
    // set up the pipe
    ///////////////////////////////////////////////////////////////////////////////

	// enable the control pipe feature and optionally debug prints
	int flags = SERVER_FLAG_EN_CONTROL_PIPE;

	// configure optional callbacks
	pipe_server_set_control_cb(AVAILABLE_APPS_CH, &_control_pipe_handler, NULL);
	pipe_server_set_connect_cb(AVAILABLE_APPS_CH, &_connect_handler, NULL);
	pipe_server_set_disconnect_cb(AVAILABLE_APPS_CH, &_disconnect_handler, NULL);

	// create the pipe
	pipe_info_t info = { \
		.name        = AVAILABLE_APPS_PIPE_NAME,\
		.location    = AVAILABLE_APPS_PIPE_LOCATION ,\
		.type        = "text",\
		.server_name = PIPE_SERVER_NAME,\
		.size_bytes  = AVAILABLE_APPS_RECOMMENDED_PIPE_SIZE};

	if (pipe_server_create(AVAILABLE_APPS_CH, info, flags)) return -1;

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	make_pid_file(PIPE_SERVER_NAME);

    // start monitor before mavlink-io
    M_VERBOSE(PREFIX "Starting Autopilot Monitor.\n" POSTFIX);
	if(autopilot_monitor_init()){
		return -1;
	}

	// most things depends on mavlink-io so start early
	M_VERBOSE(PREFIX "Starting Mavlink IO.\n" POSTFIX);
	if(mavlink_io_init()){
		return -1;
	}

    // disable terminal wrapping
    M_VERBOSE(PREFIX DISABLE_WRAP POSTFIX);

    main_running = 1;

    ///////////////////////////////////////////////////////////////////////////////
    // start thread to monitor directory of app .so files
    ///////////////////////////////////////////////////////////////////////////////

    // Create an inotify instance to monitor app files
    int inotify_fd = inotify_init();
    if (inotify_fd == -1) {
        M_ERROR(PREFIX "ERROR, couldn't create inotify instance.\n" POSTFIX);
    }

    // create watcher for app directory
    const char* apps_path = "/etc/voxl-apps";
    int watcher_fd = inotify_add_watch(inotify_fd, apps_path, IN_MODIFY | IN_MOVED_FROM | IN_MOVED_TO | IN_DELETE);
    // int watcher_fd = inotify_add_watch(inotify_fd, apps_path, IN_ACCESS | IN_MODIFY | IN_ATTRIB | IN_CLOSE_WRITE | \
    //                                                             IN_CLOSE_NOWRITE | IN_OPEN | IN_MOVED_FROM | IN_MOVED_TO | \
    //                                                             IN_CREATE | IN_DELETE);
    if (watcher_fd == -1) {
        M_ERROR(PREFIX "ERROR, couldn't create inotify watch.\n" POSTFIX);
    }

    // Create a thread to monitor inotify events
    pthread_t _apps_dir_monitor_thread_id;
    if (pthread_create(&_apps_dir_monitor_thread_id, NULL, _apps_dir_monitor_thread_func, &inotify_fd) != 0) {
        M_ERROR(PREFIX "ERROR, couldn't create app_file_monitor_thread_id thread.\n" POSTFIX);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // start thread to keep track of and execute primitives in the queue
    ///////////////////////////////////////////////////////////////////////////////

    pthread_t execute_queue_thread_id;
    int err = pthread_create(&execute_queue_thread_id, NULL, _execute_queue_thread_func, NULL);
    if (err) return -1;
    if (pthread_detach(execute_queue_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

    ///////////////////////////////////////////////////////////////////////////////
    // Initialize avaialable_apps and start thread to search for new .so apps
    ///////////////////////////////////////////////////////////////////////////////

    queue[0].type = -1;  // initialize to invalid type so that checks that happen in main.c for the previous primitives type works expected

    // set available_apps magic number so that pipe is readable/validatable
    available_apps.magic_number = AVAILABLE_APPS_MAGIC_NUMBER;
    // set variables to initial values
    available_apps.num_apps = 0;
    available_apps.active_app[0] = -1;
    available_apps.active_app[1] = -1;

    pthread_t available_apps_thread_id;
    int ret = pthread_create(&available_apps_thread_id, NULL, _available_apps_thread_func, NULL);
    if (ret) return -1;
    if (pthread_detach(available_apps_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

    ///////////////////////////////////////////////////////////////////////////////
    // start thread to monitor switch on RC (to start apps)
    ///////////////////////////////////////////////////////////////////////////////

    pthread_t rc_monitor_thread_id;
    err = pthread_create(&rc_monitor_thread_id, NULL, _rc_monitor_thread_func, NULL);
    if (err) return -1;
    if (pthread_detach(rc_monitor_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

    ///////////////////////////////////////////////////////////////////////////////
    // start thread to monitor autopilot status
    ///////////////////////////////////////////////////////////////////////////////

    pthread_t ap_monitor_thread_id;
    err = pthread_create(&ap_monitor_thread_id, NULL, _autopilot_monitor_thread_func, NULL);
    if (err) return -1;
    if (pthread_detach(ap_monitor_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

    ///////////////////////////////////////////////////////////////////////////////
    // start thread to send updates to ACTIVE and INACTIVE apps
    ///////////////////////////////////////////////////////////////////////////////

    pthread_t update_app_thread_id;
    ret = pthread_create(&update_app_thread_id, NULL, _update_app_thread_func, NULL);
    if (ret) {
        M_ERROR(PREFIX "ERROR: pthread_create errored with status: %d\n" POSTFIX, ret);
        return;
    }
    if (pthread_detach(update_app_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

    ///////////////////////////////////////////////////////////////////////////////
    // main
    ///////////////////////////////////////////////////////////////////////////////

    usleep(500000);  // wait 500 ms so there's time to populate available_apps and send some setpoints

    // warn user if PX4 still hasn't connected yet
	if(main_running && !autopilot_monitor_is_connected()){
		if(!pipe_client_is_connected(MAVLINK_ONBOARD_CH)){
			M_ERROR(PREFIX "ERROR: voxl-mavlink-server does not appear to be running.\n" POSTFIX);
			M_ERROR(PREFIX "Make sure it is enabled: systemctl enable voxl-mavlink-server. Exiting for now.\n" POSTFIX);
            goto CLEANUP;
        }
		else{
			M_ERROR(PREFIX "ERROR: connected to voxl-mavlink-server but no messages from autopilot. Please wait a second and try again. Exiting.\n" POSTFIX);
            goto CLEANUP;
		}
	}

    // if an app was given as an argument, start it
    if (app_name_argument_given) {
        // loop through available apps to see if the app given as an argument matches any of them
        for (int i=0; i<available_apps.num_apps; i++) {
            if (strncmp(app_name_argument, available_apps.apps[i].name, strlen(available_apps.apps[i].name)) == 0) {
                M_VERBOSE(PREFIX "Starting app from command line argument: %s!\n", available_apps.apps[i].name);
                
                pthread_mutex_lock(&start_app_mutex);
                start_app(i);

                // // create the thread that sends updates to apps
                // pthread_t update_app_thread_id;
                // int ret = pthread_create(&update_app_thread_id, NULL, _update_app_thread_func, NULL);
                // if (ret) {
                //     M_ERROR(PREFIX "ERROR: pthread_create errored with status: %d\n" POSTFIX, ret);
                // }
                // if (pthread_detach(update_app_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);
                pthread_mutex_unlock(&start_app_mutex);

                break;
            }
        }
        M_VERBOSE(PREFIX "Provided app name did not match an available app.\n" POSTFIX);
    }

    // run until start/stop module catches a signal and changes main_running to 0
    while (main_running) usleep(1000000);

    ///////////////////////////////////////////////////////////////////////////////
    // Stop all the threads and do cleanup
    ///////////////////////////////////////////////////////////////////////////////

CLEANUP:

	M_VERBOSE(PREFIX "Starting shutdown sequence\n" POSTFIX);

    stop_active_app(0);  // 0 to indicate that there is no app being run after active app is stopped

    // ensure all dynamically allocated memory for the queue has been freed
    for (int i=0; i<queue_end; i++) {
        if (queue[i].data != NULL) {
            free(queue[i].data);
        }
    }

    // Wait a second for apps to stop internal processes (i.e. trying to push to queue) before sending them SIGTERMs and shutting down voxl-app-manager
    usleep(1500000);

    // Terminate all app processes
    for (int i=0; i<available_apps.num_apps; i++) {
        if (available_apps.apps[i].status < TERMINATED) {  // not TERMINATED and not REMOVED
            terminate_app(available_apps.apps[i].pid, available_apps.apps[i].manager_w_pipe_fds[1]);
        }
    }

	pipe_server_close_all();
	remove_pid_file(PIPE_SERVER_NAME);
	M_VERBOSE(PREFIX "Exiting cleanly\n" POSTFIX);

    M_VERBOSE(PREFIX "Stopping mavlink io module.\n" POSTFIX);
	mavlink_io_stop();
	M_VERBOSE(PREFIX "Stopping autopilot monitor.\n" POSTFIX);
	autopilot_monitor_stop();

    // each module should have cleaned up its own pipes, but to be safe we
	// make are everything is closed up here
	M_VERBOSE(PREFIX "closing remaining client pipes\n" POSTFIX);
	pipe_client_close_all();
	M_VERBOSE(PREFIX "closing remaining server pipes\n" POSTFIX);
	pipe_server_close_all();

    // wait for all apps to finish terminating before terminating voxl-app-manager
    int ctr = 0;  // ctr serves as a timeout counter
    while (1) {
        int num_terminated_apps = 0;
        for (int i=0; i<available_apps.num_apps; i++) {
            if (available_apps.apps[i].status >= TERMINATED) {  // TERMINATED or REMOVED
                num_terminated_apps++;
            }
        }
        if (num_terminated_apps >= available_apps.num_apps || ctr > 10) {
            break;
        }

        ctr++;
        usleep(250000);
    }

    usleep(250000);  // give time for all children to finish terminating before terminating voxl-app-manager

	return 0;
}