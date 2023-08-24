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
#include <signal.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <dlfcn.h>  // dlopen, RTLD_LAZY, dlsym
#include <sys/syscall.h>

#include <voxl_cutils.h>
// #include <modal_journal.h>  // libmodal_journal is not compatible with multi-processing
#include "modal_journal_lite.h"

#include "app_runner.h"
#include "primitives.h"
#include "queue.h"
#include "voxl_app_manager.h"

static uint8_t print_threads = 0;

static uint8_t ACTIVE_FLAG;  // 1 if the app is currently running and has control of the queue

// synchronization variables to ensure all resources halted before terminating this process
static uint8_t app_running;  // 0 if TERMINATE_APP_MSG received
static uint8_t cleaned_up;   // 1 if sigterm_handler has been run

static uint8_t lib_loaded;

static int err;
static void* apphandle;

#define PREFIX   COLOR_LIT_YLW "app-runner:\t"
#define POSTFIX  RESET_COLOR

int (*init_cb)(void);
int (*start_cb)(update_t);
void (*update_cb)(update_t);
void (*exception_cb)(uint64_t, primitive_t, event_t, void*);
void (*stop_cb)();
void (*deInit_cb)(void);

// pipe sychronization variables
int8_t QUEUE_PUSH_CONFIRMED_FLAG = 0;

// global thread id so that the thread can be created from the listener thread but joined from the main thread
pthread_t run_app_thread_id;


static void sigterm_handler(__attribute__((unused)) int sig) {
    if (lib_loaded) {
        // call onDeinit function in the app
        deInit_cb();
        dlclose(apphandle);
    }
    else {
        M_WARN(PREFIX "Unable to call app's onDeinit function; .so file for app has likely been removed.\n" POSTFIX);
    }

    // close read pipe
    close(app.manager_w_pipe_fds[0]);

    // let voxl-app-manager know that termination is finished
    char term_msg[HEADER_SIZE];
    build_msg_header(term_msg, APP_TERMINATED_MSG, 0);  // takes no additional data beyond magic number and message type
    err = write(app.app_w_pipe_fds[1], term_msg, HEADER_SIZE);  // pipe write
    if (err == -1) M_ERROR(PREFIX "ERROR: Failed to notify voxl-app-manager of app termination. errno: %d.\n" POSTFIX, errno);

    // close write pipe
    close(app.app_w_pipe_fds[1]);

    cleaned_up = 1;
}


static void* _update_thread_func(void* void_star_update) {
    /**
     * Simple thread to call onUpdate() so that this doesn't have to happen in
     * _app_runner_listener_thread_func and block the listener from actually
     * listening.
     */
    if (print_threads) printf("_update_thread_func thread id: %d\n", syscall(SYS_gettid));

    update_t* update = (update_t*)void_star_update;

    update_cb(*update);

    // after onUpdate is finished, free the memory that was allocated to store the current primitive data
    free(update->current_prim);
    update->current_prim = NULL;  // pointer nullification for safety (so we won't get a silent error if we try to access this memory again)

    free(update);
    update = NULL;  // pointer nullification for safety

    return NULL;

}


static void* _run_app_thread_func(void* void_star_update) {
    /**
     * This thread function contains the code to *run* an app (not including
     * initialization and deinitialization of the app).
     * 
     * The lifetime of this thread is as long as the app is ACTIVE.
     */
    if (print_threads) printf("_run_app_thread_func thread id: %d\n", syscall(SYS_gettid));

    update_t* update = (update_t*)void_star_update;

    // run onStart()
    err = start_cb(*update);  // populate queue with app's primitives
    if (err != 0) {
        M_ERROR(PREFIX "ERROR: onStart() function returned error." POSTFIX);
    }

    // Once the primitives are pushed to queue, voxl-app-manager will immediately start executing them

    // after onStart is finished, free the memory that was allocated to store the current primitive data
    free(update->current_prim);
    update->current_prim = NULL;  // pointer nullification for safety (so we won't get a silent error if we try to access this memory again)

    free(update);
    update = NULL;  // pointer nullification for safety

    // This thread needs to keep running so long as the app is active since this
    // thread is used as a timeout (with timedjoin)
    while (app_running && ACTIVE_FLAG) {
        usleep(50000);  // 50 msecs
    }

    M_VERBOSE(PREFIX "Terminating _run_app_thread_func.\n" POSTFIX);

    return NULL;
}


static void* _app_runner_listener_thread_func(__attribute__((unused)) void* arg) {
    /**
     * Constantly listening for messages from the app process over pipe.
     * Is not explicitely joined; gets joined when the app is terminated.
     * 
     * This thread is created as soon as the app's process is forked.
     */
    if (print_threads) printf("_app_runner_listener_thread_func thread id: %d\n", syscall(SYS_gettid));

    while(1) {  // run until process termination
        /////////////////////////////////////////////////////
        // NOTE: THIS IS NOT A SPIN LOOP. read() IS BLOCKING.
        /////////////////////////////////////////////////////

        char read_buff[HEADER_SIZE];
        memset(read_buff, 0, sizeof(read_buff));  // initialize buff to 0's
        int err = read(app.manager_w_pipe_fds[0], read_buff, HEADER_SIZE);
        if (ACTIVE_FLAG == 1 && err == -1) {
            M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
            break;
        }

        uint32_t magic_num;
        uint16_t type, num_bytes;
        unpack_msg_header(read_buff, &magic_num, &type, &num_bytes);
        if (magic_num != APPS_MAGIC_NUMBER) {
            M_ERROR(PREFIX "ERROR: Received pipe message that did not begin with valid header. Received: magic number: %d; msg type: %d; number of bytes: %d. voxl-app-manager likely segfaulted.\n" POSTFIX, magic_num, type, num_bytes);
            usleep(1000000);  // prevent out of control spin loops
            continue;
        }

        // just so we don't get spammed by update messages
        if (type != UPDATE_MSG) M_VERBOSE(PREFIX "Received command from pipe of type %d, %d bytes.\n" POSTFIX, type, num_bytes);


        if (type == START_APP_MSG) {
            // read rest of the data in this message
            char buf[num_bytes];
            err = read(app.manager_w_pipe_fds[0], buf, num_bytes);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
            }

            /***********************READING UPDATE DATA************************/

            // Select the part of the buffer that corresponds to the update data
            update_t* update = (update_t*)malloc(sizeof(update_t));
            memcpy(update, buf, sizeof(update_t));

            // allocate memory to store the current primitive data
            char* current_prim_data = (char*)malloc(num_bytes - sizeof(update_t));  // allocate memory for data array
            memcpy(current_prim_data, buf + sizeof(update_t), num_bytes - sizeof(update_t));

            // set update's current_prim to point to this data
            update->current_prim = current_prim_data;

            /***************************START THE APP**************************/

            if (pthread_create(&run_app_thread_id, NULL, _run_app_thread_func, update) != 0) {
                M_ERROR(PREFIX "ERROR, couldn't create run_app_thread_id thread.\n" POSTFIX);
            }
            if (pthread_detach(run_app_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

            
            ACTIVE_FLAG = 1;  // let main thread know to now wait for this thread to join
        }
        else if (type == STOP_APP_MSG) {
            M_VERBOSE(PREFIX "Received signal to stop current app.\n" POSTFIX);
            ACTIVE_FLAG = 0;
            if (lib_loaded) {
                stop_cb();
            }
            else {
                M_WARN(PREFIX "Unable to call app's onStop function; .so file for app has likely been removed.\n" POSTFIX);
            }
        }
        else if (type == QUEUE_POS_MSG) {
            QUEUE_PUSH_CONFIRMED_FLAG = 1;

            // read rest of the data in this message
            char buf[num_bytes];
            err = read(app.manager_w_pipe_fds[0], buf, num_bytes);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
            }

            // get queue_end
            memcpy(&queue_end, buf, sizeof(queue_end));

            // get queue_pos
            memcpy(&queue_pos, buf + sizeof(queue_end), sizeof(queue_pos));
        }
        else if (type == LIB_REMOVED_MSG) {
            lib_loaded = 0;
        }
        else if (type == UPDATE_MSG) {
            // read the update struct data
            char buf[num_bytes];
            err = read(app.manager_w_pipe_fds[0], buf, num_bytes);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
            }

            /***********************READING UPDATE DATA************************/

            // Select the part of the buffer that corresponds to the update data
            update_t* update = (update_t*)malloc(sizeof(update_t));
            memcpy(update, buf, sizeof(update_t));

            // allocate memory to store the current primitive data
            char* current_prim_data = (char*)malloc(num_bytes - sizeof(update_t));  // allocate memory for data array
            memcpy(current_prim_data, buf + sizeof(update_t), num_bytes - sizeof(update_t));

            // set update's current_prim to point to this data
            update->current_prim = current_prim_data;

            /**************************CALL ONUPDATE***************************/

            // call onUpdate() with the update data in a separate thread so that it doesn't slow down listener thread
            pthread_t update_thread_id;
            int err = pthread_create(&update_thread_id, NULL, _update_thread_func, update);
            if (err != 0) {
                M_ERROR(PREFIX "couldn't create update_thread_id thread; error code %d.\n" POSTFIX, err);
            }
            // Detach the thread so that its resources are released once the thread finishes and returns
            // Otherwise, because of the amount of updates that happen, these threads will quickly eat all the memory
            if (pthread_detach(update_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);
        }
        else if (type == EXCEPTION_MSG) {
            // read the update struct data
            char buf[num_bytes];
            err = read(app.manager_w_pipe_fds[0], buf, num_bytes);
            if (err == -1) {
                M_ERROR(PREFIX "Pipe read errored.\n" POSTFIX);
            }

            // Read the exception event type from the first 4 bytes
            event_t exception_event;
            memcpy(&exception_event, buf, sizeof(exception_event));

            // Read the primitive type that caused the exception
            primitive_t primitive_type;
            memcpy(&primitive_type, buf + sizeof(exception_event), sizeof(primitive_t));

            // Read the ID of the primitive that caused the exception
            uint64_t primitive_id;
            memcpy(&primitive_id, buf + sizeof(exception_event) + sizeof(primitive_t), sizeof(uint64_t));

            // Call onException
            exception_cb(primitive_id, primitive_type, exception_event, NULL);
        }
        else if (type == TERMINATE_APP_MSG) {
            app_running = 0;
            break;
        }
    }

    M_VERBOSE(PREFIX "Terminating _app_runner_listener_thread_func.\n" POSTFIX);

    return NULL;
}



uint64_t queue_push(void* ptr) {
    // derive primitive type from ptr
    primitive_t type;
    memcpy(&type, ptr, sizeof(primitive_t));

    // Warn user and return if the queue_push is invalid due to the app being inactive
    if (ACTIVE_FLAG == 0) {
        M_WARN(PREFIX "Attempted pushing primitive of type %d to queue, but app is not active and does not have queue permissions. Aborting.\n" POSTFIX, type);
        return 0;
    }

    // Set ID of primitive
    // The ID is based on Monotonic time (so theoretically no two primitives will get assigned the same ID)
    uint64_t id;
    struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &currentTime);

    // get lower 34 bits of tv_sec and lower 30 bits of tv_nsec
    // I chose 30 and 34 specifically because all 999,999,999 nsecs can already be represented in the lower 30 bits of currentTime.tv_nsec
    // this also means that primitive ID's may start to repeat if voxl-app-manager is run for more than 2^34 seconds continuously 
    id = ((uint64_t)(currentTime.tv_sec & 0x3FFFFFFFF) << 30) | (currentTime.tv_nsec & 0x3FFFFFFF);

    char* primitive_data;
    uint16_t write_size;

    switch(type) {
        case PAUSE: {
            primitive_pause_t* prim = (primitive_pause_t*)ptr;
            prim->id = id;

            primitive_data = (char*)malloc(sizeof(primitive_pause_t));  // allocate memory for data array
            memcpy(primitive_data, (unsigned char*)prim, sizeof(primitive_pause_t));  // copy data from struct into char array
            write_size = sizeof(primitive_pause_t);
        } break;
        case LAND: {
            primitive_land_t* prim = (primitive_land_t*)ptr;
            prim->id = id;

            primitive_data = (char*)malloc(sizeof(primitive_land_t));  // allocate memory for data array
            memcpy(primitive_data, (unsigned char*)prim, sizeof(primitive_land_t));  // copy data from struct into char array
            write_size = sizeof(primitive_land_t);
        } break;
        case TAKE_OFF: {
            primitive_take_off_t* prim = (primitive_take_off_t*)ptr;
            prim->id = id;

            primitive_data = (char*)malloc(sizeof(primitive_take_off_t));  // allocate memory for data array
            memcpy(primitive_data, (unsigned char*)prim, sizeof(primitive_take_off_t));  // copy data from struct into char array
            write_size = sizeof(primitive_take_off_t);
        } break;
        case SETPOINT: {
            primitive_setpoint_t* prim = (primitive_setpoint_t*)ptr;
            prim->id = id;

            primitive_data = (char*)malloc(sizeof(primitive_setpoint_t));  // allocate memory for data array
            memcpy(primitive_data, (unsigned char*)prim, sizeof(primitive_setpoint_t));  // copy data from struct into char array
            write_size = sizeof(primitive_setpoint_t);
        } break;
        case GO_TO_WAYPOINT: {
            primitive_go_to_waypoint_t* prim = (primitive_go_to_waypoint_t*)ptr;
            prim->id = id;

            primitive_data = (char*)malloc(sizeof(primitive_go_to_waypoint_t));  // allocate memory for data array
            memcpy(primitive_data, (unsigned char*)prim, sizeof(primitive_go_to_waypoint_t));  // copy data from struct into char array
            write_size = sizeof(primitive_go_to_waypoint_t);
        } break;
        case LOCAL_POLYNOMIAL: {
            primitive_local_polynomial_t* prim = (primitive_local_polynomial_t*)ptr;
            prim->id = id;

            primitive_data = (char*)malloc(sizeof(primitive_local_polynomial_t));  // allocate memory for data array
            memcpy(primitive_data, (unsigned char*)prim, sizeof(primitive_local_polynomial_t));  // copy data from struct into char array
            write_size = sizeof(primitive_local_polynomial_t);
        } break;
        default:
            M_ERROR(PREFIX "queue_push() did not find a valid primitive type. Exiting.\n" POSTFIX);
            return 0;
    }

    // construct queue push message
    size_t queue_push_msg_len = HEADER_SIZE + write_size;
    char queue_push_msg[queue_push_msg_len];
    build_msg_header(queue_push_msg, (uint16_t)type, write_size);  // the integer version of primitive type is used as the msg type
    memcpy(queue_push_msg + HEADER_SIZE, primitive_data, write_size);  // copy primitive_data into char array

    err = write(app.app_w_pipe_fds[1], queue_push_msg, queue_push_msg_len);  // pipe write
    if (err == -1) {
        M_ERROR(PREFIX "Pipe write errored.\n" POSTFIX);
    }

    free(primitive_data);
    primitive_data = NULL;  // pointer nullification for safety (so we won't get a silent error if we try to access this memory again)

    int ctr = 0;
    while (QUEUE_PUSH_CONFIRMED_FLAG == 0) {
        usleep(10000);  // wait until voxl-app-manager confirms the queue push
        ctr++;
        if (ctr > 10) {  // more than 100ms elapsed
            M_ERROR(PREFIX "No response from voxl-app-manager after attempting to push primitive of type %d to queue. Canceling operation. Likely attempted invalid push command.\n" POSTFIX, type);
            return 0;
        }
    }

    QUEUE_PUSH_CONFIRMED_FLAG = 0;  // reset flag

    return id;
}


void run_app(app_t* app_arg) {
    if (print_threads) printf("run_app thread id: %d\n", syscall(SYS_gettid));

    app_running = 1;
    cleaned_up = 0;
    ACTIVE_FLAG = 0;

    signal(SIGTERM, sigterm_handler);  // set up handler for termination signal

    /////////////////////////////////////////////////////////////////////////////////
    // Load and perform initialization tasks for new app. Send message to voxl-app-manager when finished.
    /////////////////////////////////////////////////////////////////////////////////
    M_VERBOSE(PREFIX "Beginning initialization procedure.\n" POSTFIX);

    app = *app_arg;

    // create thread to listen to voxl-app-manager
    pthread_t self_thread_id = pthread_self();
    pthread_t app_runner_listener_thread_id;
    if (pthread_create(&app_runner_listener_thread_id, NULL, _app_runner_listener_thread_func, &self_thread_id) != 0) {
        M_ERROR(PREFIX "ERROR, couldn't create app_runner_listener_thread_id thread.\n" POSTFIX);
    }
    if (pthread_detach(app_runner_listener_thread_id) != 0) M_ERROR(PREFIX "pthread_detach failed.\n" POSTFIX);

    // close unused pipes
    close(app.app_w_pipe_fds[0]);
    close(app.manager_w_pipe_fds[1]);

    char app_full_path[MAX_APP_NAME_LENGTH + 15] = "/etc/voxl-apps/";
    strcat(app_full_path, app.name);
    
    apphandle = dlopen(app_full_path, RTLD_LAZY);  // RTLD_LAZY = functions will be loaded when I try to use them (lazy loading)
    if (apphandle == NULL) {
        M_ERROR(PREFIX "Library open failed. Quitting app.\n" POSTFIX);
    }

    lib_loaded = 1;

    // load in onInit, onStart, onUpdate, onException, and onStop functions in the app
    init_cb = dlsym(apphandle, "onInit");
    if (init_cb == NULL) {
        M_ERROR(PREFIX "Could not find onInit function in app. Quitting.\n" POSTFIX);
    }

    start_cb = dlsym(apphandle, "onStart");
    if (start_cb == NULL) {
        M_ERROR(PREFIX "Could not find onStart function in app. Quitting.\n" POSTFIX);
    }

    update_cb = dlsym(apphandle, "onUpdate");
    if (update_cb == NULL) {
        M_ERROR(PREFIX "Could not find onUpdate function in app. Quitting.\n" POSTFIX);
    }

    exception_cb = dlsym(apphandle, "onException");
    if (exception_cb == NULL) {
        M_ERROR(PREFIX "Could not find onException function in app. Quitting.\n" POSTFIX);
    }

    stop_cb = dlsym(apphandle, "onStop");
    if (stop_cb == NULL) {
        M_ERROR(PREFIX "Could not find onStop function in app. Quitting.\n" POSTFIX);
    }

    deInit_cb = dlsym(apphandle, "onDeinit");
    if (deInit_cb == NULL) {
        M_ERROR(PREFIX "Could not find onDeinit function in app. Quitting.\n" POSTFIX);
    }

    // call onInit function in the app
    err = init_cb();
    if (err != 0) {
        M_ERROR(PREFIX "ERROR: onStart() function returned error." POSTFIX);
    }

    // let voxl-app-manager know that initialization succeeded. 
    // construct INIT_DONE_MSG
    char init_done_msg[HEADER_SIZE];
    build_msg_header(init_done_msg, INIT_DONE_MSG, 0);  // takes no additional data beyond magic number and message type

    err = write(app.app_w_pipe_fds[1], init_done_msg, HEADER_SIZE);  // pipe write
    if (err == -1) {
        M_ERROR(PREFIX "Pipe write errored.\n" POSTFIX);
    }

    while(app_running) {  // run until app process is terminated

        /////////////////////////////////////////////////////////////////////////////////
        // Wait for command from voxl-app-manager to start running the app (is blocking)
        /////////////////////////////////////////////////////////////////////////////////

        // wait for ACTIVE_FLAG to be set high (START_APP_MSG)
        while (ACTIVE_FLAG != 1) {
            usleep(20000);  // 20 msecs
            if (app_running == 0) {
                goto CLEAN_UP;
            }
        }
        // M_VERBOSE(PREFIX "Received signal to start app ID: %lu.\n" POSTFIX, app.id);

        // set timed join on app thread
        struct timespec thread_timeout;
        clock_gettime(CLOCK_REALTIME, &thread_timeout);
        thread_timeout.tv_sec += TIMEOUT_APP;
        int err = pthread_timedjoin_np(run_app_thread_id, NULL, &thread_timeout);
        if (err == ETIMEDOUT) {
            M_ERROR(PREFIX "WARNING: currently running app exceeded timeout. App is being terminated.\n" POSTFIX);
            ACTIVE_FLAG = 0;
        }

        usleep(100000);  // give time for TERMINATE_APP_MSG to be received
    }

    usleep(100000);  // give some time for threads to terminate first

CLEAN_UP:

    while(cleaned_up != 1) {  // wait for onDeinit to run and for other resources to be cleaned in the sigterm handler before exiting
        usleep(10000);
    }

    M_VERBOSE(PREFIX "APP PROCESS EXITING.\n" POSTFIX);

    // exit(EXIT_SUCCESS);

    return;
}


void switch_to_app(char* new_app) {
    M_WARN("switch_to_app called.\n");
    if (ACTIVE_FLAG) {
        // construct SWITCH_TO_APP_MSG
        size_t switch_app_msg_len = HEADER_SIZE + strlen(new_app);
        char switch_app_msg[switch_app_msg_len];
        build_msg_header(switch_app_msg, SWITCH_TO_APP_MSG, switch_app_msg_len - HEADER_SIZE);
        strncpy(switch_app_msg + HEADER_SIZE, new_app, strlen(new_app));

        err = write(app.app_w_pipe_fds[1], switch_app_msg, switch_app_msg_len);  // pipe write
        if (err == -1) {
            M_ERROR(PREFIX "Pipe write errored.\n" POSTFIX);
        }
    }
    else {
        M_WARN(PREFIX "switch_to_app was called, however, the calling app is not active and therefore does not have the power to switch apps.\n" POSTFIX);
    }
    
}


void send_msg_to_app_manager(uint16_t msg_code) {
    char* msg = (char*)malloc(HEADER_SIZE);
    build_msg_header(msg, msg_code, 0);  // takes no additional data beyond magic number and message type
    
    // app is a global var defined in app_runner.c
    int err = write(app.app_w_pipe_fds[1], msg, HEADER_SIZE);  // pipe write
    if (err == -1) {
        fprintf(stderr, "Pipe write errored in send_msg_to_app_manager. Message Type: %d\n", msg_code);
    }

    free(msg);
}