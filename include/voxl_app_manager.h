/**
 * <voxl_app_manager.h>
 *
 * @brief      App manager for voxl-apps. Scans for `.so` app files available in
 *             /etc/voxl-apps, runs the respective app based on input from an RC
 *             transmitter or portal or command-line arg, and runs the 
 *             instructions in the app.
 *
 *
 * @author     Michael Zeng
 * @date       6/22/23
 */


/**
 * STANDARD FOR INTER-PROCESS COMMUNICATION
 * 
 * voxl-app-manager is asynchronously monitoring messages from app-runner in a
 * thread. Meanwhile, app-runner is synchronously executing primitives in a loop
 * and sends messages to voxl-app-manager at the appropriate times.
 * 
 * Messages sent from app-runner should be prefaced with an 8-char MSG. In the
 * listener thread in voxl-app-manager, a switch case/if statement can then be
 * used to route different actions based on which 8-char MSG was first received.
 * 
 * If app-runner seeks information from voxl-app-manager, it can perform a
 * read() after writing the 8-char MSG. If app-runner seeks to send information
 * to voxl-app-manager, after writing the 8-char MSG, it should then perform a
 * read() with a timeout to ensure that voxl-app-manager is responsive, before
 * performing additional write()'s to transmit information.
 */

#ifndef VOXL_APP_MANAGER_H
#define VOXL_APP_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <sys/inotify.h>

#include <voxl_cutils.h>

#include "primitives.h"
#include "autopilot_monitor.h"


// Set to 0 if you want to prevent MAVLink setpoint commands from actually being sent to the drone.
#define MOVE_PROPS 1



#define INIT_TIMEOUT_DEFAULT 1000  // msecs
#define TIMEOUT_APP 1920  // secs

#define UPDATE_FREQUENCY_HZ 50

#define MAX_APP_NAME_LENGTH 63
#define MAX_CONCURRENT_APPS 64  // cannot be more than 64 .so files in /etc/voxl-apps

#define AVAILABLE_APPS_MAGIC_NUMBER 0x34587243

#define EVENT_SIZE (sizeof(struct inotify_event))
#define INOTIFY_BUF_LEN (1024 * (EVENT_SIZE + 16))

mavlink_odometry_t cur_pos;
mavlink_attitude_t cur_attitude;

// Always keep track of last mavlink command so we can revert to it in case of emergency
mavlink_set_position_target_local_ned_t last_mavlink_cmd;

extern pthread_mutex_t queue_pos_mutex;
extern pthread_mutex_t cur_pos_mutex;
extern pthread_mutex_t last_mavlink_cmd_mutex;

// Synchronization variable to ensure last_mavlink_cmd is set before we try to tell the drone to fly to it
uint8_t last_mavlink_cmd_set;
// Synchronization variable to ensure last_mavlink_cmd vz is set at the proper time (just once after manager_force_arm is called)
uint8_t set_drone_vz_on_ground;

// Global variable; 1 if AP is armed and in offboard mode, 0 else
uint8_t auto_pilot_state;

/**
 * All communication is prefaced with an 8-byte "header". The first 4 bytes
 * of the header contians the magic number (used to validate that pipe
 * communication is in sync. The next 2 bytes indicate the purpose/type of
 * the message. The last 2 bytes indicate the number of bytes. The listener 
 * threads will perform additional pipe reads and perform specific actions
 * based on the header that is read.
 * 
 * Note: All queue push messages have the type equal to the int value of the
 * primitive_t type of the primitive being pushed to the queue (see
 * primitives.h).
*/
#define HEADER_SIZE 8
extern const uint32_t APPS_MAGIC_NUMBER;
extern const uint16_t START_APP_MSG;
extern const uint16_t INIT_DONE_MSG;
extern const uint16_t QUEUE_POS_MSG;
extern const uint16_t UPDATE_MSG;
extern const uint16_t EXCEPTION_MSG;
extern const uint16_t CLEAR_QUEUE_MSG;
extern const uint16_t STOP_QUEUE_MSG;
extern const uint16_t PAUSE_QUEUE_MSG;
extern const uint16_t CONTINUE_QUEUE_MSG;
extern const uint16_t LIB_REMOVED_MSG;
extern const uint16_t STOP_APP_MSG;
extern const uint16_t SWITCH_TO_APP_MSG;
extern const uint16_t FORCE_ARM_MSG;
extern const uint16_t TERMINATE_APP_MSG;
extern const uint16_t APP_TERMINATED_MSG;
extern const uint16_t KILL_POWER_MSG;


// Note: the numerical values and order of this enum are critical and cannot be changed
typedef enum app_status_t{
    UNINITIALIZED = 0,
    READY = 1,      // if app is initialized, but has not been run before 
    INACTIVE = 2,   // if app previously ran, but is not currently the active app
    ACTIVE = 3,     // if app is running now
    TERMINATED = 4, // if app process was terminated for some reason
    REMOVED = 5,    // if .so file no longer exists
} app_status_t;

typedef struct {
    char name[MAX_APP_NAME_LENGTH + 1];
    int id;  // use the inode of the .so files as the id
    int pid;
    // need 2 separate pipes or a process may read it's own writes
    int manager_w_pipe_fds[2];  // for comms from voxl-app-manager to app
    int app_w_pipe_fds[2];  // for comms from app to voxl-app-manager
    int file_monitor_fd;
    app_status_t status;
    pthread_t init_thread_id;
} __attribute__((packed)) app_t;

typedef struct {
    uint32_t magic_number;
    int num_apps;  // length of apps array
    int active_app[2];  // {Index of currently running app in apps array, ID of currently running app}; {-1, -1} if no app has been run yet
    app_t apps[MAX_CONCURRENT_APPS];
} __attribute__((packed)) available_apps_t;

available_apps_t available_apps;

/**
 * Contains data that is passed in every onUpdate call.
 * 
 * In the case that there is no current primitive (i.e. when no app has been run
 * yet), current_prim_type is set to -1 and all bytes in current_prim are set to 
 * 0.
 */
typedef struct {
    struct timespec timestamp;
    char active_app[MAX_APP_NAME_LENGTH + 1];  // set to a null terminator if there is no active app
    primitive_t current_prim_type;
    void* current_prim;  // pointer to the struct of the primitive that is running now
    int queue_len;
    float progress;  // percentage completion of current primitive
    double batt_volts;
    
    // basically copying the fields of a mavlink_odometry_t message
    int frame_id;
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float q[4];  // quaternion representing current drone orientation
    float roll;
    float pitch;
    float yaw;
    float rollspeed;
    float pitchspeed;
    float yawspeed;

    px4_main_mode autopilot_main_mode;
    px4_sub_mode autopilot_sub_mode;
    uint8_t autopilot_is_armed;

    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) update_t;


/**
 * Used to pass data into the pthread that manages initialiation procedue timeout.
 * (new_app_init_manager_thread_id)
 */
typedef struct {
    int msecs;
    app_t* app;
} thread_timeout_t;


#define AVAILABLE_APPS_RECOMMENDED_READ_BUF_SIZE (sizeof(available_apps_t)*16)

#define AVAILABLE_APPS_RECOMMENDED_PIPE_SIZE (64*1024)

/**
 * @brief     Helper function to check if an app was previously found and
 *            already has it's own process running.
 *
 * @param[in] ino    the inode of the .so file
 * @return    1 if app exists, 0 else
 */
uint8_t check_app_exists(int ino);

/**
 * @brief     For the app corresponding to the given ID, stop any existing process, and
 *            restart the app starting from initialization.
 * 
 * @param[in] app_id    the ID of the app to be restarted
 */
void restart_app(int app_id);



/**
 * @brief     Helper function to build the header that precedes every pipe write
 *            between voxl-app-manager and app-runner. This function has no
 *            return value; the header is written into the char* msg that is
 *            passed in as a param.
 *
 * @param[in] msg         pointer to buffer to write header into
 * @param[in] type_int    type of message
 * @param[in] write_size  number of bytes being written in this message
 */
void build_msg_header(char* msg, uint16_t type_int, uint16_t write_size);


/**
 * @brief     Helper function to get the magic number, message type, and number
 *            of bytes back out of a message header. This is primarily used for
 *            reading the first 8 bytes of a message out of a pipe. This
 *            function has no returns; the values of the magic number, message
 *            type, and number of bytes is written directly into the pointers
 *            passed into this function.
 *
 * @param[in] msg         pointer to buffer containing the header data
 * @param[in] magic_num   pointer where magic number will be written
 * @param[in] type_int    pointer where type of message will be written
 * @param[in] write_size  pointer where number of bytes in message will be written
 */
void unpack_msg_header(char* msg, uint32_t* magic_num, uint16_t* type_int, uint16_t* write_size);


/**
 * @brief    This function updates the control pipe with available control
 *           commands based on the available_apps struct.
*/
void update_control_commands(void);


/**
 * @brief    This function updates the available_apps struct and the available
 *           apps in the Control Pipe by scanning through /etc/voxl-apps
 *           directory for .so files.
*/
void update_available_apps(void);


/**
 * @brief     Helper function that takes the appropriate actions to start
 *            running an app.
 * 
 * @param[in] i  Index of the app to run in available_apps.apps
 */
void start_app(int i);


/**
 * @brief     Use this to simultaneously validate that the bytes from a pipe
 *            contains valid data, find the number of valid packets
 *            contained in a single read from the pipe, and cast the raw data
 *            buffer as an available_apps_t* for easy access.
 *
 *            This does NOT copy any data and the user does not need to
 *            allocate available_apps_t array separate from the pipe read buffer.
 *            The data can be read straight out of the pipe read buffer, much
 *            like reading data directly out of a mavlink_message_t message.
 *
 *            However, this does mean the user should finish processing this
 *            data before returning the pipe data callback which triggers a new
 *            read() from the pipe.
 *
 * @param[in] data        pointer to pipe read data buffer
 * @param[in] bytes       number of bytes read into that buffer
 * @param[in] n_packets   number of valid packets received
 * 
 * @return    Returns the same data pointer provided by the first argument, but
 *            cast to an available_apps_t* struct for convenience. If there was an
 *            error then NULL is returned and n_packets is set to 0
 */
static available_apps_t* pipe_validate_available_apps_t(char* data, int bytes, int* n_packets);

static available_apps_t* pipe_validate_available_apps_t(char* data, int bytes, int* n_packets) {
    // cast raw data from buffer to array of available_app_t*
    // note: there may be multiple available_app_t* if the pipe-reading falls behind the pipe writing (so n_packets > 1)
    available_apps_t* new_ptr = (available_apps_t*) data;
    *n_packets = 0;

    // basic sanity checks
    if (bytes<0) {
        fprintf(stderr, "ERROR validating available apps received through pipe: number of bytes = %d\n", bytes);
        return NULL;
    }
    if (data==NULL) {
        fprintf(stderr, "ERROR validating available apps received through pipe: got NULL data pointer\n");
        return NULL;
    }
    if (bytes%sizeof(available_apps_t)) {
        fprintf(stderr, "ERROR validating available apps received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(available_apps_t));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes/sizeof(available_apps_t);

    // check if any packets failed the magic number check
    int i, n_failed = 0;
    for (i=0;i<n_packets_tmp;i++) {
        if(new_ptr[i].magic_number != AVAILABLE_APPS_MAGIC_NUMBER) n_failed++;
    }
    if (n_failed>0) {
        fprintf(stderr, "ERROR validating available apps received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
        return NULL;
    }

    // if we get here, all good. Write out the number of packets read and return
    // the new cast pointer. It's the same pointer the user provided but cast to
    // the right type for simplicity and easy of use.
    *n_packets = n_packets_tmp;
    return new_ptr;
}


#ifdef __cplusplus
}
#endif

#endif // VOXL_APP_MANAGER_H
