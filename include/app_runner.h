/**
 * <app_runner.h>
 *
 * @brief      Critical functions and variables related to running apps for 
 *             voxl-apps.
 *
 *
 * @author     Michael Zeng
 * @date       7/5/23
 */

#ifndef APP_RUNNER_H
#define APP_RUNNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "voxl_app_manager.h"

app_t app;


/**
 * @brief    Basic helper function to help send a message (with just a header, no 
 *           payload) to voxl-app-manager.
 *
 * @param[in] msg_code    magic number representing the message type, as defined
 *                        at the top of main.c.
 */
void send_msg_to_app_manager(uint16_t msg_code);


/**
 * @brief    Helps push primitives to queue. Technically, this sends a message
 *           to voxl-app-manager containining the primitive data, then
 *           voxl-app-manager recreates the primitive and pushes it to queue. 
 *           This is because app-runner is running in a child process and does
 *           not actually have control over the queue.
 *
 * @param[in] ptr    pointer to primitive struct to push onto the queue.
 * 
 * @return   Returns the ID assigned to the primitive that is being pushed.
 */
uint64_t queue_push(void* ptr);


/**
 * @brief    The main function run when a new app's process is created.
 *
 * @param[in] app        Struct containing the apps data.
 */
void run_app(app_t* app);


/**
 * @brief    Stop the current app and start running a new app.
 * 
 * @param[in] new_app    null-terminated string ending in '.so' that is the name
 *                       of the new app to run.
 */
void switch_to_app(char* new_app);


#ifdef __cplusplus
}
#endif

#endif // APP_RUNNER_H
