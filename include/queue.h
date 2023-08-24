/**
 * <queue.h>
 *
 * @brief      Queue data structure to contain primitives to execute in an app.
 *
 *
 * @author     Michael Zeng
 * @date       6/21/23
 */

#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "primitives.h"

#define MAX_QUEUE_LEN 200000

/* Fixed length array to hold list of primitives to execute */
primitive_generic_t queue[MAX_QUEUE_LEN];

// Note: only designed to support 1 queue at a time.

/**
 * Holds the current position in the queue. This increments every time voxl
 * begins executing a new primitive.
 * 
 * This is always at least 1 less than queue_end.
 */ 
int queue_pos;

/**
 * Holds index of the last position in the queue plus 1 (which is equivalent to 
 * the index that would be occupied to any new primitive that is pushed to the 
 * queue). This increments every time a new primitive is added to the queue.
 */
int queue_end;

// Global flag variable containing whether the execution of the current primitive should be paused
uint8_t QUEUE_PAUSED_FLAG;

/* 
Global flag variable containing whether the execution of the current primitive
should be stopped.

This is set to 1 by stop_queue(), and reset to 0 every iteration of the loop in
_execute_queue_thread_func.
*/
uint8_t QUEUE_STOPPED_FLAG;


/**
 * @brief    Function for pushing a primitive struct pointer to queue.
 *           Technically, the queue is an array of primitive_generic_t, which
 *           contain char arrays. When the primitive is pushed to the queue,
 *           it is casted into a char array.
 * 
 *           Note: this should only be used internally, when called from
 *           main.c. Calling this from the app itself will not error, but it
 *           write to separate memory than what voxl-app-manager is using.
 *
 * @param[in] queue  pointer to queue
 * @param[in] ptr    pointer to primitive struct to push onto the queue
 */
void push(char* ptr, primitive_t type);


/**
 * @brief    The non-user-facing version of clear_queue(). This should only be
 *           called from voxl-app-manager.
 */
void manager_clear_queue(void);


/**
 * @brief    Function for clearing the queue. When this function is called, all
 *           future primitives in the queue will be effectively erased, however,
 *           the primitive that is currently running will continue.
 */
void clear_queue(void);

// /**
//  * @brief    Returns number of primitives in queue that have not been executed
//  *           yet (not including the currently running primitive).
//  * 
//  *           Note: this should only be used internally, when called from
//  *           main.c. Calling this from the app itself will not error, but it
//  *           write to separate memory than what voxl-app-manager is using.
//  * 
//  * @returns  integer length of the queue.
//  */
// int manager_get_queue_length(void);


// /**
//  * @brief    Returns number of primitives in queue that have not been executed
//  *           yet (not including the currently running primitive).
//  * 
//  *           THIS IS A USER-FACING FUNCTION AND SHOULD BE RUN IN EITHER
//  *           app_runner.c OR THE APP'S .c FILE ITSELF (i.e. voxl_app_template.c)
//  * 
//  * @returns  integer length of the queue.
//  */
// int get_queue_length(void);


/**
 * @brief    The non-user-facing version of stop_queue(). This should only be
 *           called from voxl-app-manager.
 */
void manager_stop_queue(void);


/**
 * @brief    Function for stopping execution of the queue. The currently running
 *           primitive will be interrupted, and the queue will be cleared. New
 *           primitives must be pushed to queue to get the drone to perform more
 *           behaviors.
 */
void stop_queue(void);


/**
 * @brief    The non-user-facing version of pause_queue(). This should only be
 *           called from voxl-app-manager.
 */
void manager_pause_queue(void);


/**
 * @brief    Function for pausing queue_execution. voxl-app-manager will
 *           pause the currently running primitive and have the drone hover
 *           in place until continue_queue() is called.
 */
void pause_queue(void);


/**
 * @brief    The non-user-facing version of continue_queue(). This should only be
 *           called from voxl-app-manager.
 */
void manager_continue_queue(void);


/**
 * @brief    Function for continuing queue execution. This can be used while
 *           a primitive_pause_t primitive is being executed, or after
 *           pause_queue() was called to continue execution of the queue
 *           or paused primitive.
 */
void continue_queue(void);


/**
 * @brief    The non-user-facing version of kill_power(). This should only be
 *           called from voxl-app-manager.
*/
void manager_kill_power(void);


/**
 * @brief    Cut power from drone's motors. (Technically, this just forces a
 *           disarm, even if the drone is in flight.)
*/
void kill_power(void);


/**
 * @brief    The non-user-facing version of force_arm(). This should only be
 *           called from voxl-app-manager.
 */
void manager_force_arm(void);


/**
 * @brief    Arm the drone. Note that this takes ~2.5 seconds.
 */
void force_arm(void);


#ifdef __cplusplus
}
#endif

#endif // QUEUE_H
