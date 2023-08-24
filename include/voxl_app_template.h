/**
 * <voxl_app_template.h>
 *
 * @brief      Defines fields and callback functions for a generic hello world
 *             app for VOXL.
 *
 *
 * @author     Michael Zeng
 * @date       6/21/23
 */

#ifndef VOXL_APP_TEMPLATE_H
#define VOXL_APP_TEMPLATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <primitives.h>
#include "voxl_app_manager.h"


/**
 * Flag that indicates whether this app is actively running or not.
 * 
 * Best practice to wrap all actions in this app in an if-ACTIVE_FLAG-statement
 * so that this app doesn't continue trying to push to queue or expend resources
 * when it doesn't have control of the drone anyway.
 * 
 * Note: this flag is automatically updated in the onUpdate() function.
 */
extern uint8_t ACTIVE_FLAG;


typedef enum frame_t
{
    LOCAL = 0,
    FIXED,
} frame_t;


/**
 * @brief     Callback function that is called after the app is initialized,
 *            meaning voxl-app-manager has selected it to be run.
 * 
 * @return    0 if success, else error.
 *
 */
int onInit(void);


/**
 * @brief     Callback function that is called when the app starts going through
 *            and running all the primitives. There may be a delay between this
 *            event and the initialization event, as the start command must wait
 *            for any currently-running app to deinit before running.
 *
 * @param[in] update  update_t struct containing the drone's state data at the
 *                    moment this app is started
 * 
 * @return    0 if success, else error.
 */
int onStart(update_t update);


/**
 * @brief     Callback function that is called periodically (at 50 Hz) to send
 *            the latest status information from voxl-app-manager. See update_t
 *            struct for more information about what is contained in the update.
 *
 * @param[in] update  update_t struct containing latest update data from
 *                    voxl-app-manager.
 * 
 */
void onUpdate(update_t update);


/**
 * @brief     Callback function that is called following an unexpected event.
 *            The full list of possible unexpected events is defined above in
 *            event_t.
 *
 * @param[in] primitive_id      id of the primitive that caused the exception.
 *                              0 if the exception was not related to a primitive.
 * @param[in] primitive_type    type of the primitive that caused the exception.
 *                              -1 if the exception was not related to a primitive.
 * @param[in] event_type        the type of exception that occurred (enum value).
 * @param[in] event_data        struct containing data related to the exception
 *                              event.
 */
void onException(uint64_t primitive_id, primitive_t primitive_type, event_t event_type, void* event_data);


/**
 * @brief     Callback function that is called when stopping the currently
 *            running primitive.
 *
 */
void onStop(void);


/**
 * @brief     Callback function that is called after all primitives on the queue
 *            are finished, and the app is finished running.
 *
 */
void onDeinit(void);


#ifdef __cplusplus
}
#endif

#endif // VOXL_APP_TEMPLATE_H
