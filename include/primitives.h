/**
 * <primitives.h>
 *
 * @brief      All primitive types, definitions, and functions for VOXL apps.
 *
 *
 * @author     Michael Zeng
 * @date       6/21/23
 */

#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <c_library_v2/common/mavlink.h>

float primitive_progress;
extern pthread_mutex_t primitive_progress_mutex;

/*
Keep track of last primitive's finish time, in order to achieve more precise
timing. For example, in a long chain of SETPOINTs, the duration fo each SETPOINT
will be dynamically set based on the last primitive's finish time to maintain
the overall timing of the whole chain.

Set to 0 to denote a NULL value (i.e. if no primitive has been run yet, or
if the queue finishes executing so the last primitive is irrelevant).
*/
uint64_t last_primitive_finish_time;

/*
Global flag variable denoting whether a primitive_pause_t is currently running.
This is used by the implementation of primitive_pause_t to recognize when to
break out of the pause.
*/
uint8_t PAUSE_PRIMITIVE_RUNNING;

typedef enum event_t {
    APP_INTERRUPTED = 1,
    QUEUE_STOPPED,
    NOT_IN_OFFBOARD_MODE,
    CRASH,
    STUCK,
    LOW_BATTERY,
    UNABLE_TO_LAND,
    FREE_FALL_WARNING,
    PRIMITIVE_TIMEOUT,
    SWITCH_TO_APP_FAILED,
} event_t;


typedef enum primitive_t {
    PAUSE = 0,
    LAND,
    TAKE_OFF,
    SETPOINT,
    GO_TO_WAYPOINT,
    LOCAL_POLYNOMIAL,
} primitive_t;



/**
 * All primitive structs need a type field so the void pointers in the queue
 * can be cast to struct pointers.
 * 
 * The ID field of primitive structs should not be modified; they are
 * automatically populated when adding the primitives to queue.
 * 
 * IMPORTANT: The type field must be the first field in each primitive struct.
 */


/**
 * Generic struct that contains certain shared fields between all primitives.
 * 
 * This is primarily used to construct the queue of primitives. The queue
 * is an array of primitive_generic_t; each primitive_generic_t is later
 * reconstructed back into instances of specific primitive types.
 */
typedef struct {
    primitive_t type;
    float status;
    char* data;
} __attribute__((packed)) primitive_generic_t;

/**
 * Continue executing the last mavlink command from the previous primitive for 
 * the given amount of time, or set duration to -1 for an indefinite pause (must
 * be continued using the continue_queue function).
 */
typedef struct {
    primitive_t type;  // PAUSE
    uint64_t id;  // do not manually assign; this is auto-assigned when the primitive is added to queue
    float duration;  // ms
    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) primitive_pause_t;


/**
 * Slowly lower to the specified height (relative to local-frame origin) and
 * disarm if the drone detects that it is on a stable surface.
 * 
 * If the drone must abort landing (does not find a surface at the specified 
 * height), it will continue to hover at its last position before abort.
 */
typedef struct {
    primitive_t type;  // LAND
    uint64_t id;   // do not manually assign; this is auto-assigned when the primitive is added to queue
    float yaw;     // yaw angle to land at relative to local frame
    float height;  // CURRENTLY UNUSED
    float abort_height;  // CURRENTLY UNUSED
    float descent_rate;  // m/s
    float accuracy;  // acceptable offset from desired lanidng position (meters) - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2)
    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) primitive_land_t;


/**
 * Go to the specified height above the takeoff point.
 * 
 * Only should be run if drone is on the ground.
 */
typedef struct {
    primitive_t type;  // TAKE_OFF
    uint64_t id;  // do not manually assign; this is auto-assigned when the primitive is added to queue
    // float yaw;     // yaw angle to land at relative to local frame
    float height;  // m (meters above the takeoff point; must be positive)
    float ascent_rate;  // m/s

    float x_accuracy;        // m; allowed distance to setpoint target before moving onto next primitive in queue
    float y_accuracy;        // m; allowed distance to setpoint target before moving onto next primitive in queue
    float z_accuracy;        // m; allowed distance to setpoint target before moving onto next primitive in queue
    float max_final_speed;   // m/s; allowed maximum final speed before moving onto next primitive in queue
    float yaw_accuracy;      // rad; allowed error before moving onto next primitive in queue
    float yaw_rate_accuracy; // rad/s; allowed maximum final yaw rate before moving onto next primitive in queue

    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) primitive_take_off_t;


/**
 * Fly to the specified setpoint in local NED frame. The application of setpoint
 * is to have a chain of a large number of setpoints that the drone follows
 * sequentially in time.
 * 
 * A setpoint should not be more than ~1 cm from the drone. For longer travel
 * distances, use primitive_go_to_waypoint_t. The recommendation is to have 30 setpoint
 * per second (to match VOXL's VIO rate), where the distance between setpoint
 * depends on the target speed.
 * 
 * Note that duration will automatically be bounded between 0.0 and 100.0.
 * 
 * IMPORTANT NOTE: If you modify this struct, do not insert or remove any fields
 * above pos_mask. See the implementation of apm_execute_primitive to understand
 * why.
 */
typedef struct {
    primitive_t type;  // SETPOINT
    uint64_t id;  // do not manually assign; this is auto-assigned when the primitive is added to queue

    float duration;  // millisecs to follow this setpoint (before moving onto the next primitive in the queue)

    /* mavlink_set_position_target_local_ned_t */
    uint32_t time_boot_ms;      // timestamp in ms since system boot. this is not actually used and can be set to 0.
    // NOTE: WITH THE CURRENT IMPLEMENTATION, DO ALL VELOCITIES/RATES AND ACCELERATIONS MUST BE MASKED TO ACCURATELY REACH THE TARGET WAYPOINT
    float x;                    // m
    float y;                    // m
    float z;                    // m
    float vx;                   // m/s
    float vy;                   // m/s
    float vz;                   // m/s
    float afx;                  // m/s/s
    float afy;                  // m/s/s
    float afz;                  // m/s/s
    float yaw;                  // rad
    float yaw_rate;             // rad/s

    // To set a mask, use bitwise OR operations with these macros:
    // https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
    uint16_t type_mask;

    uint8_t reserved;           // this is here to match _mavlink_set_position_target_local_ned_t
    uint8_t target_component;   // component ID
    // currently only supporting LOCAL_NED frame
    uint8_t coordinate_frame;   // MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9

    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) primitive_setpoint_t;


/**
 * Fly to a waypoint defined in local NED frame. The waypoint can be any
 * distance away. The drone will generate it's own path to the waypoint,
 * avoiding obstacles along the way.
 * 
 * If the drone is unable to reach the waypoint, it will throw an event_t of
 * type STUCK.
 * 
 * IMPORTANT NOTE: If you modify this struct, do not insert or remove any fields
 * above pos_mask. See the implementation of apm_execute_primitive to understand
 * why.
 */
typedef struct {
    primitive_t type;  // go_to_waypoint
    uint64_t id;  // do not manually assign; this is auto-assigned when the primitive is added to queue
    
    float x_accuracy;        // m; allowed distance to waypoint target before moving onto next primitive in queue
    float y_accuracy;        // m; allowed distance to waypoint target before moving onto next primitive in queue
    float z_accuracy;        // m; allowed distance to waypoint target before moving onto next primitive in queue
    float max_final_speed;   // ms; allowed final speed before moving onto next primitive in queue
    float yaw_accuracy;      // rad; allowed error before moving onto next primitive in queue
    float yaw_rate_accuracy; // rad/s; allowed maximum final yaw rate before moving onto next primitive in queue

    /* mavlink_set_position_target_local_ned_t */
    uint32_t time_boot_ms;      // timestamp in ms since system boot. this is not actually used and can be set to 0.
    float x;                    // m
    float y;                    // m
    float z;                    // m
    float vx;                   // m/s
    float vy;                   // m/s
    float vz;                   // m/s
    float afx;                  // m/s/s
    float afy;                  // m/s/s
    float afz;                  // m/s/s
    float yaw;                  // rad
    float yaw_rate;             // rad/s

    // To set a mask, use bitwise OR operations with these macros:
    // https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
    uint16_t type_mask;

    uint8_t reserved;           // this is here to match _mavlink_set_position_target_local_ned_t
    uint8_t target_component;   // component ID
    // currently only supporting LOCAL_NED frame
    uint8_t coordinate_frame;   // MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9

    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) primitive_go_to_waypoint_t;

/**
 * Set and follow a local polynomial.
 * 
 * coefficients are INCREASING, e.g. x(t) = a + bt + ct^2 ...
 * time begins at 0 for each coefficient, and is valid up to duration_s
 * order of the polynomial is n_coef-1
 * 
 * Use the function XXXXXXXX to generate bezier curve trajectories.
 */
#define TRAJ_MAX_COEFFICIENTS 12  // each segment is defined by up to an 11th order polynomial

typedef struct {
    primitive_t type;  // LOCAL_POLYNOMIAL
    uint64_t id;  // do not manually assign; this is auto-assigned when the primitive is added to queue
    int n_coef;
	double duration_s;
	double cx[TRAJ_MAX_COEFFICIENTS];
	double cy[TRAJ_MAX_COEFFICIENTS];
	double cz[TRAJ_MAX_COEFFICIENTS];
	double cyaw[TRAJ_MAX_COEFFICIENTS];
    uint32_t reserved1;
    uint32_t reserved2;
} __attribute__((packed)) primitive_local_polynomial_t;


/**
 * @brief   Create an instance of pause primitive with safe, default
 *          fields.
 * 
 * @param[in]  pointer to primitive whose fields will be populated.
 * @param[in]  msecs  amount of time to pause for.
 */
void primitive_default_pause(primitive_pause_t* prim, int msecs);

/**
 * @brief   Create an instance of land primitive with safe, default
 *          fields.
 * 
 * @param[in]  pointer to primitive whose fields will be populated.
 */
void primitive_default_land(primitive_land_t* prim);

/**
 * @brief   Create an instance of take_off primitive with safe, default
 *          fields. The default takeoff height is 1.5m.
 * 
 * @param[in]  pointer to primitive whose fields will be populated.
 */
void primitive_default_take_off(primitive_take_off_t* prim);

/**
 * @brief   Create an instance of setpoint primitive with safe, default
 *          fields. The default setpoint coordinates are the coordinates of the
 *          drone at time of calling.
 * 
 * @param[in]  pointer to primitive whose fields will be populated.
 */
void primitive_default_setpoint(primitive_setpoint_t* prim);

/**
 * @brief   Create an instance of go_to_waypoint primitive with safe, default
 *          fields. The default waypoint coordinates are the coordinates of the
 *          drone at time of calling.
 * 
 * @param[in]  pointer to primitive whose fields will be populated.
 */
void primitive_default_go_to_waypoint(primitive_go_to_waypoint_t* prim);

/**
 * @brief   Create an instance of local_polynomial primitive with safe, default
 *          fields. The default value is an example of a parabolic trajectory
 *          starting at (x,y,z) = (0,0,1.5) and ending at (x,y,z) = (2,4,1.5),
 *          with constant 0 yaw.
 *          Note that the default assumes the drone is starting at (x,y,z) = 
 *          (0,0,1.5).
 * 
 * @param[in]  pointer to primitive whose fields will be populated.
 */
void primitive_default_local_polynomial(primitive_local_polynomial_t* prim);



/**
 * @brief    Helper function to check if the active app has been terminated
 *           (in which case, interrupt the currently running primitive)
 *
 * @param[in] type          Type of the primitive. Just for printing purposes.                          
 * 
 * @return    0 if active app is still going, -1 if active app was terminated.
 */
int check_app_status(primitive_t type);


/**
 * @brief    Helper function to update value of primitive_progress in mutex.
 *           Just here to save a few lines of code.
 * 
 * @param[in] f  The value to set primitive_progress to.
 * @param[in] add_instead_of_set Set to 1 if the inputted float should be added
 *                               to the current value of primitive_progress.
 *                               Set to 0 if primitive_progress should be set to
 *                               the inputted float.
 */
void update_primitive_progress(float f, uint8_t add_instead_of_set);


/**
 * @brief    Helper function to check the XYZ and yaw values of an instance of 
 *           mavlink_set_position_target_local_ned_t and set default values if
 *           any are NAN.
 * 
 * @param[in] pos  The setpoints whose fields we want to check/set default values for
 */
void catch_NAN_fields(mavlink_set_position_target_local_ned_t* pos);


/**
 * @brief    Function executing all the primitives.
 *           Note: apm = "app manager"
 *
 * @param[in] params        The struct that the user defined containing 
 *                          parameters for executing the primitive.
 * @param[in] type          Type of the primitive. A switch is used with this
 *                          variable so a different action can be taken for each
 *                          primitive.                                       
 * 
 * @return    0 on success, on failure, returns a value from the enum event_t.
 */
int apm_execute_primitive(void* params, primitive_t type);



#ifdef __cplusplus
}
#endif

#endif // PRIMITIVES_H
