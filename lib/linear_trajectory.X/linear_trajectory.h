/*
 * File:   linear_trajectory.h
 * Author: Pavlo Vlastos
 * Brief:  Creates linear trajectories for a vehicle to follow
 * Note:   This module is meant to be used in place of the some-what deprecated 
 *         gnc.X library. Distances and positions are in meters, and angles are
 *         in radians
 * Created on March 11, 2021, 1:48 PM
 */

#ifndef LINEAR_TRAJECTORY_H
#define	LINEAR_TRAJECTORY_H


/*******************************************************************************
 * #INCLUDES
 ******************************************************************************/
#include "board.h"

/*******************************************************************************
 * #DEFINES
 ******************************************************************************/
#define DIM 2 /* This module only supports 2 dimensions at the moment ... */

/*******************************************************************************
 * PUBLIC DATATYPES 
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC VARIABLES 
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @function linear_trajectory(float vehicle_location[DIM], float prev_wp[DIM],
        float next_wp[DIM])
 * @brief Initialize the vehicle location and the set the previous and next 
 * waypoints
 * @param prev_wp The previous waypoint as an East and North 1x2 vector
 * @param next_wp The next waypoint as an East and North 1x2 vector
 * @param vehicle_location The vehicle location as an East and North 1x2 vector
 * @return SUCCESS or ERROR
 */
int lin_tra_init(float prev_wp[DIM], float next_wp[DIM], float vehi_pt[DIM]);

/**
 * @function lin_tra_update(float new_position[DIM])
 * @brief update the linear trajectory, including closest point to the vector 
 * connecting the previous and next waypoints, as well as the cross-track error
 * @param new_position The new vehicle position as an East and North 1x2 vector
 * @return SUCCESS or ERROR
 */
int lin_tra_update(float new_position[DIM]);


/**
 * @function lin_tra_project(void)
 * @brief Do a vector projection of the vehicles position vector onto the
 * current linear trajectory segment.
 * @param point The point off the vector to project onto the linear trajectory 
 * segment.
 * @return SUCCESS or ERROR
 */
float lin_tra_project(float point[DIM]);


/**
 * @function lin_tra_calc_cte(void)
 * @brief Calculate the cross-track error; the distance between the vehicle and
 * the closest point on the linear trajectory segment. The sign indicates left 
 * or right of the segment in the Serret-Frenet reference frame. Left is 
 * negative, right is positive
 * @return distance in meters, with 
 */
float lin_tra_calc_cte(void);

/**
 * @function lin_tra_calc_path_angle(void)
 * @brief Calculate the path angle between -pi and pi
 * @return the path angle in radians
 */
float lin_tra_calc_path_angle(void);

/**
 * @function lin_tra_rotate2d(float v[DIM], float v_new[DIM])
 * @brief rotate the vector v by the internal path angle
 * @param v The vector to rotate
 * @param angle The angle in radians between
 * @param v_new The vector after rotation
 * @return the path angle in radians
 */
int lin_tra_rotate2d(float v[DIM], float angle, float v_new[DIM]);

/**
 * @function lin_tra_calc_dist(float a[DIM], float b[DIM])
 * @brief Calculate the distance between 2 points, a and b.
 * @param a A vector that is DIM x 1 or DIM x 1
 * @param b A vector that is DIM x 1 or DIM x 1
 * @return the norm of the vectors
 */
float lin_tra_calc_dist(float a[DIM], float b[DIM]);

/**
 * @function lin_tra_get_cte(void)
 * @return The cross-track error in meters. Negative is left in the 
 * Serret-Frenet and positive is right.
 */
float lin_tra_get_cte(void);

/**
 * @function lin_tra_get_path_angle(void)
 * @return path_angle in radians between -pi and pi
 */
float lin_tra_get_path_angle(void);

/**
 * @function lin_tra_set_next_wp(float wp[DIM])
 * @brief Sets the next waypoint. Does NOT update the previous waypoint with the
 *  old next waypoint. That must be done separately with a call to 
 * lin_tra_set_prev_wp() DOES NOT DO BOUNDS CHECK
 * @param wp A new waypoint
 * @return SUCCESS or ERROR
 */
int lin_tra_set_next_wp(float wp[DIM]);

/**
 * @function lin_tra_set_next_wp(float wp[DIM])
 * @brief Sets the previous waypoint. DOES NOT DO BOUNDS CHECK
 * @param wp A new waypoint
 * @return SUCCESS or ERROR
 */
int lin_tra_set_prev_wp(float wp[DIM]);

/**
 * @function lin_tra_lla_to_ned(float lla_point[DIM + 1],
        float ref_point[DIM + 1],
        float east_north_point[DIM + 1])
 * FROM gnc.X, TESTED IN gnc.c
 * Convert latitude/longitude coordinates to East North Up (ENU) coordinates
 * in a local tangent plane. This MUST be done so that the controller is
 * operating with the correct units of distance. 
 * Based on work from:
 *     Demoz Gebre 8/18/98
 *     Modified for spherical earth - COLB 8/8/01
 * @param lla_point A point to convert from latitude/longitude coordinates to
 *        North East Down (NED) with respect to a local tangent plane based on a 
 *        reference points (can be the center of the local tangent plane).
 * @param ref_point The reference point acting as the center of the local 
 *        tangent plane
 * @param east_north_point The output point as East, North, Down, needs swap to NED
 */
void lin_tra_lla_to_enu(float lla_point[DIM + 1],
        float ref_point[DIM + 1],
        float enu_point[DIM + 1]);

/**
 * @function lin_tra_lla_to_ecef(float lla_point[DIM + 1],
        float ecef_point[DIM + 1])
 * FROM gnc.X, TESTED IN gnc.c
 * @param lla_point
 * @param ecef_point
 */
void lin_tra_lla_to_ecef(float lla_point[DIM + 1],
        float ecef_point[DIM + 1]);

#endif	/* LINEAR_TRAJECTORY_H */