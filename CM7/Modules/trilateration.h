/**
 * @file drone_triangulation.h
 * @brief Drone signal triangulation library for STM32H755
 * @author Your Name
 * @date 2025
 */

#ifndef DRONE_TRIANGULATION_H
#define DRONE_TRIANGULATION_H

#include "arm_math.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ========== CONFIGURATION ==========
// Calibrate these values for your specific drone and environment
#define RSSI_AT_1M -40.0f          // RSSI measured at 1 meter from drone (dBm)
#define PATH_LOSS_EXPONENT 2.5f    // 2.0 = free space, 2.5-3.0 = outdoor, 3.0-4.0 = indoor
#define REFERENCE_DISTANCE_M 1.0f  // Reference distance for RSSI measurement

// ========== DATA TYPES ==========

/**
 * @brief Geographic coordinate structure
 */
typedef struct {
    float32_t lat;  /**< Latitude in decimal degrees */
    float32_t lon;  /**< Longitude in decimal degrees */
} GeoCoord_t;

/**
 * @brief Cartesian coordinate structure (local frame)
 */
typedef struct {
    float32_t x;  /**< X coordinate in meters */
    float32_t y;  /**< Y coordinate in meters */
} CartesianCoord_t;

/**
 * @brief Receiver (NUCLEO board) data structure
 */
typedef struct {
    GeoCoord_t position;      /**< Geographic position of receiver */
    float32_t rssi_dbm;       /**< Received signal strength in dBm */
    float32_t distance_m;     /**< Calculated distance in meters */
} ReceiverData_t;

/**
 * @brief Drone position estimation result
 */
typedef struct {
    GeoCoord_t position;      /**< Estimated drone position */
    float32_t error;          /**< Estimation error metric (lower is better) */
    uint8_t valid;            /**< 1 if solution is valid, 0 otherwise */
} DronePosition_t;

/**
 * @brief Moving average filter for RSSI smoothing
 */
typedef struct {
    float32_t buffer[10];  /**< Sample buffer */
    uint8_t index;         /**< Current buffer index */
    uint8_t filled;        /**< Number of samples in buffer */
} MovingAvgFilter_t;

// ========== FUNCTION PROTOTYPES ==========

static inline float32_t power_of_10(float32_t exponent);

/**
 * @brief Convert RSSI to distance using log-distance path loss model
 * @param rssi_dbm Received signal strength in dBm
 * @return Distance in meters
 */
float32_t rssi_to_distance(float32_t rssi_dbm);

/**
 * @brief Convert geographic coordinates to local Cartesian meters
 * @param coord Point to convert
 * @param ref Reference point (origin)
 * @param result Output Cartesian coordinates
 */
void latlon_to_meters(const GeoCoord_t *coord, const GeoCoord_t *ref, CartesianCoord_t *result);

/**
 * @brief Convert local Cartesian meters to geographic coordinates
 * @param coord Cartesian coordinates
 * @param ref Reference point (origin)
 * @param result Output geographic coordinates
 */
void meters_to_latlon(const CartesianCoord_t *coord, const GeoCoord_t *ref, GeoCoord_t *result);

/**
 * @brief Calculate Euclidean distance between two Cartesian points
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in meters
 */
float32_t euclidean_distance(const CartesianCoord_t *p1, const CartesianCoord_t *p2);

/**
 * @brief Perform trilateration to estimate drone position
 * @param receivers Array of receiver data (minimum 3)
 * @param num_receivers Number of receivers
 * @param result Output drone position
 * @return 0 on success, -1 on error
 */
int8_t trilaterate_drone(const ReceiverData_t *receivers,
                         uint8_t num_receivers,
                         DronePosition_t *result);

/**
 * @brief Complete processing: RSSI to drone position
 * @param receivers Array of receiver data with RSSI values
 * @param num_receivers Number of receivers
 * @param result Output drone position
 * @return 0 on success, -1 on error
 */
int8_t process_drone_location(ReceiverData_t *receivers,
                              uint8_t num_receivers,
                              DronePosition_t *result);

/**
 * @brief Initialize moving average filter
 * @param filter Pointer to filter structure
 */
void moving_avg_init(MovingAvgFilter_t *filter);

/**
 * @brief Update moving average filter with new sample
 * @param filter Pointer to filter structure
 * @param new_value New RSSI sample
 * @return Filtered RSSI value
 */
float32_t moving_avg_update(MovingAvgFilter_t *filter, float32_t new_value);

#ifdef __cplusplus
}
#endif

#endif /* DRONE_TRIANGULATION_H */
