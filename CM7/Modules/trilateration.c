/*
 * Drone Signal Triangulation for STM32H755
 * Uses CMSIS-DSP for optimized computation
 *
 * Make sure to:
 * 1. Enable FPU in your project settings
 * 2. Include CMSIS-DSP library
 * 3. Define ARM_MATH_CM7 in preprocessor symbols
 */

#include "arm_math.h"
#include <math.h>
#include <stdint.h>

// ========== CONSTANTS ==========
#define EARTH_RADIUS_M 6371000.0f  // Earth radius in meters
#define PI 3.14159265358979323846f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// ========== CONFIGURATION ==========
// YOU MUST CALIBRATE THESE VALUES FOR YOUR DRONE
#define RSSI_AT_1M -73.0f          // RSSI at 1 meter (dBm)
#define PATH_LOSS_EXPONENT 2.5f    // Environment-dependent (2-4)
#define REFERENCE_DISTANCE_M 1.0f  // Reference distance

// ========== DATA STRUCTURES ==========

typedef struct {
    float32_t lat;  // Latitude in degrees
    float32_t lon;  // Longitude in degrees
} GeoCoord_t;

typedef struct {
    float32_t x;  // X coordinate in meters
    float32_t y;  // Y coordinate in meters
} CartesianCoord_t;

typedef struct {
    GeoCoord_t position;      // Board geographic position
    float32_t rssi_dbm;       // Received signal strength in dBm
    float32_t distance_m;     // Calculated distance in meters
} ReceiverData_t;

typedef struct {
    GeoCoord_t position;      // Estimated drone position
    float32_t error;          // Estimation error metric
    uint8_t valid;            // 1 if solution is valid, 0 otherwise
} DronePosition_t;

// ========== SIGNAL STRENGTH TO DISTANCE CONVERSION ==========

static inline float32_t power_of_10(float32_t exponent)
{
    return powf(10.0f, exponent);

}

/**
 * @brief Convert RSSI to distance using log-distance path loss model
 * @param rssi_dbm: Received signal strength in dBm
 * @return Distance in meters
 */
float32_t rssi_to_distance(float32_t rssi_dbm)
{
    float32_t exponent = (RSSI_AT_1M - rssi_dbm) / (10.0f * PATH_LOSS_EXPONENT);
    float32_t distance = REFERENCE_DISTANCE_M * power_of_10(exponent);
    return distance;
}

// ========== COORDINATE CONVERSION FUNCTIONS ==========

/**
 * @brief Convert latitude/longitude to local Cartesian coordinates (meters)
 * Uses equirectangular approximation - accurate for distances < 100km
 *
 * @param coord: Point to convert
 * @param ref: Reference point (origin)
 * @param result: Output Cartesian coordinates
 */
void latlon_to_meters(const GeoCoord_t *coord, const GeoCoord_t *ref, CartesianCoord_t *result)
{
    float32_t lat_rad = coord->lat * DEG_TO_RAD;
    float32_t ref_lat_rad = ref->lat * DEG_TO_RAD;

    float32_t avg_lat_rad = (lat_rad + ref_lat_rad) * 0.5f;
    float32_t cos_avg_lat;

    // Use CMSIS-DSP optimized cosine
    cos_avg_lat = arm_cos_f32(avg_lat_rad);

    result->x = EARTH_RADIUS_M * (coord->lon - ref->lon) * DEG_TO_RAD * cos_avg_lat;
    result->y = EARTH_RADIUS_M * (coord->lat - ref->lat) * DEG_TO_RAD;
}

/**
 * @brief Convert local Cartesian coordinates back to latitude/longitude
 *
 * @param coord: Cartesian coordinates in meters
 * @param ref: Reference point (origin)
 * @param result: Output geographic coordinates
 */
void meters_to_latlon(const CartesianCoord_t *coord, const GeoCoord_t *ref, GeoCoord_t *result)
{
    float32_t ref_lat_rad = ref->lat * DEG_TO_RAD;
    float32_t cos_ref_lat;

    cos_ref_lat = arm_cos_f32(ref_lat_rad);

    result->lat = ref->lat + (coord->y / EARTH_RADIUS_M) * RAD_TO_DEG;
    result->lon = ref->lon + (coord->x / (EARTH_RADIUS_M * cos_ref_lat)) * RAD_TO_DEG;
}

// ========== TRILATERATION ALGORITHM ==========

/**
 * @brief Calculate Euclidean distance between two points
 */
float32_t euclidean_distance(const CartesianCoord_t *p1, const CartesianCoord_t *p2)
{
    float32_t dx = p1->x - p2->x;
    float32_t dy = p1->y - p2->y;
    float32_t dist_sq = dx * dx + dy * dy;

    float32_t dist;
    arm_sqrt_f32(dist_sq, &dist);

    return dist;
}

/**
 * @brief Cost function for optimization (sum of squared distance errors)
 */
float32_t calculate_cost(const CartesianCoord_t *position,
                        const CartesianCoord_t *receivers,
                        const float32_t *distances,
                        uint8_t num_receivers)
{
    float32_t total_error = 0.0f;

    for (uint8_t i = 0; i < num_receivers; i++)
    {
        float32_t predicted_dist = euclidean_distance(position, &receivers[i]);
        float32_t error = predicted_dist - distances[i];
        total_error += error * error;
    }

    return total_error;
}

/**
 * @brief Simple gradient descent optimization for trilateration
 * More sophisticated than Nelder-Mead but simpler to implement on embedded
 */
void optimize_position(CartesianCoord_t *position,
                      const CartesianCoord_t *receivers,
                      const float32_t *distances,
                      uint8_t num_receivers,
                      uint16_t max_iterations,
                      float32_t learning_rate)
{
    const float32_t epsilon = 0.01f;  // Small value for numerical gradient

    for (uint16_t iter = 0; iter < max_iterations; iter++)
    {
        // Calculate gradient numerically
        CartesianCoord_t pos_dx_plus = *position;
        CartesianCoord_t pos_dy_plus = *position;

        pos_dx_plus.x += epsilon;
        pos_dy_plus.y += epsilon;

        float32_t cost_current = calculate_cost(position, receivers, distances, num_receivers);
        float32_t cost_dx = calculate_cost(&pos_dx_plus, receivers, distances, num_receivers);
        float32_t cost_dy = calculate_cost(&pos_dy_plus, receivers, distances, num_receivers);

        float32_t grad_x = (cost_dx - cost_current) / epsilon;
        float32_t grad_y = (cost_dy - cost_current) / epsilon;

        // Update position
        position->x -= learning_rate * grad_x;
        position->y -= learning_rate * grad_y;

        // Early stopping if gradient is small
        float32_t grad_mag_sq = grad_x * grad_x + grad_y * grad_y;
        if (grad_mag_sq < 0.001f)
            break;
    }
}

/**
 * @brief Main trilateration function
 *
 * @param receivers: Array of receiver data (minimum 3)
 * @param num_receivers: Number of receivers (typically 3)
 * @param result: Output drone position
 * @return 0 on success, -1 on error
 */
int8_t trilaterate_drone(const ReceiverData_t *receivers,
                         uint8_t num_receivers,
                         DronePosition_t *result)
{
    if (num_receivers < 3)
    {
        result->valid = 0;
        return -1;  // Need at least 3 receivers
    }

    // Use first receiver as reference point
    GeoCoord_t ref = receivers[0].position;

    // Convert all receivers to Cartesian coordinates
    CartesianCoord_t receivers_xy[8];  // Support up to 8 receivers
    float32_t distances[8];

    for (uint8_t i = 0; i < num_receivers; i++)
    {
        latlon_to_meters(&receivers[i].position, &ref, &receivers_xy[i]);
        distances[i] = receivers[i].distance_m;
    }

    // Initial guess: centroid of receivers
    CartesianCoord_t estimated_pos = {0.0f, 0.0f};

    for (uint8_t i = 0; i < num_receivers; i++)
    {
        estimated_pos.x += receivers_xy[i].x;
        estimated_pos.y += receivers_xy[i].y;
    }

    estimated_pos.x /= (float32_t)num_receivers;
    estimated_pos.y /= (float32_t)num_receivers;

    // Optimize position using gradient descent
    optimize_position(&estimated_pos, receivers_xy, distances, num_receivers, 500, 0.1f);

    // Convert back to lat/lon
    meters_to_latlon(&estimated_pos, &ref, &result->position);

    // Calculate final error
    result->error = calculate_cost(&estimated_pos, receivers_xy, distances, num_receivers);
    result->valid = 1;

    return 0;
}

// ========== MAIN PROCESSING FUNCTION ==========

/**
 * @brief Complete processing pipeline: RSSI to drone position
 *
 * @param receivers: Array of receiver data with RSSI values
 * @param num_receivers: Number of receivers
 * @param result: Output drone position
 * @return 0 on success, -1 on error
 */
int8_t process_drone_location(ReceiverData_t *receivers,
                              uint8_t num_receivers,
                              DronePosition_t *result)
{
    // Step 1: Convert RSSI to distances
    for (uint8_t i = 0; i < num_receivers; i++)
    {
        receivers[i].distance_m = rssi_to_distance(receivers[i].rssi_dbm);
    }

    // Step 2: Trilaterate position
    return trilaterate_drone(receivers, num_receivers, result);
}

// ========== EXAMPLE USAGE ==========

/**
 * @brief Example main function showing how to use the library
 * Replace with your actual RSSI reading code
 */
void example_usage(void)
{
    // Define your 3 NUCLEO board positions (replace with actual coordinates)
    ReceiverData_t receivers[3] = {
        {{19.4326f, -99.1332f}, -65.0f, 0.0f},  // Board 1: lat, lon, RSSI, distance
        {{19.4350f, -99.1300f}, -70.0f, 0.0f},  // Board 2
        {{19.4300f, -99.1350f}, -68.0f, 0.0f}   // Board 3
    };

    DronePosition_t drone_pos;

    // Process drone location
    int8_t status = process_drone_location(receivers, 3, &drone_pos);

    if (status == 0 && drone_pos.valid)
    {
        // Successfully calculated position
        // drone_pos.position.lat and drone_pos.position.lon contain the result
        // You can transmit this via UART, USB, or display on screen

        // Example: Print via UART (pseudo-code)
        // printf("Drone Position: %.6f, %.6f\r\n",
        //        drone_pos.position.lat,
        //        drone_pos.position.lon);
        // printf("Error metric: %.2f\r\n", drone_pos.error);
    }
    else
    {
        // Error in calculation
    }
}

// ========== FILTERING FUNCTIONS (OPTIONAL) ==========

/**
 * @brief Simple moving average filter for RSSI smoothing
 * Recommended to reduce noise in RSSI measurements
 */
typedef struct {
    float32_t buffer[10];  // Buffer size = 10 samples
    uint8_t index;
    uint8_t filled;
} MovingAvgFilter_t;

void moving_avg_init(MovingAvgFilter_t *filter)
{
    filter->index = 0;
    filter->filled = 0;
    arm_fill_f32(0.0f, filter->buffer, 10);
}

float32_t moving_avg_update(MovingAvgFilter_t *filter, float32_t new_value)
{
    filter->buffer[filter->index] = new_value;
    filter->index = (filter->index + 1) % 10;

    if (filter->filled < 10)
        filter->filled++;

    float32_t sum;
    arm_mean_f32(filter->buffer, filter->filled, &sum);

    return sum;
}
