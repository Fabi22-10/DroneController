#ifndef DRONE_CONTROL_LOOP_H
#define DRONE_CONTROL_LOOP_H

#include "GPSModule.h"
#include "Compass.h"
#include <array>
#include <mutex>
#include <chrono>
#include <atomic>
#include "SBUS.h"

class ControlLoop {

public:
    enum class PositionControlState { REACHED, ACTIVE, ABORTED };
    
    GPS gps;
    Compass compass;
    ControlLoop(float k_lat, float k_lon, float k_alt, float k_yaw);

    // Set target parameters
    void set_target(float latitude, float longitude, float altitude, float heading, float speed, float altitude_speed, float yaw_speed);


    // Compute steering signals based on current state and target
    void update_signals();

    // Get the current steering signals
    sbus_packet_t get_steering_signals();

    // Abort the control loop
    void abort();

    // Get current position control state
    PositionControlState get_position_control_state() const;
    std::string get_json_state();

    // Initialize GPS and Compass
    bool init();

private:
    // Target parameters
    float target_latitude;
    float target_longitude;
    float target_altitude;
    float target_heading; // In degrees
    float desired_speed;  // In km/h
    float desired_altitude_speed; // Altitude climbing speed (km/h)
    float desired_yaw_speed;      // Yaw rotation speed (degrees/s)
    float temp_target_latitude;  // Temporary target latitude
    float temp_target_longitude; // Temporary target longitude
    float temp_target_altitude;
    float temp_target_heading;

    // Control parameters
    float k_lat; // Proportional gain for lateral (left-right)
    float k_lon; // Proportional gain for longitudinal (front-back)
    float k_alt; // Proportional gain for altitude (up-down)
    float k_yaw; // Proportional gain for yaw (rotation)

    std::array<uint16_t, 4> steering_signals; // Output signals for the drone
    std::mutex loop_mutex;               // Protect shared data
    std::chrono::steady_clock::time_point target_start_time;
    std::atomic<PositionControlState> position_state;
    float start_latitude;   // Latitude at the time the target was set
    float start_longitude;  // Longitude at the time the target was set
    float start_altitude;   // Altitude at the time the target was set
    float start_heading;    // Heading at the time the target was set

    // Thresholds for determining if the target is reached
    static constexpr float DISTANCE_THRESHOLD = 2.0;  // Meters
    static constexpr float ALTITUDE_THRESHOLD = 5.0; // Meters
    static constexpr float HEADING_THRESHOLD = 5.0;  // Degrees


    bool validate_target_parameters(float latitude, float longitude, float altitude, float heading, float speed, float altitude_speed, float yaw_speed);

    void generate_temporary_target();

    // Utility function to constrain a value
    int constrain(int value, int min_value, int max_value);

    // Utility function to calculate distance between two GPS points (Haversine formula)
    float calculate_distance(float lat1, float lon1, float lat2, float lon2);

    // Utility function to calculate bearing between two GPS points
    float calculate_bearing(float lat1, float lon1, float lat2, float lon2);

    bool is_target_reached(float current_latitude, float current_longitude, float current_altitude, float current_heading);
};

#endif // CONTROL_LOOP_H
