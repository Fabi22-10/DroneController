#include "ControlLoop.h"
#include <cmath>
#include <math.h>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ControlLoop::ControlLoop(float k_lat, float k_lon, float k_alt, float k_yaw)
    : k_lat(k_lat), k_lon(k_lon), k_alt(k_alt), k_yaw(k_yaw),
      target_latitude(0.0), target_longitude(0.0), target_altitude(0.0), target_heading(0.0),
      desired_speed(0.0), steering_signals({1024, 1024, 1024, 1024}), position_state(PositionControlState::REACHED) {}


bool ControlLoop::init() {
    return gps.init() && compass.init();
}

bool ControlLoop::validate_target_parameters(float latitude, float longitude, float altitude, float heading, float speed, float altitude_speed, float yaw_speed) {
    
        // TODO: ...

    if (heading < 0.0 || heading > 360.0) {
        return false;
    }
    return true;
}


void ControlLoop::set_target(float latitude, float longitude, float altitude, float heading, float speed, float altitude_speed, float yaw_speed) {
    std::lock_guard<std::mutex> lock(loop_mutex);

    bool valid = validate_target_parameters(latitude, longitude, altitude, heading, speed, altitude_speed, yaw_speed);
    if (!valid) {
        position_state = PositionControlState::ABORTED;
        std::cerr << "Invalid target parameters!" << std::endl;
        std::cout << "Position Control State: ABORTED" << std::endl;
        return;
    }

    constexpr int max_retries = 50; // 5 seconds with 100ms intervals
    constexpr int retry_interval_ms = 100; // Retry every 100ms
    // Retry loop to gain a GPS signal
    bool reliable_data = false;
    for (int i = 0; i < max_retries; ++i) {
        gps.update();

        if (gps.is_data_reliable()) {
            reliable_data = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_interval_ms));
    }

    // Abort if GPS data is still not reliable after retries
    if (!reliable_data) {
        position_state = PositionControlState::ABORTED;
        std::cerr << "Failed to acquire reliable GPS data within 5 seconds!" << std::endl;
        std::cerr << "Fix quality: " << gps.get_fix_quality() << ", Satellites: " << gps.get_satellites() << std::endl;
        std::cout << "Position Control State: ABORTED" << std::endl;
        return;
    }

    // Proceed with setting the target
    start_latitude = gps.get_latitude();
    start_longitude = gps.get_longitude();
    start_altitude = gps.get_altitude_agl();
    start_heading = compass.get_heading();
    target_start_time = std::chrono::steady_clock::now();

    target_latitude = latitude;
    target_longitude = longitude;
    target_altitude = altitude;
    target_heading = heading;
    desired_speed = speed;
    desired_altitude_speed = altitude_speed;
    desired_yaw_speed = yaw_speed;

    // Set temporary targets to starting positions
    temp_target_latitude = start_latitude;
    temp_target_longitude = start_longitude;
    temp_target_altitude = start_altitude;
    temp_target_heading = start_heading;

    position_state = PositionControlState::ACTIVE;
    std::cout << "Position Control State: ACTIVE" << std::endl;
}


void ControlLoop::generate_temporary_target() {
    // Calculate elapsed time in milliseconds
    auto now = std::chrono::steady_clock::now();
    auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - target_start_time).count();

    // Convert elapsed time to seconds
    float elapsed_time_s = elapsed_time_ms / 1000.0;

    // Horizontal movement
    float distance_to_travel = (desired_speed * 1000.0 / 3600.0) * elapsed_time_s;
    float total_distance = calculate_distance(start_latitude, start_longitude, target_latitude, target_longitude);
    if (distance_to_travel >= total_distance) {
        // If the drone has (theoretically) reached or exceeded the target, set the temporary target as the final target
        temp_target_latitude = target_latitude;
        temp_target_longitude = target_longitude;
    } else {
        // Calculate the bearing from the starting position to the target
        float target_bearing = calculate_bearing(start_latitude, start_longitude, target_latitude, target_longitude);
        constexpr float R = 6371000.0; // Earth's radius in meters
        float delta_lat = (distance_to_travel / R) * (180.0 / M_PI) * std::cos(target_bearing * M_PI / 180.0);
        float delta_lon = (distance_to_travel / R) * (180.0 / M_PI) * std::sin(target_bearing * M_PI / 180.0) / std::cos(start_latitude * M_PI / 180.0);
        temp_target_latitude = start_latitude + delta_lat;
        temp_target_longitude = start_longitude + delta_lon;
    }

    // Altitude movement
    float altitude_to_climb = elapsed_time_s * (desired_altitude_speed * 1000.0 / 3600.0);
    if (target_altitude < start_altitude) {
        // sink to target altitude
        temp_target_altitude = std::max(start_altitude - altitude_to_climb, target_altitude);
    }
    else {
        // climb to target altitude
        temp_target_altitude = std::min(start_altitude + altitude_to_climb, target_altitude);
    }

    // Heading movement
    float heading_to_rotate = elapsed_time_s * desired_yaw_speed;
    float clockwiseAngle, counterClockwiseAngle;
    clockwiseAngle = fmod(target_heading - start_heading + 360.0, 360.0);
    counterClockwiseAngle = fmod(start_heading - target_heading + 360.0, 360.0);
    if (clockwiseAngle <= counterClockwiseAngle) {
        // Rotate clockwise
        temp_target_heading = start_heading + heading_to_rotate;
        // Adjust the target heading to the range [start_heading, start_heading + 360)
        float adjusted_target_heading = target_heading;
        if (target_heading < start_heading) {
            adjusted_target_heading += 360.0;
        }
        // Stop at the target if overshooting
        if (temp_target_heading > adjusted_target_heading) {
            temp_target_heading = adjusted_target_heading;
        }
        temp_target_heading = fmod(temp_target_heading, 360.0);
    } else {
        // Rotate counter-clockwise
        temp_target_heading = start_heading - heading_to_rotate;
        // Adjust the target heading to the range [start_heading - 360, start_heading)
        float adjusted_target_heading = target_heading;
        if (target_heading > start_heading) {
            adjusted_target_heading -= 360.0;
        }
        // Stop at the target if overshooting
        if (temp_target_heading < adjusted_target_heading) {
            temp_target_heading = adjusted_target_heading;
        }
        temp_target_heading = fmod(temp_target_heading + 360.0, 360.0);  // Normalize to [0, 360)
    }
    
    // std::cout << "lat: " << temp_target_latitude << ", lon: " << temp_target_longitude 
    //         << ", alt: " << temp_target_altitude << ", head:" << temp_target_heading << std::endl;
}



int ControlLoop::constrain(int value, int min_value, int max_value) {
    return std::max(min_value, std::min(value, max_value));
}

float ControlLoop::calculate_distance(float lat1, float lon1, float lat2, float lon2) {
    constexpr float R = 6371000.0; // Earth's radius in meters
    float dlat = (lat2 - lat1) * M_PI / 180.0;
    float dlon = (lon2 - lon1) * M_PI / 180.0;

    float a = std::sin(dlat / 2) * std::sin(dlat / 2) +
              std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
                  std::sin(dlon / 2) * std::sin(dlon / 2);
    float c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return R * c; // Distance in meters
}

float ControlLoop::calculate_bearing(float lat1, float lon1, float lat2, float lon2) {
    float dlon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    float y = std::sin(dlon) * std::cos(lat2);
    float x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dlon);

    float bearing = std::atan2(y, x) * 180.0 / M_PI;
    return fmod(bearing + 360.0, 360.0); // Normalize to [0, 360]
}

void ControlLoop::update_signals() {
    std::lock_guard<std::mutex> lock(loop_mutex);
    
    // Check the position control state
    if (position_state != PositionControlState::ACTIVE) {
        steering_signals = {1024, 1024, 1024, 1024}; // Default neutral signals
    //     if (position_state == PositionControlState::ABORTED) {
    //         std::cout << "Position Control State: ABORTED" << std::endl;
    //     }
    //     if (position_state == PositionControlState::REACHED) {
    //         std::cout << "Position Control State: REACHED" << std::endl;
    //     }
        return;
    }

    gps.update();
    if (!gps.is_data_reliable()) {
        std::cerr << "GPS data not reliable! Fix quality: " << gps.get_fix_quality() << ", Satellites: " << gps.get_satellites() << std::endl;
        return;
    }

    float current_latitude = gps.get_latitude();
    float current_longitude = gps.get_longitude();
    float current_altitude = gps.get_altitude_agl();
    float current_heading = compass.get_heading();

    // Check if the target is reached
    if (is_target_reached(current_latitude, current_longitude, current_altitude, current_heading)) {
        position_state = PositionControlState::REACHED;
        steering_signals = {1024, 1024, 1024, 1024}; // Default neutral signals
        std::cout << "Position Control State: REACHED" << std::endl;
        return;
    }

    // Generate the temporary target
    generate_temporary_target();

    // Calculate errors based on the temporary target
    float distance_error = calculate_distance(current_latitude, current_longitude, temp_target_latitude, temp_target_longitude);
    float altitude_error = temp_target_altitude - current_altitude;
    float target_bearing = calculate_bearing(current_latitude, current_longitude, temp_target_latitude, temp_target_longitude);
    float heading_error = temp_target_heading - current_heading;

    if (heading_error > 180.0) heading_error -= 360.0;
    if (heading_error < -180.0) heading_error += 360.0;

    // Calculate relative bearing (target bearing relative to current heading)
    float relative_bearing = target_bearing - current_heading;
    if (relative_bearing > 180.0) relative_bearing -= 360.0;
    if (relative_bearing < -180.0) relative_bearing += 360.0;

    // Normalize the relative bearing into components
    float forward_component = std::cos(relative_bearing * M_PI / 180.0);
    float lateral_component = std::sin(relative_bearing * M_PI / 180.0);

     // Generate steering signals
    steering_signals[0] = constrain(1024 + static_cast<int>(k_lat * distance_error * lateral_component), 364, 1684); // Left-right
    steering_signals[1] = constrain(1024 + static_cast<int>(k_lon * distance_error * forward_component), 364, 1684); // Front-back
    steering_signals[2] = constrain(1024 + static_cast<int>(k_alt * altitude_error), 364, 1684); // Up-down
    steering_signals[3] = constrain(1024 + static_cast<int>(k_yaw * heading_error), 364, 1684); // CW-CCW rotation

    
    // std::cout << "e_dist: " << distance_error << ", x: " << steering_signals[0] << ", y: " << steering_signals[1]
    //         << ", e_alt: " << altitude_error << ", z: " << steering_signals[2]
    //         << ", e_head: " << heading_error << ", phi: " << steering_signals[3] 
    //         << ", bearing: " << relative_bearing 
            // << ", t_loc: " << temp_target_latitude << ", " << temp_target_longitude 
            // << ", t_glob: " << target_latitude << ", " << target_longitude 
            // << ", cur_pos: " << current_latitude << ", " << current_longitude 
            // << std::endl;
}

bool ControlLoop::is_target_reached(float current_latitude, float current_longitude, float current_altitude, float current_heading) {
    float distance_error = calculate_distance(current_latitude, current_longitude, target_latitude, target_longitude);
    float altitude_error = target_altitude - current_altitude;
    float heading_error = target_heading - current_heading;

    if (std::abs(distance_error) <= DISTANCE_THRESHOLD &&
        std::abs(altitude_error) <= ALTITUDE_THRESHOLD &&
        std::abs(heading_error) <= HEADING_THRESHOLD) {
        return true;
    }
    return false;
}

void ControlLoop::abort() {
    std::lock_guard<std::mutex> lock(loop_mutex);
    if (position_state != PositionControlState::ABORTED) {
        position_state = PositionControlState::ABORTED;
        std::cout << "Position Control aborted" << std::endl;
    }
}

sbus_packet_t ControlLoop::get_steering_signals() {
    std::lock_guard<std::mutex> lock(loop_mutex);
    if (position_state == PositionControlState::ACTIVE) {
        sbus_packet_t packet = {
            .channels = {
                steering_signals[0],     // Roll (left - right)
                steering_signals[1],     // Pitch (back - front)
                steering_signals[2],     // Throttle (down - up)
                steering_signals[3],     // Yaw (counter-clockwise - clockwise)
                1684,           // Ch: 5 (not used)
                1541,           // Orientation Mode: OFF (1024 = Course Lock, 511 = Home Lock)
                1024,           // Flight Mode: Altitude Stabilized (511 = Manual, 1541 = Hold GPS Position)
                1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,           // Ch 8 - Ch 16 (not used)
            }, 
            .ch17 = false,    // Channel 17 status
            .ch18 = false,    // Channel 18 status
            .failsafe = false, // Failsafe status
            .frameLost = false // Frame lost status
        };
        return packet;
    }
    else {
        // Don't move if state is aborted or reached:
        sbus_packet_t packet = {
            .channels = {
                1024,     // Roll (left - right)
                1024,     // Pitch (back - front)
                1024,     // Throttle (down - up)
                1024,     // Yaw (counter-clockwise - clockwise)
                1684,           // Ch: 5 (not used)
                1541,           // Orientation Mode: OFF (1024 = Course Lock, 511 = Home Lock)
                1541,           // Flight Mode: Hold GPS Position (511 = Manual, 1024 = Hold altitude)
                1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024,           // Ch 8 - Ch 16 (not used)
            }, 
            .ch17 = false,    // Channel 17 status
            .ch18 = false,    // Channel 18 status
            .failsafe = false, // Failsafe status
            .frameLost = false // Frame lost status
        };
        return packet;
    }
}

ControlLoop::PositionControlState ControlLoop::get_position_control_state() const {
    return position_state.load(); // Ensure thread-safe access
}

std::string ControlLoop::get_json_state(){
    std::lock_guard<std::mutex> lock(loop_mutex); // Ensure thread safety
    json state = {
        {"type", "CONTROL_STATE"},
        {"control_loop_state", static_cast<int>(position_state.load())},
        {"target",
            {
                {"lat", target_latitude},
                {"long", target_longitude},
                {"altitude", target_altitude},
                {"heading", target_heading},
            }
        },
        {"temp_target",
            {
                {"lat", temp_target_latitude},
                {"lon", temp_target_longitude},
                {"altitude", temp_target_altitude},
                {"heading", temp_target_heading},
            }
        },
        {"desired_speed", desired_speed},
        {"desired_altitude_speed", desired_altitude_speed},
        {"desired_yaw_speed", desired_yaw_speed},
    };

    return state.dump(); // Serialize JSON to a string
}
