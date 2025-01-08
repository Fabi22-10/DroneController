#ifndef DRONE_GPS_MODULE_H
#define DRONE_GPS_MODULE_H

#include <string>
#include <queue>
#include <mutex>
#include <thread>

class GPS {
private:
    int gps_fd; // File descriptor for the serial port
    std::thread gps_thread;
    std::mutex gps_mutex;
    std::queue<std::string> gps_queue;

    // Latest values
    std::string time;
    float latitude;
    float longitude;
    float altitude_agl;
    float speed;
    float course;
    int fix_quality;
    int satellites;

    bool running;

    // Configure the serial port
    int configure_serial_port();

    // Validate NMEA checksum
    bool validate_checksum(const std::string &sentence);

    // Reader thread function
    void gps_reader();

    // Process GPS data
    void process_gps_data(const std::string &sentence);

    // Convert NMEA latitude/longitude to decimal degrees
    float convert_to_decimal_degrees(const char *coord, char direction);

public:
    GPS();
    ~GPS();

    // Initialize the GPS
    bool init();

    // Fetch the latest GPS values
    void update();
    bool is_data_reliable() const;

    // Getters for GPS values
    float get_latitude() const;
    float get_longitude() const;
    float get_altitude_agl() const;
    float get_speed() const;
    float get_course() const;
    int get_fix_quality() const;
    int get_satellites() const;
    std::string get_time() const;
};

#endif