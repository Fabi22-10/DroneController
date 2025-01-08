#include "GPSModule.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <cstring>
#include <cmath>
#include <thread>

// Constants
#define SERIAL_PORT "/dev/serial0"
#define BAUD_RATE B115200
#define LINE_BUFFER_SIZE 256

GPS::GPS() : gps_fd(-1), latitude(0.0), longitude(0.0), altitude_agl(0.0), speed(0.0), course(0.0), running(true) {}

GPS::~GPS() {
    running = false;
    fix_quality = -1;
    satellites = -1;
    if (gps_thread.joinable()) gps_thread.join();
    if (gps_fd != -1) close(gps_fd);
}

int GPS::configure_serial_port() {
    struct termios options;
    if (tcgetattr(gps_fd, &options) != 0) {
        perror("Error getting serial port attributes");
        return -1;
    }
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~OPOST;
    if (tcsetattr(gps_fd, TCSANOW, &options) != 0) {
        perror("Error setting serial port attributes");
        return -1;
    }
    return 0;
}

bool GPS::validate_checksum(const std::string &sentence) {
    if (sentence[0] != '$') return false;
    auto checksum_pos = sentence.find('*');
    if (checksum_pos == std::string::npos || checksum_pos < 1) return false;

    unsigned char checksum = 0;
    for (size_t i = 1; i < checksum_pos; ++i) {
        checksum ^= sentence[i];
    }

    unsigned int received_checksum;
    sscanf(sentence.c_str() + checksum_pos + 1, "%2x", &received_checksum);
    return checksum == received_checksum;
}

void GPS::gps_reader() {
    char line_buffer[LINE_BUFFER_SIZE];
    int line_pos = 0;
    char c;

    while (running) {
        if (read(gps_fd, &c, 1) > 0) {
            if (c == '\n') {
                line_buffer[line_pos] = '\0'; // Null-terminate the line
                std::string sentence(line_buffer);
                if (validate_checksum(sentence)) {
                    std::lock_guard<std::mutex> lock(gps_mutex);
                    gps_queue.push(sentence);
                    if (gps_queue.size() > 10) gps_queue.pop(); // Limit queue size
                }
                line_pos = 0; // Reset for the next line
            } else if (line_pos < LINE_BUFFER_SIZE - 1) {
                line_buffer[line_pos++] = c;
            }
        }
    }
}

void GPS::process_gps_data(const std::string &sentence) {
    try {
        if (sentence.find("$GNRMC") != std::string::npos) {
            char time_buf[11], lat_buf[11], lon_buf[12], speed_buf[8], cog_buf[8], date_buf[7];
            char ns = 0, ew = 0, status = 0;

            // Parse $GNRMC
            int parsed = sscanf(sentence.c_str(),
                "$GNRMC,%10[^,],%c,%10[^,],%c,%11[^,],%c,%7[^,],%7[^,],%6[^,]",
                time_buf, &status, lat_buf, &ns, lon_buf, &ew, speed_buf, cog_buf, date_buf);

            if (status != 'A' || strlen(time_buf) == 0) { // Ensure valid time and status
                std::cerr << "Skipping invalid or incomplete $GNRMC sentence." << std::endl;
                return;
            }

            // Convert latitude and longitude
            float parsed_lat = convert_to_decimal_degrees(lat_buf, ns);
            float parsed_lon = convert_to_decimal_degrees(lon_buf, ew);

            // Validate latitude and longitude ranges
            if (parsed_lat < -90.0 || parsed_lat > 90.0 || parsed_lon < -180.0 || parsed_lon > 180.0) {
                std::cerr << "Skipping out-of-range latitude or longitude in $GNRMC." << std::endl;
                return;
            }

            // Update GPS values
            time = time_buf;
            latitude = parsed_lat;
            longitude = parsed_lon;
            speed = atof(speed_buf);    // Speed over ground (knots)
            course = atof(cog_buf);    // Course over ground (degrees)

        } else if (sentence.find("$GNGGA") != std::string::npos) {
            char time_buf[11], lat_buf[11], lon_buf[12], alt_buf[8], geoid_buf[8];
            char ns = 0, ew = 0;
            int fix_quality_local = 0, satellites_local = 0;
            float hdop_local = 0.0;

            // Parse $GNGGA
            int parsed = sscanf(sentence.c_str(),
                "$GNGGA,%10[^,],%10[^,],%c,%11[^,],%c,%d,%d,%f,%7[^,],M,%7[^,],M",
                time_buf, lat_buf, &ns, lon_buf, &ew, &fix_quality_local, &satellites_local, &hdop_local, alt_buf, geoid_buf);

            if (parsed < 10 || strlen(time_buf) == 0) { // Ensure valid time
                std::cerr << "Skipping invalid or incomplete $GNGGA sentence." << std::endl;
                return;
            }

            // Convert latitude and longitude
            float parsed_lat = convert_to_decimal_degrees(lat_buf, ns);
            float parsed_lon = convert_to_decimal_degrees(lon_buf, ew);

            // Validate latitude and longitude ranges
            if (parsed_lat < -90.0 || parsed_lat > 90.0 || parsed_lon < -180.0 || parsed_lon > 180.0) {
                std::cerr << "Skipping out-of-range latitude or longitude in $GNGGA." << std::endl;
                return;
            }

            // Convert altitude and validate
            float parsed_altitude = atof(alt_buf);
            float parsed_geoid = atof(geoid_buf);
            if (parsed_altitude < -1000.0 || parsed_altitude > 10000.0) { // Sanity check altitude
                std::cerr << "Skipping invalid altitude in $GNGGA." << std::endl;
                return;
            }

            // Update GPS values
            time = time_buf;
            latitude = parsed_lat;
            longitude = parsed_lon;
            altitude_agl = parsed_altitude - parsed_geoid; // Altitude above ground level
            fix_quality = fix_quality_local;
            satellites = satellites_local;
        }

    } catch (const std::exception &e) {
        std::cerr << "Error while processing GPS data: " << e.what() << std::endl;
    }
}

float GPS::convert_to_decimal_degrees(const char *coord, char direction) {
    if (coord == nullptr || strlen(coord) < 4 || !(direction == 'N' || direction == 'S' || direction == 'E' || direction == 'W')) {
        throw std::invalid_argument("Invalid coordinate or direction");
    }

    float degrees = 0.0, minutes = 0.0;

    // Determine the format based on input length (latitude vs longitude)
    if (direction == 'E' || direction == 'W') { // Longitude (DDDMM.MMMM)
        if (sscanf(coord, "%3f%f", &degrees, &minutes) != 2) {
            throw std::invalid_argument("Failed to parse longitude coordinate");
        }
    } else { // Latitude (DDMM.MMMM)
        if (sscanf(coord, "%2f%f", &degrees, &minutes) != 2) {
            throw std::invalid_argument("Failed to parse latitude coordinate");
        }
    }

    // Convert to decimal degrees
    float decimal = degrees + (minutes / 60.0);

    // Apply hemisphere correction
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

bool GPS::init() {
    gps_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (gps_fd == -1) {
        perror("Unable to open serial port for GPS");
        return false;
    }
    if (configure_serial_port() != 0) {
        close(gps_fd);
        gps_fd = -1;
        return false;
    }
    gps_thread = std::thread(&GPS::gps_reader, this);
    return true;
}

void GPS::update() {
    std::lock_guard<std::mutex> lock(gps_mutex);
    while (!gps_queue.empty()) {
        process_gps_data(gps_queue.front());
        gps_queue.pop();
    }
}

bool GPS::is_data_reliable() const {
    if (fix_quality > 0 && satellites >= 4) {
        return true; // Good GPS data
    }
    return false; // Insufficient fix quality or satellites
}

float GPS::get_latitude() const { return latitude; }
float GPS::get_longitude() const { return longitude; }
float GPS::get_altitude_agl() const { return altitude_agl; }
float GPS::get_speed() const { return speed; }
float GPS::get_course() const { return course; }
int GPS::get_fix_quality() const { return fix_quality; }
int GPS::get_satellites() const { return satellites; }
std::string GPS::get_time() const { return time; }
