#ifndef DRONE_COMPASS_H
#define DRONE_COMPASS_H

#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>

class Compass {
private:
    int fd; // I2C file descriptor
    std::mutex compass_mutex;
    std::atomic<bool> running; // Flag to control the thread
    std::thread compass_thread;
    static constexpr float HEADING_OFFSET = 0.0; // physical offset when mounting compass on the drone in degrees

    // Latest compass data
    float heading;
    int16_t x, y, z;

    // IST8310 I2C address and register addresses
    static constexpr uint8_t IST8310_WHO_AM_I = 0x00;
    static constexpr uint8_t IST8310_ADDR = 0x0E;
    static constexpr uint8_t IST8310_CTRL1 = 0x0A;
    static constexpr uint8_t IST8310_X_LSB = 0x03;
    static constexpr uint8_t IST8310_Y_LSB = 0x05;
    static constexpr uint8_t IST8310_Z_LSB = 0x07;
    
    static constexpr uint8_t IST8310_DEVICE_ID = 0x10;

    // Helper functions
    int16_t read_2_bytes(uint8_t reg);
    void update_data(); // Thread function to update compass data

public:
    Compass();
    ~Compass();

    // Initialize the compass (returns true if successful)
    bool init();

    // Getters for compass data
    float get_heading();
};

#endif // COMPASS_H
