#include "Compass.h"
#include <wiringPiI2C.h>
#include <cmath>
#include <iostream>
#include <unistd.h>

Compass::Compass() : fd(-1), heading(0.0), x(0), y(0), z(0), running(false) {}

Compass::~Compass() {
    running = false;
    if (compass_thread.joinable()) {
        compass_thread.join();
    }
    if (fd != -1) {
        close(fd);
    }
}

bool Compass::init() {
    std::lock_guard<std::mutex> lock(compass_mutex);
    fd = wiringPiI2CSetup(IST8310_ADDR);
    if (fd == -1) {
        std::cerr << "Failed to initialize I2C for compass." << std::endl;
        return false;
    }

    if (wiringPiI2CReadReg8(fd, IST8310_WHO_AM_I) != IST8310_DEVICE_ID) {
        std::cerr << "Compass not found." << std::endl;
        return false;
    }

    // Start the update thread
    running = true;
    compass_thread = std::thread(&Compass::update_data, this);
    std::cout << "Compass initialized" << std::endl;
    return true;
}

int16_t Compass::read_2_bytes(uint8_t reg) {
    uint8_t low = wiringPiI2CReadReg8(fd, reg);
    uint8_t high = wiringPiI2CReadReg8(fd, reg + 1);
    return (high << 8) | low;
}

void Compass::update_data() {
    while (running) {
        {
            std::lock_guard<std::mutex> lock(compass_mutex);

            // Enable single measurement mode
            wiringPiI2CWriteReg8(fd, IST8310_CTRL1, 0x01);
            usleep(10000); // Wait for measurement to complete (10 ms)

            // Read x, y, z values
            x = read_2_bytes(IST8310_X_LSB);
            y = read_2_bytes(IST8310_Y_LSB);
            // z = read_2_bytes(IST8310_Z_LSB);

            // Compute heading
            heading = atan2((double)y, (double)x) * 180.0 / M_PI - 90.0 + HEADING_OFFSET;
            if (heading < 0) {
                heading += 360.0; // Normalize to [0, 360]
            }
        }
        usleep(100000); // Sleep for 100 ms (adjust as needed)
    }
}

float Compass::get_heading() {
    std::lock_guard<std::mutex> lock(compass_mutex);
    return heading;
}
