#ifndef DRONE_CONNECTOR_H
#define DRONE_CONNECTOR_H


#include "ControlLoop.h"
#include <thread>
#include <atomic>
#include <string>
#include <memory>

class Connector {
public:
    Connector(ControlLoop& controlLoop, int port);
    ~Connector();

    // Start the server
    bool start();

    // Stop the server
    void stop();

private:
    ControlLoop& controlLoop; // Reference to the control loop
    int port;                 // Port for the server
    std::thread serverThread; // Thread to handle server operations
    std::atomic<bool> running; // Flag to control server loop

    // Server logic
    void serverLoop();

    // Handle incoming commands
    std::string handleCommand(const std::string& command);

    // Helper methods to generate responses
    std::string getTelemetry();
};

#endif