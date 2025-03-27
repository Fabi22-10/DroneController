#include "Connector.h"
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

Connector::Connector(ControlLoop& controlLoop, int port)
    : controlLoop(controlLoop), port(port), running(false) {}

Connector::~Connector() {
    stop();
}

bool Connector::start() {
    if (running) return false;
    running = true;
    serverThread = std::thread(&Connector::serverLoop, this);
    return true;
}

void Connector::stop() {
    if (!running) return;
    running = false;
    if (serverThread.joinable()) serverThread.join();
}

void Connector::serverLoop() {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket failed");
        return;
    }

    // Bind socket
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        close(server_fd);
        return;
    }

    // Listen
    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        close(server_fd);
        return;
    }

    std::cout << "Server running. Waiting for connections... " << std::endl;

    while (running) {
        if ((new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
            perror("Accept failed");
            continue;
        }
        std::cout << "Client connected" << std::endl;
        struct timeval timeout;
        timeout.tv_sec = 10; // Timeout for client inactivity (10 seconds)
        timeout.tv_usec = 0;

        // Set socket options for read timeout
        setsockopt(new_socket, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

        while(true) {
            char buffer[1024] = {0};
            int bytes_read = read(new_socket, buffer, sizeof(buffer));
            if (bytes_read <= 0) {
                break;
            }
            std::string command(buffer, bytes_read);
            std::string response = handleCommand(command);
            // std::cout << "Responding: " << response << std::endl;
            send(new_socket, response.c_str(), response.size(), 0);            
        }
        std::cout << "Client disconnected" << std::endl;
        close(new_socket);
    }

    close(server_fd);
}

std::string Connector::handleCommand(const std::string& command) {
    try {
        // Parse the received JSON
        json receivedData = json::parse(command);
        std::cout << "Received: " << receivedData << std::endl;
        if (receivedData["command"] == "ABORT") {
            controlLoop.abort();
            json ackMessage = {
                {"status", "confirmed"}
            };
            return ackMessage.dump();
        }
        else if (receivedData["command"] == "TARGET") {
            float latitude = receivedData["location"]["lat"];
            float longitude = receivedData["location"]["lon"];
            float altitude = receivedData["location"]["alt"];
            float heading = receivedData["heading"];
            float linearSpeed = receivedData["speed"]["linear"];
            float yawSpeed = receivedData["speed"]["yaw"];
            float altitudeSpeed = receivedData["speed"]["altitude"];
            controlLoop.set_target(latitude, longitude, altitude, heading, linearSpeed, altitudeSpeed, yawSpeed);
            json ackMessage = {
                {"status", "confirmed"}
            };
            return ackMessage.dump();
        } else if (receivedData["command"] == "CONTROL_STATE") {
            return controlLoop.get_json_state();
        } else if (receivedData["command"] == "TELEMETRY") {
            return getTelemetry();
        }
    } catch (const json::exception &e) {
        std::cerr << "JSON Parsing Error: " << e.what() << std::endl;
        json response = {
            {"error", e.what()}
        };
        return response.dump();
    }

    json response = {
        {"error", "unknown request"},
    };
    return response.dump();
}


std::string Connector::getTelemetry() {
    controlLoop.gps.update();
    json telemetry = {
        {"type", "TELEMETRY"},
        {"gps",
            {
                {"lat", controlLoop.gps.get_latitude()},
                {"lon", controlLoop.gps.get_longitude()},
                {"altitude", controlLoop.gps.get_altitude_agl()},
                {"speed", controlLoop.gps.get_speed()},
                {"time", controlLoop.gps.get_time()},
                {"fix_quality", controlLoop.gps.get_fix_quality()},
                {"satellites", controlLoop.gps.get_satellites()},
                {"reliable", controlLoop.gps.is_data_reliable()}
            }
        },
        {"compass",
            {
                {"heading", controlLoop.compass.get_heading()}
            }
        }
    };
    return telemetry.dump();
}