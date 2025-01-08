#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "Connector.h"
#include <thread>
#include <chrono>
#include "SBUS.h"
#include "serialib.h"
#include <mutex>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <stdio.h>
#include <cstring>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "ControlLoop.h"


using namespace std;
using std::cout;
using std::cerr;
using std::endl;
using std::cin;
using std::string;
using std::chrono::steady_clock;
using std::chrono::milliseconds;

#define SERIAL_PORT "/dev/ttyUSB2"

bool remoteInactive = false;
static auto lastWrite = steady_clock::now();
static auto lastSBUSwrite = steady_clock::now();

static SBUS sbus;
static auto lastSBUSchange = steady_clock::now();
sbus_packet_t lastPacket;

ControlLoop control_loop(66.0, 66.0, 33.0, 7.33);   // k_lat, k_lon = 66 corresponds full throttle when deviation is 10 meters
                                                    // k_alt = 33 corresponds full throttle when deviation is 20 meters
                                                    // k_yaw = 73.3 corresponds full throttle when deviation is 90°


sbus_packet_t getControlSignals() {
    // mu_packet.lock();
    sbus_packet_t signals = control_loop.get_steering_signals();
    signals.channels[2] = 1024; // Don't control altitude yet
    // std::cout << "Steering Signals: ["
    //         << "Left-Right: " << signals.channels[0] << ", "
    //         << "Front-Back: " << signals.channels[1] << ", "
    //         << "Up-Down: " << signals.channels[2] << ", "
    //         << "CW-CCW: " << signals.channels[3] << ", ";
    // if (signals.channels[6] == 1541) {
    //     std::cout << "Mode: Hold GPS Position";
    // }    
    // else if (signals.channels[6] == 1024) {
    //     std::cout << "Mode: Altitude Stabilized";
    // }
    // else {
    //     std::cout << "Mode: Undefined: " << signals.channels[6];
    // }
    // std::cout << std::endl;

    // mu_packet.unlock();
    return signals;
}

static void onPacket(const sbus_packet_t &packet)
{
    static auto lastPrint = steady_clock::now();
    auto now = steady_clock::now();

    bool change = false;
    for (int i = 0; i < 16; ++i) {
        if (packet.channels[i] != lastPacket.channels[i]) {
            change = true;
            for (int i = 0; i < 16; ++i)
                printf("ch%d: %d\t", i + 1, packet.channels[i]);
            break;
        }
    }

    if(change){
        control_loop.abort();
        lastSBUSchange = now;
        remoteInactive = false;
        for (int i = 0; i < 16; ++i)
            lastPacket.channels[i] = packet.channels[i];
        lastPacket.ch17 = packet.ch17;
        lastPacket.ch18 = packet.ch18;
        lastPacket.frameLost = packet.frameLost;
        lastPacket.failsafe = packet.failsafe;

    } else if (now - lastSBUSchange > milliseconds(5000) && !remoteInactive) {
        remoteInactive = true;
        cout << "Remote inactive, internal control enabled!" << endl;
    }
}


int main() {

    // SBUS initalisieren

    string ttyPath = "/dev/ttyAMA1";

    sbus.onPacket(onPacket);

    sbus_err_t err = sbus.install(ttyPath.c_str(), true);  // false for non-blocking
    if (err != SBUS_OK)
    {
        cerr << "SBUS install error: " << err << endl;
        return err;
    }
    std::cerr << "Gestartet" << std::endl;
   
    lastSBUSchange = steady_clock::now();
    lastWrite = steady_clock::now();
    lastSBUSwrite = steady_clock::now();

    if (!control_loop.init()) {
        std::cerr << "Failed to initialize GPS or Compass." << std::endl;
        return 1;
    }
    
    // Netzwerk Thread starten
    Connector connector(control_loop, 1337);
    if (!connector.start()) {
        std::cerr << "Failed to start connector." << std::endl;
        return 1;
    }

    //Mainloop
    while(true) {

        auto now = steady_clock::now();

        if(now - lastWrite > milliseconds(2000)) {
            std::cout << "Control loop state: " << static_cast<int>(control_loop.get_position_control_state()) << std::endl;
            lastWrite = now;
        }
        
        sbus_err_t result = sbus.read();
        control_loop.update_signals();

        if (result == SBUS_OK) {
            // printf("Read successful: SBUS_OK\n");
        } else if (result == SBUS_ERR_DESYNC) {
            printf("SBUS Read error: SBUS_ERR_DESYNC (Bad packet)\n");
        } else {
            printf("SBUS Read error: %d\n", result);
        }

        if (!remoteInactive) {
            // Write last packet received from remote control
            sbus.write(lastPacket);
        } else {
            sbus_packet_t controlPacket = getControlSignals();
            sbus.write(controlPacket);
        }
    }

    connector.stop();
    return 0;
}
