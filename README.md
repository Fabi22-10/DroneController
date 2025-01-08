# DroneController
Control drone by remote control or approach defined targets in a control loop automatically.
Guide for Raspberry Pi 3:

## Prerequireties
- UART and I2C must be enabled in the raspi-config
- SBUS receiver must be connected to the raspberry with correct device name (/dev/ttyAMA1, see main.cpp)
- GPS Module must be connected on pins 3, 5, 8, 10

## How to run
1. Upload files on Raspberry Pi
2. Create build directory
```
mkdir build && cd build
```
4. Run Cmake
```
  cmake ..
```
5. Run make
```
  make
```
6. Run executable
```
  ./DroneController
```

## Usage
1. Remote control must be powered on
2. Connect to raspberry on ```100.96.1.5:1337``` via OpenVPN using the drone_app (https://github.com/TobiasBoeing/drone_app)
3. Use the drone_app to set targets or the remote control to navigate the drone
   
