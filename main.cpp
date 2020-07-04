/*
 *  TRAIL SCANNER V1
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <functional>

#include "rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

RPlidarDriver* lidar;
std::vector<RplidarScanMode> scanModes;
float scanModeRefresh = 100;

std::vector<RplidarScanMode> getScanModes() {
    // https://github.com/Slamtec/rplidar_sdk/blob/master/sdk/sdk/include/rplidar_driver.h
    std::vector<RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);
    printf("Scan Modes:\n");
    for (unsigned int i = 0; i < scanModes.size(); i++) {
        std::cout << scanModes[i].id << ' ';
        std::cout << scanModes[i].scan_mode;
        std::cout << '\n';
    }
    return scanModes;
}

void setScanMode(_u16 modeId) {
    lidar->startScanExpress(false, modeId);
}

void setScanMode(RplidarScanMode mode) {
    lidar->startScan(false, true, 0, &mode);
}

void startMotor() {
    lidar->startMotor();
}

void getData() {
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    lidar->getFrequency(scanModes[2], count, scanModeRefresh);
    std::cout << "ScanModeRefresh" << scanModeRefresh << '\n';
    
    if (IS_OK(lidar->grabScanDataHq(nodes, count, 0))) {
        lidar->ascendScanData(nodes, count);
        int pos = 0;

        std::ofstream outfile("output.csv", std::ios::out | std::ios::app);

        std::ostringstream ss;
        ss << std::fixed;

        for (pos = 0; pos < (int)count; ++pos) {
            ss << nodes[pos].angle_z_q14 * 90.f / (1 << 14);
            ss << ',';
            ss << nodes[pos].dist_mm_q2 / 1000.f / (1 << 2);
            ss << '\n';
            // printf("%03.2f, %03.2f\n",
            //     nodes[pos].angle_z_q14 * 90.f / (1 << 14),
            //     nodes[pos].dist_mm_q2 / 1000.f / (1 << 2));
            std::cout << ss.str() << std::endl;
            outfile << ss.str();
        }

        outfile.close();
    } else {
        fprintf(stderr, "Failed to get data\r\n");
    }
}

void quit() {
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);
    lidar = NULL;
    exit(0);
}

void setInterval(std::function<void(void)> func, unsigned int interval) {
    std::thread([func, interval]() {
        while (1) {
            func();
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }).detach();
}

int main(int argc, char** argv) {
    printf("Lidar Init\n");

    lidar = RPlidarDriver::CreateDriver();

    if (IS_OK(lidar->connect("/dev/ttyUSB0", 115200))) {
        printf("Connected\ns to start\nq to quit\n");
        while (1) {
            int in = getchar();
            std::cout << "Key Pressed: ";
            std::cout << in << std::endl;

            if (in == 113) { // q 
                std::cout << "Quit\n";
                quit();
            } else if (in == 115) { // s Start
                std::cout << "Start\n";

                scanModes = getScanModes();
                setScanMode(scanModes[2].id);
                std::cout << scanModes[2].scan_mode << '\n';
                startMotor();
                setInterval(getData, scanModeRefresh);
            }
        }
    } else {
        fprintf(stderr, "Error, failed to connect to Lidar\r\n");
    }
}
