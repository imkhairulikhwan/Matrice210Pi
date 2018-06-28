/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "flight_control.hpp"

#include <iostream>
#include <string>
#include <sstream>
#include <pthread.h>
#include "time.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                                DJI::OSDK::RecvContainer recvFrame,
                                DJI::OSDK::UserData      userData);

void sendData();

void displayMenu();
int getNumber(std::string message);

Vehicle*   vehicle;

#define FRAME_TO_SEND 300
#define FRAME_LENGTH 100

uint8_t frame[FRAME_LENGTH];
uint16_t frameCounter;
struct timespec lastTime;

/*! main
 *
 */
int
main(int argc, char** argv)
{
    enum State {
        init,
        loop,
        end
    };


    // MOC Frames tests
    bool running = true;
    int delayMs, frame_length;
    useconds_t delayUs;
    uint16_t framesToSend = FRAME_TO_SEND;
    uint8_t n = 0;
    while (true) {
        frame[n] = n;
        n++;
        if (n == FRAME_LENGTH) {
            break;
        }
    }
    frameCounter = 0;

    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    vehicle = linuxEnvironment.getVehicle();
    if (vehicle == NULL)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Obtain Control Authority
    vehicle->obtainCtrlAuthority(functionTimeout);

    // MOC Callback
    //setupMSDKParsing(vehicle, &linuxEnvironment);
    vehicle->moc->setFromMSDKCallback(parseFromMobileCallback);

    // Display interactive prompt


    while(running) {
        displayMenu();
        char inputChar;
        std::cin >> inputChar;

        switch (inputChar) {
            case 'a':
                monitoredTakeoff(vehicle);
                monitoredLanding(vehicle);
                break;
            case 'b':
                monitoredTakeoff(vehicle);
                moveByPositionOffset(vehicle, 0, 6, 0, 30);
                moveByPositionOffset(vehicle, 6, 0, 0, -30);
                moveByPositionOffset(vehicle, -6, -6, 0, 0);
                monitoredLanding(vehicle);
                break;
            case 'd':
                DSTATUS("Frame counter : %u", frameCounter);
                frameCounter = 0;
                break;
            case 's':
                sendData();
                break;
            case 't':
                frame_length = getNumber("Chose frame length : ");
                delayMs = getNumber("Chose delay (ms) : ");
                if(frame_length > 100) {
                    frame_length = 100;
                }

                std::cout << "Down-test running : length = " << frame_length << " bytes, delay = " << delayMs << " ms" << std::endl;
                std::cout << "..." << std::endl;
                delayUs = delayMs * (useconds_t)1000;

                while(framesToSend != 0) {
                    vehicle->moc->sendDataToMSDK(frame, (uint8_t)frame_length);
                    DSTATUS("Frame %u sent", FRAME_TO_SEND - framesToSend);
                    framesToSend--;
                    usleep(delayUs);
                }
                framesToSend = FRAME_TO_SEND;
                DSTATUS("Down-test ended : %u frames sent", FRAME_TO_SEND);
                break;
            default:
                break;

        }
    }
    return 0;
}

void displayMenu() {
    std::cout << std::endl;
    std::cout
            << "| Available commands:                                            |"
            << std::endl;
    std::cout
            << "| [a] Monitored Takeoff + Landing                                |"
            << std::endl;
    std::cout
            << "| [b] Monitored Takeoff + Position Control + Landing             |"
            << std::endl;
    std::cout
            << "| [d] Display and reset frame counter                            |"
            << std::endl;
    std::cout
            << "| [s] Send \"Hello word from M210Pi3\"                             |"
            << std::endl;
    std::cout
            << "| [t] Test speed                                                 |"
            << std::endl;
}

int
getNumber(std::string message) {
    int number;
    while (true) {
        std::string input = "";
        std::cout << message;
        std::getline(std::cin, input);

        // This code converts from string to delayMs safely.
        std::stringstream myStream(input);
        if (myStream >> number)
            break;
        std::cout << "Invalid number" << std::endl;
    }
    return number;
}

void
parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                        DJI::OSDK::RecvContainer recvFrame,
                        DJI::OSDK::UserData      userData)
{
    uint16_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;

    frameCounter++;

    if(data[0] == '#') {
        if(data[1] < 101) {
            frame[0] = data[1];
            DSTATUS("%u", data[1]);
            vehicle->moc->sendDataToMSDK(frame, 100);
        }
    } else {
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC_RAW, &currentTime);

        long ms;
        long diff = currentTime.tv_nsec - lastTime.tv_nsec;
        ms = diff / (long)1.0e6;

        lastTime = currentTime;

        DSTATUS("Diff ms = %lu", diff);
        DSTATUS("MOC - Data received : %u", msgLength);
    }
}

void
sendData()
{
    char c[] = "Hello world from M210Pi3";
    DSTATUS("Send data to mobile: %s", c);
    vehicle->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&c), sizeof(c));
}
