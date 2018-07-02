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
#include <sys/time.h>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                                DJI::OSDK::RecvContainer recvFrame,
                                DJI::OSDK::UserData      userData);

#define START_CHAR '@'
#define END_CHAR '#'

void displayMenu();
int getNumber(std::string message);
long getTimeMs();

Vehicle*   vehicle;

#define FRAME_TO_SEND 300
#define MAX_FRAME_LENGTH 100

uint8_t testFrame[MAX_FRAME_LENGTH];
uint16_t ackCounter;
long startTime;
long receivedFrames, receivedBytes;

// 4 existing test modes
// down     : launched by M210, aircraft send 300 frames of 1-100 bytes each x ms, x is chosen on launch
// up       : launched on Android, ground station send 100 frames of 1-100 bytes x ms, x is chosen on launch
// ack up   : launched on Android, ground station ask numbered frames and aircraft send 100 bytes as ack. 100 frames are asked
// ack down :launched on M210, aircraft ask numbered frames and ground station send 100 bytes as ack. 100 frames are asked
enum MocTestMode {
    none = 0,
    down,
    up,
    ackUp,
    ackDown
};
MocTestMode testMode;
void sendModeStartRequest(MocTestMode testMode);
void sendModeEndRequest(MocTestMode testMode);
void sendModeRequest(char operation, MocTestMode testMode);

/*! main
 *
 */
int
main(int argc, char** argv)
{
    enum State {
        init,
        loop,
        end,
    };

    bool running = true;

    // MOC Frame test
    uint8_t n = 0;
    while (true) {
        testFrame[n] = n;
        n++;
        if (n == MAX_FRAME_LENGTH) {
            break;
        }
    }
    testMode = none;

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
        // Get user choice
        std::cin >> inputChar;
        // Get newline char
        std::cin.get();

        switch (inputChar) {
            case '1': {
                testMode = down;
                unsigned int delayMs, frame_length;
                useconds_t delayUs;
                // Get parameters
                frame_length = (unsigned int)getNumber("Frame length (bytes) : ");
                delayMs = (unsigned int)getNumber("Send delay (ms) : ");
                delayUs = delayMs * (useconds_t)1000;
                // Max testFrame length
                if(frame_length > 100) {
                    frame_length = 100;
                }

                // Send initializing command to ground station
                DSTATUS("Down test initializing : length = %u bytes, delay = %u ms ...", frame_length, delayMs);
                sendModeStartRequest(down);
                usleep(1000000);

                // Run test
                DSTATUS("Down test running ...");
                startTime = getTimeMs();
                uint16_t framesSent = 0;
                while(framesSent < FRAME_TO_SEND) {
                    testFrame[1] = (uint8_t)framesSent; // /!\ framesSent > 255, value will overflow
                    vehicle->moc->sendDataToMSDK(testFrame, (uint8_t)frame_length);
                    DSTATUS("Down test : Frame %u sent", framesSent);
                    framesSent++;
                    usleep(delayUs);
                }
                long diffTimeMs = getTimeMs() - startTime;
                // Remove last delay
                diffTimeMs -= delayUs / 1000;
                // End of test
                usleep(500000);
                sendModeEndRequest(down);
                long bytesSent = (long)frame_length * FRAME_TO_SEND;
                DSTATUS("Down test ended (length = %u bytes, delay = %u ms) : %u frames sent (%ld Bytes) in %ld ms", frame_length, delayMs, FRAME_TO_SEND, bytesSent, diffTimeMs);
                double dataFlow = bytesSent * 1000 / diffTimeMs;
                DSTATUS("Data flow = %lf Bytes/s", dataFlow);
                testMode = none;
            }
                break;
            case '2':
                DSTATUS("MOC - Ack down test initialization");
                sendModeStartRequest(ackDown);
                break;
            case 'r':
                DSTATUS("Test status reset");
                sendModeEndRequest(none);
                testMode = none;
                break;
            case 's': {
                std::cout << "Type command to send : " << std::endl;
                std::string command;
                std::getline(std::cin, command);
                DSTATUS("Send data to mobile : %s", command.c_str());
                vehicle->moc->sendDataToMSDK((uint8_t *)command.c_str(), command.length());
            }
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
            << "| [1] Down test                                                  |"
            << std::endl;
    std::cout
            << "| [2] Ack down test                                              |"
            << std::endl;
    std::cout
            << "| [r] Reset test status                                          |"
            << std::endl;
    std::cout
            << "| [s] Send custom command                                        |"
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

    switch (data[0]) {
        // New test launched
        case START_CHAR:
            switch (data[1]) {
                case up:
                    testMode = up;
                    DSTATUS("MOC - Up test launched");
                    receivedFrames = 0;
                    receivedBytes = 0;
                    sendModeStartRequest(up);
                    startTime = getTimeMs();
                    break;
                case down:
                    // Down test can not be launch by testFrame
                    break;
                case ackUp :
                    DSTATUS("MOC - Ack up test launched");
                    testMode = ackUp;
                    ackCounter = 0;
                    sendModeStartRequest(ackUp);
                    break;
                case ackDown:
                    testMode = ackDown;
                    DSTATUS("MOC - Ack down test launched");
                    receivedFrames = 0;
                    receivedBytes = 0;
                    uint8_t b[2];
                    b[0] = '-';     // '-' to avoid START_CHAR and END_CHAR
                    b[1] = 0;
                    startTime = getTimeMs();
                    vehicle->moc->sendDataToMSDK(b, 2);
                    break;
                default:
                    DERROR("MOC - Unknown test launched");
                    break;
            }
            break;
        // Test ended
        case END_CHAR:
            switch (data[1]) {
                // Reset status mode
                case none:
                    DSTATUS("MOC - Test mode reset");
                    testMode = none;
                    break;
                case up: {
                    long diffTime = getTimeMs() - startTime - 1000;   // 1000ms before send of # end character
                    // last send delay is not subtracted !
                    DSTATUS("MOC - Up test ended : %ld frames received (%ld Bytes) in %ld ms", receivedFrames,
                            receivedBytes, diffTime);
                    double dataFlow = receivedBytes * 1000 / diffTime;
                    DSTATUS("MOC - Up flow = %lf Bytes/s", dataFlow);
                    testMode = none;
                }
                    break;
                case ackUp:
                    DSTATUS("MOC - Ack up test ended");
                    testMode = none;
                    break;
                case ackDown:
                    // No end request are supposed to be received from ground station
                    DSTATUS("MOC - Ack down test ended ??");
                    testMode = none;
                    break;
                default:
                    DERROR("MOC - Unknown test ended");
                    testMode = none;
                    break;

            }
            break;
        // Data received
        default:
            switch (testMode) {
                case down:
                    // No data are supposed to be received in this mode
                    break;
                case up:
                    // Increment received counter
                    DSTATUS("MOC - Test up : Data received (%u) : %u/%u", msgLength, data[1], receivedFrames);
                    receivedFrames++;
                    receivedBytes += msgLength;
                    break;
                case ackUp:
                    // Verify that requested testFrame is correctly numbered
                    if(data[1] == ackCounter) {
                        testFrame[0] = '-'; // '-' to avoid START_CHAR or END_CHAR
                        testFrame[1] = data[1];
                        DSTATUS("MOC - Ack up test send testFrame %u", data[1]);
                        vehicle->moc->sendDataToMSDK(testFrame, 100);
                        ackCounter++;
                    } else {
                        DSTATUS("MOC - Ack up test testFrame counter error");
                        testMode = none;
                    }
                    break;
                case ackDown: {
                    uint8_t index = data[1];
                    DSTATUS("MOC - Ack down test : Frame %u received (%u bytes)", index, msgLength);
                    receivedBytes += msgLength;
                    receivedFrames++;

                    if (index == 99) {  // 100 frames received
                        long diffTime = getTimeMs() - startTime;
                        sendModeEndRequest(ackDown);
                        DSTATUS("MOC - Ack down test ended : %ld frames received (%ld Bytes) in %ld ms", receivedFrames,
                                receivedBytes, diffTime);
                        double dataFlow = receivedBytes * 1000 / diffTime;
                        DSTATUS("MOC - Ack down flow = %lf Bytes/s", dataFlow);
                        testMode = none;
                    } else {
                        index++;
                        uint8_t b[2];
                        b[0] = '-'; // random char to avoid START_CHAR or END_CHAR
                        b[1] = index;
                        DSTATUS("MOC - Ack down test : Send request %u", index);
                        vehicle->moc->sendDataToMSDK(b, 100);
                    }
                }
                    break;
                case none:
                    // Data received displayed as string
                    if(msgLength == 100) {
                        msgLength = 99;
                    }
                    data[msgLength] = '\0';
                    DSTATUS("MOC - Data received (%u) : %s", msgLength, data);
                    displayMenu();
                    break;
            }
            break;
    }
}

long getTimeMs() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long ms = (long) tp.tv_sec * 1000L + tp.tv_usec / 1000; //get current timestamp in milliseconds
    return ms;
}

void sendModeStartRequest(MocTestMode testMode) {
    sendModeRequest(START_CHAR, testMode);

}

void sendModeEndRequest(MocTestMode testMode) {
    sendModeRequest(END_CHAR, testMode);
}

void sendModeRequest(char operation, MocTestMode testMode) {
    char initFrame[2];
    initFrame[0] = operation;
    initFrame[1] = testMode;
    vehicle->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&initFrame), sizeof(initFrame));
}