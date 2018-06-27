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
#include "mobile.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                                DJI::OSDK::RecvContainer recvFrame,
                                DJI::OSDK::UserData      userData);

void sendData();

Vehicle*   vehicle;

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

    bool running = true;

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
        << "| [s] Send data                                                  |"
        << std::endl;

    while(running) {
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
            case 's':
                sendData();
                break;
            default:
                break;
        }
    }
    return 0;
}

void
parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                        DJI::OSDK::RecvContainer recvFrame,
                        DJI::OSDK::UserData      userData)
{
    uint16_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;

    if(data[0] == '#') {
        switch (data[1]) {
            case '1':			// toggle LED
                DSTATUS("MOC - Led toggled");
                break;
            default:
                DERROR("MOC - Unknown command");
                break;
        }
    } else {
        data[msgLength] = '\0';
        DSTATUS("MOC - String received : %s", recvFrame.recvData.raw_ack_array);
    }
    vehicle->moc->sendDataToMSDK(data, msgLength);
}

void
sendData()
{
    char c[] = "Hello world from M210Pi";
    DSTATUS("Send data to mobile: %s", c);
    vehicle->moc->sendDataToMSDK(reinterpret_cast<uint8_t*>(&c), sizeof(c));
}
