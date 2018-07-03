/*! @file main.cpp
 *  @version 2.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include <iostream>
#include <string>
#include <sstream>

#include <dji_vehicle.hpp>

#include "Mission/flight_control.hpp"
#include "Mission/mobile.h"
#include "timer.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void displayMenu();
int getNumber(std::string message);

Vehicle*   vehicle;

/*! main
 *
 */
int main(int argc, char** argv)
{
    enum State {
        init,
        loop,
        end,
    };

    bool running = true;

    // Initialize variables
    int functionTimeout = 1;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    vehicle = linuxEnvironment.getVehicle();
    if (vehicle == nullptr)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Obtain Control Authority
    vehicle->obtainCtrlAuthority(functionTimeout);

    // MOC Callback
    setupMocParsing(vehicle, &linuxEnvironment);
    //*/

    // Display interactive prompt
    while(running) {
        displayMenu();
        char inputChar;
        // Get user choice
        std::cin >> inputChar;
        // Get newline char
        std::cin.get();

        switch (inputChar) {
            case '1':
                monitoredTakeoff(vehicle);
                break;
            case '2':
                monitoredLanding(vehicle);
                break;
            case '3': {
                int xOffsetDesired = getNumber("xOffsetDesired: ");
                int yOffsetDesired = getNumber("yOffsetDesired: ");
                int zOffsetDesired = getNumber("zOffsetDesired: ");
                int yawDesired = getNumber("yawDesired: ");
                moveByPositionOffset(vehicle, xOffsetDesired, yOffsetDesired, zOffsetDesired, yawDesired);
                break;
            }
            case 's': {
                std::cout << "Type command to send : " << std::endl;
                std::string command;
                std::getline(std::cin, command);
                DSTATUS("Send data to mobile : %s", command.c_str());
                vehicle->moc->sendDataToMSDK((uint8_t *)command.c_str(), reinterpret_cast<uint8_t >(command.length()));
            }
                break;

            default:
                break;

        }
    }
    return 0;
}

void displayMenuLine(const char command, const char* hint) {
    const int lineLength = 55;
    int length = strlen(hint);
    if(length < lineLength) {
        std::cout << std::endl << "| [" << command << "] ";
        std::cout << hint;
        for(int i = 0 ; i < lineLength-length; i++) {
            std::cout << " ";
        }
        std::cout << "|";
    }
}

void displayMenu() {
    std::cout << std::endl;
    std::cout << "Available commands : ";
    displayMenuLine('1', "Monitored takeoff");
    displayMenuLine('2', "Monitored landing");
    displayMenuLine('3', "moveByPositionOffset");
    displayMenuLine('s', "Send custom command");
    std::cout << std::endl;
}

int getNumber(const std::string &message) {
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


