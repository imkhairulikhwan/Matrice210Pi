/*! @file Console.cpp
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 */

#include "Console.h"

#include <iostream>
#include <sstream>

#include "../FlightController.h"
#include "../Action/Action.h"
#include "../Action/ActionData.h"
#include "../Managers/PackageManager.h"
#include "../Managers/ThreadManager.h"
#include "../util/timer.h"

Console::Console(FlightController* flightController) : flightController(flightController) {

}

void Console::launchThread() {
    ThreadManager::start("consoleThread",
                         &consoleThreadID, &consoleThreadAttr,
                         consoleThread, (void*)this);
}

void* Console::consoleThread(void* param) {
    DSTATUS("consoleThread running...");
    auto c = (Console*) param;
    // Display interactive prompt
    bool running = true;
    while(running) {
        c->displayMenu();
        char inputChar;
        // Get user choice
        cin >> inputChar;
        // Get newline char
        cin.get();
        ActionData *actionData = nullptr;
        switch (inputChar) {
            case '1':
                actionData = new ActionData(ActionData::monitoredTakeoff);
                break;
            case '2':
                actionData = new ActionData(ActionData::monitoredLanding);
                break;
            case '3': {
                Telemetry::Vector3f position;
                position.x = c->getNumber("x: ");
                position.y = c->getNumber("y: ");
                position.z = c->getNumber("z: ");
                float32_t yaw = c->getNumber("yaw: ");
                actionData = new ActionData(ActionData::moveByPosition,
                                            sizeof(Telemetry::Vector3f) + sizeof(unsigned));
                actionData->push(position);
                actionData->push(yaw);
            }
                break;
            case '4': {
                Telemetry::Vector3f position;
                position.x = c->getNumber("xOffsetDesired: ");
                position.y = c->getNumber("yOffsetDesired: ");
                position.z = c->getNumber("zOffsetDesired: ");
                float32_t yaw = c->getNumber("yawDesired: ");
                actionData = new ActionData(ActionData::moveByPositionOffset,
                                            sizeof(Telemetry::Vector3f) + sizeof(unsigned));
                actionData->push(position);
                actionData->push(yaw);
            }
                break;
            case '5': {
                Telemetry::Vector3f velocity;
                velocity.x = c->getNumber("Vx: ");
                velocity.y = c->getNumber("Vy: ");
                velocity.z = c->getNumber("Vz: ");
                float32_t yaw = c->getNumber("yaw: ");
                actionData = new ActionData(ActionData::moveByVelocity,
                                            sizeof(Telemetry::Vector3f) + sizeof(unsigned));
                actionData->push(velocity);
                actionData->push(yaw);
            }
                break;
            case 'e':
                // Emergency stop is called directly here to avoid delay
                c->flightController->emergencyStop();
                break;
            case 'm': {
                cout << "Type command to send : " << endl;
                string command;
                getline(cin, command);
                DSTATUS("Send data to mobile : %s", command.c_str());
                c->flightController->sendDataToMSDK((uint8_t *)command.c_str(), (uint8_t)command.length());
            }
                break;
            case 'r':
                actionData = new ActionData(ActionData::emergencyRelease);
                break;
            case 's':
                actionData = new ActionData(ActionData::stopAircraft,
                                            sizeof(Telemetry::Vector3f) + sizeof(unsigned));
                break;
            default:
                break;

        }
        if(actionData != nullptr)
            Action::instance().add(actionData);
    }
}

void Console::displayMenu() {
    delay_ms(500);
    cout << endl;
    cout << "Available commands : ";
    displayMenuLine('1', "Monitored takeoff");
    displayMenuLine('2', "Monitored landing");
    displayMenuLine('3', "moveByPosition");
    displayMenuLine('4', "moveByPositionOffset");
    displayMenuLine('5', "moveByVelocity");
    displayMenuLine('e', "Emergency stop");
    displayMenuLine('m', "Send custom command");
    displayMenuLine('r', "Release emergency stop");
    displayMenuLine('s', "Stop aircraft");
    cout << endl;
}

void Console::displayMenuLine(const char command, const std::string &hint) {
    const int lineLength = 55;
    int length = hint.length();
    if(length < lineLength) {
        cout << endl << "| [" << command << "] ";
        cout << hint;
        for(int i = 0 ; i < lineLength-length; i++) {
            cout << " ";
        }
        cout << "|";
    }
}

float32_t Console::getNumber(const std::string &hint) {
    float32_t number;
    while (true) {
        string input;
        cout << hint;
        getline(cin, input);

        // This code converts from string to delayMs safely.
        stringstream myStream(input);
        if (myStream >> number)
            break;
        std::cout << "Invalid number" << std::endl;
    }
    return number;
}