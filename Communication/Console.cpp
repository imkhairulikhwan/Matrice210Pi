/*! @file Console.cpp
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 */

#include "Console.h"

#include <iostream>
#include <sstream>

#include "../Aircraft/FlightController.h"
#include "../Action/Action.h"
#include "../Action/ActionData.h"
#include "../Managers/PackageManager.h"
#include "../Managers/ThreadManager.h"
#include "../util/timer.h"

using namespace M210;

Console::Console(FlightController* flightController) : flightController(flightController) {

}

void Console::launchThread() {
    ThreadManager::start("consoleThread",
                         &consoleThreadID, &consoleThreadAttr,
                         consoleThread, (void*)this);
}

void* Console::consoleThread(void* param) {
    DSTATUS("consoleThread running...");
    auto c = static_cast<Console*>(param);
    // Display interactive prompt
    bool running = true;
    bool displayMenu = true;
    while(running) {
        // Display menu except last action disable it
        if(displayMenu)
            c->displayMenu();
        displayMenu = true;
        // Get user choice
        char inputChar;
        cin >> inputChar;
        // Get newline char
        cin.get();
        ActionData *actionData = nullptr;
        switch (inputChar) {
            case '1':
                actionData = new ActionData(ActionData::takeOff);
                break;
            case '2':
                actionData = new ActionData(ActionData::landing);
                break;
            case '3': {
                Telemetry::Vector3f position;
                position.x = c->getNumber("x: ");
                position.y = c->getNumber("y: ");
                position.z = c->getNumber("z: ");
                float yaw = c->getNumber("yaw: ");
                actionData = new ActionData(ActionData::mission,
                                            sizeof(Telemetry::Vector3f) // position
                                            + sizeof(unsigned)          // mission action
                                            + 2 * sizeof(char));        // mission kind
                actionData->push(position);
                actionData->push(yaw);
                actionData->push((char)1);    // action
                actionData->push((char)2);    // mission kind
            }
                break;
            case '4': {
                Telemetry::Vector3f position;
                position.x = c->getNumber("xOffsetDesired: ");
                position.y = c->getNumber("yOffsetDesired: ");
                position.z = c->getNumber("zOffsetDesired: ");
                float yaw = c->getNumber("yawDesired: ");
                actionData = new ActionData(ActionData::mission,
                                            sizeof(Telemetry::Vector3f) // position offset
                                            + sizeof(unsigned)          // mission action
                                            + 2 * sizeof(char));        // mission kind
                actionData->push(position);
                actionData->push(yaw);
                actionData->push((char)1);    // action
                actionData->push((char)3);    // mission kind
            }
                break;
            case '5': {
                Telemetry::Vector3f velocity;
                velocity.x = c->getNumber("Vx: ");
                velocity.y = c->getNumber("Vy: ");
                velocity.z = c->getNumber("Vz: ");
                float yaw = c->getNumber("yaw: ");
                actionData = new ActionData(ActionData::mission,
                                            sizeof(Telemetry::Vector3f) // velocity
                                            + sizeof(unsigned)          // mission action
                                            + 2 * sizeof(char));        // mission kind
                actionData->push(velocity);
                actionData->push(yaw);
                actionData->push((char)1);    // action
                actionData->push((char)1);    // mission kind
            }
                break;
            case '6': {
                unsigned u = (unsigned)c->getNumber("1pol 2res 3pau 4sto 5sav 6ret");
                c->flightController->waypointMissionAction(u);
                displayMenu = false;
            }
                break;
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
                // TODO Add to action queue
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
                DERROR("Unknown command");
                break;

        }
        if(actionData != nullptr)
            Action::instance().add(actionData);
    }
}

void Console::displayMenu() const {
    delay_ms(500);
    cout << endl;
    cout << "Available commands : ";
    displayMenuLine('1', "Take-off");
    displayMenuLine('2', "Landing");
    displayMenuLine('3', "moveByPosition");
    displayMenuLine('4', "moveByPositionOffset");
    displayMenuLine('5', "moveByVelocity");
    displayMenuLine('6', "Waypoint mission");
    displayMenuLine('e', "Emergency stop");
    displayMenuLine('m', "Send custom command");
    displayMenuLine('p', "Get current position");
    displayMenuLine('r', "Release emergency stop");
    displayMenuLine('s', "Stop aircraft");
    cout << endl;
}

void Console::displayMenuLine(char command, const std::string &hint) const{
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

float Console::getNumber(const std::string &hint) const {
    float number;
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