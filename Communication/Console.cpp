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
#include "../util/Log.h"
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
                // Move by position
                Telemetry::Vector3f position;
                position.x = c->getNumber("x: ");
                position.y = c->getNumber("y: ");
                position.z = c->getNumber("z: ");
                float yaw = c->getNumber("yaw: ");
                actionData = new ActionData(ActionData::ActionId::mission,
                                            sizeof(Telemetry::Vector3f) // position
                                            + sizeof(unsigned)          // mission action
                                            + 2 * sizeof(char));        // mission kind
                actionData->push(position);
                actionData->push(yaw);
                actionData->push((char)Action::MissionAction::START);    // action
                actionData->push((char)Action::MissionType::POSITION);   // mission kind
            }
                break;
            case '4': {
                // Move by position offset
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
                actionData->push((char)Action::MissionAction::START);           // action
                actionData->push((char)Action::MissionType::POSITION_OFFSET);   // mission kind
            }
                break;
            case '5': {
                // Move by velocity
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
                actionData->push((char)Action::MissionAction::START);    // action
                actionData->push((char)Action::MissionType::VELOCITY);    // mission kind
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
                // TODO Add to action queue
                c->flightController->sendDataToMSDK(reinterpret_cast<const uint8_t *>(command.c_str()), (uint8_t)command.length());
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
    displayMenuLine('e', "Emergency stop");
    displayMenuLine('m', "Send custom command");
    displayMenuLine('r', "Release emergency stop");
    displayMenuLine('s', "Stop aircraft");
    cout << endl;
}

void Console::displayMenuLine(char command, const std::string &hint) const{
    // Display line as follow :
    // | [1] Take-off                                               |
    // If hint is longer than lineLength end border char "|" will be shifted
    const int lineLength = 55;
    int length = hint.length();
    cout << endl << "| [" << command << "] ";
    cout << hint;
    for(int i = 0 ; i < lineLength-length; i++) {
        cout << " ";
    }
    cout << "|";
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