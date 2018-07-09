/*! @file Console.cpp
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 */

#include "Console.h"

#include "../PackageManager/PackageManager.h"
#include "../FlightController.h"

Console::Console(FlightController* flightController) : flightController(flightController) {

}

void Console::launchThread() {
    pthread_attr_init(&consoleThreadAttr);
    pthread_attr_setdetachstate(&consoleThreadAttr, PTHREAD_CREATE_JOINABLE);
    int ret  = pthread_create(&consoleThreadID, nullptr, consoleThread, (void*)this);
    string threadName = "consoleThread";

    if (ret != 0)
        DERROR("Fail to create thread for %s !", threadName.c_str());

    ret = pthread_setname_np(consoleThreadID, threadName.c_str());
    if (ret != 0)
        DERROR("Fail to set thread name for %s !", threadName.c_str());
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
        switch (inputChar) {
            case '1':
                c->flightController->monitoredTakeoff();
                break;
            case '2':
                c->flightController->monitoredLanding();
                break;
            case '3': {
                Telemetry::Vector3f position;
                position.x = c->getNumber("x: ");
                position.y = c->getNumber("y: ");
                position.z = c->getNumber("z: ");
                float32_t yaw = c->getNumber("yaw: ");
                c->flightController->moveByPosition(&position, yaw);
            }
                break;
            case '4': {
                Telemetry::Vector3f position;
                position.x = c->getNumber("xOffsetDesired: ");
                position.y = c->getNumber("yOffsetDesired: ");
                position.z = c->getNumber("zOffsetDesired: ");
                float32_t yaw = c->getNumber("yawDesired: ");
                c->flightController->moveByPositionOffset(&position, yaw);
            }
                break;
            case '5': {
                Telemetry::Vector3f velocity;
                velocity.x = c->getNumber("Vx: ");
                velocity.y = c->getNumber("Vy: ");
                velocity.z = c->getNumber("Vz: ");
                float32_t yaw = c->getNumber("yaw: ");
                c->flightController->moveByVelocity(&velocity, yaw);
            }
                break;
            case 'e':
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
            case 'r': {
                int packageNb = (int)c->getNumber("Package number :");
                PackageManager::getInstance()->unsubscribe(packageNb);
            }
                break;
            case 's':
                DSTATUS("Stop aircraft");
                c->flightController->stopAircraft();
                break;

            default:
                break;

        }
    }
}

void Console::displayMenu() {
    cout << endl;
    cout << "Available commands : ";
    displayMenuLine('1', "Monitored takeoff");
    displayMenuLine('2', "Monitored landing");
    displayMenuLine('3', "moveByPosition");
    displayMenuLine('4', "moveByPositionOffset");
    displayMenuLine('5', "moveByVelocity");
    displayMenuLine('e', "Emergency stop");
    displayMenuLine('m', "Send custom command");
    displayMenuLine('r', "Remove package");
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