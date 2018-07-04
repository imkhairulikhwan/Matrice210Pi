/*! @file Console.hpp
 *  @version 1.0
 *  @date Jul 04 2018
 *  @author Jonathan Michel
 */

#include "Console.h"

Console::Console(FlightController* flightController) : flightController(flightController) {

}

void Console::launchThread() {
    pthread_attr_init(&consoleThreadAttr);
    pthread_attr_setdetachstate(&consoleThreadAttr, PTHREAD_CREATE_JOINABLE);
    int ret  = pthread_create(&consoleThreadID, nullptr, consoleThread, (void*)this);
    string infoStr = "consoleThread";

    if (0 != ret)
        cout << "Fail to create thread for " << infoStr.c_str() << "!\n";

    ret = pthread_setname_np(consoleThreadID, infoStr.c_str());
    if (0 != ret)
        cout << "Fail to set thread name for " << infoStr.c_str() << "!\n";
}

void* Console::consoleThread(void* param) {
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
                position.x = c->getNumber("xOffsetDesired: ");
                position.y = c->getNumber("yOffsetDesired: ");
                position.z = c->getNumber("zOffsetDesired: ");
                float32_t yaw = c->getNumber("yawDesired: ");
                c->flightController->moveByPositionOffset(&position, yaw);
                break;
            }
            case '4': {
                Telemetry::Vector3f velocity;
                velocity.x = c->getNumber("Vx: ");
                velocity.y = c->getNumber("Vy: ");
                velocity.z = c->getNumber("Vz: ");
                float32_t yaw = c->getNumber("yaw: ");
                c->flightController->moveByVelocity(&velocity, yaw);
                break;
            }
            case '5':
                DSTATUS("Stop velocityAndYawRateCtrl");
                c->flightController->stopVelocity();
                break;
            case 's': {
                cout << "Type command to send : " << endl;
                string command;
                getline(cin, command);
                DSTATUS("Send data to mobile : %s", command.c_str());
                c->flightController->sendDataToMSDK((uint8_t *)command.c_str(), (uint8_t)command.length());
            }
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
    displayMenuLine('3', "moveByPositionOffset");
    displayMenuLine('4', "Init velocityAndYawRateCtrl");
    displayMenuLine('5', "Stop velocityAndYawRateCtrl");
    displayMenuLine('s', "Send custom command");
    cout << endl;
}

void Console::displayMenuLine(const char command, const char* hint) {
    const int lineLength = 55;
    int length = strlen(hint);
    if(length < lineLength) {
        cout << endl << "| [" << command << "] ";
        cout << hint;
        for(int i = 0 ; i < lineLength-length; i++) {
            cout << " ";
        }
        cout << "|";
    }
}

float32_t Console::getNumber(const std::string &message) {
    float32_t number;
    while (true) {
        string input;
        cout << message;
        getline(cin, input);

        // This code converts from string to delayMs safely.
        stringstream myStream(input);
        if (myStream >> number)
            break;
        std::cout << "Invalid number" << std::endl;
    }
    return number;
}
