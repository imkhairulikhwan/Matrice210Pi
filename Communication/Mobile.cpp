/*! @file Mobile.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 *  @brief Mobile.h implementation
 */

#include "Mobile.h"

#include <string>

#include "../util/Log.h"
#include "../Aircraft/FlightController.h"
#include "../Aircraft/Watchdog.h"
#include "../Action/Action.h"
#include "../Action/ActionData.h"

using namespace std;
using namespace M210;

Mobile::Mobile(FlightController *flightController) : flightController(flightController) {

}

void Mobile::setup() {
    // Mobile object is passed as parameter to grant access to FlightController
    DSTATUS("MOBSDK Setup");
    flightController->getVehicle()->moc->setFromMSDKCallback(mobileCallback, (void *) this);
}

void Mobile::mobileCallback(Vehicle *vehicle, RecvContainer recvFrame,
                            UserData userData) {
    // Cast userData to Mobile object to have access to corresponding FlightController
    auto *m = static_cast<Mobile *>(userData);

    uint8_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;

    ActionData* actionData = nullptr;
    // Command char at the beginning of a frame indicates frame is a command
    if(data[0] == COMMAND_CHAR) {
        // Get command char
        if(msgLength >= 2) {
            switch (data[1]) {
                case 'e':
                    // Emergency stop is called directly here to avoid delay
                    m->getFlightController()->emergencyStop();
                    break;
                case 'r':
                    actionData = new ActionData(ActionData::ActionId::emergencyRelease);
                    break;
                case 's':
                    actionData = new ActionData(ActionData::ActionId::stopAircraft);
                    break;
                case 't':
                    actionData = new ActionData(ActionData::ActionId::takeOff);
                    break;
                case 'l':
                    actionData = new ActionData(ActionData::ActionId::landing);
                    break;
                case 'm':   // mission
                    if(msgLength >= 4) { // 4 command bytes and unknown data length
                        size_t dataLength =  msgLength - (size_t)2;
                        actionData = new ActionData(ActionData::ActionId::mission, dataLength);
                        actionData->push(data+4, dataLength-2); // mission parameters
                        actionData->push((char)data[3]);        // mission action
                        actionData->push((char)data[2]);        // mission type
                    } else {
                        LERROR("Mission data format error");
                    }
                    break;
                case 'o':
                    actionData = new ActionData(ActionData::ActionId::obtainControlAuthority);
                    break;
                case 'w':
                    actionData = new ActionData(ActionData::ActionId::watchdog);
                    break;
                default:
                    LERROR("Unknown command received from MOSDK");
                    break;
            }
            if(actionData != nullptr)
                Action::instance().add(actionData);
        } else {
            LERROR("Unknown command format received from MOSDK");
        }
    } else {
        // Data received displayed as string
        if(msgLength == 100) {
            msgLength = 99;
        }
        data[msgLength] = '\0';
        DSTATUS("String received: %s", recvFrame.recvData.raw_ack_array);

        // Hello world response for fun
        string msg = string(reinterpret_cast<char*>(data));
        string hw = "Hello world from Android";
        if(msg == hw) {
            actionData = new ActionData(ActionData::ActionId::helloWorld);
            Action::instance().add(actionData);
        }

    }
}