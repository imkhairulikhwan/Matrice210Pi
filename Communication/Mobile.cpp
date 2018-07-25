/*! @file Mobile.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "Mobile.h"

#include "../Aircraft/FlightController.h"
#include "../Aircraft/Watchdog.h"
#include "../Action/Action.h"
#include "../Action/ActionData.h"

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
    if(data[0] == COMMAND_CHAR) {
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
                        actionData->push(data+4, dataLength-2);
                        actionData->push((char)data[3]);    // action
                        actionData->push((char)data[2]);    // mission kind
                    } else {
                        DERROR("Mission data format error");
                    }
                    break;
                case 'w':
                    actionData = new ActionData(ActionData::ActionId::watchdog);
                    //m->getFlightController()->getWatchdog()->reset();
                    break;
                default:
                    DERROR("Unknown command");
                    break;
            }
            if(actionData != nullptr)
                Action::instance().add(actionData);
        } else {
            DERROR("Unknown command format");
        }
    } else {
        // Data received displayed as string
        if(msgLength == 100) {
            msgLength = 99;
        }
        data[msgLength] = '\0';
        DSTATUS("String received : %s", recvFrame.recvData.raw_ack_array);
    }
}