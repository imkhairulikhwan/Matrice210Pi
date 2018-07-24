/*! @file Mobile.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "Mobile.h"

#include "../FlightController.h"
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
        if(msgLength == 2) {
            switch (data[1]) {
                case 'e':
                    // Emergency stop is called directly here to avoid delay
                    m->getFlightController()->emergencyStop();
                    break;
                case 'r':
                    actionData = new ActionData(ActionData::emergencyRelease);
                    break;
                case 's':
                    actionData = new ActionData(ActionData::stopAircraft);
                    break;
                case 't':
                    actionData = new ActionData(ActionData::monitoredTakeoff);
                    break;
                case 'l':
                    actionData = new ActionData(ActionData::monitoredLanding);
                    break;
                case 'm':
                    DSTATUS("mission !");
                    break;
                default:
                    DERROR("MOC - Unknown command");
                    break;
            }
            if(actionData != nullptr)
                Action::instance().add(actionData);
        } else {
            DERROR("MOC - Unknown command format");
        }
    } else {
        // Data received displayed as string
        if(msgLength == 100) {
            msgLength = 99;
        }
        data[msgLength] = '\0';
        DSTATUS("MOC - String received :) : %s", recvFrame.recvData.raw_ack_array);
    }
}