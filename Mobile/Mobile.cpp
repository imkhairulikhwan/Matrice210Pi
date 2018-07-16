/*! @file Mobile.cpp
 *  @version 1.0
 *  @date Jul 03 2018
 *  @author Jonathan Michel
 */

#include "Mobile.h"

#include "../FlightController.h"

Mobile::Mobile(FlightController *flightController) : flightController(flightController) {

}

void Mobile::setup() {
    // Mobile object is passed as parameter to grant access to FlightController
    DSTATUS("MOBSDK Setup");
    flightController->getVehicle()->moc->setFromMSDKCallback(mobileCallback, (void *) this);
}

void Mobile::mobileCallback(Vehicle *vehicle, RecvContainer recvFrame,
                            UserData userData) {
    // Cast userData to Mobile object to have access to used FlightController
    auto *m = (Mobile *) userData;

    uint8_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;

    // TODO Better protocol choice, ensure data size
    if(data[0] == '#') {
        switch (data[1]) {
            case 'e':
                m->getFlightController()->emergencyStop();
                break;
            case 'r':
                m->getFlightController()->emergencyRelease();
                break;
            case 's':
                m->getFlightController()->stopAircraft();
                break;
            case 't':
                m->getFlightController()->monitoredTakeoff();
                break;
            case 'l':
                m->getFlightController()->monitoredLanding();
            default:
                DERROR("MOC - Unknown command");
                break;
        }
    } else {
        // Data received displayed as string
        if(msgLength == 100) {
            msgLength = 99;
        }
        data[msgLength] = '\0';
        DSTATUS("MOC - String received :) : %s", recvFrame.recvData.raw_ack_array);
    }
    //vehicle->moc->sendDataToMSDK(data, (uint8_t)msgLength);
}