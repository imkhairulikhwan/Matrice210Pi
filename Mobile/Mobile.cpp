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
    // Register the callback for parsing mobile data
    flightController->getVehicle()->moc->setFromMSDKCallback(parseFromMobileCallback, (void*)this);
}

void Mobile::parseFromMobileCallback(Vehicle *vehicle, RecvContainer recvFrame,
                                                  UserData userData) {
    // First, lets cast the userData to LinuxSetup*
    auto *m = (Mobile *) userData;

    uint8_t formatFrameLength = OpenProtocol::PackageMin + OpenProtocol::CRCHead;
    uint16_t msgLength = recvFrame.recvInfo.len - formatFrameLength;
    uint8_t* data = recvFrame.recvData.raw_ack_array;


    if(data[0] == '#') {
        switch (data[1]) {
            case '!':
                m->getFlightController()->emergencyStop();
                break;
            case '#':
                m->getFlightController()->emergencyRelease();
                break;
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
        DSTATUS("MOC - String received : %s", recvFrame.recvData.raw_ack_array);
    }
    vehicle->moc->sendDataToMSDK(data, (uint8_t)msgLength);
}