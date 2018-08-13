/*! @file AvalancheMission.cpp
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 */

#include "AvalancheMission.h"

#include "../Gps/GeodeticCoord.h"
#include "../Gps/GpsManip.h"
#include "../Gps/GpsAxis.h"

using namespace M210;

void AvalancheMission::updateAxis(GeodeticCoord &P1, GeodeticCoord &P2) {
    // Create two points
    Vector2 offset = GpsManip::offsetFromGpsOffset(P1, P2);
    GpsAxis::instance().updateFrontVector(offset);
}
