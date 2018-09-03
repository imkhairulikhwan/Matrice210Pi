/*! @file GpsAxis.cpp
 *  @version 1.0
 *  @date Aou 12 2018
 *  @author Jonathan Michel
 *  @brief GpsAxis.h implementation
 */

#include "GpsAxis.h"

#include "GpsManip.h"

using namespace M210;

GpsAxis::GpsAxis() {
    rotationAxisAngle = 0;
}

void GpsAxis::setRotationAngle(double angle) {
    rotationAxisAngle = angle;
}

void GpsAxis::updateFrontVector(const Vector2 &v) {
    // todo mutex
    // Reference base is x face to north and y to east
    // South-North vector (1,0)
    Vector2 SNVec{1, 0};
    double angle = GpsManip::angleVector(SNVec, v);
    // If new vector y coordinates is < 0, angle is negative (Counterclockwise)
    // It works only with SNVec{1,0}
    if(v.y < SNVec.y)
        angle = -angle;
    rotationAxisAngle = angle;
}

Vector2 GpsAxis::projectVector(Vector2 &v) const {
    return GpsManip::rotateVector(v, rotationAxisAngle);
}

Vector2 GpsAxis::revertVector(Vector2 &v) const {
    return GpsManip::rotateVector(v, -rotationAxisAngle);
}