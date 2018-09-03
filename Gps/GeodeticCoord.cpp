/*! @file GeodeticCoord.cpp
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 *  @brief GeodeticCoord.h implementation
 */

#include "GeodeticCoord.h"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace M210;

GeodeticCoord::GeodeticCoord(double latitude, double longitude) : latitude(latitude), longitude(longitude) {
}

void GeodeticCoord::setFromRad(double latitude, double longitude) {
    this->latitude = latitude;
    this->longitude = longitude;
}

void GeodeticCoord::setFromDeg(double latitude, double longitude) {
    this->latitude = latitude / RAD2DEG;
    this->longitude = longitude / RAD2DEG;
}

void GeodeticCoord::setFromDms(Dms *latitude, Dms *longitude) {
    this->latitude = DmsToRad(latitude);
    this->longitude = DmsToRad(longitude);
}

void GeodeticCoord::set(GeodeticCoord &p)
{
    this->latitude = p.latitudeRad();
    this->longitude = p.longitudeRad();
}

void GeodeticCoord::dms(Dms &latitude, Dms &longitude) const {
    latitude = RadToDms(this->latitude);
    longitude = RadToDms(this->longitude);
}

Dms GeodeticCoord::longitudeDms() const {
    Dms dms = RadToDms(longitude);
    return dms;
}

Dms GeodeticCoord::latitudeDms() const {
    Dms dms = RadToDms(latitude);
    return dms;
}

double GeodeticCoord::longitudeRad() const {
    return longitude;
}

double GeodeticCoord::latitudeRad() const {
    return latitude;
}

double GeodeticCoord::longitudeDeg() const {
    return longitudeRad() * RAD2DEG;
}

double GeodeticCoord::latitudeDeg() const {
    return latitudeRad() * RAD2DEG;
}

double GeodeticCoord::DmsToDeg(Dms *dms) const {
    // Convert decimal value in deg/min/sec
    return dms->deg + (dms->min / 60.0) + (dms->sec / 3600.0);;
}

double GeodeticCoord::DmsToRad(Dms *dms) const
{
    return DmsToDeg(dms) / RAD2DEG;
}

Dms GeodeticCoord::RadToDms(double rad) const {
    Dms dms{};
    dms = DegToDms(rad * RAD2DEG);
    return dms;
}

Dms GeodeticCoord::DegToDms(double deg) const {
    // Convert deg/min/sec value in decimal
    Dms dms{};
    dms.deg = (int)deg;
    dms.min = (int)((deg - dms.deg) * 60.0);
    dms.sec = (deg - dms.deg - dms.min/60.0) * 3600.0;
    return dms;
}

GeodeticCoord GeodeticCoord::operator+(const Vector2 &vec)
{
    // See GpsManip::offsetFromGpsOffset()
    // Inverse operation here
    GeodeticCoord GeodeticCoord;
    double deltaLat = vec.x / R_EARTH;
    double rEarthAtLat = R_EARTH * cos(this->latitude);
    double deltaLon = vec.y / rEarthAtLat;
    GeodeticCoord.latitude = this->latitude + deltaLat;
    GeodeticCoord.longitude = this->longitude + deltaLon;
    return GeodeticCoord;
}

GeodeticCoord GeodeticCoord::operator-(const Vector2 &vec)
{
    Vector2 v{-vec.x, -vec.y};
    return *this + v;
}

GeodeticCoord GeodeticCoord::operator+=(const Vector2 &vec){
    *this = *this + vec;
    return *this;
}

GeodeticCoord GeodeticCoord::operator-=(const Vector2 &vec)
{
    *this = *this - vec;
    return *this;
}


void GeodeticCoord::unitTest() {
    Dms lat1{}, lon1{}, lat2{}, lon2{};
    GeodeticCoord P1;
    P1.setFromDeg(46.21555833286709, 7.3381027777989702);
    P1.dms(lat1, lon1);

    GeodeticCoord P2;
    P2.setFromDms(&lat1, &lon1);
    P2.dms(lat2, lon2);

    assert(abs(P1.longitudeRad() - P2.longitudeRad()) < 0.000000001);
    assert(abs(P1.latitudeRad() - P2.latitudeRad()) < 0.000000001);
    assert(abs(P1.longitudeDeg() - P2.longitudeDeg()) < 0.0000001);
    assert(abs(P1.latitudeDeg() - P2.latitudeDeg()) < 0.0000001);
    assert(lat1.deg == lat2.deg);
    assert(lat1.min == lat2.min);
    assert(lat1.sec == lat2.sec);
    assert(lat1.deg == lat2.deg);
    assert(lat1.min == lat2.min);
    assert(lat1.sec == lat2.sec);
}