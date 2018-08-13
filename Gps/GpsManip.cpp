/*! @file GpsManip.cpp
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 */


#include "GpsManip.h"

#include <cmath>

#include "GeodeticCoord.h"

using namespace M210;

double GpsManip::vectorNorm(const Vector2 &v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

double GpsManip::scalarProduct(const Vector2 &v1, const Vector2 &v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}


double GpsManip::angleVector(const Vector2 &v1, const Vector2 &v2) {
    double v1_n = GpsManip::vectorNorm(v1);
    double v2_n = GpsManip::vectorNorm(v2);
    if(v1_n == 0 || v2_n == 0)
        return 0;
    double frac = GpsManip::scalarProduct(v1, v2) / (v1_n * v2_n);
    double angle = acos(frac);
    return angle;
}

Vector2 GpsManip::rotateVector(const Vector2 &initVector, double rotationAngle) {
    double cosRot = cos(rotationAngle);
    double sinRot = sin(rotationAngle);

    Vector2 rotatedVector{};
    rotatedVector.x = initVector.x * cosRot + initVector.y * (-sinRot);
    rotatedVector.y = initVector.x * sinRot + initVector.y * cosRot;

    return rotatedVector;
}
Vector2 GpsManip::offsetFromGpsOffset(const GeodeticCoord &origin,
                                      const GeodeticCoord &target) {
    Vector2 delta{};
    // Calculate latitude/longitude delta
    double deltaLon = target.longitudeRad() - origin.longitudeRad();
    double deltaLat = target.latitudeRad() - origin.latitudeRad();
    // x-y movements between two geodetic coordinates points are calculated with arc formula
    // x (S-N) movement is proportional to delta latitude
    // Independently of the longitude of the point
    delta.x = deltaLat * R_EARTH;
    // y (W-E) movement is proportional to delta longitude
    // Points latitude impact section earth cutting
    // Section radius at a specified latitude is approximated with cos
    double lat_avg = (origin.latitudeRad() + target.latitudeRad()) / 2.0;
    double rEarthAtLat = R_EARTH * cos(lat_avg);
    delta.y = deltaLon * rEarthAtLat;
    return delta;
}

Telemetry::Vector3f GpsManip::offsetFromGpsOffset(const Telemetry::GPSFused &origin,
                                    const Telemetry::GPSFused &target) {
    GeodeticCoord ori(origin.latitude, origin.longitude);
    GeodeticCoord tar(target.latitude, target.longitude);
    Vector2 delta = GpsManip::offsetFromGpsOffset(ori, tar);

    Telemetry::Vector3f deltaNed;
    deltaNed.x = (float32_t) delta.x;
    deltaNed.y = (float32_t) delta.y;
    deltaNed.z = target.altitude - origin.altitude;
    return deltaNed;
}

Telemetry::Vector3f GpsManip::toEulerAngle(const Telemetry::Quaternion &quaternion) {
    // Formulas are provided by DJI
    Telemetry::Vector3f ans;

    double q2sqr = quaternion.q2 * quaternion.q2;
    double t0 =
            -2.0 * (q2sqr + quaternion.q3 * quaternion.q3) + 1.0;
    double t1 =
            +2.0 * (quaternion.q1 * quaternion.q2 + quaternion.q0 * quaternion.q3);
    double t2 =
            -2.0 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2);
    double t3 =
            +2.0 * (quaternion.q2 * quaternion.q3 + quaternion.q0 * quaternion.q1);
    double t4 =
            -2.0 * (quaternion.q1 * quaternion.q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = (float32_t) asin(t2);       // pitch
    ans.y = (float32_t) atan2(t3, t4);  // roll
    ans.z = (float32_t) atan2(t1, t0);  // yaw

    return ans;
}