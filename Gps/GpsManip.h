/*! @file GpsManip.h
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_GPSMANIP_H
#define MATRICE210_GPSMANIP_H

#include "dji_vehicle.hpp"

#include "../util/define.h"

using namespace DJI::OSDK;

namespace M210 {
    class GeodeticCoord;

    class GpsManip {
    public:
        // Static math methods
        static double vectorNorm(const Vector2 &v);
        static Vector2 rotateVector(const Vector2 &initVector, double rotationAngle);
        static double angleVector(const Vector2 &v1, const Vector2 &v2);
        static double scalarProduct(const Vector2 &v1, const Vector2 &v2);
        static Vector2 offsetFromGpsOffset(const GeodeticCoord &origin,
                                           const GeodeticCoord &target);

        /**
         * Calculate local NED offset between two pairs of GPS coordinates.
         * Accurate when distances are small.
         * @param origin Origin GPS coordinates
         * @param target Target GPS coordinates
         * @return Float Vector value
         */
        static Telemetry::Vector3f offsetFromGpsOffset(const Telemetry::GPSFused &origin,
                                        const Telemetry::GPSFused &target );

        /**
         * Calculate euler angle from quaternion data
         * @param quaternionData quaternion
         * @return Rad euler angle
         */
        static Telemetry::Vector3f toEulerAngle(const Telemetry::Quaternion &quaternion);
            private:
        GpsManip() = default;;
    };
}

#endif //MATRICE210_GPSMANIP_H
