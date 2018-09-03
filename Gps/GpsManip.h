/*! @file GpsManip.h
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 *  @brief Vector and coordinates calculation methods
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

        /**
         * Calculate 2d vector norm
         * @param v Vector
         * @return Vector norm
         */
        static double vectorNorm(const Vector2 &v);

        /**
         * Rotate 2d vector
         * @param initVector Vector to rotate
         * @param rotationAngle Clockwise rotation angle
         * @return Rotated vector
         */
        static Vector2 rotateVector(const Vector2 &initVector, double rotationAngle);

        /**
         * Calculate angle between two vectors
         * @param v1 First vector
         * @param v2 Second vector
         * @return Angle, always positive [rad]
         */
        static double angleVector(const Vector2 &v1, const Vector2 &v2);

        /**
         * Calculate scalar product between two vectors
         * @param v1 First vector
         * @param v2 Second vector
         * @return Scalar product
         */
        static double scalarProduct(const Vector2 &v1, const Vector2 &v2);

        /**
         * Calculate local NED offset between two pairs of geodetic coordinates.
         * Accurate when distances are small.
         * @param origin Origin GPS coordinates
         * @param target Target GPS coordinates
         * @return Float 2d vector value
         */
        static Vector2 offsetFromGpsOffset(const GeodeticCoord &origin,
                                           const GeodeticCoord &target);

        /**
         * Calculate local NED offset between two pairs of GPS coordinates.
         * Accurate when distances are small.
         * @param origin Origin GPS coordinates
         * @param target Target GPS coordinates
         * @return Float 3d vector value
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
        GpsManip() = default;
    };
}

#endif //MATRICE210_GPSMANIP_H
