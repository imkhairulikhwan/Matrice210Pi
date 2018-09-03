/*! @file GpsAxis.h
 *  @version 1.0
 *  @date Aou 12 2018
 *  @author Jonathan Michel
 *  @brief This class allows user to rotate X-Y coordinates
 *  in NED vector depending on a pre-defined rotation angle.
 *  This is useful to change ground referential.
 *  By default, X axe face to North. With a rotation angle of
 *  90°, it will face to East.
 */

#ifndef MATRICE210_GPSAXIS_H
#define MATRICE210_GPSAXIS_H

#include <dji_vehicle.hpp>

#include "../util/define.h"

using namespace DJI::OSDK;

namespace M210 {
    class GpsAxis : public Singleton<GpsAxis> {
    private:
        double rotationAxisAngle;       /*!< Rotation angle of custom base [rad] */
    public:
        GpsAxis();
        /**
         * Set rotation angle
         * @param angle Rotation angle to save [rad]
         */
        void setRotationAngle(double angle);

        /**
         * Calculate rotation angle to use from 2d vector who indicates
         * new x positive direction
         * @param v Direction vector in NED
         */
        void updateFrontVector(const Vector2 &v);

        /**
         * Project vector from custom base to NED
         * @param v Vector to project
         * @return Projected vector
         */
        Vector2 projectVector(Vector2 &v) const;

        /**
         * Project vector from NED to custom base
         * @param v Vector to project
         * @return Projected vector
         */
        Vector2 revertVector(Vector2 &v) const;
    };
}

#endif //MATRICE210_GPSAXIS_H
