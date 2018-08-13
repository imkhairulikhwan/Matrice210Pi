/*! @file GpsAxis.h
 *  @version 1.0
 *  @date Aou 12 2018
 *  @author Jonathan Michel
 */

#ifndef MATRICE210_GPSAXIS_H
#define MATRICE210_GPSAXIS_H

#include <dji_vehicle.hpp>

#include "../util/define.h"

using namespace DJI::OSDK;

namespace M210 {
    class GpsAxis : public Singleton<GpsAxis> {
    private:
        double rotationAxisAngle;
    public:
        GpsAxis();
        void setRotationAngle(double angle);

        void updateFrontVector(const Vector2 &v);

        Vector2 projectVector(Vector2 &v) const;
        Vector2 revertVector(Vector2 &v) const;
    };
}

#endif //MATRICE210_GPSAXIS_H
