/*! @file GeodeticCoord.h
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 *  @brief This class stores geodetic coordinates and allow user to add
 *  local NED vector to a point.
 *  Note that current implementation is only in 2D - without altitude/Z axis.
 *
 *  ---- Geodetic coordinates ----
 *  The geodetic coordinate system  is widely used in GPS-based navigation.
 *  It characterizes a coordinate point near the earth’s surface in terms of
 *  longitude (λ), latitude (ϕ), and height (h) (or altitude).
 *  The longitude measures the angle (−180° to 180°) between the Prime Meridian
 *  and the measured point.
 *  The latitude measures the angle (−90° to 90°) between the equatorial plane
 *  and the normal of the reference ellipsoid that passes through the measured point.
 *  ---- Local North-East-Down Coordinate (NED) ---
 *  The local NED coordinate system is also known as a navigation or ground coordinate
 *  system. It is a coordinate frame fixed to the earth’s surface.
 *  Its origin and axes are defined as the following :
 *      1. The origin (On) is arbitrarily fixed to a point on the earth’s surface
 *      2. The X-axis (Xn) points toward the geodetic north
 *      3. The Y-axis (Yn) points toward the geodetic east
 *      4. The Z-axis (Zn) points downward along the ellipsoid normal.
 */

#ifndef MATRICE210_GEODETICCOORD_H
#define MATRICE210_GEODETICCOORD_H

#include <dji_vehicle.hpp>

#include "../util/define.h"

using namespace DJI::OSDK;

namespace M210 {
    struct Dms {    /*!< Degree° minutes' seconds" notation */
        int deg;
        int min;
        double sec;
    };

    class GeodeticCoord {
    private:
        double latitude{};        /*!< Latitude stored in [rad] */
        double longitude{};       /*!< Longitude stored in [rad] */
        /**
         * Convert given position to deg
         * @param dms Dms value
         * @return Deg value [deg]
         */
        double DmsToDeg(Dms *dms) const;
        /**
         * Convert given position to rad
         * @param dms Dms value
         * @return Rad value [rad]
         */
        double DmsToRad(Dms *dms) const;
        /**
         * Convert given position to dms
         * @param rad Rad value [rad]
         * @return Dms value
         */
        Dms RadToDms(double rad) const;
        /**
         * Convert given position to dms
         * @param deg Deg value [rad]
         * @return Dms value
         */
        Dms DegToDms(double deg) const;
    public:
        GeodeticCoord() = default;
        GeodeticCoord(double latitude, double longitude);
        // Setters
        /**
         * Set current position
         * @param lat Latitude [rad]
         * @param lon Longitude [rad]
         */
        void setFromRad(double lat, double lon);
        /**
         * Set current position
         * @param lat Latitude [deg]
         * @param lon Longitude [deg]
         */
        void setFromDeg(double lat, double lon);
        /**
         * Set current position
         * @param lat Latitude [Dms]
         * @param lon Longitude [Dms]
         */
        void setFromDms(Dms *lat, Dms *lon);
        /**
         * Set current position from an other GeodeticCoord object
         * @param p Object whose position has to be copied
         */
        void set(GeodeticCoord &p);
        // Getters
        /**
         * Get current position
         * @param lat Dms structure to store latitude
         * @param lon Dms structure to store longitude
         */
        void dms(Dms &lat, Dms &lon) const;
        /**
         * Get current longitude
         * @return Longitude [Dms]
         */
        Dms longitudeDms() const;
        /**
         * Get current latitude
         * @return Latitude [Dms}
         */
        Dms latitudeDms() const;
        /**
         * Get current longitude
         * @return Longitude [rad]
         */
        double longitudeRad() const;
        /**
         * Get current latitude
         * @return Latitude [rad]
         */
        double latitudeRad() const;
        /**
         * Get current longitude
         * @return Longitude [deg]
         */
        double longitudeDeg() const;
        /**
         * Get current latitude
         * @return Latitude [deg]
         */
        double latitudeDeg() const;
        /**
        * Unit test to check that class is working. Called at the
        * beginning of the program. Assert if a test fails
        */
        static void unitTest();
        // Operator
        /**
         * Add a NED vector to a geodetic position
         * @param vec Vector to add [m]
         * @return Geodetic coordinates of the new point
         */
        GeodeticCoord operator+(const Vector2& vec);
        /**
         * Subtract a NED vector to a geodetic position
         * @param vec Vector to Subtract [m]
         * @return Geodetic coordinates of the new point
         */
        GeodeticCoord operator-(const Vector2& vec);
        /**
         * Add a NED vector to the current geodetic position
         * @param vec Vector to add [m]
         * @return Current geodetic coordinates object
         */
        GeodeticCoord operator+=(const Vector2 &vec);
         /**
          * Subtract a NED vector to the current geodetic position
          * @param vec Vector to Subtract [m]
          * @return Current geodetic coordinates object
          */
        GeodeticCoord operator-=(const Vector2 &vec);
    };
}

#endif //MATRICE210_GEODETICCOORD_H