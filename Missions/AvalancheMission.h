/*! @file AvalancheMission.h
 *  @version 1.0
 *  @date Aou 11 2018
 *  @author Jonathan Michel
 */
#ifndef MATRICE210_AVALANCHEMISSION_H
#define MATRICE210_AVALANCHEMISSION_H

namespace M210 {
    class GeodeticCoord;

    class AvalancheMission {
    private:
        GeodeticCoord* corners;
    public:
        AvalancheMission() = default;
        void updateAxis(GeodeticCoord &P1, GeodeticCoord &P2);
    };
}


#endif //MATRICE210_AVALANCHEMISSION_H
