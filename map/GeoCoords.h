#ifndef GEOCOORDS_H
#define GEOCOORDS_H

#include <QString>

namespace MapAbstraction
{
    typedef float SingleCoordinate;

    class GeoCoords
    {
    public:
        GeoCoords();
        GeoCoords(SingleCoordinate longitude, SingleCoordinate latitude, SingleCoordinate altitude = 0);

        SingleCoordinate longitude() const;
        void setLongitude(SingleCoordinate longitude);
        SingleCoordinate latitude() const;
        void setLatitude(SingleCoordinate latitude);
        SingleCoordinate altitude() const;
        void setAltitude(SingleCoordinate altitude);
        QString positionString() const;

        void invalidate();
        bool valid() const;

        bool operator==(const GeoCoords &other) const;

    private:
        SingleCoordinate mLon;
        SingleCoordinate mLat;
        SingleCoordinate mAlt;
    };
}

#endif // GEOCOORDS_H
