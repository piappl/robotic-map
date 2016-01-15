#include "GeoCoords.h"
#include "MapLibraryHelpers.h"

namespace
{
    const MapAbstraction::GeoCoords invalidCoords(0, 0, 100);
}

namespace MapAbstraction
{
    SingleCoordinate GeoCoords::longitude() const { return mLon; }
    void GeoCoords::setLongitude(SingleCoordinate longitude) { mLon = longitude; }

    SingleCoordinate GeoCoords::latitude() const { return mLat; }
    void GeoCoords::setLatitude(SingleCoordinate latitude) { mLat = latitude; }

    SingleCoordinate GeoCoords::altitude() const { return mAlt; }
    void GeoCoords::setAltitude(SingleCoordinate altitude) { mAlt = altitude; }

    QString GeoCoords::positionString() const
    {
        return MapLibraryHelpers::coordsString(*this);
    }

    bool GeoCoords::valid() const
    {
        bool invalid = *this == invalidCoords;
        return !invalid;
    }

    void GeoCoords::invalidate()
    {
        mLon = invalidCoords.longitude();
        mLat = invalidCoords.latitude();
        mAlt = invalidCoords.altitude();
    }

    bool GeoCoords::operator==(const GeoCoords &other) const
    {
        return (longitude() == other.longitude()
                && latitude() == other.latitude()
                && altitude() == other.altitude());
    }

    GeoCoords::GeoCoords(SingleCoordinate longitude, SingleCoordinate latitude, SingleCoordinate altitude)
        : mLon(longitude), mLat(latitude), mAlt(altitude) {}

    GeoCoords::GeoCoords() : mLon(invalidCoords.longitude()),
        mLat(invalidCoords.latitude()), mAlt(invalidCoords.altitude()) {}
}


