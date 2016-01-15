#include "PlacemarkTracker.h"
using namespace Marble;

PlacemarkTracker::PlacemarkTracker(PlacemarkConstPtr placemark) : mPlacemark(placemark)
{
}

GeoDataLatLonBox PlacemarkTracker::calculateBox() const
{
    qreal lon, lat;
    mPlacemark->coordinate().geoCoordinates(lon, lat, GeoDataCoordinates::Degree);
    const qreal m = 0.0002;
    return GeoDataLatLonBox(lat + m, lat - m, lon + m, lon - m, GeoDataCoordinates::Degree);
}
