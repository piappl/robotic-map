#include "MapLibraryHelpers.h"

using namespace Marble;
using namespace MapAbstraction;

namespace MapLibraryHelpers
{
    GeoDataCoordinates transformCoords(const GeoCoords &in)
    {
        GeoDataCoordinates coords(in.longitude(), in.latitude(), 0.0, GeoDataCoordinates::Degree);
        return coords;
    }

    GeoCoords transformCoords(const GeoDataCoordinates &in)
    {
        GeoCoords coords(in.longitude(GeoDataCoordinates::Degree),
                         in.latitude(GeoDataCoordinates::Degree));
        return coords;
    }

    QString coordsString(const GeoCoords &in, int precision)
    {
        return coordsString(in.longitude(), in.latitude(), precision);
    }

    QString coordsString(const GeoDataCoordinates &in, int precision)
    {
        return coordsString(in.longitude(GeoDataCoordinates::Degree),
                            in.latitude(GeoDataCoordinates::Degree),
                            precision);
    }

    QString coordsString(qreal lon, qreal lat, int precision)
    {
        QString positionDisplayString = QString::number(lon,'f', precision);
        positionDisplayString.append(" " + QString::number(lat, 'f', precision));
        return positionDisplayString;
    }
}
