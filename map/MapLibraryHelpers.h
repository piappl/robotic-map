#ifndef MAPLIBRARYHELPERS_H
#define MAPLIBRARYHELPERS_H

#include <marble/GeoDataCoordinates.h>
#include "GeoCoords.h"

namespace
{
    const int defaultPrecision = 6;
}

namespace MapLibraryHelpers
{
    Marble::GeoDataCoordinates transformCoords(const MapAbstraction::GeoCoords &in);
    MapAbstraction::GeoCoords transformCoords(const Marble::GeoDataCoordinates &in);
    QString coordsString(const MapAbstraction::GeoCoords &in, int precision = defaultPrecision);
    QString coordsString(const Marble::GeoDataCoordinates &in, int precision = defaultPrecision);
    QString coordsString(qreal lon, qreal lat, int precision = defaultPrecision);
}

#endif // MAPLIBRARYHELPERS_H
