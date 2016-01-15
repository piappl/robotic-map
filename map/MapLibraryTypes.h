#ifndef MAPLIBRARYTYPES_H
#define MAPLIBRARYTYPES_H

#include <QTime>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataDocument.h>

typedef Marble::GeoDataPlacemark* PlacemarkPtr;
typedef const Marble::GeoDataPlacemark* PlacemarkConstPtr;
typedef Marble::GeoDataDocument* DocumentPtr;
typedef Marble::GeoDataCoordinates Coordinates;
typedef QPair<QTime, Coordinates> TimedCoords;

#endif // MAPLIBRARYTYPES_H
