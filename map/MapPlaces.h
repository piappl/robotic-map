#ifndef MAPPLACES_H
#define MAPPLACES_H

#include <QVector>
#include "MapObjectsFwd.h"
#include "MapPlaceObject.h"

namespace MapAbstraction
{
    typedef QVector<MapPlaceObjectConstPtr> MapPlaces;
    typedef QVector<MapWaypointObjectConstPtr> MapWaypoints;
}

#endif // MAPPLACES_H
