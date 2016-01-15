#ifndef MAPOBJECTSFWD_H
#define MAPOBJECTSFWD_H

#include <QSharedPointer>

namespace MapAbstraction
{
    class MapObject;
    typedef QSharedPointer<MapObject> MapObjectPtr;
    typedef QSharedPointer<const MapObject> MapObjectConstPtr;

    class MapRobotObject;
    typedef QSharedPointer<MapRobotObject> MapRobotObjectPtr;
    typedef QSharedPointer<const MapRobotObject> MapRobotObjectConstPtr;

    class MapPlaceObject;
    typedef QSharedPointer<MapPlaceObject> MapPlaceObjectPtr;
    typedef QSharedPointer<const MapPlaceObject> MapPlaceObjectConstPtr;

    class MapOrderedObject;
    typedef QSharedPointer<MapOrderedObject> MapOrderedObjectPtr;
    typedef QSharedPointer<const MapOrderedObject> MapOrderedObjectConstPtr;

    class MapWaypointObject;
    typedef QSharedPointer<MapWaypointObject> MapWaypointObjectPtr;
    typedef QSharedPointer<const MapWaypointObject> MapWaypointObjectConstPtr;

    class MapPolygonNodeObject;
    typedef QSharedPointer<MapPolygonNodeObject> MapPolygonNodeObjectPtr;
    typedef QSharedPointer<const MapPolygonNodeObject> MapPolygonNodeObjectConstPtr;
}

#endif // MAPOBJECTSFWD_H
