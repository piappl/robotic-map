#include <marble/GeoPainter.h>
#include <marble/ViewportParams.h>
#include <marble/GeoDataLineString.h>
#include "MapLibraryHelpers.h"
#include "MapRobotObject.h"
#include "MapWaypointObject.h"
#include "WaypointsLayer.h"
#include "RoboticsMap.h"

using namespace Marble;
using namespace MapAbstraction;

WaypointsLayer::WaypointsLayer(RoboticsMap *rm)
    : MapLayerInterface(rm), mGeoManager(rm->geoManager())
{
}

QStringList WaypointsLayer::renderPosition() const
{
    return QStringList() << "HOVERS_ABOVE_SURFACE";
}

bool WaypointsLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
{
    if (!visible())
        return true;

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setOpacity(0.9);

    OrderedPoints waypoints = mGeoManager->orderedPoints(PlacemarkWaypoint);
    if (waypoints.size() > 0)
    {
        Marble::GeoDataLineString waypointsPath(Tessellate);
        if (mGeoManager->isAnyRobotConnected() && mGeoManager->getConnectedRobot())
        {
            waypointsPath.append(MapLibraryHelpers::transformCoords(mGeoManager->getConnectedRobot()->coords()));
        }

        QBrush brush(Oxygen::skyBlue2);
        QPen pen(brush, 2, Qt::DashLine);
        painter->setPen(pen);
        painter->setBrush(brush);

        foreach (int num, waypoints.keys())
        {
           PlacemarkPtr placemark = waypoints.value(num);
           MapObjectPtr mapObject = mGeoManager->getMapObjectForPlacemark(placemark);
           MapWaypointObjectPtr waypoint = mapObject.staticCast<MapWaypointObject>();

           if (!waypoint->reached())
           {
               GeoCoords coord = waypoint->coords();
               GeoDataCoordinates marbleCoords = MapLibraryHelpers::transformCoords(coord);
               waypointsPath << marbleCoords;
           }
        }
        if (waypointsPath.size() > 1)
            painter->drawPolyline(waypointsPath);
    }

    painter->restore();
    return true;
}
