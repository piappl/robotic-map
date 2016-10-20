#include <marble/GeoPainter.h>
#include <marble/GeoDataCoordinates.h>
#include <marble/GeoDataLineString.h>

#include "GeoObjectsManager.h"
#include "MapLogPlacemarkData.h"
#include "PathsLayer.h"
#include "MapLibraryHelpers.h"
#include "MapRobotObject.h"
#include "MapWaypointObject.h"

using namespace MapAbstraction;

namespace Marble
{
    PathsLayer::PathsLayer(MapLogPlacemarkDataPtr mapLogData
                                       ,GeoObjectsManagerPtr geoObjectsManager)
        : mMapLogData(mapLogData), mGeoObjectsManager(geoObjectsManager),
          mRobotPathsVisible(false), mWaypointPathsVisible(true)
    {
    }

    void PathsLayer::setRobotPathsVisibility(bool visible)
    {
        mRobotPathsVisible = visible;
    }

    bool PathsLayer::robotPathsVisible() const
    {
        return mRobotPathsVisible;
    }

    void PathsLayer::setWaypointPathsVisibility(bool visible)
    {
        mWaypointPathsVisible = visible;
    }

    bool PathsLayer::waypointPathsVisible() const
    {
        return mWaypointPathsVisible;
    }

    QStringList PathsLayer::renderPosition() const
    {
        return QStringList() << "HOVERS_ABOVE_SURFACE";
    }

    bool PathsLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
    {
        if (!mRobotPathsVisible && !mWaypointPathsVisible)
            return true;

        painter->save();
        painter->setRenderHint(QPainter::Antialiasing, true);
        painter->setOpacity(0.9);

        if (mRobotPathsVisible)
        {
            foreach(const GeoObjectID &objectID, mMapLogData->recordedObjects())
            {
                ObjectDataHistory history = mMapLogData->getData(objectID);
                PlacemarkPtr placemark = mGeoObjectsManager->getPlacemark(objectID);
                if (!history.empty() && placemark->isVisible())
                {
                    QBrush brush(Oxygen::skyBlue2, Qt::SolidPattern);
                    QPen pen(brush, 2);

                    painter->setPen(pen);
                    painter->setBrush(brush);

                    GeoDataLineString drawnPath(Tessellate);
                    foreach(const SinglePointDataPtr point, history)
                    {
                        GeoCoords coord = point->coords();
                        GeoDataCoordinates marbleCoords = MapLibraryHelpers::transformCoords(coord);
                        painter->drawEllipse(marbleCoords, 3.0, 3.0);
                        drawnPath << marbleCoords;
                    }
                    painter->drawPolyline(drawnPath);
                }
            }
        }

        if (mWaypointPathsVisible)
        {
            OrderedPoints waypoints = mGeoObjectsManager->orderedPoints(PlacemarkWaypoint);
            if (waypoints.size() > 0)
            {
                GeoDataLineString waypointsPath(Tessellate);
                {
                    if (mGeoObjectsManager->isAnyRobotConnected())
                    {
                        waypointsPath.append(MapLibraryHelpers::transformCoords(
                                                 mGeoObjectsManager->getConnectedRobot()->coords()));
                    }
                }

                QBrush brush(Oxygen::skyBlue2);
                QPen pen(brush, 2, Qt::DashLine);
                painter->setPen(pen);
                painter->setBrush(brush);

                foreach (int num, waypoints.keys())
                {
                   PlacemarkPtr placemark = waypoints.value(num);
                   MapObjectPtr mapObject = mGeoObjectsManager->getMapObjectForPlacemark(placemark);
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
        }

        painter->restore();
        return true;
    }
}
