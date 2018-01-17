#include <marble/GeoPainter.h>
#include <marble/GeoDataCoordinates.h>
#include <marble/GeoDataLineString.h>

#include "GeoObjectsManager.h"
#include "MapLogPlacemarkData.h"
#include "PathsLayer.h"
#include "MapLibraryHelpers.h"
#include "MapRobotObject.h"
#include "MapWaypointObject.h"
#include "RoboticsMap.h"

using namespace MapAbstraction;
using namespace Marble;

PathsLayer::PathsLayer(RoboticsMap *rm) : MapLayerInterface(rm), mMapLogData(rm->mapLog()),
    mGeoObjectsManager(rm->geoManager())
{
}

QStringList PathsLayer::renderPosition() const
{
    return QStringList() << "HOVERS_ABOVE_SURFACE";
}

bool PathsLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
{
    if (!visible() || mMapLogData->recordedObjects().isEmpty())
        return true;

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setOpacity(0.9);
    foreach(const GeoObjectID &objectID, mMapLogData->recordedObjects())
    {
        ObjectDataHistory history = mMapLogData->getData(objectID);
        if (!history.empty())
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
    painter->restore();
}
