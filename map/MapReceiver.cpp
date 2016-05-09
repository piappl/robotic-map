#include "MapReceiver.h"
#include "Orientation.h"

namespace MapAbstraction
{
    void MapReceiver::updateLocalMap(QString localMapPng, qreal resolution,
                                             GeoCoords approxCenter, Orientation rotation, QPointF origin)
    {
        emit localMapChanged(localMapPng, resolution, approxCenter, rotation, origin);
    }

    void MapReceiver::updateLaserPointsCloud(int robotID, LaserScanPoints points)
    {   //TODO Do some performance filtering maybe
        //qDebug("Received points %d", points.size());
        emit laserPointsCloudChanged(robotID, points);
    }

    void MapReceiver::updateDynamicObjects(int robotID, DynamicObjects dynamic)
    {
        emit dynamicObjectsChanged(robotID, dynamic);
    }
}
