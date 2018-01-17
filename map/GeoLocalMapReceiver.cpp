#include "GeoLocalMapReceiver.h"
#include "Orientation.h"

namespace MapAbstraction
{
    void GeoLocalMapReceiver::updateLocalMap(QString localMapPng, qreal resolution,
                                             GeoCoords approxCenter, Orientation rotation, QPointF origin)
    {
        emit localMapChanged(localMapPng, resolution, approxCenter, rotation, origin);
    }

    void GeoLocalMapReceiver::updateLaserPointsCloud(int robotID, LaserScanPoints points)
    {   //TODO Do some performance filtering maybe
        //qDebug("Received points %d", points.size());
        emit laserPointsCloudChanged(robotID, points);
    }

    void GeoLocalMapReceiver::updateNamedPlaces(MapPlaces mapPlaces)
    {
        emit namedPlacesChanged(mapPlaces);
    }
}
