#ifndef MAPRECEIVER_H
#define MAPRECEIVER_H

#include <QString>
#include <QObject>
#include <QPointF>

#include "GeoCoords.h"
#include "LaserScanPoint.h"
#include "DynamicObject.h"
#include "Orientation.h"

namespace MapAbstraction
{
    class MapReceiver : public QObject
    {
        Q_OBJECT
        signals:
            void localMapChanged(QString newLocalMap, qreal resolution, MapAbstraction::GeoCoords approxCenter,
                                 MapAbstraction::Orientation rotation, QPointF origin);
            void laserPointsCloudChanged(int robotID, MapAbstraction::LaserScanPoints points);
            void dynamicObjectsChanged(int robotID, MapAbstraction::DynamicObjects dynamic);

        public slots:
            //Local map - The map has a starting location but can be moved by the user
            //resolution is meters per pixel.
            //approxCenter can be invalid coords (created with default constructor). In such case an arbitrary
            //point is selected as the first approximation.
            void updateLocalMap(QString localMapPng, qreal resolution, MapAbstraction::GeoCoords approxCenter,
                                Orientation rotation, QPointF origin);

            //A laser scan is received
            void updateLaserPointsCloud(int robotID, MapAbstraction::LaserScanPoints points);

            //A list of dynamic objects (such as pedestrians) detected by robot is received
            void updateDynamicObjects(int robotID, MapAbstraction::DynamicObjects dynamic);
    };
}
#endif // MAPRECEIVER
