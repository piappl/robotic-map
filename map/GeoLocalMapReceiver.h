#ifndef GEOLOCALMAPRECEIVER_H
#define GEOLOCALMAPRECEIVER_H

#include <QString>
#include <QObject>
#include <QPointF>

#include "GeoCoords.h"
#include "LaserScanPoint.h"
#include "Orientation.h"

namespace MapAbstraction
{
    class GeoLocalMapReceiver : public QObject
    {
        Q_OBJECT
        signals:
            void localMapChanged(QString newLocalMap, qreal resolution, MapAbstraction::GeoCoords approxCenter,
                                 MapAbstraction::Orientation rotation, QPointF origin);
            void laserPointsCloudChanged(int robotID, MapAbstraction::LaserScanPoints points);

        public slots:
            //Local map - The map has a starting location but can be moved by the user
            //resolution is meters per pixel.
            //approxCenter can be invalid coords (created with default constructor). In such case an arbitrary
            //point is selected as the first approximation.
            void updateLocalMap(QString localMapPng, qreal resolution, MapAbstraction::GeoCoords approxCenter,
                                Orientation rotation, QPointF origin);

            //A laser scan is received
            void updateLaserPointsCloud(int robotID, MapAbstraction::LaserScanPoints points);
    };
}
#endif // GEOLOCALMAPRECEIVER
