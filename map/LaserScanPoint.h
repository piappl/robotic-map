#ifndef LASERSCANPOINT_H
#define LASERSCANPOINT_H

#include <QPoint>
#include <QVector>
#include "GeoCoords.h"

namespace MapAbstraction
{
    typedef QPointF LaserScanPoint; //always in local map coordinates
    typedef QVector<LaserScanPoint> LaserScanPoints; //implicit memory sharing
}

#endif // LASERSCANPOINT_H
