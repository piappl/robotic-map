#ifndef DYNAMICOBJECT_H
#define DYNAMICOBJECT_H

#include <QPoint>
#include <QVector>
#include "GeoCoords.h"
#include "MapObject.h"

namespace MapAbstraction
{
    enum DynamicObjectType
    {
        DynamicObjectUnknown,
        DynamicObjectPedestrian
    };

    struct DynamicObject
    {
        QString name;
        GeoCoords position;   //a triplet of coords (x, y, z) for the object. Global are in radians, local in meters
        double velocityX;
        double velocityY;
        double reliability;
        MapObject::LocalizationType localizationType;
        DynamicObjectType objectType;
    };

    typedef QList<DynamicObject> DynamicObjects;
}


#endif // DYNAMICOBJECT_H
