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
        GeoCoords midCoords;   //a triplet of coords (x, y, z) for the object middle. Global are in radians, local in meters
        MapObject::LocalizationType localizationType;
        qreal radius;   //object radius
        DynamicObjectType objectType;
        qint32 objectId;
    };

    typedef QList<DynamicObject> DynamicObjects;
}


#endif // DYNAMICOBJECT_H
