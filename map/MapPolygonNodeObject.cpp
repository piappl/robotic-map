#include "MapPolygonNodeObject.h"

namespace MapAbstraction
{
    MapPolygonNodeObject::MapPolygonNodeObject(GeoCoords coords, short number)
        : MapOrderedObject(coords, number)
    {
    }

    MapObject *MapPolygonNodeObject::Clone() const
    {
        return new MapPolygonNodeObject(*this);
    }

    PlacemarkType MapPolygonNodeObject::category() const
    {
        return PlacemarkPolygonNode;
    }

    bool MapPolygonNodeObject::compare(const MapObject &other) const
    {
        return MapOrderedObject::compare(other);
    }

    QString MapPolygonNodeObject::displayText() const
    {
        return QString::number(number());
    }
}
