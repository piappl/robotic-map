#include "MapPlaceObject.h"

namespace MapAbstraction
{
    MapPlaceObject::MapPlaceObject(GeoCoords coords, QString type,
                                   QString name)
        : MapObject(coords, type, name)
    {
    }

    MapObject *MapPlaceObject::Clone() const
    {
        return new MapPlaceObject(*this);
    }

    PlacemarkType MapPlaceObject::category() const
    {
        return PlacemarkPlace;
    }

    bool MapPlaceObject::compare(const MapObject &/*other*/) const
    {
        return true;
    }

    QString MapPlaceObject::displayText() const
    {
        QString returned;

        if (!type().isEmpty())
            returned += type() + " ";

        returned += name();
        return returned;
    }
}
