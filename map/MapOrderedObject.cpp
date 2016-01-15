#include "MapOrderedObject.h"

namespace
{
    const QString typeName = "node";
}

namespace MapAbstraction
{
    MapOrderedObject::MapOrderedObject(GeoCoords coords, short number)
        : MapObject(coords, typeName, typeName + QString::number(number)),
          mNumber(number)
    {
    }

    MapOrderedObject::~MapOrderedObject()
    {
    }

    bool MapOrderedObject::compare(const MapObject &other) const
    {
        bool ret = false;
        const MapOrderedObject* otherObject = dynamic_cast<const MapOrderedObject*>(&other);
        if (otherObject)
        {
            ret = number() == otherObject->number();
        }
        return ret;
    }

    QString MapOrderedObject::displayText() const
    {
        return QString("");
    }

    short MapOrderedObject::number() const
    {
        return mNumber;
    }

    void MapOrderedObject::setNumber(short number)
    {
        mNumber = number;
    }
}

