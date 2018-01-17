#ifndef MAPPLACE_H
#define MAPPLACE_H

#include "MapObject.h"

namespace MapAbstraction
{
    class MapPlaceObject : public MapObject
    {
    public:
        MapPlaceObject(GeoCoords coords, QString type, QString name);
        MapObject *Clone()  const;
        PlacemarkType category() const;
    protected:
        bool compare(const MapObject &other) const;

    private:
        virtual QString displayText() const;
    };
}

#endif // MAPPLACE_H
