#ifndef MAPORDEREDOBJECT_H
#define MAPORDEREDOBJECT_H

#include "MapObject.h"

namespace MapAbstraction
{   //Abstract
    class MapOrderedObject : public MapObject
    {
    public:
        MapOrderedObject(GeoCoords coords, short number);
        virtual ~MapOrderedObject();
        short number() const;
        void setNumber(short number);
        virtual QString displayText() const;

    protected:
        virtual bool compare(const MapObject &other) const;

    private:
        short mNumber;
    };
}

#endif // MAPORDEREDOBJECT_H
