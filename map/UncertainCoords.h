#ifndef UNCERTAINCOORDS_H
#define UNCERTAINCOORDS_H

#include "GeoCoords.h"

namespace MapAbstraction
{
    struct UncertainCoords
    {
        GeoCoords mCoords;
        float mUncertaintity;
    };
}

#endif // UNCERTAINCOORDS_H
