#ifndef MAPFACTORY_H
#define MAPFACTORY_H

#include "IGeoMapFwd.h"

class QWidget;
class QQuickItem;

class MapFactory
{
public:
    MapAbstraction::IGeoMapPtr createMapItem(QQuickItem *viewMapItem);
};

#endif // MAPFACTORY_H
