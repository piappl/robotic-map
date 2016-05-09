#ifndef MAPLAYERSFWD_H
#define MAPLAYERSFWD_H

#include <QSharedPointer>

namespace Marble
{
    class LocalMapLayer;
    typedef QSharedPointer<LocalMapLayer> LocalMapLayerPtr;

    class PathsLayer;
    typedef QSharedPointer<PathsLayer> PathsLayerPtr;

    class CrosshairLayer;
    typedef QSharedPointer<CrosshairLayer> CrosshairLayerPtr;

    class RegionsLayer;
    typedef QSharedPointer<RegionsLayer> RegionsLayerPtr;

    class RobotManualPlacementLayer;
    typedef QSharedPointer<RobotManualPlacementLayer> RobotManualPlacementLayerPtr;

    class LaserCloudLayer;
    typedef QSharedPointer<LaserCloudLayer> LaserCloudLayerPtr;

    class DynamicObjectsLayer;
    typedef QSharedPointer<DynamicObjectsLayer> DynamicObjectsLayerPtr;
}


#endif // MAPLAYERSFWD_H
