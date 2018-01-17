#ifndef WAYPOINTSLAYER_H
#define WAYPOINTSLAYER_H

#include "MapLayerInterface.h"
#include "GeoObjectsManager.h"

namespace MapAbstraction
{
    class WaypointsLayer : public MapLayerInterface
    {
    public:
        WaypointsLayer(RoboticsMap *map);
        LayerType type() const { return LayerWaypoints; }
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport,
                    const QString &renderPos, Marble::GeoSceneLayer *layer);
        QStringList renderPosition() const;
    private:
        GeoObjectsManagerPtr mGeoManager;
    };
    typedef QSharedPointer<WaypointsLayer> WaypointsLayerPtr;
}

#endif // WAYPOINTSLAYER_H
