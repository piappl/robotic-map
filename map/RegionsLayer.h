#ifndef REGIONSLAYER_H
#define REGIONSLAYER_H

#include "MapLayerInterface.h"
#include "GeoObjectsManager.h"

namespace MapAbstraction
{
    class RegionsLayer : public MapLayerInterface
    {
    public:
        RegionsLayer(RoboticsMap *rm);
        QStringList renderPosition() const;
        LayerType type() const { return LayerRegions; }
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);

    private:
        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
    };
    typedef QSharedPointer<RegionsLayer> RegionsLayerPtr;
}

#endif // REGIONSLAYER_H
