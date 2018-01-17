#ifndef PATHSLAYER_H
#define PATHSLAYER_H

#include "MapLayerInterface.h"
#include "MapLogPlacemarkData.h"
#include "GeoObjectsManager.h"

namespace MapAbstraction
{
    class PathsLayer : public MapLayerInterface
    {
    public: //LayerInterface
        PathsLayer(RoboticsMap *rm);
        LayerType type() const { return LayerPaths; }
        QStringList renderPosition() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);
    private:
        MapAbstraction::MapLogPlacemarkDataPtr mMapLogData;
        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
    };
    typedef QSharedPointer<PathsLayer> PathsLayerPtr;
}

#endif // PATHSLAYER_H
