#ifndef CROSSHAIRLAYER_H
#define CROSSHAIRLAYER_H

#include "MapLayerInterface.h"

namespace MapAbstraction
{
    class CrosshairLayer : public MapLayerInterface
    {
    public:
        CrosshairLayer(RoboticsMap *rm);
        LayerType type() const { return LayerCrosshair; }
        QStringList renderPosition() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);
    };
    typedef QSharedPointer<CrosshairLayer> CrosshairLayerPtr;
}
#endif // CROSSHAIRLAYER_H
