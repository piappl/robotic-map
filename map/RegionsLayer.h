#ifndef REGIONSLAYER_H
#define REGIONSLAYER_H

#include <marble/LayerInterface.h>
#include "InternalTypesFwd.h"

namespace Marble
{
    class RegionsLayer : public LayerInterface
    {
    public:
        RegionsLayer(MapAbstraction::GeoObjectsManagerPtr geoObjectsManager);
        QStringList renderPosition() const;
        bool render(GeoPainter *painter, ViewportParams *viewport, const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setVisible(bool visible);
        bool visible() const;

    private:
        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
        bool mVisible;
    };
}

#endif // REGIONSLAYER_H
