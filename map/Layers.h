#ifndef LAYERS_H
#define LAYERS_H

#include <marble/MarbleMap.h>
#include "MapLayerInterface.h"
#include <QMap>
#include <QSharedPointer>

namespace MapAbstraction
{
    class Layers
    {
    public:
        Layers(RoboticsMap *map);
        MapLayerInterfacePtr getLayer(LayerType type);
        void setVisibility(LayerType type, bool visible);
        void toggleVisibility(LayerType type);
        bool hasLayer(LayerType type) const;

    private:
        QMap<LayerType, MapLayerInterfacePtr> mLayers;
    };
    typedef QSharedPointer<Layers> LayersPtr;
}

#endif // LAYERS_H
