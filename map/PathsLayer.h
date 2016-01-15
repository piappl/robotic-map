#ifndef PATHSLAYER_H
#define PATHSLAYER_H

#include <marble/LayerInterface.h>
#include "InternalTypesFwd.h"

namespace Marble
{
    class PathsLayer : public LayerInterface
    {
    public: //LayerInterface
        PathsLayer(MapAbstraction::MapLogPlacemarkDataPtr mapLogData,
                         MapAbstraction::GeoObjectsManagerPtr geoObjectsManager);
        QStringList renderPosition() const;
        bool render(GeoPainter *painter, ViewportParams *viewport, const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setRobotPathsVisibility(bool visible);
        bool robotPathsVisible() const;
        void setWaypointPathsVisibility(bool visible);
        bool waypointPathsVisible() const;

    private:
        MapAbstraction::MapLogPlacemarkDataPtr mMapLogData;
        MapAbstraction::GeoObjectsManagerPtr mGeoObjectsManager;
        bool mRobotPathsVisible;
        bool mWaypointPathsVisible;
    };
}

#endif // PATHSLAYER_H
