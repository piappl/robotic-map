#ifndef LASERCLOUDLAYER_H
#define LASERCLOUDLAYER_H

#include "MapLayerInterface.h"
#include "LaserScanPoint.h"
#include "LocalMapLayer.h"

namespace MapAbstraction
{
    class LaserCloudLayer : public MapLayerInterface
    {
    Q_OBJECT
    public:
        LaserCloudLayer(RoboticsMap *rm, LocalMapLayerPtr map);
        void updateContent(MapAbstraction::LaserScanPoints points, MapAbstraction::MapRobotObjectConstPtr robot);
        LayerType type() const { return LayerLaserCloud; }

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(Marble::GeoPainter *painter, Marble::ViewportParams *viewport, const QString &renderPos = "NONE",
                    Marble::GeoSceneLayer *layer = 0);
        bool hasContent() const;

    private:
        LocalMapLayerPtr mLocalMap;
        MapAbstraction::LaserScanPoints mLaserPoints;
        MapAbstraction::MapRobotObjectConstPtr mRobot;

        bool mHasContent = false;
    };
    typedef QSharedPointer<LaserCloudLayer> LaserCloudLayerPtr;
}

#endif // LASERCLOUDLAYER_H
