#ifndef LASERCLOUDLAYER_H
#define LASERCLOUDLAYER_H

#include <QImage>
#include <marble/LayerInterface.h>
#include "InternalTypesFwd.h"
#include "MapAbstractionFwd.h"
#include "LaserScanPoint.h"
#include "MapLayersFwd.h"

class RoboticsMap;

namespace Marble
{
    class LaserCloudLayer : public QObject, public LayerInterface
    {
    Q_OBJECT
    public:
        LaserCloudLayer(LocalMapLayerPtr map);
        void updateContent(MapAbstraction::LaserScanPoints points, MapAbstraction::MapRobotObjectConstPtr robot);

        QStringList renderPosition() const;
        qreal zValue() const;
        bool render(GeoPainter *painter, ViewportParams *viewport, const QString &renderPos = "NONE", GeoSceneLayer *layer = 0);
        void setVisible(bool visible);
        bool visible() const;
        bool hasContent() const;

    private:
        LocalMapLayerPtr mLocalMap;
        MapAbstraction::LaserScanPoints mLaserPoints;
        MapAbstraction::MapRobotObjectConstPtr mRobot;

        bool mVisible;
        bool mHasContent;
    };
}

#endif // LASERCLOUDLAYER_H
