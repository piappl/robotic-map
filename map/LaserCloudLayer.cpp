#include <marble/ViewportParams.h>
#include <marble/GeoPainter.h>

#include "LaserCloudLayer.h"
#include "LocalMapLayer.h"
#include "MapLibraryHelpers.h"
#include "MapRobotObject.h"
#include "RoboticsMap.h"

using namespace MapAbstraction;
using namespace Marble;

LaserCloudLayer::LaserCloudLayer(RoboticsMap *rm, LocalMapLayerPtr map)
    : MapLayerInterface(rm), mLocalMap(map)
{
}

void LaserCloudLayer::updateContent(MapAbstraction::LaserScanPoints points,
                                    MapAbstraction::MapRobotObjectConstPtr robot)
{
    mLaserPoints = points;
    mHasContent = true;
    mRobot = robot;
    emit requestUpdate();
}

QStringList LaserCloudLayer::renderPosition() const
{
    return QStringList() << "HOVERS_ABOVE_SURFACE";
}

qreal LaserCloudLayer::zValue() const
{
    const qreal LaserCloudLayerZPosition = 2.0;
    return LaserCloudLayerZPosition;
}

bool LaserCloudLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
{
    if (!visible() || !mHasContent || !mLocalMap->visible() || !mLocalMap->hasContent()) //TODO
        return true;

    if (!mRobot->connected())  //Only display the cloud for currently active robot
        return true;

    painter->save();
    painter->setOpacity(1);
    QPen pen(Oxygen::brickRed2);
    pen.setWidth(3);
    painter->setPen(pen);

    const int optimizeToOrder = 500;
    int every = mLaserPoints.size() / optimizeToOrder;
    if (every < 1)
        every = 1;
    int counter = 1;

    foreach(LaserScanPoint p, mLaserPoints)
    {   //Should be optimized (make a png);
        if (counter >= every)
        {
            GeoDataCoordinates marbleCoords = mLocalMap->robotToGlobal(p, mRobot);
            painter->drawPoint(marbleCoords);
            counter = 1;
        }
        else
        {
            counter++;
        }
    }

    painter->restore();
    return true;
}
