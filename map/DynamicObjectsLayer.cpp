#include <marble/ViewportParams.h>
#include <marble/GeoPainter.h>

#include "DynamicObjectsLayer.h"
#include "LocalMapLayer.h"
#include "MapLibraryHelpers.h"
#include "MapRobotObject.h"
#include "MapIconProvider.h"

using namespace MapAbstraction;
using namespace Marble;

namespace
{
    const int disappearTime = 5000;
}

DynamicObjectsLayer::DynamicObjectsLayer(RoboticsMap *rm, LocalMapLayerPtr localMap)
    : MapLayerInterface(rm), mLocalMap(localMap)
{
    connect(&mCleanupTimer, SIGNAL(timeout()), this, SLOT(cleanDynamicObjects()));
}

void DynamicObjectsLayer::updateContent(MapAbstraction::DynamicObjects objects,
                                    MapAbstraction::MapRobotObjectConstPtr robot)
{
    mCleanupTimer.stop();
    mDynamicObjects = objects;
    mHasContent = true;
    mRobot = robot;
    mCleanupTimer.start(disappearTime);
    emit requestUpdate();
}

void DynamicObjectsLayer::cleanDynamicObjects()
{
    mDynamicObjects.clear();
}

QStringList DynamicObjectsLayer::renderPosition() const
{
    return QStringList() << "HOVERS_ABOVE_SURFACE";
}

qreal DynamicObjectsLayer::zValue() const
{
    const qreal DynamicObjectsLayerZPosition = 3.0;
    return DynamicObjectsLayerZPosition;
}

bool DynamicObjectsLayer::render(GeoPainter *painter, ViewportParams*, const QString&, GeoSceneLayer*)
{
    if (!visible() || !mHasContent)
        return true;

    if (!mRobot->connected())  //Only display objects for currently active robot
        return true;

    //painter->save();
    //painter->setOpacity(1);

    MapIconProvider iconProvider;
    foreach(DynamicObject o, mDynamicObjects)
    {
        if (o.localizationType != MapObject::Global && (!mLocalMap->visible() || !mLocalMap->hasContent()))
            continue; //No local map to display local objects, skipping

        GeoDataCoordinates marbleCoords = mLocalMap->robotToGlobal(
                    QPointF(o.midCoords.longitude(), o.midCoords.latitude()), mRobot);
        painter->drawImage(marbleCoords, iconProvider.getDynamicObjectIcon(DynamicObjectPedestrian));
    }

    //painter->restore();
    return true;
}
